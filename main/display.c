#include "display.h"
#include "session.h"
#include "temperature.h"
#include "config.h"
#include "esp_log.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <stdio.h>

static const char *TAG = "DISPLAY";

/* ── ST7789V registers ─────────────────────────────────── */
#define ST_SWRESET  0x01
#define ST_SLPOUT   0x11
#define ST_COLMOD   0x3A
#define ST_MADCTL   0x36
#define ST_CASET    0x2A
#define ST_RASET    0x2B
#define ST_RAMWR    0x2C
#define ST_DISPON   0x29
#define ST_INVON    0x21

#define DISPLAY_W   240
#define DISPLAY_H   320

/* 16-bit colour (RGB565) helpers */
#define COLOR_BLACK   0x0000
#define COLOR_WHITE   0xFFFF
#define COLOR_RED     0xF800
#define COLOR_GREEN   0x07E0
#define COLOR_AMBER   0xFD20
#define COLOR_BLUE    0x001F
#define COLOR_GRAY    0x7BEF
#define COLOR_BG      0x0841  /* Dark background */

static spi_device_handle_t s_spi;

/* ── SPI helpers ───────────────────────────────────────── */

static void dc_cmd(void)  { gpio_set_level(PIN_SPI_DC, 0); }
static void dc_data(void) { gpio_set_level(PIN_SPI_DC, 1); }

static void spi_write(const uint8_t *data, size_t len)
{
    if (len == 0) return;
    spi_transaction_t t = {
        .length    = len * 8,
        .tx_buffer = data,
    };
    spi_device_transmit(s_spi, &t);
}

static void cmd(uint8_t c)
{
    dc_cmd();
    spi_write(&c, 1);
}

static void data1(uint8_t d)
{
    dc_data();
    spi_write(&d, 1);
}

static void data16(uint16_t d)
{
    dc_data();
    uint8_t buf[2] = { d >> 8, d & 0xFF };
    spi_write(buf, 2);
}

/* ── Window + fill ─────────────────────────────────────── */

static void set_window(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
    cmd(ST_CASET);
    dc_data();
    uint8_t ca[4] = { x0 >> 8, x0, x1 >> 8, x1 };
    spi_write(ca, 4);

    cmd(ST_RASET);
    dc_data();
    uint8_t ra[4] = { y0 >> 8, y0, y1 >> 8, y1 };
    spi_write(ra, 4);

    cmd(ST_RAMWR);
    dc_data();
}

static void fill_rect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color)
{
    set_window(x, y, x + w - 1, y + h - 1);
    uint8_t buf[2] = { color >> 8, color & 0xFF };
    uint32_t pixels = (uint32_t)w * h;
    for (uint32_t i = 0; i < pixels; i++) {
        spi_write(buf, 2);
    }
}

/* ── Minimal 5×7 pixel font ────────────────────────────── */

static const uint8_t font5x7[][5] = {
    {0x00,0x00,0x00,0x00,0x00}, /* ' ' */
    {0x00,0x00,0x5F,0x00,0x00}, /* '!' */
    {0x3E,0x51,0x49,0x45,0x3E}, /* '0' - index 2 */
    {0x00,0x42,0x7F,0x40,0x00}, /* '1' */
    {0x42,0x61,0x51,0x49,0x46}, /* '2' */
    {0x21,0x41,0x45,0x4B,0x31}, /* '3' */
    {0x18,0x14,0x12,0x7F,0x10}, /* '4' */
    {0x27,0x45,0x45,0x45,0x39}, /* '5' */
    {0x3C,0x4A,0x49,0x49,0x30}, /* '6' */
    {0x01,0x71,0x09,0x05,0x03}, /* '7' */
    {0x36,0x49,0x49,0x49,0x36}, /* '8' */
    {0x06,0x49,0x49,0x29,0x1E}, /* '9' */
    {0x00,0x36,0x36,0x00,0x00}, /* ':' */
    {0x08,0x14,0x22,0x14,0x08}, /* '°' placeholder */
    {0x7F,0x09,0x09,0x09,0x06}, /* 'F' */
    {0x7F,0x09,0x09,0x09,0x01}, /* 'E' */
    {0x7F,0x49,0x49,0x49,0x41}, /* 'H' */
    {0x00,0x41,0x7F,0x41,0x00}, /* 'I' */
    {0x7F,0x09,0x19,0x29,0x46}, /* 'R' */
    {0x38,0x44,0x44,0x44,0x38}, /* 'O' */
    {0x7F,0x09,0x09,0x09,0x06}, /* 'P' → reuse F glyph */
    {0x36,0x49,0x55,0x22,0x50}, /* 'S' */
    {0x01,0x01,0x7F,0x01,0x01}, /* 'T' */
    {0x7F,0x40,0x40,0x40,0x7F}, /* 'U' */
    {0x06,0x09,0x09,0x09,0x06}, /* 'C' */
    {0x20,0x54,0x54,0x54,0x78}, /* 'a' */
    {0x7F,0x48,0x44,0x44,0x38}, /* 'b' */
    {0x00,0x08,0x08,0x00,0x00}, /* '-' */
};

static void draw_char(uint16_t x, uint16_t y, char c, uint16_t fg, uint16_t bg, uint8_t scale)
{
    /* Map character to glyph index (very small set for status display) */
    int idx = 0;
    if (c >= '0' && c <= '9') idx = 2 + (c - '0');
    else if (c == ':')  idx = 12;
    else if (c == 'C')  idx = 24;
    else if (c == 'E')  idx = 15;
    else if (c == 'H')  idx = 16;
    else if (c == 'I')  idx = 17;
    else if (c == 'O')  idx = 19;
    else if (c == 'R')  idx = 18;
    else if (c == 'S')  idx = 21;
    else if (c == 'T')  idx = 22;
    else if (c == '-')  idx = 27;
    else idx = 0; /* space */

    for (int col = 0; col < 5; col++) {
        uint8_t line = font5x7[idx][col];
        for (int row = 0; row < 7; row++) {
            uint16_t color = (line & (1 << row)) ? fg : bg;
            fill_rect(x + col * scale, y + row * scale, scale, scale, color);
        }
    }
}

static void draw_string(uint16_t x, uint16_t y, const char *s,
                         uint16_t fg, uint16_t bg, uint8_t scale)
{
    while (*s) {
        draw_char(x, y, *s, fg, bg, scale);
        x += (5 + 1) * scale;
        s++;
    }
}

/* ── Display init ──────────────────────────────────────── */

esp_err_t display_init(void)
{
    spi_bus_config_t buscfg = {
        .mosi_io_num   = PIN_SPI_MOSI,
        .miso_io_num   = -1,
        .sclk_io_num   = PIN_SPI_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = DISPLAY_W * DISPLAY_H * 2,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO));

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 40 * 1000 * 1000,  /* 40MHz */
        .mode           = 0,
        .spics_io_num   = PIN_SPI_CS,
        .queue_size     = 7,
    };
    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &devcfg, &s_spi));

    gpio_set_direction(PIN_SPI_DC,  GPIO_MODE_OUTPUT);
    gpio_set_direction(PIN_SPI_RST, GPIO_MODE_OUTPUT);
    gpio_set_direction(PIN_SPI_BL,  GPIO_MODE_OUTPUT);

    /* Hardware reset */
    gpio_set_level(PIN_SPI_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(PIN_SPI_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(120));

    cmd(ST_SWRESET);  vTaskDelay(pdMS_TO_TICKS(150));
    cmd(ST_SLPOUT);   vTaskDelay(pdMS_TO_TICKS(10));
    cmd(ST_COLMOD);   data1(0x55);   /* 16-bit RGB565 */
    cmd(ST_MADCTL);   data1(0x00);
    cmd(ST_INVON);
    cmd(ST_DISPON);   vTaskDelay(pdMS_TO_TICKS(10));

    /* Clear to background */
    fill_rect(0, 0, DISPLAY_W, DISPLAY_H, COLOR_BG);

    /* Backlight on */
    gpio_set_level(PIN_SPI_BL, 1);

    ESP_LOGI(TAG, "ST7789V ready");
    return ESP_OK;
}

/* ── Display update ────────────────────────────────────── */

void display_update(void)
{
    cabir_state_t state   = session_get_state();
    int16_t plate_raw     = session_get_plate_temp();
    int16_t liquid_raw    = session_get_liquid_temp();
    uint8_t target        = session_get_target_temp();
    uint16_t remaining    = session_get_timer_remaining();
    uint8_t alarms        = session_get_alarm_flags();

    /* Top bar: state label */
    const char *state_str;
    uint16_t state_color;
    switch (state) {
        case STATE_HEATING: state_str = "HEATING"; state_color = COLOR_AMBER;  break;
        case STATE_HOLDING: state_str = "HOLDING"; state_color = COLOR_GREEN;  break;
        case STATE_COOLING: state_str = "COOLING"; state_color = COLOR_BLUE;   break;
        case STATE_ALARM:   state_str = "ALARM";   state_color = COLOR_RED;    break;
        default:            state_str = "IDLE";    state_color = COLOR_GRAY;   break;
    }
    fill_rect(0, 0, DISPLAY_W, 30, state_color);
    draw_string(10, 8, state_str, COLOR_BLACK, state_color, 2);

    /* Plate temperature */
    fill_rect(0, 40, DISPLAY_W, 80, COLOR_BG);
    char buf[24];
    if (plate_raw == CABIR_TEMP_FAULT) {
        draw_string(10, 60, "SENSOR ERR", COLOR_RED, COLOR_BG, 2);
    } else {
        snprintf(buf, sizeof(buf), "%d.%dC",
                 plate_raw / 10, (plate_raw < 0 ? -plate_raw : plate_raw) % 10);
        draw_string(10, 50, buf, COLOR_WHITE, COLOR_BG, 3);
        draw_string(10, 90, "PLATE", COLOR_GRAY, COLOR_BG, 1);
    }

    /* Liquid temperature */
    fill_rect(0, 110, DISPLAY_W, 50, COLOR_BG);
    if (liquid_raw == (int16_t)NTC_DISCONNECTED) {
        draw_string(10, 120, "EXT:--", COLOR_GRAY, COLOR_BG, 2);
    } else {
        snprintf(buf, sizeof(buf), "EXT:%d.%dC",
                 liquid_raw / 10,
                 (liquid_raw < 0 ? -liquid_raw : liquid_raw) % 10);
        draw_string(10, 120, buf, COLOR_GREEN, COLOR_BG, 2);
    }

    /* Target */
    fill_rect(0, 170, DISPLAY_W, 40, COLOR_BG);
    snprintf(buf, sizeof(buf), "TGT:%dC", target);
    draw_string(10, 178, buf, COLOR_AMBER, COLOR_BG, 2);

    /* Timer remaining */
    fill_rect(0, 220, DISPLAY_W, 40, COLOR_BG);
    if (remaining == 0xFFFF) {
        draw_string(10, 228, "HOLD", COLOR_GRAY, COLOR_BG, 2);
    } else {
        uint16_t mm = remaining / 60;
        uint16_t ss = remaining % 60;
        snprintf(buf, sizeof(buf), "%02d:%02d", mm, ss);
        draw_string(10, 228, buf, COLOR_WHITE, COLOR_BG, 2);
    }

    /* Alarm indicator */
    fill_rect(0, 270, DISPLAY_W, 50, COLOR_BG);
    if (alarms) {
        snprintf(buf, sizeof(buf), "ALARM:%02X", alarms);
        draw_string(10, 280, buf, COLOR_RED, COLOR_BG, 2);
    }
}
