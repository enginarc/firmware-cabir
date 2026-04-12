#include "tpc.h"
#include "config.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <stdbool.h>
#include <stdint.h>

static const char *TAG = "TPC";

static volatile float s_duty      = 0.0f;   /* 0.0 – 100.0 */
static volatile bool  s_forced_off = false;
static SemaphoreHandle_t s_mutex;

static void ssr_set(bool on)
{
    gpio_set_level(PIN_SSR_EN, on ? 1 : 0);
}

static void tpc_task(void *arg)
{
    const TickType_t window_ticks = pdMS_TO_TICKS(TPC_WINDOW_MS);

    while (1) {
        float duty;
        bool  forced;

        xSemaphoreTake(s_mutex, portMAX_DELAY);
        duty   = s_duty;
        forced = s_forced_off;
        xSemaphoreGive(s_mutex);

        if (forced || duty <= 0.0f) {
            ssr_set(false);
            vTaskDelay(window_ticks);
            continue;
        }

        if (duty >= 100.0f) {
            ssr_set(true);
            vTaskDelay(window_ticks);
            continue;
        }

        /* ON phase */
        uint32_t on_ms = (uint32_t)((duty / 100.0f) * TPC_WINDOW_MS);
        ssr_set(true);
        vTaskDelay(pdMS_TO_TICKS(on_ms));

        /* OFF phase — re-check forced_off */
        xSemaphoreTake(s_mutex, portMAX_DELAY);
        forced = s_forced_off;
        xSemaphoreGive(s_mutex);

        if (forced) {
            ssr_set(false);
        } else {
            uint32_t off_ms = TPC_WINDOW_MS - on_ms;
            /* ssr already on; wait off period with SSR off */
            ssr_set(false);
            vTaskDelay(pdMS_TO_TICKS(off_ms));
        }
    }
}

esp_err_t tpc_init(void)
{
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << PIN_SSR_EN),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&io));
    ssr_set(false);

    s_mutex = xSemaphoreCreateMutex();
    if (!s_mutex) return ESP_ERR_NO_MEM;

    xTaskCreate(tpc_task, "tpc", 2048, NULL, 10, NULL);
    ESP_LOGI(TAG, "TPC ready — window %dms", TPC_WINDOW_MS);
    return ESP_OK;
}

void tpc_set_duty(float duty_percent)
{
    if (duty_percent < 0.0f) duty_percent = 0.0f;
    if (duty_percent > 100.0f) duty_percent = 100.0f;
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    s_duty = duty_percent;
    xSemaphoreGive(s_mutex);
}

void tpc_force_off(void)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    s_forced_off = true;
    xSemaphoreGive(s_mutex);
    ssr_set(false);
    ESP_LOGI(TAG, "SSR forced OFF");
}

void tpc_release(void)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    s_forced_off = false;
    xSemaphoreGive(s_mutex);
    ESP_LOGI(TAG, "SSR released");
}

bool tpc_is_forced_off(void)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    bool v = s_forced_off;
    xSemaphoreGive(s_mutex);
    return v;
}
