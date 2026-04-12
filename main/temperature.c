#include "temperature.h"
#include "config.h"
#include "esp_log.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include <math.h>
#include <stdint.h>
#include <stdbool.h>

static const char *TAG = "TEMP";

static adc_oneshot_unit_handle_t s_adc1_handle;
static adc_cali_handle_t         s_cali_handle;
static bool                      s_cali_enabled = false;
static bool                      s_plate_fault  = false;

/* ── ADC calibration ─────────────────────────────────────── */

static void cali_init(void)
{
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    adc_cali_curve_fitting_config_t cfg = {
        .unit_id  = ADC_UNIT_1,
        .atten    = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12,
    };
    if (adc_cali_create_scheme_curve_fitting(&cfg, &s_cali_handle) == ESP_OK) {
        s_cali_enabled = true;
        return;
    }
#endif
#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    adc_cali_line_fitting_config_t cfg = {
        .unit_id  = ADC_UNIT_1,
        .atten    = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12,
    };
    if (adc_cali_create_scheme_line_fitting(&cfg, &s_cali_handle) == ESP_OK) {
        s_cali_enabled = true;
    }
#endif
    if (!s_cali_enabled) {
        ESP_LOGW(TAG, "ADC calibration not available — using raw values");
    }
}

/* ── NTC conversion ──────────────────────────────────────── */

/*
 * Voltage divider: 3.3V ─ R_fixed(10K) ─ ADC_PIN ─ NTC ─ GND
 *
 * R_ntc = R_fixed × V_adc / (V_supply − V_adc)
 *
 * Steinhart-Hart simplified (Beta equation):
 * 1/T = 1/T0 + (1/B) × ln(R/R0)
 */
static int16_t raw_to_celsius_x10(int raw_mv, bool *fault)
{
    *fault = false;

    /* Clamp: open circuit (NTC disconnected → R→∞ → V_adc→0) */
    if (raw_mv < 50) {
        *fault = true;
        return CABIR_TEMP_FAULT;
    }
    /* Clamp: short circuit (NTC shorted → R→0 → V_adc→3300mV) */
    if (raw_mv > 3250) {
        *fault = true;
        return CABIR_TEMP_FAULT;
    }

    float v_adc    = (float)raw_mv / 1000.0f;           /* volts */
    float v_supply = 3.3f;
    float r_ntc    = (float)NTC_R_FIXED * v_adc / (v_supply - v_adc);

    float temp_k = 1.0f / (
        1.0f / NTC_T0_K +
        (1.0f / (float)NTC_BETA) * logf(r_ntc / (float)NTC_R0)
    );

    float temp_c = temp_k - 273.15f;
    return (int16_t)(temp_c * 10.0f);
}

/* ── Oversampled read ────────────────────────────────────── */

static int read_mv(adc_channel_t ch)
{
    int sum = 0;
    int raw = 0;
    for (int i = 0; i < ADC_OVERSAMPLE; i++) {
        adc_oneshot_read(s_adc1_handle, ch, &raw);
        sum += raw;
    }
    int avg_raw = sum / ADC_OVERSAMPLE;

    if (s_cali_enabled) {
        int mv = 0;
        adc_cali_raw_to_voltage(s_cali_handle, avg_raw, &mv);
        return mv;
    }
    /* Fallback: linear approximation */
    return (int)((float)avg_raw * 3300.0f / 4095.0f);
}

/* ── Public API ──────────────────────────────────────────── */

esp_err_t temperature_init(void)
{
    adc_oneshot_unit_init_cfg_t unit_cfg = {
        .unit_id  = ADC_UNIT_1,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&unit_cfg, &s_adc1_handle));

    adc_oneshot_chan_cfg_t ch_cfg = {
        .atten    = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(s_adc1_handle,
                                               ADC_CHANNEL_PLATE, &ch_cfg));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(s_adc1_handle,
                                               ADC_CHANNEL_LIQUID, &ch_cfg));
    cali_init();
    ESP_LOGI(TAG, "Temperature subsystem ready");
    return ESP_OK;
}

int16_t temperature_read_plate(void)
{
    int mv = read_mv(ADC_CHANNEL_PLATE);
    bool fault = false;
    int16_t t  = raw_to_celsius_x10(mv, &fault);
    s_plate_fault = fault;
    if (fault) {
        ESP_LOGW(TAG, "Plate NTC fault (mv=%d)", mv);
    }
    return t;
}

int16_t temperature_read_liquid(void)
{
    int mv = read_mv(ADC_CHANNEL_LIQUID);

    /* Near-rail → open circuit → sensor not connected */
    if (mv < 50 || mv > 3250) {
        return (int16_t)NTC_DISCONNECTED;
    }

    bool fault = false;
    int16_t t  = raw_to_celsius_x10(mv, &fault);
    return fault ? (int16_t)NTC_DISCONNECTED : t;
}

bool temperature_plate_fault(void)
{
    return s_plate_fault;
}
