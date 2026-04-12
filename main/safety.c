#include "safety.h"
#include "tpc.h"
#include "config.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

static const char *TAG = "SAFETY";

/*
 * Hardware-clearable flags: ALARM_PLATE_FAULT, ALARM_HW_CUTOFF
 * Software-clearable flags: ALARM_OVERSHOOT, ALARM_LIQUID_LOST, ALARM_BLE_WATCHDOG
 */
#define ALARM_HW_MASK  (ALARM_PLATE_FAULT | ALARM_HW_CUTOFF)
#define ALARM_SW_MASK  (ALARM_OVERSHOOT | ALARM_LIQUID_LOST | ALARM_BLE_WATCHDOG)

static volatile uint8_t s_alarms = 0;
static SemaphoreHandle_t s_mutex;

esp_err_t safety_init(void)
{
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << PIN_KSD_WARN),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,   /* Active LOW when KSD301 NO closes */
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&io));
    s_mutex = xSemaphoreCreateMutex();
    if (!s_mutex) return ESP_ERR_NO_MEM;
    ESP_LOGI(TAG, "Safety subsystem ready");
    return ESP_OK;
}

uint8_t safety_check(float plate_c, float target_c, bool plate_fault)
{
    uint8_t new_alarms = 0;

    /* Plate sensor fault */
    if (plate_fault) {
        new_alarms |= ALARM_PLATE_FAULT;
        ESP_LOGE(TAG, "Plate NTC fault — forcing SSR off");
        tpc_force_off();
    }

    /* Plate over-temp (firmware ceiling) */
    if (!plate_fault && plate_c >= (float)TEMP_ALARM_PLATE_C) {
        new_alarms |= ALARM_PLATE_FAULT;
        ESP_LOGE(TAG, "Plate over-temp %.1f°C — forcing SSR off", plate_c);
        tpc_force_off();
    }

    /* Target overshoot */
    if (!plate_fault && plate_c > (target_c + (float)TEMP_OVERSHOOT_C)) {
        new_alarms |= ALARM_OVERSHOOT;
    }

    /* KSD301 NC hardware cutoff detection (indirect):
     * If KSD_WARN (NO 60°C) is active but the plate fault alarm has already
     * fired and SSR is forced off, flag ALARM_HW_CUTOFF to indicate the
     * electromechanical chain has intervened. */
    if (safety_ksd_warn_active() && plate_c >= 70.0f) {
        new_alarms |= ALARM_HW_CUTOFF;
        ESP_LOGW(TAG, "KSD301 NC region reached (%.1f°C) — hardware cutoff active", plate_c);
    }

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    s_alarms |= new_alarms;
    uint8_t current = s_alarms;
    xSemaphoreGive(s_mutex);

    return current;
}

uint8_t safety_get_alarms(void)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    uint8_t v = s_alarms;
    xSemaphoreGive(s_mutex);
    return v;
}

bool safety_clear_alarms(uint8_t key)
{
    if (key != ALARM_CLEAR_KEY) return false;
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    /* Only software-clearable bits can be cleared */
    s_alarms &= ALARM_HW_MASK;
    bool cleared = true;
    xSemaphoreGive(s_mutex);
    ESP_LOGI(TAG, "Software alarms cleared");

    /* If no hardware alarms remain, release SSR forced-off */
    if ((s_alarms & ALARM_HW_MASK) == 0) {
        tpc_release();
    }
    return cleared;
}

bool safety_ksd_warn_active(void)
{
    /* KSD301 NO: active LOW (closed = GPIO reads 0) */
    return gpio_get_level(PIN_KSD_WARN) == 0;
}

void safety_set_ble_watchdog(bool active)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    if (active) {
        s_alarms |= ALARM_BLE_WATCHDOG;
    } else {
        s_alarms &= ~ALARM_BLE_WATCHDOG;
    }
    xSemaphoreGive(s_mutex);
    if (active) {
        ESP_LOGW(TAG, "BLE watchdog alarm — SSR forced off");
        tpc_force_off();
    }
}
