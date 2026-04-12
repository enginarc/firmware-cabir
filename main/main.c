#include <stdio.h>
#include "esp_log.h"
#include "esp_err.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "config.h"
#include "temperature.h"
#include "tpc.h"
#include "safety.h"
#include "session.h"
#include "display.h"
#include "ble_server.h"

static const char *TAG = "CABIR";

/* ── Display update task ─────────────────────────────────── */

static void display_task(void *arg)
{
    while (1) {
        display_update();
        vTaskDelay(pdMS_TO_TICKS(DISPLAY_UPDATE_MS));
    }
}

/* ── app_main ────────────────────────────────────────────── */

void app_main(void)
{
    ESP_LOGI(TAG, "Cabir v%s starting", FIRMWARE_VERSION);

    /* 1. NVS — required by BLE */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    /* 2. Temperature ADC */
    ESP_ERROR_CHECK(temperature_init());

    /* 3. Time-proportional SSR control */
    ESP_ERROR_CHECK(tpc_init());

    /* 4. Safety monitor (KSD301 GPIO) */
    ESP_ERROR_CHECK(safety_init());

    /* 5. Session state machine (PID loop) */
    ESP_ERROR_CHECK(session_init());

    /* 6. Display */
    ESP_ERROR_CHECK(display_init());
    xTaskCreate(display_task, "display", 4096, NULL, 4, NULL);

    /* 7. BLE GATT server */
    ESP_ERROR_CHECK(ble_server_init());

    ESP_LOGI(TAG, "All subsystems ready — Cabir is online");

    /* app_main can return; all work is done in tasks */
}
