#include "session.h"
#include "config.h"
#include "temperature.h"
#include "pid.h"
#include "tpc.h"
#include "safety.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <string.h>
#include <math.h>

static const char *TAG = "SESSION";

/* ── State ─────────────────────────────────────────────── */
typedef struct {
    cabir_state_t state;
    uint8_t       target_temp_c;
    uint16_t      duration_s;        /* 0 = hold mode */
    uint16_t      timer_remaining_s; /* 0xFFFF = no timer */
    uint16_t      delayed_start_s;
    uint16_t      delay_remaining_s;
    uint16_t      notify_interval_ms;
    uint16_t      ble_watchdog_s;
    uint16_t      ble_watchdog_remaining_s;
    bool          ble_connected;
    int16_t       plate_temp;        /* °C × 10, last reading */
    int16_t       liquid_temp;       /* °C × 10 */
    float         pid_output;

    /* Time-to-target estimation */
    uint16_t      time_to_target_s;
    float         prev_plate_c;
    int           tte_sample_count;

    uint32_t      uptime_s;
} session_ctx_t;

static session_ctx_t s_ctx;
static pid_t         s_pid;
static SemaphoreHandle_t s_mutex;

/* ── Helpers ────────────────────────────────────────────── */

static float plate_c(void) { return (float)s_ctx.plate_temp / 10.0f; }

static void transition(cabir_state_t next)
{
    if (s_ctx.state == next) return;
    ESP_LOGI(TAG, "State %d → %d", s_ctx.state, next);
    s_ctx.state = next;
}

static void enter_alarm(void)
{
    tpc_force_off();
    transition(STATE_ALARM);
}

/* ── Time-to-target estimation ──────────────────────────── */

static void update_tte(float current_c, float target_c)
{
    float delta = target_c - current_c;
    if (delta <= 0.5f) {
        s_ctx.time_to_target_s = 0;
        return;
    }

    float rate = current_c - s_ctx.prev_plate_c; /* °C per TEMP_SAMPLE_MS */
    s_ctx.prev_plate_c = current_c;
    s_ctx.tte_sample_count++;

    /* Need at least 3 samples for a meaningful rate */
    if (s_ctx.tte_sample_count < 3 || rate <= 0.0f) {
        s_ctx.time_to_target_s = 0xFFFF;
        return;
    }

    /* Rate is °C per sample interval; convert to °C/s */
    float rate_per_s = rate / ((float)TEMP_SAMPLE_MS / 1000.0f);
    if (rate_per_s < 0.001f) {
        s_ctx.time_to_target_s = 0xFFFF;
        return;
    }

    uint32_t tte = (uint32_t)(delta / rate_per_s);
    s_ctx.time_to_target_s = (tte > 0xFFFE) ? 0xFFFE : (uint16_t)tte;
}

/* ── Main session task ──────────────────────────────────── */

static void session_task(void *arg)
{
    TickType_t      last_temp_tick    = xTaskGetTickCount();
    TickType_t      last_second_tick  = xTaskGetTickCount();
    const TickType_t temp_period      = pdMS_TO_TICKS(TEMP_SAMPLE_MS);
    const TickType_t second_period    = pdMS_TO_TICKS(1000);

    while (1) {
        TickType_t now = xTaskGetTickCount();

        /* ── Temperature sampling (every TEMP_SAMPLE_MS) ── */
        if ((now - last_temp_tick) >= temp_period) {
            last_temp_tick = now;

            xSemaphoreTake(s_mutex, portMAX_DELAY);

            s_ctx.plate_temp  = temperature_read_plate();
            s_ctx.liquid_temp = temperature_read_liquid();
            bool plate_fault  = temperature_plate_fault();
            float tc          = plate_c();
            float tgt         = (float)s_ctx.target_temp_c;

            /* Safety check */
            uint8_t alarms = safety_check(tc, tgt, plate_fault);

            if (alarms & ALARM_PLATE_FAULT) {
                enter_alarm();
            } else if (s_ctx.state == STATE_HEATING ||
                       s_ctx.state == STATE_HOLDING) {

                /* PID → TPC */
                float pid_out = pid_compute(&s_pid, tgt, tc);
                s_ctx.pid_output = pid_out;
                tpc_set_duty(pid_out);

                /* State transitions */
                float err = fabsf(tgt - tc);
                if (s_ctx.state == STATE_HEATING && err < 1.0f) {
                    pid_reset(&s_pid);
                    transition(STATE_HOLDING);
                } else if (s_ctx.state == STATE_HOLDING && err > 3.0f) {
                    transition(STATE_HEATING);
                }

                update_tte(tc, tgt);
            }

            xSemaphoreGive(s_mutex);
        }

        /* ── 1-second tick (timers, watchdog, uptime) ── */
        if ((now - last_second_tick) >= second_period) {
            last_second_tick = now;

            xSemaphoreTake(s_mutex, portMAX_DELAY);

            s_ctx.uptime_s++;

            /* Delayed start countdown */
            if (s_ctx.state == STATE_IDLE &&
                s_ctx.delay_remaining_s > 0) {
                s_ctx.delay_remaining_s--;
                if (s_ctx.delay_remaining_s == 0) {
                    pid_reset(&s_pid);
                    s_ctx.prev_plate_c    = plate_c();
                    s_ctx.tte_sample_count = 0;
                    tpc_release();
                    transition(STATE_HEATING);
                }
            }

            /* Session timer countdown */
            if ((s_ctx.state == STATE_HEATING ||
                 s_ctx.state == STATE_HOLDING) &&
                s_ctx.timer_remaining_s != 0xFFFF) {
                if (s_ctx.timer_remaining_s > 0) {
                    s_ctx.timer_remaining_s--;
                }
                if (s_ctx.timer_remaining_s == 0) {
                    ESP_LOGI(TAG, "Session timer expired — entering COOLING");
                    tpc_set_duty(0.0f);
                    tpc_force_off();
                    transition(STATE_COOLING);
                }
            }

            /* BLE watchdog */
            if (!s_ctx.ble_connected &&
                (s_ctx.state == STATE_HEATING ||
                 s_ctx.state == STATE_HOLDING) &&
                s_ctx.ble_watchdog_s > 0) {
                if (s_ctx.ble_watchdog_remaining_s > 0) {
                    s_ctx.ble_watchdog_remaining_s--;
                }
                if (s_ctx.ble_watchdog_remaining_s == 0) {
                    ESP_LOGW(TAG, "BLE watchdog expired");
                    safety_set_ble_watchdog(true);
                    enter_alarm();
                }
            }

            xSemaphoreGive(s_mutex);
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

/* ── Public API ─────────────────────────────────────────── */

esp_err_t session_init(void)
{
    memset(&s_ctx, 0, sizeof(s_ctx));
    s_ctx.state               = STATE_IDLE;
    s_ctx.target_temp_c       = TEMP_DEFAULT_TARGET_C;
    s_ctx.duration_s          = SESSION_DURATION_DEF_S;
    s_ctx.timer_remaining_s   = 0xFFFF;
    s_ctx.notify_interval_ms  = BLE_NOTIFY_MS_DEF;
    s_ctx.ble_watchdog_s      = BLE_WATCHDOG_DEF_S;
    s_ctx.time_to_target_s    = 0xFFFF;
    s_ctx.plate_temp          = 250;  /* Assume 25°C at start */
    s_ctx.liquid_temp         = (int16_t)NTC_DISCONNECTED;
    s_ctx.prev_plate_c        = 25.0f;

    float dt_s = (float)TEMP_SAMPLE_MS / 1000.0f;
    pid_init(&s_pid, PID_KP, PID_KI, PID_KD,
             PID_OUTPUT_MIN, PID_OUTPUT_MAX,
             PID_INTEGRAL_MAX, dt_s);

    s_mutex = xSemaphoreCreateMutex();
    if (!s_mutex) return ESP_ERR_NO_MEM;

    xTaskCreate(session_task, "session", 4096, NULL, 8, NULL);
    ESP_LOGI(TAG, "Session subsystem ready");
    return ESP_OK;
}

void session_set_target_temp(uint8_t target_c)
{
    if (target_c > TEMP_MAX_USER_C) target_c = TEMP_MAX_USER_C;
    if (target_c < 30) target_c = 30;
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    s_ctx.target_temp_c = target_c;
    xSemaphoreGive(s_mutex);
}

void session_set_duration(uint16_t seconds)
{
    if (seconds > SESSION_DURATION_MAX_S) seconds = SESSION_DURATION_MAX_S;
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    s_ctx.duration_s = seconds;
    /* 0 = hold mode */
    s_ctx.timer_remaining_s = (seconds == 0) ? 0xFFFF : seconds;
    xSemaphoreGive(s_mutex);
}

void session_set_delayed_start(uint16_t seconds)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    s_ctx.delayed_start_s   = seconds;
    s_ctx.delay_remaining_s = seconds;
    xSemaphoreGive(s_mutex);
}

void session_set_notify_interval(uint16_t ms)
{
    if (ms < 500) ms = 500;
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    s_ctx.notify_interval_ms = ms;
    xSemaphoreGive(s_mutex);
}

void session_set_ble_watchdog(uint16_t seconds)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    s_ctx.ble_watchdog_s = seconds;
    xSemaphoreGive(s_mutex);
}

void session_cmd_start(void)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);

    if (s_ctx.state == STATE_ALARM) {
        xSemaphoreGive(s_mutex);
        ESP_LOGW(TAG, "Cannot start in ALARM state");
        return;
    }

    pid_reset(&s_pid);
    s_ctx.prev_plate_c     = plate_c();
    s_ctx.tte_sample_count = 0;
    s_ctx.time_to_target_s = 0xFFFF;

    if (s_ctx.duration_s > 0) {
        s_ctx.timer_remaining_s = s_ctx.duration_s;
    } else {
        s_ctx.timer_remaining_s = 0xFFFF;
    }

    if (s_ctx.delay_remaining_s > 0) {
        /* Will auto-start after delay in tick handler */
        ESP_LOGI(TAG, "Delayed start: %ds", s_ctx.delay_remaining_s);
    } else {
        tpc_release();
        transition(STATE_HEATING);
    }

    xSemaphoreGive(s_mutex);
}

void session_cmd_stop(void)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    tpc_set_duty(0.0f);
    tpc_force_off();
    pid_reset(&s_pid);
    s_ctx.timer_remaining_s = 0xFFFF;
    s_ctx.delay_remaining_s = 0;
    transition(STATE_IDLE);
    xSemaphoreGive(s_mutex);
}

void session_ble_connected(void)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    s_ctx.ble_connected            = true;
    s_ctx.ble_watchdog_remaining_s = s_ctx.ble_watchdog_s;
    xSemaphoreGive(s_mutex);
}

void session_ble_disconnected(void)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    s_ctx.ble_connected            = false;
    s_ctx.ble_watchdog_remaining_s = s_ctx.ble_watchdog_s;
    xSemaphoreGive(s_mutex);
}

/* ── Getters ─────────────────────────────────────────────── */

cabir_state_t session_get_state(void)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    cabir_state_t v = s_ctx.state; xSemaphoreGive(s_mutex); return v;
}
uint8_t session_get_target_temp(void)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    uint8_t v = s_ctx.target_temp_c; xSemaphoreGive(s_mutex); return v;
}
int16_t session_get_plate_temp(void)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    int16_t v = s_ctx.plate_temp; xSemaphoreGive(s_mutex); return v;
}
int16_t session_get_liquid_temp(void)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    int16_t v = s_ctx.liquid_temp; xSemaphoreGive(s_mutex); return v;
}
uint16_t session_get_timer_remaining(void)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    uint16_t v = s_ctx.timer_remaining_s; xSemaphoreGive(s_mutex); return v;
}
uint16_t session_get_time_to_target(void)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    uint16_t v = s_ctx.time_to_target_s; xSemaphoreGive(s_mutex); return v;
}
uint8_t session_get_alarm_flags(void)
{
    return safety_get_alarms();
}
uint16_t session_get_notify_interval(void)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    uint16_t v = s_ctx.notify_interval_ms; xSemaphoreGive(s_mutex); return v;
}
uint16_t session_get_duration(void)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    uint16_t v = s_ctx.duration_s; xSemaphoreGive(s_mutex); return v;
}
uint16_t session_get_delayed_start(void)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    uint16_t v = s_ctx.delayed_start_s; xSemaphoreGive(s_mutex); return v;
}
uint16_t session_get_ble_watchdog(void)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    uint16_t v = s_ctx.ble_watchdog_s; xSemaphoreGive(s_mutex); return v;
}
uint32_t session_get_uptime(void)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    uint32_t v = s_ctx.uptime_s; xSemaphoreGive(s_mutex); return v;
}
