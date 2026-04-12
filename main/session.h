#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "config.h"

/* Initialise the session subsystem (PID, TPC, state machine) */
esp_err_t session_init(void);

/* ── Setters (called from BLE server) ── */
void session_set_target_temp(uint8_t target_c);    /* Clamped to TEMP_MAX_USER_C */
void session_set_duration(uint16_t seconds);        /* 0 = hold indefinitely */
void session_set_delayed_start(uint16_t seconds);
void session_set_notify_interval(uint16_t ms);
void session_set_ble_watchdog(uint16_t seconds);
void session_cmd_start(void);
void session_cmd_stop(void);
void session_ble_connected(void);
void session_ble_disconnected(void);

/* ── Getters (called from BLE server & display) ── */
cabir_state_t session_get_state(void);
uint8_t       session_get_target_temp(void);
int16_t       session_get_plate_temp(void);    /* °C × 10 */
int16_t       session_get_liquid_temp(void);   /* °C × 10, 0x7FFF if absent */
uint16_t      session_get_timer_remaining(void);  /* seconds, 0xFFFF if no timer */
uint16_t      session_get_time_to_target(void);   /* seconds, 0xFFFF if unknown */
uint8_t       session_get_alarm_flags(void);
uint16_t      session_get_notify_interval(void);
uint16_t      session_get_duration(void);
uint16_t      session_get_delayed_start(void);
uint16_t      session_get_ble_watchdog(void);
uint32_t      session_get_uptime(void);
