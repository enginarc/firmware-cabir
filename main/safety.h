#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "config.h"

/* Initialise KSD301 NO GPIO and safety subsystem */
esp_err_t safety_init(void);

/*
 * Run safety checks. Call every SAFETY_CHECK_MS.
 * Updates alarm flags and forces SSR off when needed.
 * plate_c: current plate temperature in °C (float).
 * target_c: current target temperature in °C.
 * plate_fault: true if NTC read failed.
 * Returns current alarm bitmask.
 */
uint8_t safety_check(float plate_c, float target_c, bool plate_fault);

/* Return current alarm flags bitmask */
uint8_t safety_get_alarms(void);

/* Clear software-clearable alarm bits (0xAC key required) */
bool safety_clear_alarms(uint8_t key);

/* True if KSD301 NO 60°C has closed (plate ≥ 60°C warning) */
bool safety_ksd_warn_active(void);

/* Set/clear BLE watchdog alarm bit */
void safety_set_ble_watchdog(bool active);
