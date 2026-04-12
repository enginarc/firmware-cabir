#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

/* Initialise GPIO for SSR output */
esp_err_t tpc_init(void);

/*
 * Set duty cycle for the next TPC window.
 * duty: 0.0 (fully off) to 100.0 (fully on).
 * Call this once per PID computation cycle.
 */
void tpc_set_duty(float duty_percent);

/* Force SSR off immediately (alarm / safety cutoff) */
void tpc_force_off(void);

/* Release forced-off state (resume TPC operation) */
void tpc_release(void);

/* True if TPC is currently forced off */
bool tpc_is_forced_off(void);
