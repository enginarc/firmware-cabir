#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

/* Initialise ADC1 channels for plate and liquid NTCs */
esp_err_t temperature_init(void);

/*
 * Read plate NTC temperature in °C × 10.
 * Returns CABIR_TEMP_FAULT (INT16_MIN) on sensor fault.
 */
int16_t temperature_read_plate(void);

/*
 * Read liquid NTC temperature in °C × 10.
 * Returns NTC_DISCONNECTED (0x7FFF) when sensor is absent or open-circuit.
 */
int16_t temperature_read_liquid(void);

/* True if the last plate reading indicated a sensor fault */
bool temperature_plate_fault(void);

#define CABIR_TEMP_FAULT  INT16_MIN
