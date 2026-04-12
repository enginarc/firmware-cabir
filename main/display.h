#pragma once
#include "esp_err.h"
#include "config.h"
#include <stdint.h>

/* Initialise SPI bus and ST7789V */
esp_err_t display_init(void);

/*
 * Update the display with current session state.
 * Call every DISPLAY_UPDATE_MS.
 */
void display_update(void);
