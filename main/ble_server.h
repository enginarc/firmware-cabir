#pragma once
#include "esp_err.h"

/* Initialise and start NimBLE GATT server */
esp_err_t ble_server_init(void);

/*
 * Notify all subscribed characteristics with latest session data.
 * Call every session_get_notify_interval() ms from the notify task.
 */
void ble_server_notify_all(void);
