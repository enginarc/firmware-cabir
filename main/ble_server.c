#include "ble_server.h"
#include "config.h"
#include "session.h"
#include "safety.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "host/ble_gatt.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include <string.h>
#include <stdio.h>

static const char *TAG = "BLE";

static char s_device_name[16];   /* "Cabir-XXXX\0" */
static uint16_t   s_conn_handle = BLE_HS_CONN_HANDLE_NONE;

/* Notify characteristic handles */
static uint16_t h_plate_temp;
static uint16_t h_liquid_temp;
static uint16_t h_device_state;
static uint16_t h_time_to_target;
static uint16_t h_timer_remaining;
static uint16_t h_alarm_flags;

/* ── Helper: pack/unpack little-endian ─────────────────── */
static void pack_u16_le(uint8_t *buf, uint16_t v)
{
    buf[0] = v & 0xFF;
    buf[1] = (v >> 8) & 0xFF;
}
static void pack_u32_le(uint8_t *buf, uint32_t v)
{
    buf[0] = v & 0xFF;
    buf[1] = (v >> 8) & 0xFF;
    buf[2] = (v >> 16) & 0xFF;
    buf[3] = (v >> 24) & 0xFF;
}
static uint16_t unpack_u16_le(const uint8_t *buf)
{
    return (uint16_t)buf[0] | ((uint16_t)buf[1] << 8);
}

/* ── GATT read/write callbacks ─────────────────────────── */

static int gatt_read_plate_temp(uint16_t conn, uint16_t attr,
                                 struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    int16_t t = session_get_plate_temp();
    uint8_t buf[2]; pack_u16_le(buf, (uint16_t)t);
    os_mbuf_append(ctxt->om, buf, 2);
    return 0;
}

static int gatt_read_liquid_temp(uint16_t conn, uint16_t attr,
                                   struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    int16_t t = session_get_liquid_temp();
    uint8_t buf[2]; pack_u16_le(buf, (uint16_t)t);
    os_mbuf_append(ctxt->om, buf, 2);
    return 0;
}

static int gatt_rw_notify_interval(uint16_t conn, uint16_t attr,
                                     struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
        uint8_t buf[2];
        uint16_t len = OS_MBUF_PKTLEN(ctxt->om);
        if (len < 2) return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
        os_mbuf_copydata(ctxt->om, 0, 2, buf);
        session_set_notify_interval(unpack_u16_le(buf));
        return 0;
    }
    uint8_t buf[2];
    pack_u16_le(buf, session_get_notify_interval());
    os_mbuf_append(ctxt->om, buf, 2);
    return 0;
}

static int gatt_rw_target_temp(uint16_t conn, uint16_t attr,
                                 struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
        uint8_t t;
        if (OS_MBUF_PKTLEN(ctxt->om) < 1) return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
        os_mbuf_copydata(ctxt->om, 0, 1, &t);
        session_set_target_temp(t);
        return 0;
    }
    uint8_t t = session_get_target_temp();
    os_mbuf_append(ctxt->om, &t, 1);
    return 0;
}

static int gatt_write_heater_cmd(uint16_t conn, uint16_t attr,
                                   struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    uint8_t cmd_byte;
    if (OS_MBUF_PKTLEN(ctxt->om) < 1) return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
    os_mbuf_copydata(ctxt->om, 0, 1, &cmd_byte);
    if (cmd_byte == HEATER_CMD_START) {
        session_cmd_start();
    } else {
        session_cmd_stop();
    }
    return 0;
}

static int gatt_read_device_state(uint16_t conn, uint16_t attr,
                                    struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    uint8_t s = (uint8_t)session_get_state();
    os_mbuf_append(ctxt->om, &s, 1);
    return 0;
}

static int gatt_read_time_to_target(uint16_t conn, uint16_t attr,
                                      struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    uint8_t buf[2]; pack_u16_le(buf, session_get_time_to_target());
    os_mbuf_append(ctxt->om, buf, 2);
    return 0;
}

static int gatt_rw_session_duration(uint16_t conn, uint16_t attr,
                                      struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
        uint8_t buf[2];
        if (OS_MBUF_PKTLEN(ctxt->om) < 2) return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
        os_mbuf_copydata(ctxt->om, 0, 2, buf);
        session_set_duration(unpack_u16_le(buf));
        return 0;
    }
    uint8_t buf[2]; pack_u16_le(buf, session_get_duration());
    os_mbuf_append(ctxt->om, buf, 2);
    return 0;
}

static int gatt_read_timer_remaining(uint16_t conn, uint16_t attr,
                                       struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    uint8_t buf[2]; pack_u16_le(buf, session_get_timer_remaining());
    os_mbuf_append(ctxt->om, buf, 2);
    return 0;
}

static int gatt_rw_delayed_start(uint16_t conn, uint16_t attr,
                                   struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
        uint8_t buf[2];
        if (OS_MBUF_PKTLEN(ctxt->om) < 2) return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
        os_mbuf_copydata(ctxt->om, 0, 2, buf);
        session_set_delayed_start(unpack_u16_le(buf));
        return 0;
    }
    uint8_t buf[2]; pack_u16_le(buf, session_get_delayed_start());
    os_mbuf_append(ctxt->om, buf, 2);
    return 0;
}

static int gatt_read_alarm_flags(uint16_t conn, uint16_t attr,
                                   struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    uint8_t f = session_get_alarm_flags();
    os_mbuf_append(ctxt->om, &f, 1);
    return 0;
}

static int gatt_rw_ble_watchdog(uint16_t conn, uint16_t attr,
                                  struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
        uint8_t buf[2];
        if (OS_MBUF_PKTLEN(ctxt->om) < 2) return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
        os_mbuf_copydata(ctxt->om, 0, 2, buf);
        session_set_ble_watchdog(unpack_u16_le(buf));
        return 0;
    }
    uint8_t buf[2]; pack_u16_le(buf, session_get_ble_watchdog());
    os_mbuf_append(ctxt->om, buf, 2);
    return 0;
}

static int gatt_write_alarm_clear(uint16_t conn, uint16_t attr,
                                    struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    uint8_t key;
    if (OS_MBUF_PKTLEN(ctxt->om) < 1) return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
    os_mbuf_copydata(ctxt->om, 0, 1, &key);
    if (!safety_clear_alarms(key)) {
        return BLE_ATT_ERR_WRITE_NOT_PERMITTED;
    }
    return 0;
}

static int gatt_read_fw_version(uint16_t conn, uint16_t attr,
                                  struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    os_mbuf_append(ctxt->om, FIRMWARE_VERSION, strlen(FIRMWARE_VERSION));
    return 0;
}

static int gatt_read_uptime(uint16_t conn, uint16_t attr,
                              struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    uint8_t buf[4]; pack_u32_le(buf, session_get_uptime());
    os_mbuf_append(ctxt->om, buf, 4);
    return 0;
}

/* ── GATT service table ────────────────────────────────── */

static const struct ble_gatt_svc_def s_gatt_svcs[] = {
  { .type = BLE_GATT_SVC_TYPE_PRIMARY,
    .uuid = BLE_UUID16_DECLARE(CABIR_SVC_UUID),
    .characteristics = (struct ble_gatt_chr_def[]) {

      /* 0xCA11 — Plate temperature: Read + Notify */
      { .uuid       = BLE_UUID16_DECLARE(CHAR_PLATE_TEMP),
        .access_cb  = gatt_read_plate_temp,
        .flags      = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
        .val_handle = &h_plate_temp },

      /* 0xCA12 — Liquid temperature: Read + Notify */
      { .uuid       = BLE_UUID16_DECLARE(CHAR_LIQUID_TEMP),
        .access_cb  = gatt_read_liquid_temp,
        .flags      = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
        .val_handle = &h_liquid_temp },

      /* 0xCA14 — Notify interval: Read + Write */
      { .uuid      = BLE_UUID16_DECLARE(CHAR_NOTIFY_INTERVAL),
        .access_cb = gatt_rw_notify_interval,
        .flags     = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE },

      /* 0xCA21 — Target temperature: Read + Write */
      { .uuid      = BLE_UUID16_DECLARE(CHAR_TARGET_TEMP),
        .access_cb = gatt_rw_target_temp,
        .flags     = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE },

      /* 0xCA22 — Heater command: Write */
      { .uuid      = BLE_UUID16_DECLARE(CHAR_HEATER_CMD),
        .access_cb = gatt_write_heater_cmd,
        .flags     = BLE_GATT_CHR_F_WRITE },

      /* 0xCA23 — Device state: Read + Notify */
      { .uuid       = BLE_UUID16_DECLARE(CHAR_DEVICE_STATE),
        .access_cb  = gatt_read_device_state,
        .flags      = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
        .val_handle = &h_device_state },

      /* 0xCA24 — Time to target: Read + Notify */
      { .uuid       = BLE_UUID16_DECLARE(CHAR_TIME_TO_TARGET),
        .access_cb  = gatt_read_time_to_target,
        .flags      = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
        .val_handle = &h_time_to_target },

      /* 0xCA31 — Session duration: Read + Write */
      { .uuid      = BLE_UUID16_DECLARE(CHAR_SESSION_DURATION),
        .access_cb = gatt_rw_session_duration,
        .flags     = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE },

      /* 0xCA32 — Timer remaining: Read + Notify */
      { .uuid       = BLE_UUID16_DECLARE(CHAR_TIMER_REMAINING),
        .access_cb  = gatt_read_timer_remaining,
        .flags      = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
        .val_handle = &h_timer_remaining },

      /* 0xCA33 — Delayed start: Read + Write */
      { .uuid      = BLE_UUID16_DECLARE(CHAR_DELAYED_START),
        .access_cb = gatt_rw_delayed_start,
        .flags     = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE },

      /* 0xCA41 — Alarm flags: Read + Notify */
      { .uuid       = BLE_UUID16_DECLARE(CHAR_ALARM_FLAGS),
        .access_cb  = gatt_read_alarm_flags,
        .flags      = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
        .val_handle = &h_alarm_flags },

      /* 0xCA42 — BLE watchdog: Read + Write */
      { .uuid      = BLE_UUID16_DECLARE(CHAR_BLE_WATCHDOG),
        .access_cb = gatt_rw_ble_watchdog,
        .flags     = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE },

      /* 0xCA43 — Alarm clear: Write */
      { .uuid      = BLE_UUID16_DECLARE(CHAR_ALARM_CLEAR),
        .access_cb = gatt_write_alarm_clear,
        .flags     = BLE_GATT_CHR_F_WRITE },

      /* 0xCA51 — Firmware version: Read */
      { .uuid      = BLE_UUID16_DECLARE(CHAR_FW_VERSION),
        .access_cb = gatt_read_fw_version,
        .flags     = BLE_GATT_CHR_F_READ },

      /* 0xCA52 — Uptime: Read */
      { .uuid      = BLE_UUID16_DECLARE(CHAR_UPTIME),
        .access_cb = gatt_read_uptime,
        .flags     = BLE_GATT_CHR_F_READ },

      { 0 } /* sentinel */
    }
  },
  { 0 } /* sentinel */
};

/* ── Forward declaration ─────────────────────────────────── */
static int gap_event_cb(struct ble_gap_event *event, void *arg);

/* ── Advertising ─────────────────────────────────────────── */

static void start_advertising(void)
{
    struct ble_hs_adv_fields fields = {0};

    /* Flags */
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;

    /* Device name */
    fields.name     = (const uint8_t *)s_device_name;
    fields.name_len = strlen(s_device_name);
    fields.name_is_complete = 1;

    /* ── ADD THIS BLOCK ── */
    static const ble_uuid16_t svc_uuid = BLE_UUID16_INIT(CABIR_SVC_UUID);
    fields.uuids16             = &svc_uuid;
    fields.num_uuids16         = 1;
    fields.uuids16_is_complete = 1;
    /* ────────────────────── */

    int rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gap_adv_set_fields failed: %d", rc);
        return;
    }

    struct ble_gap_adv_params params = {0};
    params.conn_mode = BLE_GAP_CONN_MODE_UND;
    params.disc_mode = BLE_GAP_DISC_MODE_GEN;

    rc = ble_gap_adv_start(BLE_OWN_ADDR_PUBLIC, NULL, BLE_HS_FOREVER,
                           &params, gap_event_cb, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gap_adv_start failed: %d", rc);
        return;
    }
    ESP_LOGI(TAG, "Advertising as \"%s\" with service 0x%04X", s_device_name, CABIR_SVC_UUID);
}

/* ── GAP event handler ───────────────────────────────────── */

static int gap_event_cb(struct ble_gap_event *event, void *arg)
{
    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status == 0) {
            s_conn_handle = event->connect.conn_handle;
            session_ble_connected();
            ESP_LOGI(TAG, "Connected handle=%d", s_conn_handle);
        } else {
            s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
            start_advertising();
        }
        break;

    case BLE_GAP_EVENT_DISCONNECT:
        s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
        session_ble_disconnected();
        ESP_LOGI(TAG, "Disconnected reason=0x%02x", event->disconnect.reason);
        start_advertising();
        break;

    case BLE_GAP_EVENT_ADV_COMPLETE:
        start_advertising();
        break;

    default:
        break;
    }
    return 0;
}

static void build_device_name(void)
{
    uint8_t addr[6] = {0};
    int rc = ble_hs_id_copy_addr(BLE_ADDR_PUBLIC, addr, NULL);
    if (rc != 0) {
        /* Fallback if public address unavailable */
        ble_hs_id_copy_addr(BLE_ADDR_RANDOM, addr, NULL);
    }
    /*
     * Use the last two bytes of the MAC address as a 4-character
     * uppercase hex suffix — unique per device, stable across reboots.
     * addr[0] is the LSB of the MAC in NimBLE's byte order.
     */
    snprintf(s_device_name, sizeof(s_device_name),
             "Cabir-%02X%02X", addr[1], addr[0]);
    ESP_LOGI("BLE", "Device name: %s", s_device_name);
}

/* ── NimBLE host callbacks ───────────────────────────────── */

static void ble_on_sync(void)
{
    ble_hs_util_ensure_addr(0);
    build_device_name();                          /* ← add this line */
    ble_svc_gap_device_name_set(s_device_name);   /* ← add this line */
    start_advertising();
}

static void ble_on_reset(int reason)
{
    ESP_LOGE(TAG, "BLE host reset reason=%d", reason);
}

static void nimble_host_task(void *param)
{
    nimble_port_run();
    nimble_port_freertos_deinit();
}

/* ── Notify task ─────────────────────────────────────────── */

static void notify_task(void *arg)
{
    while (1) {
        uint16_t interval = session_get_notify_interval();
        vTaskDelay(pdMS_TO_TICKS(interval));
        ble_server_notify_all();
    }
}

/* ── Notify helper ───────────────────────────────────────── */

void ble_server_notify_all(void)
{
    if (s_conn_handle == BLE_HS_CONN_HANDLE_NONE) return;

    struct os_mbuf *om;

    /* Plate temp */
    int16_t pt = session_get_plate_temp();
    uint8_t buf2[2];
    pack_u16_le(buf2, (uint16_t)pt);
    om = ble_hs_mbuf_from_flat(buf2, 2);
    if (om) ble_gatts_notify_custom(s_conn_handle, h_plate_temp, om);

    /* Liquid temp */
    int16_t lt = session_get_liquid_temp();
    pack_u16_le(buf2, (uint16_t)lt);
    om = ble_hs_mbuf_from_flat(buf2, 2);
    if (om) ble_gatts_notify_custom(s_conn_handle, h_liquid_temp, om);

    /* Device state */
    uint8_t state = (uint8_t)session_get_state();
    om = ble_hs_mbuf_from_flat(&state, 1);
    if (om) ble_gatts_notify_custom(s_conn_handle, h_device_state, om);

    /* Time to target */
    pack_u16_le(buf2, session_get_time_to_target());
    om = ble_hs_mbuf_from_flat(buf2, 2);
    if (om) ble_gatts_notify_custom(s_conn_handle, h_time_to_target, om);

    /* Timer remaining */
    pack_u16_le(buf2, session_get_timer_remaining());
    om = ble_hs_mbuf_from_flat(buf2, 2);
    if (om) ble_gatts_notify_custom(s_conn_handle, h_timer_remaining, om);

    /* Alarm flags */
    uint8_t alarms = session_get_alarm_flags();
    om = ble_hs_mbuf_from_flat(&alarms, 1);
    if (om) ble_gatts_notify_custom(s_conn_handle, h_alarm_flags, om);
}

/* ── Public init ─────────────────────────────────────────── */

esp_err_t ble_server_init(void)
{
    ESP_ERROR_CHECK(nimble_port_init());

    ble_hs_cfg.sync_cb  = ble_on_sync;
    ble_hs_cfg.reset_cb = ble_on_reset;

    ble_svc_gap_init();
    ble_svc_gatt_init();

    int rc = ble_gatts_count_cfg(s_gatt_svcs);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gatts_count_cfg failed: %d", rc);
        return ESP_FAIL;
    }
    rc = ble_gatts_add_svcs(s_gatt_svcs);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gatts_add_svcs failed: %d", rc);
        return ESP_FAIL;
    }

    nimble_port_freertos_init(nimble_host_task);

    xTaskCreate(notify_task, "ble_notify", 4096, NULL, 5, NULL);
    ESP_LOGI(TAG, "BLE server ready");
    return ESP_OK;
}
