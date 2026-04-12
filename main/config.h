#pragma once
#include <stdint.h>

/* ═══════════════════════════════════════════════════════
   GPIO PIN ASSIGNMENTS — ESP32-S3 Super Mini
   ═══════════════════════════════════════════════════════ */

/* AC/Heater control */
#define PIN_SSR_EN          GPIO_NUM_4   /* TPC output → MMBT2222A base */

/* Safety input */
#define PIN_KSD_WARN        GPIO_NUM_5   /* KSD301 NO 60°C → pull-up, active LOW */

/* Temperature ADC (ADC1, 12-bit, 0–3.3V, atten DB_12) */
#define PIN_ADC_PLATE       GPIO_NUM_1   /* ADC1_CH0 — plate NTC */
#define PIN_ADC_LIQUID      GPIO_NUM_2   /* ADC1_CH1 — external liquid NTC */
#define ADC_CHANNEL_PLATE   ADC_CHANNEL_0
#define ADC_CHANNEL_LIQUID  ADC_CHANNEL_1

/* ST7789V SPI display (SPI2 / HSPI) */
#define PIN_SPI_SCLK        GPIO_NUM_12
#define PIN_SPI_MOSI        GPIO_NUM_11
#define PIN_SPI_CS          GPIO_NUM_10
#define PIN_SPI_DC          GPIO_NUM_9
#define PIN_SPI_RST         GPIO_NUM_8
#define PIN_SPI_BL          GPIO_NUM_7   /* Backlight: HIGH = on */

/* ═══════════════════════════════════════════════════════
   TEMPERATURE CONSTANTS
   ═══════════════════════════════════════════════════════ */

#define NTC_BETA            3950         /* Beta coefficient */
#define NTC_R0              10000        /* 10K at T0 */
#define NTC_T0_K            298.15f      /* 25°C in Kelvin */
#define NTC_R_FIXED         10000        /* Series resistor value (Ω) */
#define ADC_MAX_RAW         4095         /* 12-bit */
#define ADC_OVERSAMPLE      16           /* Readings averaged per sample */

/* Sentinel: liquid NTC disconnected */
#define NTC_DISCONNECTED    0x7FFF       /* INT16 sentinel value */
/* Open circuit threshold: raw ADC near max = NTC open (very high R = very cold) */
#define ADC_OPEN_THRESHOLD  4050
/* Short circuit threshold: raw ADC near 0 = NTC shorted */
#define ADC_SHORT_THRESHOLD 50

/* ═══════════════════════════════════════════════════════
   SAFETY LIMITS
   ═══════════════════════════════════════════════════════ */

#define TEMP_MAX_USER_C         75       /* Max user-settable target (°C) */
#define TEMP_DEFAULT_TARGET_C   60       /* Default target (°C) */
#define TEMP_ALARM_PLATE_C      75       /* Software alarm threshold (°C) */
#define TEMP_OVERSHOOT_C        5        /* Alert if plate > target + this */

/* ═══════════════════════════════════════════════════════
   PID PARAMETERS (initial — tune after hardware test)
   ═══════════════════════════════════════════════════════ */

#define PID_KP              2.5f
#define PID_KI              0.08f
#define PID_KD              0.5f
#define PID_OUTPUT_MIN      0.0f
#define PID_OUTPUT_MAX      100.0f
#define PID_INTEGRAL_MAX    50.0f        /* Anti-windup clamp */

/* ═══════════════════════════════════════════════════════
   TIME-PROPORTIONAL CONTROL
   ═══════════════════════════════════════════════════════ */

#define TPC_WINDOW_MS       10000        /* 10-second TPC window */

/* ═══════════════════════════════════════════════════════
   TASK TIMING
   ═══════════════════════════════════════════════════════ */

#define TEMP_SAMPLE_MS      500          /* Temperature sampling interval */
#define DISPLAY_UPDATE_MS   1000         /* Display refresh interval */
#define BLE_NOTIFY_MS_DEF   2000         /* Default BLE notification interval */
#define SAFETY_CHECK_MS     200          /* Safety monitor interval */
#define SESSION_TICK_MS     1000         /* Session timer tick */

/* ═══════════════════════════════════════════════════════
   SESSION DEFAULTS
   ═══════════════════════════════════════════════════════ */

#define SESSION_DURATION_DEF_S  2700     /* 45 minutes */
#define SESSION_DURATION_MAX_S  14400    /* 4 hours */
#define BLE_WATCHDOG_DEF_S      120      /* 2 minutes */

/* ═══════════════════════════════════════════════════════
   BLE GATT UUIDs
   ═══════════════════════════════════════════════════════ */

#define CABIR_SVC_UUID          0xCA01

#define CHAR_PLATE_TEMP         0xCA11
#define CHAR_LIQUID_TEMP        0xCA12
#define CHAR_NOTIFY_INTERVAL    0xCA14
#define CHAR_TARGET_TEMP        0xCA21
#define CHAR_HEATER_CMD         0xCA22
#define CHAR_DEVICE_STATE       0xCA23
#define CHAR_TIME_TO_TARGET     0xCA24
#define CHAR_SESSION_DURATION   0xCA31
#define CHAR_TIMER_REMAINING    0xCA32
#define CHAR_DELAYED_START      0xCA33
#define CHAR_ALARM_FLAGS        0xCA41
#define CHAR_BLE_WATCHDOG       0xCA42
#define CHAR_ALARM_CLEAR        0xCA43
#define CHAR_FW_VERSION         0xCA51
#define CHAR_UPTIME             0xCA52

/* Alarm flag bits */
#define ALARM_PLATE_FAULT       (1 << 0)
#define ALARM_LIQUID_LOST       (1 << 1)
#define ALARM_OVERSHOOT         (1 << 2)
#define ALARM_BLE_WATCHDOG      (1 << 3)
#define ALARM_HW_CUTOFF         (1 << 4)  /* KSD301 NC has tripped */

/* Device states */
typedef enum {
    STATE_IDLE    = 0,
    STATE_HEATING = 1,
    STATE_HOLDING = 2,
    STATE_COOLING = 3,
    STATE_ALARM   = 4,
} cabir_state_t;

/* Heater commands */
#define HEATER_CMD_STOP   0x00
#define HEATER_CMD_START  0x01
#define ALARM_CLEAR_KEY   0xAC

#define FIRMWARE_VERSION  "1.0.0"
