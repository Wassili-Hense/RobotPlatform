#ifndef RIO_H
#define RIO_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    RIO_DATA_STAT_USB_CONN = 0,
    RIO_DATA_STAT_BL_ON    = 1,

    RIO_DATA_JOY_X         = 10,
    RIO_DATA_JOY_Y         = 11,
    RIO_DATA_ADC_V         = 12,

    RIO_DATA_BTN_ON        = 20,
    RIO_DATA_BTN_FIRE      = 21,
    RIO_DATA_BTN_UP        = 22,
    RIO_DATA_BTN_DOWN      = 23,
    RIO_DATA_BTN_BACK      = 24,
    RIO_DATA_BTN_OK        = 25,
    RIO_DATA_BTN_LUP       = 26,
    RIO_DATA_BTN_LDN       = 27,
    RIO_DATA_BTN_RUP       = 28,
    RIO_DATA_BTN_RDN       = 29,
    RIO_DATA_BTN_ANYKEY    = 30,

    RIO_DATA_COUNT         = 31
} rio_data_idx_t;

typedef enum {
    RIO_TICK_OK                  = 0x0000,
    RIO_TICK_ERR_NOT_INITIALIZED = 0x0001,
    RIO_TICK_ERR_I2C_REQUEST     = 0x0002,
    RIO_TICK_ERR_I2C_READ        = 0x0004
} rio_tick_result_t;

typedef enum {
    RIO_CMD_OK                  = 0,
    RIO_CMD_ERR_NOT_INITIALIZED = 1,
    RIO_CMD_ERR_INVALID_ARG     = 2,
    RIO_CMD_ERR_I2C_TX          = 3
} rio_cmd_result_t;

typedef enum {
    RIO_MELODY_POWER_ON     = 1,
    RIO_MELODY_CONNECTED    = 2,
    RIO_MELODY_DISCONNECTED = 3
} rio_melody_t;

typedef void (*rio_log_callback_t)(const char *text, bool emergency);

void rio_init(rio_log_callback_t log_callback);
rio_tick_result_t rio_tick(void);
uint16_t rio_get(rio_data_idx_t idx);
bool rio_changed(rio_data_idx_t idx);
void rio_sysSend(void);

void rio_cmd_set_backlight_timeout(uint32_t timeout_ms);
void rio_cmd_set_brightness(uint8_t level);
void rio_cmd_play_tone(uint16_t divider, uint16_t delay_ms);
void rio_cmd_play_melody(rio_melody_t melody);
void rio_cmd_power_off(void);

#ifdef __cplusplus
}
#endif

#endif // RIO_H
