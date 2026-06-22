#ifndef HMI_H
#define HMI_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    HMI_DATA_STAT_LCD_BUSY = 0,
    HMI_DATA_STAT_USB_CONN = 1,
    HMI_DATA_STAT_BL_ON    = 2,
    HMI_DATA_JOY_X         = 10,
    HMI_DATA_JOY_Y         = 11,
    HMI_DATA_BTN_ON        = 20,
    HMI_DATA_BTN_FIRE      = 21,
    HMI_DATA_BTN_UP        = 22,
    HMI_DATA_BTN_DOWN      = 23,
    HMI_DATA_BTN_BACK      = 24,
    HMI_DATA_BTN_OK        = 25,
    HMI_DATA_BTN_LUP       = 26,
    HMI_DATA_BTN_LDN       = 27,
    HMI_DATA_BTN_RUP       = 28,
    HMI_DATA_BTN_RDN       = 29,
    HMI_DATA_COUNT         = 30
} hmi_data_idx_t;

typedef enum {
    HMI_TICK_OK                    = 0x0000,
    HMI_TICK_ERR_NOT_INITIALIZED   = 0x0001,
    HMI_TICK_ERR_I2C_REQUEST       = 0x0002,
    HMI_TICK_ERR_I2C_READ          = 0x0004
} hmi_tick_result_t;

typedef enum {
    HMI_CMD_OK                     = 0,
    HMI_CMD_ERR_NOT_INITIALIZED    = 1,
    HMI_CMD_ERR_INVALID_ARG        = 2,
    HMI_CMD_ERR_NOT_READY          = 3,
    HMI_CMD_ERR_I2C_TX             = 4
} hmi_cmd_result_t;

typedef void (*hmi_log_callback_t)(const char *text, bool emergency);

void hmi_init(hmi_log_callback_t log_callback);
hmi_tick_result_t hmi_tick(void);
uint16_t hmi_get(hmi_data_idx_t idx);
bool hmi_changed(hmi_data_idx_t idx);

hmi_cmd_result_t hmi_cmd_set_backlight_timeout(uint32_t timeout_ms);
hmi_cmd_result_t hmi_cmd_set_brightness(uint8_t level);
hmi_cmd_result_t hmi_cmd_play_tone(uint16_t divider, uint16_t delay_ms);
hmi_cmd_result_t hmi_cmd_lcd_clear(uint16_t rgb565_color);
hmi_cmd_result_t hmi_cmd_lcd_set_bg(uint16_t rgb565_color);
hmi_cmd_result_t hmi_cmd_lcd_draw_text(uint8_t x, uint8_t y, uint16_t rgb565_color, const char *text);
hmi_cmd_result_t hmi_cmd_lcd_draw_marker(uint8_t x, uint8_t y, uint8_t index, uint16_t rgb565_color);
hmi_cmd_result_t hmi_cmd_lcd_set_indicator(uint8_t index, bool state);
hmi_cmd_result_t hmi_cmd_lcd_set_progress(uint8_t index, uint8_t value);

#ifdef __cplusplus
}
#endif

#endif // HMI_H
