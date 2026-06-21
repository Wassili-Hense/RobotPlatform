#include "hmi.h"

#include <Arduino.h>
#include <Wire.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

static constexpr uint8_t I2C_ADDR = 0x14;
static constexpr int I2C_SDA = 21;
static constexpr int I2C_SCL = 22;
static constexpr uint8_t I2C_READ_LEN = 6;

static constexpr uint8_t STATUS_BIT_USB_CONNECTED = 12;
static constexpr uint8_t STATUS_BIT_BACKLIGHT_ON  = 14;
static constexpr uint8_t STATUS_BIT_LCD_BUSY      = 15;

static constexpr uint8_t CMD_BACKLIGHT_TIMEOUT    = 0x03;
static constexpr uint8_t CMD_BACKLIGHT_BRIGHTNESS = 0x04;
static constexpr uint8_t CMD_TONE                 = 0x07;
static constexpr uint8_t CMD_LCD_CLEAR            = 0x10;
static constexpr uint8_t CMD_LCD_DRAW_MARKER      = 0x12;
static constexpr uint8_t CMD_LCD_DRAW_TEXT        = 0x13;
static constexpr uint8_t CMD_LCD_SET_BG           = 0x14;
static constexpr uint8_t CMD_LCD_INDICATOR        = 0x20;
static constexpr uint8_t CMD_LCD_PROGRESS         = 0x21;

static constexpr uint8_t LCD_X_MIN = 5U;
static constexpr uint8_t LCD_X_MAX = 154U;
static constexpr uint8_t LCD_Y_MIN = 5U;
static constexpr uint8_t LCD_Y_MAX = 79U;
static constexpr uint8_t FONT_W = 7U;
static constexpr uint8_t FONT_H = 10U;

static bool s_initialized = false;
static bool s_lcdSendAllowed = false;
static uint32_t s_dataBits = 0U;
static uint32_t s_changed = 0U;
static uint16_t s_joyX = 0U;
static uint16_t s_joyY = 0U;
static hmi_log_callback_t s_logCallback = nullptr;

static uint32_t set_bit(uint32_t data, uint8_t idx, uint8_t value)
{
    return (data & ~(1UL << idx)) | (((uint32_t)(value & 1U)) << idx);
}

#ifdef DEBUG_HMI
static void LogMessage(bool emergency, const char* fmt, ...)
{
    if (s_logCallback == nullptr || fmt == nullptr)
    {
        return;
    }

    char buf[160];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    s_logCallback(buf, emergency);
}

static void LogError(const char* funcName, const char* errName)
{
    if (funcName == nullptr || errName == nullptr)
    {
        return;
    }
    LogMessage(true, "%s - %s", funcName, errName);
}

static const char* ButtonNameFromIndex(uint8_t idx)
{
    switch (idx)
    {
        case HMI_DATA_BTN_ON:   return "ON";
        case HMI_DATA_BTN_FIRE: return "FIRE";
        case HMI_DATA_BTN_UP:   return "UP";
        case HMI_DATA_BTN_DOWN: return "DOWN";
        case HMI_DATA_BTN_BACK: return "BACK";
        case HMI_DATA_BTN_OK:   return "OK";
        case HMI_DATA_BTN_LUP:  return "LUP";
        case HMI_DATA_BTN_LDN:  return "LDN";
        case HMI_DATA_BTN_RUP:  return "RUP";
        case HMI_DATA_BTN_RDN:  return "RDN";
        default:                return "NONE";
    }
}

static void LogStateIfChanged(void)
{
    if (s_changed == 0U)
    {
        return;
    }

    const char* buttonName = "NONE";
    for (uint8_t idx = (uint8_t)HMI_DATA_BTN_ON; idx <= (uint8_t)HMI_DATA_BTN_RDN; ++idx)
    {
        if ((s_dataBits & (1UL << idx)) != 0UL)
        {
            buttonName = ButtonNameFromIndex(idx);
            break;
        }
    }

    LogMessage(false,
               "status=%1X X=%5u Y=%5u button=%s",
               (unsigned int)(s_dataBits & 0x0FU),
               (unsigned int)s_joyX,
               (unsigned int)s_joyY,
               buttonName);
}

static void LogTxBytes(const uint8_t* data, uint8_t len)
{
    if (data == nullptr || len == 0U)
    {
        return;
    }

    char buf[160];
    int pos = snprintf(buf, sizeof(buf), "TX");
    for (uint8_t i = 0U; i < len && pos >= 0 && pos < (int)sizeof(buf); ++i)
    {
        pos += snprintf(&buf[pos], sizeof(buf) - (size_t)pos, " %02X", data[i]);
    }
    LogMessage(false, "%s", buf);
}
#else
static void LogError(const char*, const char*) {}
static void LogStateIfChanged(void) {}
static void LogTxBytes(const uint8_t*, uint8_t) {}
#endif

static bool IsPrintableAscii(const char* text)
{
    if (text == nullptr)
    {
        return false;
    }

    for (size_t i = 0U; text[i] != '\0'; ++i)
    {
        const uint8_t c = (uint8_t)text[i];
        if (c < 32U || c > 126U)
        {
            return false;
        }
    }
    return true;
}

static bool IsTextPosValid(uint8_t x, uint8_t y, size_t textLen)
{
    if (x < LCD_X_MIN || x > LCD_X_MAX || y < LCD_Y_MIN || y > LCD_Y_MAX)
    {
        return false;
    }
    if (textLen == 0U)
    {
        return true;
    }

    const uint16_t xEnd = (uint16_t)x + (uint16_t)(textLen * FONT_W) - 1U;
    const uint16_t yEnd = (uint16_t)y + (uint16_t)FONT_H - 1U;
    return (xEnd <= LCD_X_MAX) && (yEnd <= LCD_Y_MAX);
}

static bool IsPointInLcdArea(uint8_t x, uint8_t y)
{
    return (x >= LCD_X_MIN) && (x <= LCD_X_MAX) && (y >= LCD_Y_MIN) && (y <= LCD_Y_MAX);
}

static hmi_cmd_result_t SendCommand(const char* funcName, const uint8_t* data, uint8_t len, bool isLcd)
{
    if (!s_initialized)
    {
        LogError(funcName, "NOT_INITIALIZED");
        return HMI_CMD_ERR_NOT_INITIALIZED;
    }
    if (data == nullptr || len == 0U || len > 32U)
    {
        LogError(funcName, "INVALID_ARG");
        return HMI_CMD_ERR_INVALID_ARG;
    }
    if (isLcd && !s_lcdSendAllowed)
    {
        LogError(funcName, "NOT_READY");
        return HMI_CMD_ERR_NOT_READY;
    }

    Wire.beginTransmission(I2C_ADDR);
    const size_t written = Wire.write(data, len);
    const uint8_t rc = Wire.endTransmission(true);
    if (rc != 0U || written != len)
    {
        LogError(funcName, "I2C_TX");
        return HMI_CMD_ERR_I2C_TX;
    }

    if (isLcd)
    {
        s_lcdSendAllowed = false;
    }

    LogTxBytes(data, len);
    return HMI_CMD_OK;
}

static void ParsePacket(const uint8_t* rx)
{
    const uint16_t word0 = (uint16_t)((uint16_t)rx[0] | ((uint16_t)rx[1] << 8));
    const uint16_t joyX = (uint16_t)(((uint16_t)(rx[3] & 0x0FU) << 8) | (uint16_t)rx[2]);
    const uint16_t joyY = (uint16_t)(((uint16_t)(rx[5] & 0x0FU) << 8) | (uint16_t)rx[4]);
    const bool joyXChanged = ((rx[3] & 0x80U) != 0U);
    const bool joyYChanged = ((rx[5] & 0x80U) != 0U);

    uint32_t newBits = s_dataBits;
    newBits = set_bit(newBits, HMI_DATA_STAT_LCD_BUSY, (word0 & (1U << STATUS_BIT_LCD_BUSY)) ? 1U : 0U);
    newBits = set_bit(newBits, HMI_DATA_STAT_USB_CONN, (word0 & (1U << STATUS_BIT_USB_CONNECTED)) ? 1U : 0U);
    newBits = set_bit(newBits, HMI_DATA_STAT_BL_ON, (word0 & (1U << STATUS_BIT_BACKLIGHT_ON)) ? 1U : 0U);

    const uint32_t newButtons = (uint32_t)(word0 & 0x03FFU);
    newBits &= ~(0x03FFUL << (uint8_t)HMI_DATA_BTN_ON);
    newBits |= (newButtons << (uint8_t)HMI_DATA_BTN_ON);

    s_lcdSendAllowed = ((word0 & (1U << STATUS_BIT_LCD_BUSY)) == 0U);
    s_changed |= (s_dataBits ^ newBits);

    if (joyX != s_joyX || joyXChanged)
    {
        s_joyX = joyX;
        s_changed |= (1UL << HMI_DATA_JOY_X);
    }
    if (joyY != s_joyY || joyYChanged)
    {
        s_joyY = joyY;
        s_changed |= (1UL << HMI_DATA_JOY_Y);
    }

    s_dataBits = newBits;
}

void hmi_init(hmi_log_callback_t log_callback)
{
    s_initialized = false;
    s_lcdSendAllowed = false;
    s_dataBits = 0U;
    s_changed = 0U;
    s_joyX = 0U;
    s_joyY = 0U;
    s_logCallback = log_callback;

    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.setClock(100000);

    s_initialized = true;
}

hmi_tick_result_t hmi_tick(void)
{
    if (!s_initialized)
    {
        LogError("hmi_tick", "NOT_INITIALIZED");
        return HMI_TICK_ERR_NOT_INITIALIZED;
    }

    uint8_t rx[I2C_READ_LEN];
    uint8_t len = 0U;
    hmi_tick_result_t result = HMI_TICK_OK;

    const int requested = Wire.requestFrom((int)I2C_ADDR, (int)I2C_READ_LEN, (int)true);
    while (Wire.available() && len < I2C_READ_LEN)
    {
        rx[len++] = (uint8_t)Wire.read();
    }

    if (requested != I2C_READ_LEN)
    {
        LogError("hmi_tick", "I2C_REQUEST");
        result = (hmi_tick_result_t)(result | HMI_TICK_ERR_I2C_REQUEST);
    }
    if (len != I2C_READ_LEN)
    {
        LogError("hmi_tick", "I2C_READ");
        result = (hmi_tick_result_t)(result | HMI_TICK_ERR_I2C_READ);
    }
    if (result != HMI_TICK_OK)
    {
        return result;
    }

    ParsePacket(rx);
    LogStateIfChanged();
    return HMI_TICK_OK;
}

uint16_t hmi_get(hmi_data_idx_t idx)
{
    if ((uint8_t)idx >= (uint8_t)HMI_DATA_COUNT)
    {
        return 0U;
    }
    if (idx == HMI_DATA_JOY_X)
    {
        return s_joyX;
    }
    if (idx == HMI_DATA_JOY_Y)
    {
        return s_joyY;
    }
    return ((s_dataBits & (1UL << (uint8_t)idx)) != 0UL) ? 1U : 0U;
}

hmi_data_idx_t hmi_next_changed(void)
{
    for (uint8_t i = 0U; i < (uint8_t)HMI_DATA_COUNT; ++i)
    {
        if ((s_changed & (1UL << i)) != 0UL)
        {
            s_changed &= ~(1UL << i);
            return (hmi_data_idx_t)i;
        }
    }
    return HMI_DATA_COUNT;
}

hmi_cmd_result_t hmi_cmd_set_backlight_timeout(uint32_t timeout_ms)
{
    uint8_t data[5] = {
        CMD_BACKLIGHT_TIMEOUT,
        (uint8_t)(timeout_ms & 0xFFU),
        (uint8_t)((timeout_ms >> 8) & 0xFFU),
        (uint8_t)((timeout_ms >> 16) & 0xFFU),
        (uint8_t)((timeout_ms >> 24) & 0xFFU)
    };
    return SendCommand("hmi_cmd_set_backlight_timeout", data, sizeof(data), false);
}

hmi_cmd_result_t hmi_cmd_set_brightness(uint8_t level)
{
    const uint8_t data[2] = { CMD_BACKLIGHT_BRIGHTNESS, level };
    return SendCommand("hmi_cmd_set_brightness", data, sizeof(data), false);
}

hmi_cmd_result_t hmi_cmd_play_tone(uint16_t divider, uint16_t delay_ms)
{
    uint8_t data[5];
    data[0] = CMD_TONE;
    data[1] = (uint8_t)(divider & 0xFFU);
    data[2] = (uint8_t)(divider >> 8);
    data[3] = (uint8_t)(delay_ms & 0xFFU);
    data[4] = (uint8_t)(delay_ms >> 8);
    return SendCommand("hmi_cmd_play_tone", data, sizeof(data), false);
}

hmi_cmd_result_t hmi_cmd_lcd_clear(uint16_t rgb565_color)
{
    uint8_t data[3];
    data[0] = CMD_LCD_CLEAR;
    data[1] = (uint8_t)(rgb565_color & 0xFFU);
    data[2] = (uint8_t)(rgb565_color >> 8);
    return SendCommand("hmi_cmd_lcd_clear", data, sizeof(data), true);
}

hmi_cmd_result_t hmi_cmd_lcd_set_bg(uint16_t rgb565_color)
{
    uint8_t data[3];
    data[0] = CMD_LCD_SET_BG;
    data[1] = (uint8_t)(rgb565_color & 0xFFU);
    data[2] = (uint8_t)(rgb565_color >> 8);
    return SendCommand("hmi_cmd_lcd_set_bg", data, sizeof(data), true);
}

hmi_cmd_result_t hmi_cmd_lcd_draw_text(uint8_t x, uint8_t y, uint16_t rgb565_color, const char* text)
{
    if (text == nullptr)
    {
        LogError("hmi_cmd_lcd_draw_text", "INVALID_ARG");
        return HMI_CMD_ERR_INVALID_ARG;
    }

    const size_t textLen = strlen(text);
    if (textLen > 26U || !IsPrintableAscii(text) || !IsTextPosValid(x, y, textLen))
    {
        LogError("hmi_cmd_lcd_draw_text", "INVALID_ARG");
        return HMI_CMD_ERR_INVALID_ARG;
    }

    uint8_t data[32] = { 0 };
    data[0] = CMD_LCD_DRAW_TEXT;
    data[1] = x;
    data[2] = y;
    data[3] = (uint8_t)(rgb565_color & 0xFFU);
    data[4] = (uint8_t)(rgb565_color >> 8);
    memcpy(&data[5], text, textLen);
    data[5U + textLen] = 0U;
    return SendCommand("hmi_cmd_lcd_draw_text", data, (uint8_t)(6U + textLen), true);
}

hmi_cmd_result_t hmi_cmd_lcd_draw_marker(uint8_t x, uint8_t y, uint8_t index, uint16_t rgb565_color)
{
    if (!IsPointInLcdArea(x, y))
    {
        LogError("hmi_cmd_lcd_draw_marker", "INVALID_ARG");
        return HMI_CMD_ERR_INVALID_ARG;
    }

    uint8_t data[6];
    data[0] = CMD_LCD_DRAW_MARKER;
    data[1] = x;
    data[2] = y;
    data[3] = index;
    data[4] = (uint8_t)(rgb565_color & 0xFFU);
    data[5] = (uint8_t)(rgb565_color >> 8);
    return SendCommand("hmi_cmd_lcd_draw_marker", data, sizeof(data), true);
}

hmi_cmd_result_t hmi_cmd_lcd_set_indicator(uint8_t index, bool state)
{
    const uint8_t data[3] = { CMD_LCD_INDICATOR, index, (uint8_t)(state ? 1U : 0U) };
    return SendCommand("hmi_cmd_lcd_set_indicator", data, sizeof(data), true);
}

hmi_cmd_result_t hmi_cmd_lcd_set_progress(uint8_t index, uint8_t value)
{
    const uint8_t data[3] = { CMD_LCD_PROGRESS, index, value };
    return SendCommand("hmi_cmd_lcd_set_progress", data, sizeof(data), true);
}
