#include "hmi.h"

#include <Arduino.h>
#include <Wire.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

// #define DEBUG_HMI 1

static constexpr uint8_t I2C_ADDR = 0x14;
static constexpr int I2C_SDA = 21;
static constexpr int I2C_SCL = 22;
static constexpr uint8_t I2C_READ_LEN = 6;
static constexpr uint32_t I2C_WAIT_RETRY_MS = 50U;

static constexpr uint8_t STATUS_BIT_USB_CONNECTED = 12;
static constexpr uint8_t STATUS_BIT_BACKLIGHT_ON = 14;
static constexpr uint8_t STATUS_BIT_LCD_BUSY = 15;

static constexpr uint8_t CMD_BACKLIGHT_TIMEOUT = 0x03;
static constexpr uint8_t CMD_BACKLIGHT_BRIGHTNESS = 0x04;
static constexpr uint8_t CMD_TONE = 0x07;
static constexpr uint8_t CMD_MELODY = 0x08;
static constexpr uint8_t CMD_POWER_OFF = 0x0F;
static constexpr uint8_t CMD_LCD_CLEAR = 0x10;
static constexpr uint8_t CMD_LCD_DRAW_MARKER = 0x12;
static constexpr uint8_t CMD_LCD_DRAW_TEXT = 0x13;
static constexpr uint8_t CMD_LCD_SET_BG = 0x14;
static constexpr uint8_t CMD_LCD_INDICATOR = 0x20;
static constexpr uint8_t CMD_LCD_PROGRESS = 0x21;

enum hmi_sys_cmd_type_t {
  HMI_SYS_CMD_BEEP = 0,
  HMI_SYS_CMD_MELODY,
  HMI_SYS_CMD_POWER_OFF,
  HMI_SYS_CMD_BRIGHTNESS,
  HMI_SYS_CMD_BL_TIMEOUT,
  HMI_SYS_CMD_INDICATOR0,
  HMI_SYS_CMD_INDICATOR1,
  HMI_SYS_CMD_PROGRESS0,
  HMI_SYS_CMD_PROGRESS1,
  HMI_SYS_CMD_PROGRESS2,
  HMI_SYS_CMD_COUNT
};

typedef struct {
  uint16_t divider;
  uint16_t delayMs;
  bool hasData;
} hmi_sys_beep_t;
typedef struct {
  uint32_t timeoutMs;
  bool hasData;
} hmi_sys_bl_timeout_t;

typedef struct {
  uint8_t value;
  bool hasData;
} hmi_sys_u8_t;

static bool s_initialized = false;
static bool s_lcdSendAllowed = false;
static uint32_t s_dataBits = 0U;
static uint32_t s_changed = 0U;
static uint16_t s_joyX = 0U;
static uint16_t s_joyY = 0U;
static hmi_log_callback_t s_logCallback = nullptr;

static hmi_sys_beep_t s_sysBeep = { 0U, 0U, false };
static hmi_sys_u8_t s_sysMelody = { 0U, false };
static bool s_sysPowerOff = false;
static hmi_sys_u8_t s_sysBrightness = { 0U, false };
static hmi_sys_bl_timeout_t s_sysBlTimeout = { 0U, false };
static hmi_sys_u8_t s_sysIndicator[2] = { { 0U, false }, { 0U, false } };
static hmi_sys_u8_t s_sysProgress[3] = { { 0U, false }, { 0U, false }, { 0U, false } };

static uint32_t set_bit(uint32_t data, uint8_t idx, uint8_t value) {
  return (data & ~(1UL << idx)) | (((uint32_t)(value & 1U)) << idx);
}

static void LogMessage(bool emergency, const char* fmt, ...) {
  if ((s_logCallback == nullptr) || (fmt == nullptr)) {
    return;
  }

  char buf[160];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);
  s_logCallback(buf, emergency);
}

static void LogError(const char* funcName, const char* errName) {
  if ((funcName == nullptr) || (errName == nullptr)) {
    return;
  }
  LogMessage(false, "%s - %s", funcName, errName);
}

#ifdef DEBUG_HMI
static const char* ButtonNameFromIndex(uint8_t idx) {
  switch (idx) {
    case HMI_DATA_BTN_ON: return "ON";
    case HMI_DATA_BTN_FIRE: return "FIRE";
    case HMI_DATA_BTN_UP: return "UP";
    case HMI_DATA_BTN_DOWN: return "DOWN";
    case HMI_DATA_BTN_BACK: return "BACK";
    case HMI_DATA_BTN_OK: return "OK";
    case HMI_DATA_BTN_LUP: return "LUP";
    case HMI_DATA_BTN_LDN: return "LDN";
    case HMI_DATA_BTN_RUP: return "RUP";
    case HMI_DATA_BTN_RDN: return "RDN";
    default: return "NONE";
  }
}

static void LogStateIfChanged(void) {
  if (s_changed == 0U) {
    return;
  }

  const char* buttonName = "NONE";
  for (uint8_t idx = (uint8_t)HMI_DATA_BTN_ON; idx <= (uint8_t)HMI_DATA_BTN_RDN; ++idx) {
    if ((s_dataBits & (1UL << idx)) != 0UL) {
      buttonName = ButtonNameFromIndex(idx);
      break;
    }
  }

  LogMessage(false,
             "%1X %5u %5u %s",
             (unsigned int)(s_dataBits & 0x0FU),
             (unsigned int)s_joyX,
             (unsigned int)s_joyY,
             buttonName);
}

static void LogTxBytes(const uint8_t* data, uint8_t len) {
  if ((data == nullptr) || (len == 0U)) {
    return;
  }

  char buf[160];
  int pos = snprintf(buf, sizeof(buf), "TX");
  for (uint8_t i = 0U; (i < len) && (pos >= 0) && (pos < (int)sizeof(buf)); ++i) {
    pos += snprintf(&buf[pos], sizeof(buf) - (size_t)pos, " %02X", data[i]);
  }
  LogMessage(false, "%s", buf);
}
#else
static void LogStateIfChanged(void) {}
static void LogTxBytes(const uint8_t*, uint8_t) {}
#endif

static bool WaitForI2cDevice(void) {
  for (;;) {
    Wire.beginTransmission(I2C_ADDR);
    const uint8_t rc = Wire.endTransmission(true);
    if (rc == 0U) return true;
    LogError("hmi_init", "I2C_WAIT");
    delay(I2C_WAIT_RETRY_MS);
  }
}

static hmi_cmd_result_t SendCommand(const char* funcName, const uint8_t* data, uint8_t len, bool isLcd) {
  if (!s_initialized) {
    LogError(funcName, "NOT_INITIALIZED");
    return HMI_CMD_ERR_NOT_INITIALIZED;
  }
  if ((data == nullptr) || (len == 0U) || (len > 32U)) {
    LogError(funcName, "INVALID_ARG");
    return HMI_CMD_ERR_INVALID_ARG;
  }
  if (isLcd && !s_lcdSendAllowed) {
    LogError(funcName, "NOT_READY");
    return HMI_CMD_ERR_NOT_READY;
  }

  Wire.beginTransmission(I2C_ADDR);
  const size_t written = Wire.write(data, len);
  const uint8_t rc = Wire.endTransmission(true);
  if ((rc != 0U) || (written != len)) {
    LogError(funcName, "I2C_TX");
    return HMI_CMD_ERR_I2C_TX;
  }

  if (isLcd) {
    s_lcdSendAllowed = false;
  }

  LogTxBytes(data, len);
  return HMI_CMD_OK;
}

static uint16_t abs2(uint16_t a, uint16_t b) {
  return (a > b) ? (a - b) : (b - a);
}

static void ParsePacket(const uint8_t* rx) {
  const uint16_t word0 = (uint16_t)((uint16_t)rx[0] | ((uint16_t)rx[1] << 8));
  const uint16_t joyX = (uint16_t)(((uint16_t)(rx[3] & 0x0FU) << 8) | (uint16_t)rx[2]);
  const uint16_t joyY = (uint16_t)(((uint16_t)(rx[5] & 0x0FU) << 8) | (uint16_t)rx[4]);

  uint32_t newBits = s_dataBits;
  newBits = set_bit(newBits, HMI_DATA_STAT_LCD_BUSY, (word0 & (1U << STATUS_BIT_LCD_BUSY)) ? 1U : 0U);
  newBits = set_bit(newBits, HMI_DATA_STAT_USB_CONN, (word0 & (1U << STATUS_BIT_USB_CONNECTED)) ? 1U : 0U);
  newBits = set_bit(newBits, HMI_DATA_STAT_BL_ON, (word0 & (1U << STATUS_BIT_BACKLIGHT_ON)) ? 1U : 0U);

  const uint32_t newButtons = (uint32_t)(word0 & 0x03FFU);
  newBits &= ~(0x03FFUL << (uint8_t)HMI_DATA_BTN_ON);
  newBits |= (newButtons << (uint8_t)HMI_DATA_BTN_ON);
  newBits = set_bit(newBits, HMI_DATA_BTN_ANYKEY, (newButtons != 0U) ? 1U : 0U);

  s_lcdSendAllowed = ((word0 & (1U << STATUS_BIT_LCD_BUSY)) == 0U);
  s_changed = (s_dataBits ^ newBits);

  if (abs2(joyX, s_joyX) > 2) {
    s_joyX = joyX;
    s_changed |= (1UL << HMI_DATA_JOY_X);
  }
  if (abs2(joyY, s_joyY) > 2) {
    s_joyY = joyY;
    s_changed |= (1UL << HMI_DATA_JOY_Y);
  }
  s_dataBits = newBits;
}

static hmi_sys_cmd_type_t FindNextSysCmd(void) {
  if (s_sysBeep.hasData) return HMI_SYS_CMD_BEEP;
  if (s_sysMelody.hasData) return HMI_SYS_CMD_MELODY;
  if (s_sysPowerOff) return HMI_SYS_CMD_POWER_OFF;
  if (s_sysBrightness.hasData) return HMI_SYS_CMD_BRIGHTNESS;
  if (s_sysBlTimeout.hasData) return HMI_SYS_CMD_BL_TIMEOUT;
  if (s_sysIndicator[0].hasData) return HMI_SYS_CMD_INDICATOR0;
  if (s_sysIndicator[1].hasData) return HMI_SYS_CMD_INDICATOR1;
  if (s_sysProgress[0].hasData) return HMI_SYS_CMD_PROGRESS0;
  if (s_sysProgress[1].hasData) return HMI_SYS_CMD_PROGRESS1;
  if (s_sysProgress[2].hasData) return HMI_SYS_CMD_PROGRESS2;
  return HMI_SYS_CMD_COUNT;
}

static void ClearSysCmd(hmi_sys_cmd_type_t type) {
  switch (type) {
    case HMI_SYS_CMD_BEEP:
      s_sysBeep.hasData = false;
      break;
    case HMI_SYS_CMD_MELODY:
      s_sysMelody.hasData = false;
      break;
    case HMI_SYS_CMD_POWER_OFF:
      s_sysPowerOff = false;
      break;
    case HMI_SYS_CMD_BRIGHTNESS:
      s_sysBrightness.hasData = false;
      break;
    case HMI_SYS_CMD_BL_TIMEOUT:
      s_sysBlTimeout.hasData = false;
      break;
    case HMI_SYS_CMD_INDICATOR0:
      s_sysIndicator[0].hasData = false;
      break;
    case HMI_SYS_CMD_INDICATOR1:
      s_sysIndicator[1].hasData = false;
      break;
    case HMI_SYS_CMD_PROGRESS0:
      s_sysProgress[0].hasData = false;
      break;
    case HMI_SYS_CMD_PROGRESS1:
      s_sysProgress[1].hasData = false;
      break;
    case HMI_SYS_CMD_PROGRESS2:
      s_sysProgress[2].hasData = false;
      break;
    default:
      break;
  }
}

void hmi_init(hmi_log_callback_t log_callback) {
  s_initialized = false;
  s_lcdSendAllowed = false;
  s_dataBits = 0U;
  s_changed = 0U;
  s_joyX = 0U;
  s_joyY = 0U;
  s_logCallback = log_callback;

  s_sysBeep = { 0U, 0U, false };
  s_sysMelody = { 0U, false };
  s_sysPowerOff = false;
  s_sysBrightness = { 0U, false };
  s_sysBlTimeout = { 0U, false };
  s_sysIndicator[0] = { 0U, false };
  s_sysIndicator[1] = { 0U, false };
  s_sysProgress[0] = { 0U, false };
  s_sysProgress[1] = { 0U, false };
  s_sysProgress[2] = { 0U, false };

  if (!Wire.begin(I2C_SDA, I2C_SCL)) {
    LogError("hmi_init", "WIRE_BEGIN");
    return;
  }
  if (!Wire.setClock(100000U)) {
    LogError("hmi_init", "WIRE_CLOCK");
    return;
  }
  Wire.setTimeOut(10);
  (void)WaitForI2cDevice();
  s_initialized = true;
}

hmi_tick_result_t hmi_tick(void) {
  if (!s_initialized) {
    LogError("hmi_tick", "NOT_INITIALIZED");
    return HMI_TICK_ERR_NOT_INITIALIZED;
  }

  uint8_t rx[I2C_READ_LEN];
  uint8_t len = 0U;
  hmi_tick_result_t result = HMI_TICK_OK;

  const int requested = Wire.requestFrom((int)I2C_ADDR, (int)I2C_READ_LEN, (int)true);
  if (requested != I2C_READ_LEN) {
    LogError("hmi_tick", "I2C_REQUEST");
    result = (hmi_tick_result_t)(result | HMI_TICK_ERR_I2C_REQUEST);
  }

  while (Wire.available() && (len < I2C_READ_LEN)) {
    rx[len++] = (uint8_t)Wire.read();
  }

  if (len != I2C_READ_LEN) {
    LogError("hmi_tick", "I2C_READ");
    result = (hmi_tick_result_t)(result | HMI_TICK_ERR_I2C_READ);
  }

  if (result != HMI_TICK_OK) {
    return result;
  }

  ParsePacket(rx);
  LogStateIfChanged();
  return HMI_TICK_OK;
}

uint16_t hmi_get(hmi_data_idx_t idx) {
  if ((uint8_t)idx >= (uint8_t)HMI_DATA_COUNT) return 0U;
  if (idx == HMI_DATA_JOY_X) return s_joyX;
  if (idx == HMI_DATA_JOY_Y) return s_joyY;
  return ((s_dataBits & (1UL << (uint8_t)idx)) != 0UL) ? 1U : 0U;
}

bool hmi_changed(hmi_data_idx_t idx) {
  if ((uint8_t)idx >= (uint8_t)HMI_DATA_COUNT) return false;
  return ((s_changed & (1UL << (uint8_t)idx)) != 0UL);
}

void hmi_sysSend(void) {
  const hmi_sys_cmd_type_t type = FindNextSysCmd();
  if (type == HMI_SYS_CMD_COUNT) {
    return;
  }

  hmi_cmd_result_t rc = HMI_CMD_ERR_INVALID_ARG;
  switch (type) {
    case HMI_SYS_CMD_BEEP: {
      uint8_t data[5];
      data[0] = CMD_TONE;
      data[1] = (uint8_t)(s_sysBeep.divider & 0xFFU);
      data[2] = (uint8_t)(s_sysBeep.divider >> 8);
      data[3] = (uint8_t)(s_sysBeep.delayMs & 0xFFU);
      data[4] = (uint8_t)(s_sysBeep.delayMs >> 8);
      rc = SendCommand("hmi_sysSend", data, sizeof(data), false);
      break;
    }
    case HMI_SYS_CMD_MELODY: {
      const uint8_t data[2] = { CMD_MELODY, s_sysMelody.value };
      rc = SendCommand("hmi_sysSend", data, sizeof(data), false);
      break;
    }
    case HMI_SYS_CMD_POWER_OFF: {
      const uint8_t data[2] = { CMD_POWER_OFF, 0xAAU };
      rc = SendCommand("hmi_sysSend", data, sizeof(data), false);
      break;
    }
    case HMI_SYS_CMD_BRIGHTNESS: {
      const uint8_t data[2] = { CMD_BACKLIGHT_BRIGHTNESS, s_sysBrightness.value };
      rc = SendCommand("hmi_sysSend", data, sizeof(data), false);
      break;
    }
    case HMI_SYS_CMD_BL_TIMEOUT: {
      uint8_t data[5] = {
        CMD_BACKLIGHT_TIMEOUT,
        (uint8_t)(s_sysBlTimeout.timeoutMs & 0xFFU),
        (uint8_t)((s_sysBlTimeout.timeoutMs >> 8) & 0xFFU),
        (uint8_t)((s_sysBlTimeout.timeoutMs >> 16) & 0xFFU),
        (uint8_t)((s_sysBlTimeout.timeoutMs >> 24) & 0xFFU)
      };
      rc = SendCommand("hmi_sysSend", data, sizeof(data), false);
      break;
    }
    case HMI_SYS_CMD_INDICATOR0:
    case HMI_SYS_CMD_INDICATOR1: {
      const uint8_t index = (type == HMI_SYS_CMD_INDICATOR0) ? 0U : 1U;
      const uint8_t data[3] = { CMD_LCD_INDICATOR, index, s_sysIndicator[index].value };
      rc = SendCommand("hmi_sysSend", data, sizeof(data), true);
      break;
    }
    case HMI_SYS_CMD_PROGRESS0:
    case HMI_SYS_CMD_PROGRESS1:
    case HMI_SYS_CMD_PROGRESS2: {
      const uint8_t index = (uint8_t)(type - HMI_SYS_CMD_PROGRESS0);
      const uint8_t data[3] = { CMD_LCD_PROGRESS, index, s_sysProgress[index].value };
      rc = SendCommand("hmi_sysSend", data, sizeof(data), true);
      break;
    }
    default:
      return;
  }

  if ((rc == HMI_CMD_OK) || (rc == HMI_CMD_ERR_INVALID_ARG)) {
    ClearSysCmd(type);
  }
}

void hmi_cmd_set_backlight_timeout(uint32_t timeout_ms) {
  if (!s_initialized) {
    LogError("hmi_cmd_set_backlight_timeout", "NOT_INITIALIZED");
    return;
  }
  s_sysBlTimeout.timeoutMs = timeout_ms;
  s_sysBlTimeout.hasData = true;
}

void hmi_cmd_set_brightness(uint8_t level) {
  if (!s_initialized) {
    LogError("hmi_cmd_set_brightness", "NOT_INITIALIZED");
    return;
  }
  s_sysBrightness.value = level;
  s_sysBrightness.hasData = true;
}

void hmi_cmd_play_tone(uint16_t divider, uint16_t delay_ms) {
  if (!s_initialized) {
    LogError("hmi_cmd_play_tone", "NOT_INITIALIZED");
    return;
  }
  s_sysBeep.divider = divider;
  s_sysBeep.delayMs = delay_ms;
  s_sysBeep.hasData = true;
}

void hmi_cmd_play_melody(hmi_melody_t melody) {
  if (!s_initialized) {
    LogError("hmi_cmd_play_melody", "NOT_INITIALIZED");
    return;
  }
  if ((melody != HMI_MELODY_POWER_ON) &&
      (melody != HMI_MELODY_CONNECTED) &&
      (melody != HMI_MELODY_DISCONNECTED)) {
    LogError("hmi_cmd_play_melody", "INVALID_ARG");
    return;
  }
  s_sysMelody.value = (uint8_t)melody;
  s_sysMelody.hasData = true;
}

void hmi_cmd_power_off(void) {
  if (!s_initialized) {
    LogError("hmi_cmd_power_off", "NOT_INITIALIZED");
    return;
  }
  s_sysPowerOff = true;
}

hmi_cmd_result_t hmi_cmd_lcd_clear(uint16_t rgb565_color) {
  uint8_t data[3];
  data[0] = CMD_LCD_CLEAR;
  data[1] = (uint8_t)(rgb565_color & 0xFFU);
  data[2] = (uint8_t)(rgb565_color >> 8);
  return SendCommand("hmi_cmd_lcd_clear", data, sizeof(data), true);
}

hmi_cmd_result_t hmi_cmd_lcd_set_bg(uint16_t rgb565_color) {
  uint8_t data[3];
  data[0] = CMD_LCD_SET_BG;
  data[1] = (uint8_t)(rgb565_color & 0xFFU);
  data[2] = (uint8_t)(rgb565_color >> 8);
  return SendCommand("hmi_cmd_lcd_set_bg", data, sizeof(data), true);
}

hmi_cmd_result_t hmi_cmd_lcd_draw_text(uint8_t x, uint8_t y, uint16_t rgb565_color, const char* text) {
  if (text == nullptr) {
    LogError("hmi_cmd_lcd_draw_text", "INVALID_ARG");
    return HMI_CMD_ERR_INVALID_ARG;
  }

  const size_t textLen = strlen(text);
  if (textLen > 26U) {
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

hmi_cmd_result_t hmi_cmd_lcd_draw_marker(uint8_t x, uint8_t y, uint8_t index, uint16_t rgb565_color) {
  uint8_t data[6];
  data[0] = CMD_LCD_DRAW_MARKER;
  data[1] = x;
  data[2] = y;
  data[3] = index;
  data[4] = (uint8_t)(rgb565_color & 0xFFU);
  data[5] = (uint8_t)(rgb565_color >> 8);
  return SendCommand("hmi_cmd_lcd_draw_marker", data, sizeof(data), true);
}

void hmi_cmd_lcd_set_indicator(uint8_t index, bool state) {
  if (index > 1U) {
    LogError("hmi_cmd_lcd_set_indicator", "INVALID_ARG");
    return;
  }
  if (!s_initialized) {
    LogError("hmi_cmd_lcd_set_indicator", "NOT_INITIALIZED");
    return;
  }
  s_sysIndicator[index].value = state ? 1U : 0U;
  s_sysIndicator[index].hasData = true;
}

void hmi_cmd_lcd_set_progress(uint8_t index, uint8_t value) {
  if (index > 2U) {
    LogError("hmi_cmd_lcd_set_progress", "INVALID_ARG");
    return;
  }
  if (!s_initialized) {
    LogError("hmi_cmd_lcd_set_progress", "NOT_INITIALIZED");
    return;
  }
  s_sysProgress[index].value = value;
  s_sysProgress[index].hasData = true;
}
