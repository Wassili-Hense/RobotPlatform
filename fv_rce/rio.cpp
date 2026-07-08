#include "rio.h"

#include <Arduino.h>
#include <Wire.h>
#include <stdio.h>
#include <string.h>

//#define DEBUG_RIO 1

static constexpr uint8_t I2C_ADDR = 0x14;
static constexpr int I2C_SDA = 21;
static constexpr int I2C_SCL = 22;
static constexpr uint8_t I2C_READ_LEN = 8;
static constexpr uint32_t I2C_WAIT_RETRY_MS = 50U;
static constexpr uint8_t I2C_WAIT_MAX_ATTEMPTS = 5U;

static constexpr uint8_t STATUS_BIT_USB_CONNECTED = 12;
static constexpr uint8_t STATUS_BIT_BACKLIGHT_ON = 14;

static constexpr uint8_t ADC_CHANGED_MASK = 0x80U;
static constexpr uint8_t ADC_HI_MASK = 0x0FU;

static constexpr uint8_t CMD_BACKLIGHT_TIMEOUT = 0x03;
static constexpr uint8_t CMD_BACKLIGHT_BRIGHTNESS = 0x04;
static constexpr uint8_t CMD_TONE = 0x07;
static constexpr uint8_t CMD_MELODY = 0x08;
static constexpr uint8_t CMD_POWER_OFF = 0x0F;

enum rio_sys_cmd_type_t {
  RIO_SYS_CMD_BEEP = 0,
  RIO_SYS_CMD_MELODY,
  RIO_SYS_CMD_POWER_OFF,
  RIO_SYS_CMD_BRIGHTNESS,
  RIO_SYS_CMD_BL_TIMEOUT,
  RIO_SYS_CMD_COUNT
};

typedef struct {
  uint16_t divider;
  uint16_t delayMs;
  bool hasData;
} rio_sys_beep_t;

typedef struct {
  uint32_t timeoutMs;
  bool hasData;
} rio_sys_bl_timeout_t;

typedef struct {
  uint8_t value;
  bool hasData;
} rio_sys_u8_t;

static bool s_initialized = false;
static uint32_t s_dataBits = 0U;
static uint32_t s_changed = 0U;
static uint16_t s_adcX = 0U;
static uint16_t s_adcY = 0U;
static uint16_t s_adcV = 0U;
static rio_log_callback_t s_logCallback = nullptr;

static rio_sys_beep_t s_sysBeep = { 0U, 0U, false };
static rio_sys_u8_t s_sysMelody = { 0U, false };
static bool s_sysPowerOff = false;
static rio_sys_u8_t s_sysBrightness = { 0U, false };
static rio_sys_bl_timeout_t s_sysBlTimeout = { 0U, false };

static uint32_t set_bit(uint32_t data, uint8_t idx, uint8_t value) {
  return (data & ~(1UL << idx)) | (((uint32_t)(value & 1U)) << idx);
}

static rio_result_t LogError(const char* funcName, rio_result_t result) {
  if ((s_logCallback != nullptr) && (funcName != nullptr)) {
    rio_log_event_t ev = {};
    ev.result = result;
    ev.data.error.funcName = funcName;
    s_logCallback(&ev);
  }
  return result;
}

static void LogBytes(rio_result_t result, const uint8_t* data, uint8_t len) {
#ifdef DEBUG_RIO
  if ((s_logCallback == nullptr) || (data == nullptr) || (len == 0U)) return;
  if ((result != RIO_EVT_RX) && (result != RIO_EVT_TX)) return;

  rio_log_event_t ev = {};
  ev.result = result;
  ev.data.bytes.data = data;
  ev.data.bytes.len = len;
  s_logCallback(&ev);
#endif
}

static bool WaitForI2cDevice(void) {
  for (uint8_t attempt = 0U; attempt < I2C_WAIT_MAX_ATTEMPTS; ++attempt) {
    Wire.beginTransmission(I2C_ADDR);
    const uint8_t rc = Wire.endTransmission(true);
    if (rc == 0U) return true;
    if ((attempt + 1U) < I2C_WAIT_MAX_ATTEMPTS) {
      delay(I2C_WAIT_RETRY_MS);
    }
  }
  return false;
}

static rio_result_t SendCommand(const char* funcName, const uint8_t* data, uint8_t len) {
  if (!s_initialized) return LogError(funcName, RIO_ERR_NOT_INITIALIZED);
  if ((data == nullptr) || (len == 0U) || (len > 32U)) return LogError(funcName, RIO_ERR_INVALID_ARG);

  Wire.beginTransmission(I2C_ADDR);
  const size_t written = Wire.write(data, len);
  const uint8_t rc = Wire.endTransmission(true);
  if ((rc != 0U) || (written != len)) return LogError(funcName, RIO_ERR_I2C_TX);

  LogBytes(RIO_EVT_TX, data, len);
  return RIO_OK;
}

static uint16_t abs2(uint16_t a, uint16_t b) {
  return (a > b) ? (a - b) : (b - a);
}

static void UpdateAdc(const uint8_t* rx, uint8_t offset, uint16_t* storage, rio_data_idx_t idx) {
  if (storage == nullptr) return;
  uint16_t value = (uint16_t)(((uint16_t)(rx[offset + 1U] & ADC_HI_MASK) << 8) | (uint16_t)rx[offset]);
  if (abs2(value, *storage) > 4U) {
    *storage = value;
    s_changed |= (1UL << (uint8_t)idx);
  }
}

static void ParsePacket(const uint8_t* rx) {
  const uint16_t word0 = (uint16_t)((uint16_t)rx[0] | ((uint16_t)rx[1] << 8));

  uint32_t newBits = s_dataBits;
  newBits = set_bit(newBits, RIO_DATA_STAT_USB_CONN, (word0 & (1U << STATUS_BIT_USB_CONNECTED)) ? 1U : 0U);
  newBits = set_bit(newBits, RIO_DATA_STAT_BL_ON, (word0 & (1U << STATUS_BIT_BACKLIGHT_ON)) ? 1U : 0U);

  const uint32_t newButtons = (uint32_t)(word0 & 0x03FFU);
  newBits &= ~(0x03FFUL << (uint8_t)RIO_DATA_BTN_ON);
  newBits |= (newButtons << (uint8_t)RIO_DATA_BTN_ON);
  newBits = set_bit(newBits, RIO_DATA_BTN_ANYKEY, (newButtons != 0U) ? 1U : 0U);

  s_changed = (s_dataBits ^ newBits);
  UpdateAdc(rx, 2U, &s_adcX, RIO_DATA_JOY_X);
  UpdateAdc(rx, 4U, &s_adcY, RIO_DATA_JOY_Y);
  UpdateAdc(rx, 6U, &s_adcV, RIO_DATA_ADC_V);
  s_dataBits = newBits;
}

static rio_sys_cmd_type_t FindNextSysCmd(void) {
  if (s_sysBeep.hasData) return RIO_SYS_CMD_BEEP;
  if (s_sysMelody.hasData) return RIO_SYS_CMD_MELODY;
  if (s_sysBrightness.hasData) return RIO_SYS_CMD_BRIGHTNESS;
  if (s_sysBlTimeout.hasData) return RIO_SYS_CMD_BL_TIMEOUT;
  if (s_sysPowerOff) return RIO_SYS_CMD_POWER_OFF;
  return RIO_SYS_CMD_COUNT;
}

static void ClearSysCmd(rio_sys_cmd_type_t type) {
  switch (type) {
    case RIO_SYS_CMD_BEEP:
      s_sysBeep.hasData = false;
      break;
    case RIO_SYS_CMD_MELODY:
      s_sysMelody.hasData = false;
      break;
    case RIO_SYS_CMD_POWER_OFF:
      s_sysPowerOff = false;
      break;
    case RIO_SYS_CMD_BRIGHTNESS:
      s_sysBrightness.hasData = false;
      break;
    case RIO_SYS_CMD_BL_TIMEOUT:
      s_sysBlTimeout.hasData = false;
      break;
    default:
      break;
  }
}

rio_result_t rio_init(rio_log_callback_t log_callback) {
  s_initialized = false;
  s_dataBits = 0U;
  s_changed = 0U;
  s_adcX = 0U;
  s_adcY = 0U;
  s_adcV = 0U;
  s_logCallback = log_callback;

  if (!Wire.begin(I2C_SDA, I2C_SCL)) return LogError("rio_init", RIO_ERR_WIRE_BEGIN);
  if (!Wire.setClock(100000U)) return LogError("rio_init", RIO_ERR_WIRE_CLOCK);
  Wire.setTimeOut(10);
  if (!WaitForI2cDevice()) return LogError("rio_init", RIO_ERR_NOT_INITIALIZED);
  s_initialized = true;
  return RIO_OK;
}

rio_result_t rio_tick(void) {
  if (!s_initialized) return LogError("rio_tick", RIO_ERR_NOT_INITIALIZED);
  uint8_t rx[I2C_READ_LEN];
  uint8_t len = 0U;
  rio_result_t result = RIO_OK;

  const int requested = Wire.requestFrom((int)I2C_ADDR, (int)I2C_READ_LEN, (int)true);
  if (requested != I2C_READ_LEN) {
    result = (rio_result_t)(result | LogError("rio_tick", RIO_ERR_I2C_REQUEST));
  }
  while (Wire.available() && (len < I2C_READ_LEN)) {
    rx[len++] = (uint8_t)Wire.read();
  }
  if (len != I2C_READ_LEN) {
    result = (rio_result_t)(result | LogError("rio_tick", RIO_ERR_I2C_READ));
  }
  if (result != RIO_OK) return result;
  LogBytes(RIO_EVT_RX, rx, I2C_READ_LEN);
  ParsePacket(rx);
  return RIO_OK;
}

uint16_t rio_get(rio_data_idx_t idx) {
  if ((uint8_t)idx >= (uint8_t)RIO_DATA_COUNT) return 0U;
  if ((idx == RIO_DATA_JOY_X)) return s_adcX;
  if ((idx == RIO_DATA_JOY_Y)) return s_adcY;
  if (idx == RIO_DATA_ADC_V) return s_adcV;
  return ((s_dataBits & (1UL << (uint8_t)idx)) != 0UL) ? 1U : 0U;
}

bool rio_changed(rio_data_idx_t idx) {
  if ((uint8_t)idx >= (uint8_t)RIO_DATA_COUNT) return false;
  return ((s_changed & (1UL << (uint8_t)idx)) != 0UL);
}

void rio_sysSend(void) {
  const rio_sys_cmd_type_t type = FindNextSysCmd();
  if (type == RIO_SYS_CMD_COUNT) return;

  rio_result_t rc = RIO_ERR_INVALID_ARG;
  switch (type) {
    case RIO_SYS_CMD_BEEP: {
      uint8_t data[5];
      data[0] = CMD_TONE;
      data[1] = (uint8_t)(s_sysBeep.divider & 0xFFU);
      data[2] = (uint8_t)(s_sysBeep.divider >> 8);
      data[3] = (uint8_t)(s_sysBeep.delayMs & 0xFFU);
      data[4] = (uint8_t)(s_sysBeep.delayMs >> 8);
      rc = SendCommand("rio_Beep", data, sizeof(data));
      break;
    }
    case RIO_SYS_CMD_MELODY: {
      const uint8_t data[2] = { CMD_MELODY, s_sysMelody.value };
      rc = SendCommand("rio_Melody", data, sizeof(data));
      break;
    }
    case RIO_SYS_CMD_POWER_OFF: {
      const uint8_t data[2] = { CMD_POWER_OFF, 0xAAU };
      rc = SendCommand("rio_Power", data, sizeof(data));
      break;
    }
    case RIO_SYS_CMD_BRIGHTNESS: {
      const uint8_t data[2] = { CMD_BACKLIGHT_BRIGHTNESS, s_sysBrightness.value };
      rc = SendCommand("rio_BR", data, sizeof(data));
      break;
    }
    case RIO_SYS_CMD_BL_TIMEOUT: {
      uint8_t data[5] = {
        CMD_BACKLIGHT_TIMEOUT,
        (uint8_t)(s_sysBlTimeout.timeoutMs & 0xFFU),
        (uint8_t)((s_sysBlTimeout.timeoutMs >> 8) & 0xFFU),
        (uint8_t)((s_sysBlTimeout.timeoutMs >> 16) & 0xFFU),
        (uint8_t)((s_sysBlTimeout.timeoutMs >> 24) & 0xFFU)
      };
      rc = SendCommand("rio_TO", data, sizeof(data));
      break;
    }
    default:
      return;
  }

  if ((rc == RIO_OK) || (rc == RIO_ERR_INVALID_ARG)) ClearSysCmd(type);
}

void rio_cmd_set_backlight_timeout(uint32_t timeout_ms) {
  s_sysBlTimeout.timeoutMs = timeout_ms;
  s_sysBlTimeout.hasData = true;
}

void rio_cmd_set_brightness(uint8_t level) {
  s_sysBrightness.value = level;
  s_sysBrightness.hasData = true;
}

void rio_cmd_play_tone(uint16_t divider, uint16_t delay_ms) {
  s_sysBeep.divider = divider;
  s_sysBeep.delayMs = delay_ms;
  s_sysBeep.hasData = true;
}

void rio_cmd_play_melody(rio_melody_t melody) {
  if ((melody != RIO_MELODY_POWER_ON) &&
      (melody != RIO_MELODY_CONNECTED) &&
      (melody != RIO_MELODY_DISCONNECTED)) {
    (void)LogError("rio_melody", RIO_ERR_INVALID_ARG);
    return;
  }
  s_sysMelody.value = (uint8_t)melody;
  s_sysMelody.hasData = true;
}

void rio_cmd_power_off(void) {
  s_sysPowerOff = true;
}
