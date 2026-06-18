
#include <Wire.h>
#include <Preferences.h>
#include <ctype.h>
#include <string.h>
#include <stdlib.h>

static constexpr uint8_t  I2C_ADDR       = 0x14;
static constexpr int      SDA_PIN        = 21;
static constexpr int      SCL_PIN        = 22;
static constexpr uint32_t POLL_PERIOD_MS = 5;
static constexpr uint32_t SERIAL_BAUD    = 115200;

// Read indexes
static constexpr uint8_t IDX_STATUS   = 0;
static constexpr uint8_t IDX_ADC_X    = 1;
static constexpr uint8_t IDX_ADC_Y    = 2;
static constexpr uint8_t IDX_BUTTON_0 = 3;
static constexpr uint8_t IDX_BUTTON_9 = 12;

// STATUS bits
static constexpr uint16_t STATUS_LCD_BUSY      = (1U << 0);
static constexpr uint16_t STATUS_USB_CONNECTED = (1U << 1);

// Button protocol indexes
static constexpr uint8_t BTN_ON   = 3;
static constexpr uint8_t BTN_FIRE = 4;
static constexpr uint8_t BTN_UP   = 5;
static constexpr uint8_t BTN_DOWN = 6;
static constexpr uint8_t BTN_BACK = 7;
static constexpr uint8_t BTN_OK   = 8;
static constexpr uint8_t BTN_LUP  = 9;
static constexpr uint8_t BTN_LDN  = 10;
static constexpr uint8_t BTN_RUP  = 11;
static constexpr uint8_t BTN_RDN  = 12;

// Write commands
static constexpr uint8_t CMD_BACKLIGHT_TIMEOUT    = 0x03;
static constexpr uint8_t CMD_BACKLIGHT_BRIGHTNESS = 0x04;
static constexpr uint8_t CMD_TONE                 = 0x07;
static constexpr uint8_t CMD_LCD_CLEAR            = 0x10;
static constexpr uint8_t CMD_LCD_FILL_CIRCLE      = 0x12;
static constexpr uint8_t CMD_LCD_DRAW_TEXT        = 0x13;
static constexpr uint8_t CMD_LCD_INDICATOR        = 0x20;
static constexpr uint8_t CMD_LCD_PROGRESS_BAR     = 0x21;

// LCD_Indicator params
static constexpr uint8_t LCD_INDICATOR_CMD = 0;
static constexpr uint8_t LCD_INDICATOR_USB = 1;
static constexpr uint8_t LCD_INDICATOR_OFF = 0;
static constexpr uint8_t LCD_INDICATOR_ON  = 1;

// Colors RGB565
static constexpr uint16_t LCD_BLACK = 0x0000;
static constexpr uint16_t LCD_WHITE = 0xFFFF;
static constexpr uint16_t LCD_GREEN = 0x07E0;
static constexpr uint16_t LCD_BLUE  = 0x001F;

// Display areas
static constexpr uint8_t JOY_X0 = 10;
static constexpr uint8_t JOY_Y0 = 10;
static constexpr uint8_t JOY_X1 = 79;
static constexpr uint8_t JOY_Y1 = 79;
static constexpr uint8_t JOY_R  = 3;

static constexpr uint8_t MENU_TEXT_X  = 10;
static constexpr uint8_t MENU_TEXT_Y0 = 10;
static constexpr uint8_t MENU_TEXT_DY = 12;
static constexpr uint8_t MENU_BL_Y    = MENU_TEXT_Y0 + 3U * MENU_TEXT_DY;

static constexpr uint8_t CAL_TEXT_X  = 86;
static constexpr uint8_t CAL_TEXT_Y0 = 10;
static constexpr uint8_t CAL_TEXT_DY = 12;

// Timing
static constexpr uint32_t MENU_TIMEOUT_MS           = 60000;
static constexpr uint32_t MENU_BACKLIGHT_TIMEOUT_MS = 15000;
static constexpr uint32_t MENU_BACKLIGHT_REFRESH_MS = 10000;

// Brightness
static constexpr uint8_t BACKLIGHT_STEP_COUNT = 11;
static const uint8_t kBacklightLevels[BACKLIGHT_STEP_COUNT] = { 1, 2, 4, 7, 11, 18, 29, 46, 74, 107, 127 };

// LCD queue
static constexpr uint8_t LCD_QUEUE_SIZE    = 32;
static constexpr uint8_t LCD_CMD_MAX_LEN   = 40;
static constexpr uint8_t LCD_DRAW_TEXT_MAX = 32;

// Serial commands
static constexpr uint8_t SERIAL_LINE_MAX = 48;

struct AxisCal
{
  uint16_t min;
  uint16_t cMin;
  uint16_t cMax;
  uint16_t max;
};

struct LcdCmd
{
  uint8_t len;
  uint8_t data[LCD_CMD_MAX_LEN];
};

struct MarkerState
{
  bool visible;
  bool pending;
  uint8_t drawX;
  uint8_t drawY;
  uint8_t targetX;
  uint8_t targetY;
};

enum AppMode : uint8_t
{
  MODE_VIEW = 0,
  MODE_MENU,
  MODE_CAL_CENTER,
  MODE_CAL_EDGE
};

enum MenuItem : uint8_t
{
  MENU_CAL_CENTER = 0,
  MENU_CAL_EDGE,
  MENU_COUNT
};

static Preferences s_prefs;

static uint16_t s_status = 0;
static uint16_t s_adcX = 2047;
static uint16_t s_adcY = 2047;
static uint8_t  s_buttons[10] = {0};

static bool s_adcXValid = false;
static bool s_adcYValid = false;
static bool s_serialActive = false;
static bool s_lcdSendAllowed = false;
static bool s_seenFirstStatus = false;
static bool s_initialBrightnessPending = true;

static bool s_usbIndicatorPending = false;
static uint8_t s_usbIndicatorState = LCD_INDICATOR_OFF;

static MarkerState s_marker =
{
  false,
  false,
  JOY_X0,
  JOY_Y0,
  JOY_X0,
  JOY_Y0
};

static AxisCal s_calX = { 0, 2000, 2095, 4095 };
static AxisCal s_calY = { 0, 2000, 2095, 4095 };
static AxisCal s_calXCenterCandidate = { 0, 0, 0, 0 };
static AxisCal s_calYCenterCandidate = { 0, 0, 0, 0 };
static AxisCal s_calXEdgeCandidate   = { 0, 0, 0, 0 };
static AxisCal s_calYEdgeCandidate   = { 0, 0, 0, 0 };

static bool s_calCenterCandidateValid = false;
static bool s_calEdgeCandidateValid = false;

static AppMode s_mode = MODE_VIEW;
static uint8_t s_menuIndex = MENU_CAL_CENTER;
static uint8_t s_backlightLevel = 46;
static uint8_t s_backlightIndex = 7;

static LcdCmd s_lcdQueue[LCD_QUEUE_SIZE];
static uint8_t s_lcdQueueHead = 0;
static uint8_t s_lcdQueueTail = 0;
static uint8_t s_lcdQueueCount = 0;

static uint32_t s_nextPollMs = 0;
static uint32_t s_menuLastActivityMs = 0;
static uint32_t s_nextBacklightRefreshMs = 0;

static char s_serialLine[SERIAL_LINE_MAX + 1U];
static uint8_t s_serialLineLen = 0;

static const char* btnNames[] =
{
  "", "", "", "ON", "Fire", "UP", "DOWN", "BACK", "OK",
  "LUP", "LDN", "RUP", "RDN"
};

static const char* menuNames[MENU_COUNT] =
{
  "Cal center",
  "Cal edge"
};

static bool IsUiActive(void)
{
  return (s_mode == MODE_MENU) || (s_mode == MODE_CAL_CENTER) || (s_mode == MODE_CAL_EDGE);
}

static int FindNearestBacklightIndex(uint8_t level)
{
  int bestIndex = 0;
  int bestDiff = 1000;

  for (int i = 0; i < (int)BACKLIGHT_STEP_COUNT; ++i)
  {
    const int diff = abs((int)level - (int)kBacklightLevels[i]);
    if (diff < bestDiff)
    {
      bestDiff = diff;
      bestIndex = i;
    }
  }

  return bestIndex;
}

static void NormalizeBacklightSettings(void)
{
  s_backlightIndex = (uint8_t)FindNearestBacklightIndex(s_backlightLevel);
  s_backlightLevel = kBacklightLevels[s_backlightIndex];
}

static uint8_t GetCurrentBrightnessForMode(void)
{
  if (IsUiActive())
  {
    return s_backlightLevel;
  }

  const uint8_t half = (uint8_t)(s_backlightLevel / 2U);
  return (half == 0U) ? 1U : half;
}

static void SaveSettings(void)
{
  s_prefs.putUShort("x_min", s_calX.min);
  s_prefs.putUShort("x_cmin", s_calX.cMin);
  s_prefs.putUShort("x_cmax", s_calX.cMax);
  s_prefs.putUShort("x_max", s_calX.max);

  s_prefs.putUShort("y_min", s_calY.min);
  s_prefs.putUShort("y_cmin", s_calY.cMin);
  s_prefs.putUShort("y_cmax", s_calY.cMax);
  s_prefs.putUShort("y_max", s_calY.max);

  s_prefs.putUChar("bl", s_backlightLevel);
}

static void LoadSettings(void)
{
  s_backlightLevel = s_prefs.getUChar("bl", 46);
  if (s_backlightLevel < 1U)
  {
    s_backlightLevel = 1U;
  }
  if (s_backlightLevel > 127U)
  {
    s_backlightLevel = 127U;
  }
  NormalizeBacklightSettings();

  s_calX.min  = s_prefs.getUShort("x_min", 0);
  s_calX.cMin = s_prefs.getUShort("x_cmin", 2000);
  s_calX.cMax = s_prefs.getUShort("x_cmax", 2095);
  s_calX.max  = s_prefs.getUShort("x_max", 4095);

  s_calY.min  = s_prefs.getUShort("y_min", 0);
  s_calY.cMin = s_prefs.getUShort("y_cmin", 2000);
  s_calY.cMax = s_prefs.getUShort("y_cmax", 2095);
  s_calY.max  = s_prefs.getUShort("y_max", 4095);
}

static float ClampNorm(float v)
{
  if (v < -1.0f)
  {
    return -1.0f;
  }

  if (v > 1.0f)
  {
    return 1.0f;
  }

  return v;
}

static float ApplyCalibration(uint16_t raw, const AxisCal& cal)
{
  if (raw <= cal.min)
  {
    return -1.0f;
  }

  if (raw >= cal.max)
  {
    return 1.0f;
  }

  if (raw <= cal.cMin)
  {
    const int32_t denom = (int32_t)cal.cMin - (int32_t)cal.min;
    if (denom <= 0)
    {
      return -1.0f;
    }

    const float t = (float)((int32_t)raw - (int32_t)cal.min) / (float)denom;
    return ClampNorm(-1.0f + t);
  }

  if (raw < cal.cMax)
  {
    return 0.0f;
  }

  const int32_t denom = (int32_t)cal.max - (int32_t)cal.cMax;
  if (denom <= 0)
  {
    return 1.0f;
  }

  const float t = (float)((int32_t)raw - (int32_t)cal.cMax) / (float)denom;
  return ClampNorm(t);
}

static uint8_t NormToRange(float norm, uint8_t outMin, uint8_t outMax)
{
  const float t = (ClampNorm(norm) + 1.0f) * 0.5f;
  const float scaled = (float)outMin + t * (float)((int32_t)outMax - (int32_t)outMin);
  int32_t iv = (int32_t)(scaled + 0.5f);

  if (iv < outMin)
  {
    iv = outMin;
  }

  if (iv > outMax)
  {
    iv = outMax;
  }

  return (uint8_t)iv;
}

static float GetNormX(void)
{
  return ApplyCalibration(s_adcX, s_calX);
}

static float GetNormY(void)
{
  return ApplyCalibration(s_adcY, s_calY);
}

static const char* GetPressedButtonName(void)
{
  for (uint8_t index = IDX_BUTTON_0; index <= IDX_BUTTON_9; ++index)
  {
    if (s_buttons[index - IDX_BUTTON_0] != 0U)
    {
      return btnNames[index];
    }
  }

  return "";
}

static void LogI2cTx(const uint8_t* data, size_t len)
{
  if (!s_serialActive)
  {
    return;
  }

  Serial.print("TX");
  for (size_t i = 0; i < len; ++i)
  {
    Serial.print(' ');
    if (data[i] < 0x10U)
    {
      Serial.print('0');
    }
    Serial.print(data[i], HEX);
  }
  Serial.println();
}

static bool I2C_WriteBytes(const uint8_t* data, size_t len)
{
  LogI2cTx(data, len);

  Wire.beginTransmission(I2C_ADDR);
  Wire.write(data, len);
  const uint8_t err = Wire.endTransmission();
  return (err == 0U);
}

static bool I2C_ReadChangedItem(uint8_t& index, uint16_t& value)
{
  const uint8_t readCount = Wire.requestFrom((int)I2C_ADDR, 2);

  if (readCount != 2U)
  {
    while (Wire.available() > 0)
    {
      (void)Wire.read();
    }
    return false;
  }

  const uint8_t b0 = (uint8_t)Wire.read();
  const uint8_t b1 = (uint8_t)Wire.read();

  index = (uint8_t)(b0 >> 4);
  value = (uint16_t)(((uint16_t)(b0 & 0x0FU) << 8) | b1);
  return true;
}

static bool SendBacklightTimeout(uint32_t timeoutMs)
{
  const uint8_t cmd[5] =
  {
    CMD_BACKLIGHT_TIMEOUT,
    (uint8_t)(timeoutMs & 0xFFU),
    (uint8_t)((timeoutMs >> 8) & 0xFFU),
    (uint8_t)((timeoutMs >> 16) & 0xFFU),
    (uint8_t)((timeoutMs >> 24) & 0xFFU)
  };

  return I2C_WriteBytes(cmd, sizeof(cmd));
}

static bool SendBacklightBrightness(uint8_t level)
{
  const uint8_t cmd[2] = { CMD_BACKLIGHT_BRIGHTNESS, level };
  return I2C_WriteBytes(cmd, sizeof(cmd));
}

static bool SendTone(uint16_t divider, uint16_t delayMs)
{
  const uint8_t cmd[5] =
  {
    CMD_TONE,
    (uint8_t)(divider & 0xFFU),
    (uint8_t)(divider >> 8),
    (uint8_t)(delayMs & 0xFFU),
    (uint8_t)(delayMs >> 8)
  };

  return I2C_WriteBytes(cmd, sizeof(cmd));
}

static bool LCD_Indicator(uint8_t index, uint8_t state)
{
  const uint8_t cmd[3] = { CMD_LCD_INDICATOR, index, state };
  return I2C_WriteBytes(cmd, sizeof(cmd));
}

static bool LCD_FillCircle(uint8_t x, uint8_t y, uint8_t r, uint16_t color)
{
  const uint8_t cmd[6] =
  {
    CMD_LCD_FILL_CIRCLE,
    x,
    y,
    r,
    (uint8_t)(color & 0xFFU),
    (uint8_t)(color >> 8)
  };

  return I2C_WriteBytes(cmd, sizeof(cmd));
}

static bool LcdQueuePush(const uint8_t* data, uint8_t len)
{
  if ((len == 0U) || (len > LCD_CMD_MAX_LEN) || (s_lcdQueueCount >= LCD_QUEUE_SIZE))
  {
    return false;
  }

  s_lcdQueue[s_lcdQueueTail].len = len;
  memcpy(s_lcdQueue[s_lcdQueueTail].data, data, len);
  s_lcdQueueTail = (uint8_t)((s_lcdQueueTail + 1U) % LCD_QUEUE_SIZE);
  ++s_lcdQueueCount;
  return true;
}

static bool LcdQueuePeek(LcdCmd& cmd)
{
  if (s_lcdQueueCount == 0U)
  {
    return false;
  }

  cmd = s_lcdQueue[s_lcdQueueHead];
  return true;
}

static void LcdQueuePop(void)
{
  if (s_lcdQueueCount == 0U)
  {
    return;
  }

  s_lcdQueueHead = (uint8_t)((s_lcdQueueHead + 1U) % LCD_QUEUE_SIZE);
  --s_lcdQueueCount;
}

static void LcdQueueClearAll(void)
{
  s_lcdQueueHead = 0;
  s_lcdQueueTail = 0;
  s_lcdQueueCount = 0;
}

static bool QueueLcdClear(uint16_t color)
{
  const uint8_t cmd[3] =
  {
    CMD_LCD_CLEAR,
    (uint8_t)(color & 0xFFU),
    (uint8_t)(color >> 8)
  };

  return LcdQueuePush(cmd, sizeof(cmd));
}

static bool QueueLcdDrawText(uint8_t x, uint8_t y, const char* text)
{
  uint8_t cmd[LCD_CMD_MAX_LEN];
  uint8_t len = 0;

  cmd[len++] = CMD_LCD_DRAW_TEXT;
  cmd[len++] = x;
  cmd[len++] = y;

  for (uint8_t i = 0; (i < LCD_DRAW_TEXT_MAX) && (text[i] != '\0'); ++i)
  {
    if (len >= (LCD_CMD_MAX_LEN - 1U))
    {
      break;
    }

    char ch = text[i];
    if ((ch < 32) || (ch > 126))
    {
      ch = '?';
    }
    cmd[len++] = (uint8_t)ch;
  }

  cmd[len++] = 0;
  return LcdQueuePush(cmd, len);
}

static bool QueueLcdProgressBar(uint8_t index, uint8_t value)
{
  const uint8_t cmd[3] = { CMD_LCD_PROGRESS_BAR, index, value };
  return LcdQueuePush(cmd, sizeof(cmd));
}

static bool QueueLcdIndicator(uint8_t index, uint8_t value)
{
  const uint8_t cmd[3] = { CMD_LCD_INDICATOR, index, value };
  return LcdQueuePush(cmd, sizeof(cmd));
}

static void QueueViewScreen(void)
{
  LcdQueueClearAll();
  (void)QueueLcdClear(LCD_BLACK);
}

static void QueueMenuBrightnessField(void)
{
  char line[12];
  snprintf(line, sizeof(line), "BL %3u", s_backlightLevel);
  (void)QueueLcdDrawText(MENU_TEXT_X, MENU_BL_Y, line);
}

static void QueueMenuScreen(void)
{
  char line[20];

  LcdQueueClearAll();
  (void)QueueLcdClear(LCD_BLACK);

  for (uint8_t i = 0; i < MENU_COUNT; ++i)
  {
    snprintf(line, sizeof(line), "%c%s", (i == s_menuIndex) ? '>' : ' ', menuNames[i]);
    (void)QueueLcdDrawText(MENU_TEXT_X, (uint8_t)(MENU_TEXT_Y0 + i * MENU_TEXT_DY), line);
  }

  QueueMenuBrightnessField();
}

static void QueueCalibrationScreen(const char* title)
{
  LcdQueueClearAll();
  (void)QueueLcdClear(LCD_BLACK);
  (void)QueueLcdDrawText(CAL_TEXT_X, CAL_TEXT_Y0, title);
  (void)QueueLcdDrawText(CAL_TEXT_X, (uint8_t)(CAL_TEXT_Y0 + CAL_TEXT_DY), "OK Save");
  (void)QueueLcdDrawText(CAL_TEXT_X, (uint8_t)(CAL_TEXT_Y0 + 2U * CAL_TEXT_DY), "BACK");
}

static void TouchMenuActivity(void)
{
  s_menuLastActivityMs = millis();
}

static void RefreshMenuBacklightTimer(void)
{
  if (!IsUiActive())
  {
    return;
  }

  (void)SendBacklightTimeout(MENU_BACKLIGHT_TIMEOUT_MS);
  s_nextBacklightRefreshMs = millis() + MENU_BACKLIGHT_REFRESH_MS;
}

static void ApplyMenuBrightness(void)
{
  if (s_seenFirstStatus)
  {
    (void)SendBacklightBrightness(s_backlightLevel);
  }
}

static void ApplyViewBrightness(void)
{
  if (s_seenFirstStatus)
  {
    (void)SendBacklightBrightness(GetCurrentBrightnessForMode());
  }
}

static void UpdateMarkerTarget(void)
{
  if (!s_adcXValid || !s_adcYValid)
  {
    return;
  }

  s_marker.targetX = NormToRange(GetNormX(), JOY_X0, JOY_X1);
  s_marker.targetY = NormToRange(GetNormY(), JOY_Y0, JOY_Y1);

  switch (s_mode)
  {
    case MODE_VIEW:
      if (!s_marker.visible || (s_marker.drawX != s_marker.targetX) || (s_marker.drawY != s_marker.targetY))
      {
        s_marker.pending = true;
      }
      break;

    case MODE_CAL_CENTER:
    case MODE_CAL_EDGE:
      s_marker.pending = true;
      break;

    default:
      s_marker.pending = false;
      break;
  }
}

static uint16_t GetMarkerColor(void)
{
  switch (s_mode)
  {
    case MODE_CAL_CENTER:
      return LCD_GREEN;

    case MODE_CAL_EDGE:
      return LCD_BLUE;

    default:
      return LCD_WHITE;
  }
}

static void EnterMenu(void)
{
  s_mode = MODE_MENU;
  s_marker.visible = false;
  s_marker.pending = false;
  TouchMenuActivity();
  QueueMenuScreen();
  ApplyMenuBrightness();
  RefreshMenuBacklightTimer();
}

static void ExitToView(void)
{
  s_mode = MODE_VIEW;
  s_marker.visible = false;
  s_marker.pending = false;
  QueueViewScreen();
  ApplyViewBrightness();
  UpdateMarkerTarget();
}

static void StartCalCenter(void)
{
  s_mode = MODE_CAL_CENTER;
  s_calCenterCandidateValid = s_adcXValid && s_adcYValid;

  if (s_calCenterCandidateValid)
  {
    s_calXCenterCandidate = s_calX;
    s_calYCenterCandidate = s_calY;
    s_calXCenterCandidate.cMin = s_adcX;
    s_calXCenterCandidate.cMax = s_adcX;
    s_calYCenterCandidate.cMin = s_adcY;
    s_calYCenterCandidate.cMax = s_adcY;
  }

  s_marker.visible = false;
  s_marker.pending = false;
  TouchMenuActivity();
  QueueCalibrationScreen("Center");
  RefreshMenuBacklightTimer();
  UpdateMarkerTarget();
}

static void StartCalEdge(void)
{
  s_mode = MODE_CAL_EDGE;
  s_calEdgeCandidateValid = s_adcXValid && s_adcYValid;

  if (s_calEdgeCandidateValid)
  {
    s_calXEdgeCandidate = s_calX;
    s_calYEdgeCandidate = s_calY;
    s_calXEdgeCandidate.min = s_adcX;
    s_calXEdgeCandidate.max = s_adcX;
    s_calYEdgeCandidate.min = s_adcY;
    s_calYEdgeCandidate.max = s_adcY;
  }

  s_marker.visible = false;
  s_marker.pending = false;
  TouchMenuActivity();
  QueueCalibrationScreen("Edge");
  RefreshMenuBacklightTimer();
  UpdateMarkerTarget();
}

static void SaveCalCenter(void)
{
  if (!s_calCenterCandidateValid)
  {
    return;
  }

  if (s_calXCenterCandidate.cMin > s_calXCenterCandidate.cMax)
  {
    const uint16_t t = s_calXCenterCandidate.cMin;
    s_calXCenterCandidate.cMin = s_calXCenterCandidate.cMax;
    s_calXCenterCandidate.cMax = t;
  }

  if (s_calYCenterCandidate.cMin > s_calYCenterCandidate.cMax)
  {
    const uint16_t t = s_calYCenterCandidate.cMin;
    s_calYCenterCandidate.cMin = s_calYCenterCandidate.cMax;
    s_calYCenterCandidate.cMax = t;
  }

  if (s_calXCenterCandidate.cMin < s_calX.min)
  {
    s_calXCenterCandidate.cMin = s_calX.min;
  }
  if (s_calXCenterCandidate.cMax > s_calX.max)
  {
    s_calXCenterCandidate.cMax = s_calX.max;
  }
  if (s_calYCenterCandidate.cMin < s_calY.min)
  {
    s_calYCenterCandidate.cMin = s_calY.min;
  }
  if (s_calYCenterCandidate.cMax > s_calY.max)
  {
    s_calYCenterCandidate.cMax = s_calY.max;
  }

  s_calX.cMin = s_calXCenterCandidate.cMin;
  s_calX.cMax = s_calXCenterCandidate.cMax;
  s_calY.cMin = s_calYCenterCandidate.cMin;
  s_calY.cMax = s_calYCenterCandidate.cMax;
  SaveSettings();
}

static void SaveCalEdge(void)
{
  if (!s_calEdgeCandidateValid)
  {
    return;
  }

  if (s_calXEdgeCandidate.min > s_calXEdgeCandidate.max)
  {
    const uint16_t t = s_calXEdgeCandidate.min;
    s_calXEdgeCandidate.min = s_calXEdgeCandidate.max;
    s_calXEdgeCandidate.max = t;
  }
  if (s_calYEdgeCandidate.min > s_calYEdgeCandidate.max)
  {
    const uint16_t t = s_calYEdgeCandidate.min;
    s_calYEdgeCandidate.min = s_calYEdgeCandidate.max;
    s_calYEdgeCandidate.max = t;
  }

  if (s_calXEdgeCandidate.min > s_calX.cMin)
  {
    s_calXEdgeCandidate.min = s_calX.cMin;
  }
  if (s_calXEdgeCandidate.max < s_calX.cMax)
  {
    s_calXEdgeCandidate.max = s_calX.cMax;
  }
  if (s_calYEdgeCandidate.min > s_calY.cMin)
  {
    s_calYEdgeCandidate.min = s_calY.cMin;
  }
  if (s_calYEdgeCandidate.max < s_calY.cMax)
  {
    s_calYEdgeCandidate.max = s_calY.cMax;
  }

  s_calX.min = s_calXEdgeCandidate.min;
  s_calX.max = s_calXEdgeCandidate.max;
  s_calY.min = s_calYEdgeCandidate.min;
  s_calY.max = s_calYEdgeCandidate.max;
  SaveSettings();
}

static void StepBacklight(int delta)
{
  int nextIndex = (int)s_backlightIndex + delta;

  if (nextIndex < 0)
  {
    nextIndex = 0;
  }
  if (nextIndex >= (int)BACKLIGHT_STEP_COUNT)
  {
    nextIndex = (int)BACKLIGHT_STEP_COUNT - 1;
  }
  if ((uint8_t)nextIndex == s_backlightIndex)
  {
    return;
  }

  s_backlightIndex = (uint8_t)nextIndex;
  s_backlightLevel = kBacklightLevels[s_backlightIndex];
  SaveSettings();

  if (IsUiActive())
  {
    ApplyMenuBrightness();
    RefreshMenuBacklightTimer();
  }

  if (s_mode == MODE_MENU)
  {
    (void)QueueLcdDrawText(MENU_TEXT_X, MENU_BL_Y, "      ");
    QueueMenuBrightnessField();
  }
}

static void HandleButtonPressed(uint8_t btnIndex)
{
  switch (s_mode)
  {
    case MODE_VIEW:
      if (btnIndex == BTN_OK)
      {
        EnterMenu();
      }
      break;

    case MODE_MENU:
      switch (btnIndex)
      {
        case BTN_UP:
          if (s_menuIndex > 0U)
          {
            --s_menuIndex;
            QueueMenuScreen();
          }
          break;

        case BTN_DOWN:
          if (s_menuIndex + 1U < MENU_COUNT)
          {
            ++s_menuIndex;
            QueueMenuScreen();
          }
          break;

        case BTN_OK:
          if (s_menuIndex == MENU_CAL_CENTER)
          {
            StartCalCenter();
          }
          else
          {
            StartCalEdge();
          }
          break;

        case BTN_BACK:
          ExitToView();
          break;

        case BTN_LUP:
          StepBacklight(1);
          break;

        case BTN_LDN:
          StepBacklight(-1);
          break;

        default:
          break;
      }
      break;

    case MODE_CAL_CENTER:
      if (btnIndex == BTN_OK)
      {
        SaveCalCenter();
        EnterMenu();
      }
      else if (btnIndex == BTN_BACK)
      {
        EnterMenu();
      }
      else if (btnIndex == BTN_LUP)
      {
        StepBacklight(1);
      }
      else if (btnIndex == BTN_LDN)
      {
        StepBacklight(-1);
      }
      break;

    case MODE_CAL_EDGE:
      if (btnIndex == BTN_OK)
      {
        SaveCalEdge();
        EnterMenu();
      }
      else if (btnIndex == BTN_BACK)
      {
        EnterMenu();
      }
      else if (btnIndex == BTN_LUP)
      {
        StepBacklight(1);
      }
      else if (btnIndex == BTN_LDN)
      {
        StepBacklight(-1);
      }
      break;

    default:
      break;
  }
}

static void UpdateCalibrationCandidates(void)
{
  if (!s_adcXValid || !s_adcYValid)
  {
    return;
  }

  if ((s_mode == MODE_CAL_CENTER) && s_calCenterCandidateValid)
  {
    if (s_adcX < s_calXCenterCandidate.cMin)
    {
      s_calXCenterCandidate.cMin = s_adcX;
    }
    if (s_adcX > s_calXCenterCandidate.cMax)
    {
      s_calXCenterCandidate.cMax = s_adcX;
    }
    if (s_adcY < s_calYCenterCandidate.cMin)
    {
      s_calYCenterCandidate.cMin = s_adcY;
    }
    if (s_adcY > s_calYCenterCandidate.cMax)
    {
      s_calYCenterCandidate.cMax = s_adcY;
    }
  }

  if ((s_mode == MODE_CAL_EDGE) && s_calEdgeCandidateValid)
  {
    if (s_adcX < s_calXEdgeCandidate.min)
    {
      s_calXEdgeCandidate.min = s_adcX;
    }
    if (s_adcX > s_calXEdgeCandidate.max)
    {
      s_calXEdgeCandidate.max = s_adcX;
    }
    if (s_adcY < s_calYEdgeCandidate.min)
    {
      s_calYEdgeCandidate.min = s_adcY;
    }
    if (s_adcY > s_calYEdgeCandidate.max)
    {
      s_calYEdgeCandidate.max = s_adcY;
    }
  }
}

static void HandleUsbStateChange(uint16_t oldStatus, uint16_t newStatus)
{
  const bool oldUsb = ((oldStatus & STATUS_USB_CONNECTED) != 0U);
  const bool newUsb = ((newStatus & STATUS_USB_CONNECTED) != 0U);

  if (oldUsb == newUsb)
  {
    return;
  }

  if (!newUsb)
  {
    if (s_serialActive)
    {
      Serial.end();
      s_serialActive = false;
    }

    s_usbIndicatorState = LCD_INDICATOR_OFF;
    s_usbIndicatorPending = true;
  }
  else
  {
    if (!s_serialActive)
    {
      Serial.begin(SERIAL_BAUD);
      s_serialActive = true;
      s_serialLineLen = 0;
    }

    s_usbIndicatorState = LCD_INDICATOR_ON;
    s_usbIndicatorPending = true;
  }
}

static bool ShouldPrintPacket(uint8_t index, uint16_t oldStatus, uint16_t newValue)
{
  if (index != IDX_STATUS)
  {
    return true;
  }

  return (oldStatus != newValue);
}

static void PrintState(uint8_t changedIndex)
{
  if (!s_serialActive)
  {
    return;
  }

  Serial.printf(
    "%1x %d %.3f %.3f %s\n",
    changedIndex & 0x0FU,
    (int)s_status,
    GetNormX(),
    GetNormY(),
    GetPressedButtonName()
  );
}

static bool ApplyChangedItem(uint8_t index, uint16_t value)
{
  const uint16_t oldStatus = s_status;
  const bool shouldPrint = ShouldPrintPacket(index, oldStatus, value);

  switch (index)
  {
    case IDX_STATUS:
      s_status = value;
      s_lcdSendAllowed = ((s_status & STATUS_LCD_BUSY) == 0U);
      HandleUsbStateChange(oldStatus, s_status);
      s_seenFirstStatus = true;

      if (s_initialBrightnessPending)
      {
        (void)SendBacklightBrightness(GetCurrentBrightnessForMode());
        if (IsUiActive())
        {
          (void)SendBacklightTimeout(MENU_BACKLIGHT_TIMEOUT_MS);
          s_nextBacklightRefreshMs = millis() + MENU_BACKLIGHT_REFRESH_MS;
        }
        s_initialBrightnessPending = false;
      }
      break;

    case IDX_ADC_X:
      s_adcX = value;
      s_adcXValid = true;
      UpdateCalibrationCandidates();
      UpdateMarkerTarget();
      break;

    case IDX_ADC_Y:
      s_adcY = value;
      s_adcYValid = true;
      UpdateCalibrationCandidates();
      UpdateMarkerTarget();
      break;

    default:
      if ((index >= IDX_BUTTON_0) && (index <= IDX_BUTTON_9))
      {
        const uint8_t btnIdx = (uint8_t)(index - IDX_BUTTON_0);
        const uint8_t oldPressed = s_buttons[btnIdx];
        const uint8_t newPressed = (value != 0U) ? 1U : 0U;

        if (oldPressed != newPressed)
        {
          if (IsUiActive())
          {
            TouchMenuActivity();
          }

          s_buttons[btnIdx] = newPressed;

          if (newPressed != 0U)
          {
            HandleButtonPressed(index);
          }
        }
        else
        {
          s_buttons[btnIdx] = newPressed;
        }
      }
      break;
  }

  return shouldPrint;
}

static bool TrySendMarker(void)
{
  if (!s_marker.pending)
  {
    return false;
  }

  if (s_mode == MODE_VIEW)
  {
    if (s_marker.visible)
    {
      if (LCD_FillCircle(s_marker.drawX, s_marker.drawY, JOY_R, LCD_BLACK))
      {
        s_marker.visible = false;
        s_lcdSendAllowed = false;
      }
      return true;
    }

    if (LCD_FillCircle(s_marker.targetX, s_marker.targetY, JOY_R, LCD_WHITE))
    {
      s_marker.drawX = s_marker.targetX;
      s_marker.drawY = s_marker.targetY;
      s_marker.visible = true;
      s_marker.pending = false;
      s_lcdSendAllowed = false;
    }
    return true;
  }

  if ((s_mode == MODE_CAL_CENTER) || (s_mode == MODE_CAL_EDGE))
  {
    if (LCD_FillCircle(s_marker.targetX, s_marker.targetY, JOY_R, GetMarkerColor()))
    {
      s_marker.drawX = s_marker.targetX;
      s_marker.drawY = s_marker.targetY;
      s_marker.visible = true;
      s_marker.pending = false;
      s_lcdSendAllowed = false;
    }
    return true;
  }

  s_marker.pending = false;
  return false;
}

static void TrySendPendingLcdCommands(void)
{
  if (!s_lcdSendAllowed)
  {
    return;
  }

  if (TrySendMarker())
  {
    return;
  }

  if (s_usbIndicatorPending)
  {
    if (LCD_Indicator(LCD_INDICATOR_USB, s_usbIndicatorState))
    {
      s_usbIndicatorPending = false;
      s_lcdSendAllowed = false;
    }
    return;
  }

  LcdCmd cmd;
  if (LcdQueuePeek(cmd))
  {
    if (I2C_WriteBytes(cmd.data, cmd.len))
    {
      LcdQueuePop();
      s_lcdSendAllowed = false;
    }
  }
}

static void ServiceMenuTimeout(uint32_t now)
{
  if (!IsUiActive())
  {
    return;
  }

  if ((int32_t)(now - s_menuLastActivityMs) >= (int32_t)MENU_TIMEOUT_MS)
  {
    ExitToView();
  }
}

static void ServiceMenuBacklightTimer(uint32_t now)
{
  if (!IsUiActive())
  {
    return;
  }

  if ((int32_t)(now - s_nextBacklightRefreshMs) >= 0)
  {
    (void)SendBacklightTimeout(MENU_BACKLIGHT_TIMEOUT_MS);
    s_nextBacklightRefreshMs = now + MENU_BACKLIGHT_REFRESH_MS;
  }
}

static bool ParseUInt(const char* s, long& value)
{
  if ((s == nullptr) || (*s == '\0'))
  {
    return false;
  }

  char* endPtr = nullptr;
  value = strtol(s, &endPtr, 10);
  return (endPtr != s) && (*endPtr == '\0');
}

static void HandleSerialCommand(const char* line)
{
  if ((line == nullptr) || (*line == '\0'))
  {
    return;
  }

  if ((line[0] == 'A') || (line[0] == 'B') || (line[0] == 'C'))
  {
    long value = 0;
    if (!ParseUInt(line + 1, value))
    {
      return;
    }
    if (value < 0)
    {
      value = 0;
    }
    if (value > 70)
    {
      value = 70;
    }

    uint8_t index = 0;
    if (line[0] == 'B')
    {
      index = 1;
    }
    else if (line[0] == 'C')
    {
      index = 2;
    }

    (void)QueueLcdProgressBar(index, (uint8_t)value);
    return;
  }

  if (line[0] == 'D')
  {
    long value = 0;
    if (!ParseUInt(line + 1, value))
    {
      return;
    }
    (void)QueueLcdIndicator(LCD_INDICATOR_CMD, (value != 0) ? 1U : 0U);
    return;
  }

  if (line[0] == 'T')
  {
    const char* comma = strchr(line + 1, ',');
    if (comma == nullptr)
    {
      return;
    }

    char hzText[16];
    const size_t hzLen = (size_t)(comma - (line + 1));
    if ((hzLen == 0U) || (hzLen >= sizeof(hzText)))
    {
      return;
    }

    memcpy(hzText, line + 1, hzLen);
    hzText[hzLen] = '\0';

    long hz = 0;
    long duration = 0;
    if (!ParseUInt(hzText, hz) || !ParseUInt(comma + 1, duration))
    {
      return;
    }
    if ((hz <= 0) || (duration < 0))
    {
      return;
    }

    long divider = 1000000L / hz;
    if (divider < 1L)
    {
      divider = 1L;
    }
    if (divider > 65535L)
    {
      divider = 65535L;
    }
    if (duration > 65535L)
    {
      duration = 65535L;
    }

    (void)SendTone((uint16_t)divider, (uint16_t)duration);
  }
}

static void ServiceSerialInput(void)
{
  if (!s_serialActive)
  {
    return;
  }

  while (Serial.available() > 0)
  {
    const int ch = Serial.read();
    if (ch < 0)
    {
      break;
    }

    if ((ch == '\r') || (ch == '\n'))
    {
      if (s_serialLineLen > 0U)
      {
        s_serialLine[s_serialLineLen] = '\0';
        HandleSerialCommand(s_serialLine);
        s_serialLineLen = 0U;
      }
      continue;
    }

    if ((uint8_t)ch < 32U)
    {
      continue;
    }

    if (s_serialLineLen < SERIAL_LINE_MAX)
    {
      s_serialLine[s_serialLineLen++] = (char)toupper(ch);
    }
  }
}

static void PollOnce(void)
{
  uint8_t index = 0;
  uint16_t value = 0;

  if (!I2C_ReadChangedItem(index, value))
  {
    return;
  }

  const bool shouldPrint = ApplyChangedItem(index, value);

  if (shouldPrint)
  {
    PrintState(index);
  }

  TrySendPendingLcdCommands();
}

void setup(void)
{
  s_prefs.begin("joyui", false);
  LoadSettings();

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);

  QueueViewScreen();
  s_menuLastActivityMs = millis();
  s_nextBacklightRefreshMs = millis() + MENU_BACKLIGHT_REFRESH_MS;
  s_nextPollMs = millis();
}

void loop(void)
{
  const uint32_t now = millis();

  ServiceSerialInput();
  ServiceMenuTimeout(now);
  ServiceMenuBacklightTimer(now);

  if ((int32_t)(now - s_nextPollMs) >= 0)
  {
    s_nextPollMs += POLL_PERIOD_MS;
    PollOnce();
  }
}
