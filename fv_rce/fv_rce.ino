
#include <Arduino.h>
#include <Wire.h>
#include <Preferences.h>

static constexpr uint8_t I2C_ADDR = 0x14;
static constexpr int I2C_SDA = 21;
static constexpr int I2C_SCL = 22;
static constexpr uint32_t I2C_POLL_MS = 5;
static constexpr uint8_t I2C_READ_LEN = 26;
static constexpr uint8_t I2C_ITEM_COUNT_MAX = 13;

static constexpr uint8_t APP_I2C_INDEX_STATUS       = 0;
static constexpr uint8_t APP_I2C_INDEX_ADC_X        = 1;
static constexpr uint8_t APP_I2C_INDEX_ADC_Y        = 2;
static constexpr uint8_t APP_I2C_INDEX_BUTTON_0     = 3;
static constexpr uint8_t APP_I2C_INDEX_BUTTON_UP    = 5;
static constexpr uint8_t APP_I2C_INDEX_BUTTON_DOWN  = 6;
static constexpr uint8_t APP_I2C_INDEX_BUTTON_BACK  = 7;
static constexpr uint8_t APP_I2C_INDEX_BUTTON_OK    = 8;
static constexpr uint8_t APP_I2C_INDEX_BUTTON_LUP   = 9;
static constexpr uint8_t APP_I2C_INDEX_BUTTON_LDN   = 10;

static constexpr uint8_t STATUS_BIT_LCD_BUSY      = 0;
static constexpr uint8_t STATUS_BIT_USB_CONNECTED = 1;
static constexpr uint8_t STATUS_BIT_BACKLIGHT_ON  = 3;

static constexpr uint16_t COLOR_BLACK = 0x0000;
static constexpr uint16_t COLOR_WHITE = 0xFFFF;
static constexpr uint16_t COLOR_GREEN = 0x07E0;
static constexpr uint16_t COLOR_BLUE  = 0x001F;

static constexpr int JOY_AREA_X0 = 40;
static constexpr int JOY_AREA_Y0 = 10;
static constexpr int JOY_AREA_X1 = 119;
static constexpr int JOY_AREA_Y1 = 79;

static constexpr uint8_t CMD_BACKLIGHT_TIMEOUT    = 0x03;
static constexpr uint8_t CMD_BACKLIGHT_BRIGHTNESS = 0x04;
static constexpr uint8_t CMD_TONE                 = 0x07;
static constexpr uint8_t CMD_LCD_CLEAR            = 0x10;
static constexpr uint8_t CMD_LCD_DRAW_MARKER      = 0x12;
static constexpr uint8_t CMD_LCD_DRAW_TEXT        = 0x13;
static constexpr uint8_t CMD_LCD_SET_BG           = 0x14;
static constexpr uint8_t CMD_LCD_INDICATOR        = 0x20;
static constexpr uint8_t CMD_LCD_PROGRESS         = 0x21;

static constexpr uint8_t PRIO_LOW  = 0;
static constexpr uint8_t PRIO_MED  = 1;
static constexpr uint8_t PRIO_HIGH = 2;

static constexpr uint32_t MENU_INACTIVITY_MS = 60000UL;
static constexpr uint32_t MENU_TIMEOUT_REFRESH_MS = 10000UL;
static constexpr uint32_t MENU_TIMEOUT_VALUE_MS = 15000UL;

static constexpr size_t CMD_MAX_LEN = 32;
static constexpr size_t CMD_QUEUE_CAPACITY = 56;

struct CmdItem
{
  bool used = false;
  uint8_t data[CMD_MAX_LEN]{};
  uint8_t len = 0;
  uint8_t prio = PRIO_MED;
  bool isLcd = false;
  bool requiresLcdReady = false;
  uint32_t seq = 0;
};

struct MarkerState
{
  bool visible = false;
  uint8_t x = 0;
  uint8_t y = 0;
};

struct AxisCal
{
  uint16_t min = 0;
  uint16_t cMin = 1955;
  uint16_t cMax = 1955;
  uint16_t max = 4095;
};

enum UiMode : uint8_t
{
  UI_NORMAL = 0,
  UI_MENU,
  UI_CAL_CENTER,
  UI_CAL_EDGE,
};

enum MenuItem : uint8_t
{
  MENU_CAL_CENTER = 0,
  MENU_CAL_EDGE = 1,
  MENU_ITEM_COUNT = 2,
};

static Preferences s_prefs;
static bool s_prefsOpened = false;
static bool s_initialBrightnessQueued = false;

static CmdItem s_cmdQueue[CMD_QUEUE_CAPACITY];
static uint32_t s_cmdSeq = 1;

static uint32_t s_nextPollMs = 0;
static uint32_t s_lastMenuActivityMs = 0;
static uint32_t s_lastMenuTimeoutRefreshMs = 0;
static String s_serialLine;

static uint8_t s_status = 0;
static uint8_t s_lastLen1StatusPrinted = 0xFF;
static bool s_usbKnown = false;
static bool s_usbConnected = false;
static bool s_backlightOn = false;
static bool s_lcdSendAllowed = false;
static uint32_t s_rxSeq = 0;
static uint32_t s_lastTxRxSeq = 0;

static uint16_t s_rawX = 2048;
static uint16_t s_rawY = 2048;
static bool s_haveRawX = false;
static bool s_haveRawY = false;
static bool s_buttons[13] = { false };

static MarkerState s_markerCurrent;
static int s_markerTargetX = -1;
static int s_markerTargetY = -1;
static bool s_markerTargetVisible = false;
static bool s_skipMarkerUpdateOnce = false;
static UiMode s_uiMode = UI_NORMAL;
static uint8_t s_menuIndex = MENU_CAL_CENTER;
static uint8_t s_brightnessStep = 7;

static AxisCal s_calX{177, 1955, 1956, 4025};
static AxisCal s_calY{0, 1958, 1959, 4027};
static AxisCal s_tmpCalX;
static AxisCal s_tmpCalY;
static int s_calMarkerLastX = -1;
static int s_calMarkerLastY = -1;
static uint32_t s_calStaticSeqBarrier = 0;

static const char* kButtonNames[13] = {
  "", "", "", "ON", "Fire", "UP", "DOWN", "BACK", "OK", "LUP", "LDN", "RUP", "RDN"
};

static const uint8_t kBrightnessLevels[11] = { 1, 2, 3, 5, 8, 13, 20, 32, 50, 79, 127 };

static bool SerialEnabled()
{
  return s_usbKnown && s_usbConnected;
}

static uint8_t BrightnessLevel()
{
  return kBrightnessLevels[s_brightnessStep];
}

static uint8_t DimmedBrightnessLevel()
{
  const uint8_t full = BrightnessLevel();
  return (uint8_t)max(1, (int)full / 2);
}

static void OpenPrefs()
{
  if (!s_prefsOpened)
  {
    s_prefs.begin("ui", false);
    s_prefsOpened = true;
  }
}

static void LoadPrefs()
{
  OpenPrefs();

  s_brightnessStep = s_prefs.getUChar("bl_step", 7);
  if (s_brightnessStep > 10)
  {
    s_brightnessStep = 10;
  }

  s_calX.min  = s_prefs.getUShort("x_min", 177);
  s_calX.cMin = s_prefs.getUShort("x_cmin", 1955);
  s_calX.cMax = s_prefs.getUShort("x_cmax", 1956);
  s_calX.max  = s_prefs.getUShort("x_max", 4025);

  s_calY.min  = s_prefs.getUShort("y_min", 0);
  s_calY.cMin = s_prefs.getUShort("y_cmin", 1958);
  s_calY.cMax = s_prefs.getUShort("y_cmax", 1959);
  s_calY.max  = s_prefs.getUShort("y_max", 4027);
}

static void SaveBrightnessPrefs()
{
  OpenPrefs();
  s_prefs.putUChar("bl_step", s_brightnessStep);
}

static void SaveCalibrationPrefs()
{
  OpenPrefs();
  s_prefs.putUShort("x_min",  s_calX.min);
  s_prefs.putUShort("x_cmin", s_calX.cMin);
  s_prefs.putUShort("x_cmax", s_calX.cMax);
  s_prefs.putUShort("x_max",  s_calX.max);
  s_prefs.putUShort("y_min",  s_calY.min);
  s_prefs.putUShort("y_cmin", s_calY.cMin);
  s_prefs.putUShort("y_cmax", s_calY.cMax);
  s_prefs.putUShort("y_max",  s_calY.max);
}

static float MapAxisRawToNorm(uint16_t raw, const AxisCal& cal)
{
  if (raw <= cal.cMin)
  {
    const int32_t denom = (int32_t)cal.cMin - (int32_t)cal.min;
    if (denom <= 0)
    {
      return -1.0f;
    }
    float t = (float)((int32_t)raw - (int32_t)cal.min) / (float)denom;
    if (t < 0.0f) t = 0.0f;
    if (t > 1.0f) t = 1.0f;
    return t - 1.0f;
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
  float t = (float)((int32_t)raw - (int32_t)cal.cMax) / (float)denom;
  if (t < 0.0f) t = 0.0f;
  if (t > 1.0f) t = 1.0f;
  return t;
}

static float GetNormX()
{
  return MapAxisRawToNorm(s_rawX, s_calX);
}

static float GetNormY()
{
  return -MapAxisRawToNorm(s_rawY, s_calY);
}

static int NormToPixel(float v, int lo, int hi)
{
  if (v < -1.0f) v = -1.0f;
  if (v >  1.0f) v =  1.0f;
  const float t = (v + 1.0f) * 0.5f;
  return lo + (int)lroundf(t * (float)(hi - lo));
}

static int MapRawWindowToPixel(uint16_t raw, int center, int width, int px0, int px1)
{
  int half = width / 2;
  if (half < 1)
  {
    half = 1;
  }
  const int left = center - half;
  const int right = center + half;
  if ((int)raw <= left)
  {
    return px0;
  }
  if ((int)raw >= right)
  {
    return px1;
  }
  float t = (float)((int)raw - left) / (float)(right - left);
  return px0 + (int)lroundf(t * (float)(px1 - px0));
}

static int MapRawFullToPixel(uint16_t raw, int px0, int px1)
{
  float t = (float)raw / 4095.0f;
  if (t < 0.0f) t = 0.0f;
  if (t > 1.0f) t = 1.0f;
  return px0 + (int)lroundf(t * (float)(px1 - px0));
}

static const char* FirstPressedButtonName()
{
  for (uint8_t i = APP_I2C_INDEX_BUTTON_0; i < 13; ++i)
  {
    if (s_buttons[i])
    {
      return kButtonNames[i];
    }
  }
  return "";
}

static bool QueueCommand(const uint8_t* data, uint8_t len, uint8_t prio, bool requiresLcdReady, bool isLcd)
{
  if (data == nullptr || len == 0 || len > CMD_MAX_LEN)
  {
    return false;
  }

  for (size_t i = 0; i < CMD_QUEUE_CAPACITY; ++i)
  {
    if (!s_cmdQueue[i].used)
    {
      s_cmdQueue[i].used = true;
      memcpy(s_cmdQueue[i].data, data, len);
      s_cmdQueue[i].len = len;
      s_cmdQueue[i].prio = prio;
      s_cmdQueue[i].requiresLcdReady = requiresLcdReady;
      s_cmdQueue[i].isLcd = isLcd;
      s_cmdQueue[i].seq = s_cmdSeq++;
      return true;
    }
  }
  return false;
}

static bool QueueBacklightTimeout(uint32_t timeoutMs)
{
  uint8_t data[5];
  data[0] = CMD_BACKLIGHT_TIMEOUT;
  data[1] = (uint8_t)(timeoutMs & 0xFF);
  data[2] = (uint8_t)((timeoutMs >> 8) & 0xFF);
  data[3] = (uint8_t)((timeoutMs >> 16) & 0xFF);
  data[4] = (uint8_t)((timeoutMs >> 24) & 0xFF);
  return QueueCommand(data, sizeof(data), PRIO_MED, false, false);
}

static bool QueueBacklightBrightness(uint8_t level)
{
  const uint8_t data[2] = { CMD_BACKLIGHT_BRIGHTNESS, level };
  return QueueCommand(data, sizeof(data), PRIO_MED, false, false);
}

static bool QueueTone(uint16_t divider, uint16_t delayMs)
{
  uint8_t data[5];
  data[0] = CMD_TONE;
  data[1] = (uint8_t)(divider & 0xFF);
  data[2] = (uint8_t)(divider >> 8);
  data[3] = (uint8_t)(delayMs & 0xFF);
  data[4] = (uint8_t)(delayMs >> 8);
  return QueueCommand(data, sizeof(data), PRIO_HIGH, false, false);
}

static bool QueueLcdIndicator(uint8_t index, uint8_t state)
{
  const uint8_t data[3] = { CMD_LCD_INDICATOR, index, (uint8_t)(state ? 1 : 0) };
  return QueueCommand(data, sizeof(data), PRIO_LOW, true, true);
}

static bool QueueLcdProgressBar(uint8_t index, uint8_t value)
{
  const uint8_t data[3] = { CMD_LCD_PROGRESS, index, value };
  return QueueCommand(data, sizeof(data), PRIO_LOW, true, true);
}

static bool QueueLcdClear(uint16_t color)
{
  uint8_t data[3];
  data[0] = CMD_LCD_CLEAR;
  data[1] = (uint8_t)(color & 0xFF);
  data[2] = (uint8_t)(color >> 8);
  return QueueCommand(data, sizeof(data), PRIO_MED, true, true);
}

static bool QueueLcdSetBackgroundColor(uint16_t color)
{
  uint8_t data[3];
  data[0] = CMD_LCD_SET_BG;
  data[1] = (uint8_t)(color & 0xFF);
  data[2] = (uint8_t)(color >> 8);
  return QueueCommand(data, sizeof(data), PRIO_MED, true, true);
}

static bool QueueLcdDrawMarker(uint8_t x, uint8_t y, uint8_t idx, uint16_t color)
{
  uint8_t data[6];
  data[0] = CMD_LCD_DRAW_MARKER;
  data[1] = x;
  data[2] = y;
  data[3] = idx;
  data[4] = (uint8_t)(color & 0xFF);
  data[5] = (uint8_t)(color >> 8);
  return QueueCommand(data, sizeof(data), PRIO_HIGH, true, true);
}

static bool QueueLcdDrawText(uint8_t x, uint8_t y, const char* text)
{
  if (text == nullptr)
  {
    return false;
  }
  const size_t textLen = strlen(text);
  if (textLen > 21)
  {
    return false;
  }

  uint8_t data[24] = { 0 };
  data[0] = CMD_LCD_DRAW_TEXT;
  data[1] = x;
  data[2] = y;
  memcpy(&data[3], text, textLen);
  data[3 + textLen] = 0;
  return QueueCommand(data, (uint8_t)(4 + textLen), PRIO_MED, true, true);
}

static void LogTx(const uint8_t* data, uint8_t len)
{
#ifdef DEBUG_I2C
  if (!SerialEnabled())
  {
    return;
  }

  String line = "TX";
  char tmp[8];
  for (uint8_t i = 0; i < len; ++i)
  {
    snprintf(tmp, sizeof(tmp), " %02X", data[i]);
    line += tmp;
  }
  Serial.println(line);
#else
  (void)data;
  (void)len;
#endif
}

static bool QueueHasPendingBefore(uint32_t barrierSeq)
{
  if (barrierSeq == 0U)
  {
    return false;
  }

  for (size_t i = 0; i < CMD_QUEUE_CAPACITY; ++i)
  {
    if (s_cmdQueue[i].used && s_cmdQueue[i].seq < barrierSeq)
    {
      return true;
    }
  }
  return false;
}

static int FindBestQueuedCommand()
{
  int bestIndex = -1;
  uint8_t bestPrio = 0;
  uint32_t bestSeq = 0xFFFFFFFFu;

  for (size_t i = 0; i < CMD_QUEUE_CAPACITY; ++i)
  {
    const CmdItem& item = s_cmdQueue[i];
    if (!item.used)
    {
      continue;
    }
    if (item.requiresLcdReady && !s_lcdSendAllowed)
    {
      continue;
    }
    if (bestIndex < 0 || item.prio > bestPrio || (item.prio == bestPrio && item.seq < bestSeq))
    {
      bestIndex = (int)i;
      bestPrio = item.prio;
      bestSeq = item.seq;
    }
  }

  return bestIndex;
}

static bool SendOneQueuedCommand()
{
  const int index = FindBestQueuedCommand();
  if (index < 0)
  {
    return false;
  }

  const CmdItem item = s_cmdQueue[index];
  Wire.beginTransmission(I2C_ADDR);
  const size_t written = Wire.write(item.data, item.len);
  const uint8_t rc = Wire.endTransmission(true);
  if (rc != 0 || written != item.len)
  {
    return false;
  }

  if (item.isLcd)
  {
    s_lcdSendAllowed = false;
  }

  LogTx(item.data, item.len);
  s_cmdQueue[index].used = false;
  return true;
}

static void PrintPacketLine(uint8_t itemCount)
{
#ifdef DEBUG_I2C
  if (!SerialEnabled())
  {
    return;
  }

  if (itemCount == 1U)
  {
    if (s_status == s_lastLen1StatusPrinted)
    {
      return;
    }
    s_lastLen1StatusPrinted = s_status;
  }

  char line[96];
  snprintf(
    line,
    sizeof(line),
    "%2u %02x %.3f %.3f %s",
    itemCount,
    s_status,
    GetNormX(),
    GetNormY(),
    FirstPressedButtonName()
  );
  Serial.println(line);
#else
  (void)itemCount;
#endif
}

static void PrintCalibrationState(const char* tag)
{
  if (!SerialEnabled() || tag == nullptr)
  {
    return;
  }
  char line[160];
  snprintf(line, sizeof(line), "%s X[min=%u cMin=%u cMax=%u max=%u] Y[min=%u cMin=%u cMax=%u max=%u]",
           tag,
           s_calX.min, s_calX.cMin, s_calX.cMax, s_calX.max,
           s_calY.min, s_calY.cMin, s_calY.cMax, s_calY.max);
  Serial.println(line);
}

static void HandleUsbState(bool usbConnected)
{
  if (!s_usbKnown)
  {
    s_usbKnown = true;
    s_usbConnected = usbConnected;
    if (usbConnected)
    {
      Serial.begin(115200);
      delay(20);
      QueueLcdIndicator(1, 1);
    }
    return;
  }

  if (s_usbConnected == usbConnected)
  {
    return;
  }

  if (!usbConnected)
  {
    Serial.end();
    s_usbConnected = false;
    QueueLcdIndicator(1, 0);
  }
  else
  {
    s_usbConnected = true;
    Serial.begin(115200);
    delay(20);
    QueueLcdIndicator(1, 1);
  }
}

static uint8_t MenuItemY(uint8_t idx)
{
  return (idx == MENU_CAL_CENTER) ? 28U : 42U;
}

static const char* MenuItemText(uint8_t idx, bool selected)
{
  if (idx == MENU_CAL_CENTER)
  {
    return selected ? "> Cal center" : "  Cal center";
  }
  return selected ? "> Cal edge" : "  Cal edge";
}

static void QueueBrightnessField()
{
  char line[22];
  snprintf(line, sizeof(line), "Brightness %3u   ", BrightnessLevel());
  QueueLcdDrawText(10, 56, line);
}

static void QueueMenuFrame()
{
  QueueLcdSetBackgroundColor(COLOR_BLACK);
  QueueLcdClear(COLOR_BLACK);
  QueueLcdDrawText(10, 10, "MENU");
  QueueLcdDrawText(10, MenuItemY(MENU_CAL_CENTER), MenuItemText(MENU_CAL_CENTER, s_menuIndex == MENU_CAL_CENTER));
  QueueLcdDrawText(10, MenuItemY(MENU_CAL_EDGE), MenuItemText(MENU_CAL_EDGE, s_menuIndex == MENU_CAL_EDGE));
  QueueBrightnessField();
}

static void QueueMenuSelectionUpdate(uint8_t oldIndex, uint8_t newIndex)
{
  if (oldIndex == newIndex)
  {
    return;
  }
  QueueLcdDrawText(10, MenuItemY(oldIndex), MenuItemText(oldIndex, false));
  QueueLcdDrawText(10, MenuItemY(newIndex), MenuItemText(newIndex, true));
}

static void QueueCalCenterStatic()
{
  QueueLcdSetBackgroundColor(COLOR_BLACK);
  QueueLcdClear(COLOR_BLACK);
  QueueLcdDrawText(10, 10, "c\na\nn\nc\ne\nl");
  QueueLcdDrawText(142, 10, "a\nc\nc\ne\np\nt");
}

static void QueueCalEdgeStatic()
{
  QueueLcdSetBackgroundColor(COLOR_BLACK);
  QueueLcdClear(COLOR_BLACK);
  QueueLcdDrawText(10, 10, "c\na\nn\nc\ne\nl");
  QueueLcdDrawText(142, 10, "a\nc\nc\ne\np\nt");
}

static void ResetCalUiState()
{
  s_calMarkerLastX = -1;
  s_calMarkerLastY = -1;
  s_calStaticSeqBarrier = 0;
}

static void QueueMenuModeBrightness()
{
  QueueBacklightBrightness(BrightnessLevel());
}

static void QueueNormalModeBrightness()
{
  QueueBacklightBrightness(DimmedBrightnessLevel());
}

static void EnterMenu()
{
  if (s_uiMode == UI_MENU)
  {
    return;
  }

  s_uiMode = UI_MENU;
  s_menuIndex = MENU_CAL_CENTER;
  s_lastMenuActivityMs = millis();
  s_lastMenuTimeoutRefreshMs = 0;


  QueueMenuModeBrightness();

  s_markerTargetVisible = false;
  if (s_markerCurrent.visible)
  {
    if (QueueLcdDrawMarker(s_markerCurrent.x, s_markerCurrent.y, 3, COLOR_BLACK))
    {
      s_markerCurrent.visible = false;
    }
  }
  QueueMenuFrame();
}

static void ExitMenu()
{
  if (s_uiMode == UI_NORMAL)
  {
    return;
  }

  s_uiMode = UI_NORMAL;
  s_skipMarkerUpdateOnce = true;
  s_markerCurrent.visible = false;

  QueueNormalModeBrightness();
  QueueLcdSetBackgroundColor(COLOR_BLACK);
  QueueLcdClear(COLOR_BLACK);
}

static void EnterCalCenter()
{
  s_uiMode = UI_CAL_CENTER;
  s_tmpCalX = s_calX;
  s_tmpCalY = s_calY;
  s_tmpCalX.cMin = s_rawX;
  s_tmpCalX.cMax = s_rawX;
  s_tmpCalY.cMin = s_rawY;
  s_tmpCalY.cMax = s_rawY;
  ResetCalUiState();
  s_lastMenuActivityMs = millis();
  QueueCalCenterStatic();
  s_calStaticSeqBarrier = s_cmdSeq;
}

static void EnterCalEdge()
{
  s_uiMode = UI_CAL_EDGE;
  s_tmpCalX = s_calX;
  s_tmpCalY = s_calY;
  s_tmpCalX.min = s_rawX;
  s_tmpCalX.max = s_rawX;
  s_tmpCalY.min = s_rawY;
  s_tmpCalY.max = s_rawY;
  ResetCalUiState();
  s_lastMenuActivityMs = millis();
  QueueCalEdgeStatic();
  s_calStaticSeqBarrier = s_cmdSeq;
}

static void ReturnToMenu()
{
  s_uiMode = UI_MENU;
  ResetCalUiState();
  s_lastMenuActivityMs = millis();
  QueueMenuModeBrightness();
  QueueMenuFrame();
}

static void SaveCalCenterAndReturn()
{
  if (s_tmpCalX.cMin > s_tmpCalX.cMax)
  {
    const uint16_t t = s_tmpCalX.cMin; s_tmpCalX.cMin = s_tmpCalX.cMax; s_tmpCalX.cMax = t;
  }
  if (s_tmpCalY.cMin > s_tmpCalY.cMax)
  {
    const uint16_t t = s_tmpCalY.cMin; s_tmpCalY.cMin = s_tmpCalY.cMax; s_tmpCalY.cMax = t;
  }

  s_calX.cMin = s_tmpCalX.cMin;
  s_calX.cMax = s_tmpCalX.cMax;
  s_calY.cMin = s_tmpCalY.cMin;
  s_calY.cMax = s_tmpCalY.cMax;
  SaveCalibrationPrefs();
  PrintCalibrationState("CAL center saved");
  ReturnToMenu();
}

static void SaveCalEdgeAndReturn()
{
  if (s_tmpCalX.min > s_tmpCalX.max)
  {
    const uint16_t t = s_tmpCalX.min; s_tmpCalX.min = s_tmpCalX.max; s_tmpCalX.max = t;
  }
  if (s_tmpCalY.min > s_tmpCalY.max)
  {
    const uint16_t t = s_tmpCalY.min; s_tmpCalY.min = s_tmpCalY.max; s_tmpCalY.max = t;
  }

  s_calX.min = s_tmpCalX.min;
  s_calX.max = s_tmpCalX.max;
  s_calY.min = s_tmpCalY.min;
  s_calY.max = s_tmpCalY.max;
  SaveCalibrationPrefs();
  PrintCalibrationState("CAL edge saved");
  ReturnToMenu();
}

static void AdjustBrightness(int delta)
{
  int next = (int)s_brightnessStep + delta;
  if (next < 0)
  {
    next = 0;
  }
  if (next > 10)
  {
    next = 10;
  }
  if ((uint8_t)next == s_brightnessStep)
  {
    return;
  }

  s_brightnessStep = (uint8_t)next;
  SaveBrightnessPrefs();
  QueueBacklightBrightness(BrightnessLevel());

  if (s_uiMode == UI_MENU)
  {
    QueueBrightnessField();
  }
}

static void UpdateMarkerTargetFromAdc()
{
  if (s_uiMode != UI_NORMAL)
  {
    s_markerTargetVisible = false;
    return;
  }

  if (!s_haveRawX || !s_haveRawY)
  {
    s_markerTargetVisible = false;
    return;
  }

  if (!s_backlightOn)
  {
    s_markerTargetVisible = false;
    return;
  }

  const int x = NormToPixel(GetNormX(), JOY_AREA_X0, JOY_AREA_X1);
  const int y = NormToPixel(-GetNormY(), JOY_AREA_Y0, JOY_AREA_Y1);

  s_markerTargetVisible = true;
  s_markerTargetX = x;
  s_markerTargetY = y;
}

static bool QueueMarkerUpdateIfNeeded()
{
  if (s_skipMarkerUpdateOnce)
  {
    s_skipMarkerUpdateOnce = false;
    return false;
  }

  UpdateMarkerTargetFromAdc();

  if (!s_markerTargetVisible)
  {
    if (s_markerCurrent.visible)
    {
      if (QueueLcdDrawMarker(s_markerCurrent.x, s_markerCurrent.y, 3, COLOR_BLACK))
      {
        s_markerCurrent.visible = false;
        return true;
      }
    }
    return false;
  }

  if (s_markerCurrent.visible && s_markerCurrent.x == (uint8_t)s_markerTargetX && s_markerCurrent.y == (uint8_t)s_markerTargetY)
  {
    return false;
  }

  if (s_markerCurrent.visible)
  {
    if (!QueueLcdDrawMarker(s_markerCurrent.x, s_markerCurrent.y, 3, COLOR_BLACK))
    {
      return false;
    }
  }

  if (!QueueLcdDrawMarker((uint8_t)s_markerTargetX, (uint8_t)s_markerTargetY, 3, COLOR_WHITE))
  {
    return false;
  }

  s_markerCurrent.visible = true;
  s_markerCurrent.x = (uint8_t)s_markerTargetX;
  s_markerCurrent.y = (uint8_t)s_markerTargetY;
  return true;
}

static void UpdateCalibrationRuntime()
{
  if (!s_haveRawX || !s_haveRawY || !s_backlightOn)
  {
    return;
  }
  if ((s_uiMode == UI_CAL_CENTER || s_uiMode == UI_CAL_EDGE) && QueueHasPendingBefore(s_calStaticSeqBarrier))
  {
    return;
  }

  int px = -1;
  int py = -1;

  if (s_uiMode == UI_CAL_CENTER)
  {
    if (s_rawX < s_tmpCalX.cMin) s_tmpCalX.cMin = s_rawX;
    if (s_rawX > s_tmpCalX.cMax) s_tmpCalX.cMax = s_rawX;
    if (s_rawY < s_tmpCalY.cMin) s_tmpCalY.cMin = s_rawY;
    if (s_rawY > s_tmpCalY.cMax) s_tmpCalY.cMax = s_rawY;

    const int widthX = (int)s_calX.cMax - (int)s_calX.cMin;
    const int widthY = (int)s_calY.cMax - (int)s_calY.cMin;
    const int w = 2 * max(max(widthX, widthY), 16);
    const int cx = ((int)s_calX.cMin + (int)s_calX.cMax) / 2;
    const int cy = ((int)s_calY.cMin + (int)s_calY.cMax) / 2;

    px = MapRawWindowToPixel(s_rawX, cx, w, JOY_AREA_X0, JOY_AREA_X1);
    py = MapRawWindowToPixel(s_rawY, cy, w, JOY_AREA_Y0, JOY_AREA_Y1);

    if (px != s_calMarkerLastX || py != s_calMarkerLastY)
    {
      QueueLcdDrawMarker((uint8_t)px, (uint8_t)py, 5, COLOR_GREEN);
      s_calMarkerLastX = px;
      s_calMarkerLastY = py;
    }
  }
  else if (s_uiMode == UI_CAL_EDGE)
  {
    if (s_rawX < s_tmpCalX.min) s_tmpCalX.min = s_rawX;
    if (s_rawX > s_tmpCalX.max) s_tmpCalX.max = s_rawX;
    if (s_rawY < s_tmpCalY.min) s_tmpCalY.min = s_rawY;
    if (s_rawY > s_tmpCalY.max) s_tmpCalY.max = s_rawY;

    px = MapRawFullToPixel(s_rawX, JOY_AREA_X0, JOY_AREA_X1);
    py = MapRawFullToPixel(s_rawY, JOY_AREA_Y0, JOY_AREA_Y1);

    if (px != s_calMarkerLastX || py != s_calMarkerLastY)
    {
      QueueLcdDrawMarker((uint8_t)px, (uint8_t)py, 3, COLOR_BLUE);
      s_calMarkerLastX = px;
      s_calMarkerLastY = py;
    }
  }
}

static void TouchMenuActivity(const bool* oldButtons)
{
  if (s_uiMode == UI_NORMAL)
  {
    return;
  }

  for (uint8_t i = APP_I2C_INDEX_BUTTON_0; i < 13; ++i)
  {
    if (oldButtons[i] != s_buttons[i])
    {
      s_lastMenuActivityMs = millis();
      break;
    }
  }
}

static void HandleButtonEdges(const bool* oldButtons)
{
  const bool okRising   = (!oldButtons[APP_I2C_INDEX_BUTTON_OK]   && s_buttons[APP_I2C_INDEX_BUTTON_OK]);
  const bool backRising = (!oldButtons[APP_I2C_INDEX_BUTTON_BACK] && s_buttons[APP_I2C_INDEX_BUTTON_BACK]);
  const bool upRising   = (!oldButtons[APP_I2C_INDEX_BUTTON_UP]   && s_buttons[APP_I2C_INDEX_BUTTON_UP]);
  const bool downRising = (!oldButtons[APP_I2C_INDEX_BUTTON_DOWN] && s_buttons[APP_I2C_INDEX_BUTTON_DOWN]);
  const bool lupRising  = (!oldButtons[APP_I2C_INDEX_BUTTON_LUP]  && s_buttons[APP_I2C_INDEX_BUTTON_LUP]);
  const bool ldnRising  = (!oldButtons[APP_I2C_INDEX_BUTTON_LDN]  && s_buttons[APP_I2C_INDEX_BUTTON_LDN]);

  if (s_uiMode != UI_NORMAL)
  {
    if (lupRising)
    {
      AdjustBrightness(+1);
    }
    if (ldnRising)
    {
      AdjustBrightness(-1);
    }
  }

  switch (s_uiMode)
  {
    case UI_NORMAL:
      if (okRising)
      {
        EnterMenu();
      }
      break;

    case UI_MENU:
      if (backRising)
      {
        ExitMenu();
        break;
      }
      if (upRising)
      {
        const uint8_t oldIndex = s_menuIndex;
        s_menuIndex = (s_menuIndex == 0U) ? (MENU_ITEM_COUNT - 1U) : (uint8_t)(s_menuIndex - 1U);
        QueueMenuSelectionUpdate(oldIndex, s_menuIndex);
      }
      if (downRising)
      {
        const uint8_t oldIndex = s_menuIndex;
        s_menuIndex = (uint8_t)((s_menuIndex + 1U) % MENU_ITEM_COUNT);
        QueueMenuSelectionUpdate(oldIndex, s_menuIndex);
      }
      if (okRising)
      {
        if (s_menuIndex == MENU_CAL_CENTER)
        {
          EnterCalCenter();
        }
        else
        {
          EnterCalEdge();
        }
      }
      break;

    case UI_CAL_CENTER:
      if (backRising)
      {
        ReturnToMenu();
      }
      else if (okRising)
      {
        SaveCalCenterAndReturn();
      }
      break;

    case UI_CAL_EDGE:
      if (backRising)
      {
        ReturnToMenu();
      }
      else if (okRising)
      {
        SaveCalEdgeAndReturn();
      }
      break;

    default:
      break;
  }
}

static void HandleMenuTimers()
{
  if (s_uiMode == UI_NORMAL)
  {
    return;
  }

  const uint32_t now = millis();
  if ((uint32_t)(now - s_lastMenuActivityMs) >= MENU_INACTIVITY_MS)
  {
    ExitMenu();
    return;
  }

  if ((uint32_t)(now - s_lastMenuTimeoutRefreshMs) >= MENU_TIMEOUT_REFRESH_MS)
  {
    QueueBacklightTimeout(MENU_TIMEOUT_VALUE_MS);
    s_lastMenuTimeoutRefreshMs = now;
  }
}

static void HandleSerialCommand(const String& lineIn)
{
  String line = lineIn;
  line.trim();
  if (line.length() < 2)
  {
    return;
  }

  if (line[0] == 'A' || line[0] == 'B' || line[0] == 'C')
  {
    const int value = line.substring(1).toInt();
    const uint8_t index = (line[0] == 'A') ? 0U : (line[0] == 'B' ? 1U : 2U);
    uint8_t out = (uint8_t)value;
    if (out > 70U) out = 70U;
    QueueLcdProgressBar(index, out);
    return;
  }

  if (line[0] == 'D')
  {
    const int value = line.substring(1).toInt();
    QueueLcdIndicator(0, value ? 1U : 0U);
    return;
  }

  if (line[0] == 'T')
  {
    const int comma = line.indexOf(',');
    if (comma <= 1)
    {
      return;
    }
    const long hz = line.substring(1, comma).toInt();
    const long ms = line.substring(comma + 1).toInt();
    if (hz <= 0 || ms < 0)
    {
      return;
    }
    const uint32_t divider32 = 1000000UL / (uint32_t)hz;
    const uint16_t divider = (divider32 > 0xFFFFu) ? 0xFFFFu : (uint16_t)divider32;
    const uint16_t delayMs = (ms > 0xFFFFL) ? 0xFFFFu : (uint16_t)ms;
    QueueTone(divider, delayMs);
  }
}

static void PumpSerialRx()
{
  if (!SerialEnabled())
  {
    return;
  }

  while (Serial.available() > 0)
  {
    const char c = (char)Serial.read();
    if (c == '\r')
    {
      continue;
    }
    if (c == '\n')
    {
      if (!s_serialLine.isEmpty())
      {
        HandleSerialCommand(s_serialLine);
        s_serialLine = "";
      }
    }
    else if (s_serialLine.length() < 96)
    {
      s_serialLine += c;
    }
  }
}

static void ParsePacket(const uint8_t* rx)
{
  const uint8_t itemCount = (uint8_t)(rx[0] & 0x0F);
  const uint8_t index0 = (uint8_t)(rx[0] >> 4);
  if (index0 != APP_I2C_INDEX_STATUS || itemCount == 0U || itemCount > I2C_ITEM_COUNT_MAX)
  {
    return;
  }

  const uint8_t expectedLen = (uint8_t)(2U + 2U * (itemCount - 1U));
  if (expectedLen > I2C_READ_LEN)
  {
    return;
  }

  bool oldButtons[13];
  memcpy(oldButtons, s_buttons, sizeof(oldButtons));

  s_status = rx[1];
  s_lcdSendAllowed = ((s_status & (1U << STATUS_BIT_LCD_BUSY)) == 0U);
  s_backlightOn = ((s_status & (1U << STATUS_BIT_BACKLIGHT_ON)) != 0U);

  if (!s_initialBrightnessQueued)
  {
    QueueNormalModeBrightness();
    s_initialBrightnessQueued = true;
  }

  for (uint8_t i = 1U; i < itemCount; ++i)
  {
    const uint8_t off = (uint8_t)(2U + 2U * (i - 1U));
    const uint8_t index = (uint8_t)(rx[off] >> 4);
    const uint16_t value = (uint16_t)(((uint16_t)(rx[off] & 0x0F) << 8) | rx[off + 1U]);

    if (index == APP_I2C_INDEX_ADC_X)
    {
      s_rawX = value;
      s_haveRawX = true;
    }
    else if (index == APP_I2C_INDEX_ADC_Y)
    {
      s_rawY = value;
      s_haveRawY = true;
    }
    else if (index >= APP_I2C_INDEX_BUTTON_0 && index < 13U)
    {
      s_buttons[index] = (value != 0U);
    }
  }

  HandleUsbState((s_status & (1U << STATUS_BIT_USB_CONNECTED)) != 0U);
  TouchMenuActivity(oldButtons);
  HandleButtonEdges(oldButtons);
  UpdateCalibrationRuntime();
  QueueMarkerUpdateIfNeeded();
  PrintPacketLine(itemCount);
  ++s_rxSeq;
}

static void PollI2C()
{
  uint8_t rx[I2C_READ_LEN];
  uint8_t len = 0;

  const int requested = Wire.requestFrom((int)I2C_ADDR, (int)I2C_READ_LEN, (int)true);
  while (Wire.available() && len < I2C_READ_LEN)
  {
    rx[len++] = (uint8_t)Wire.read();
  }

  if (requested != I2C_READ_LEN || len != I2C_READ_LEN)
  {
    return;
  }

  ParsePacket(rx);
}

static void TrySendQueuedCommandAfterFreshRx()
{
  // Do not send more than one command per freshly received status packet.
  if (s_lastTxRxSeq == s_rxSeq)
  {
    return;
  }

  if (SendOneQueuedCommand())
  {
    s_lastTxRxSeq = s_rxSeq;
  }
}

void setup()
{
  LoadPrefs();
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(100000);
  s_nextPollMs = millis();
  s_lastMenuActivityMs = s_nextPollMs;
  s_lastMenuTimeoutRefreshMs = s_nextPollMs;
}

void loop()
{
  const uint32_t now = millis();
  if ((int32_t)(now - s_nextPollMs) >= 0)
  {
    s_nextPollMs += I2C_POLL_MS;
    PollI2C();
  }

  HandleMenuTimers();
  PumpSerialRx();
  TrySendQueuedCommandAfterFreshRx();
}
