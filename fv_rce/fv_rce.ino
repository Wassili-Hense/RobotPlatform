#include <Arduino.h>
#include <Preferences.h>
#include "hmi.h"

static constexpr uint32_t HMI_POLL_MS = 5;

static constexpr uint16_t COLOR_BLACK  = 0x0000;
static constexpr uint16_t COLOR_WHITE  = 0xFFFF;
static constexpr uint16_t COLOR_GREEN  = 0x07E0;
static constexpr uint16_t COLOR_BLUE   = 0x001F;
static constexpr uint16_t COLOR_YELLOW = 0xFFE0;
static constexpr uint16_t COLOR_CYAN   = 0x07FF;

static constexpr int JOY_AREA_X0 = 45;
static constexpr int JOY_AREA_Y0 = 10;
static constexpr int JOY_AREA_X1 = 114;
static constexpr int JOY_AREA_Y1 = 79;

static constexpr uint32_t MENU_INACTIVITY_MS = 60000UL;
static constexpr uint32_t MENU_TIMEOUT_REFRESH_MS = 10000UL;
static constexpr uint32_t MENU_TIMEOUT_VALUE_MS = 15000UL;

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
  uint16_t cMax = 1956;
  uint16_t max = 4095;
};

struct ButtonEvents
{
  bool anyChange = false;
  bool okRise = false;
  bool backRise = false;
  bool upRise = false;
  bool downRise = false;
  bool lupRise = false;
  bool ldnRise = false;
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
static uint32_t s_nextPollMs = 0;
static uint32_t s_lastMenuActivityMs = 0;
static uint32_t s_lastMenuTimeoutRefreshMs = 0;
static String s_serialLine;

static bool s_usbConnected = false;
static bool s_hmiDataValid = false;
static bool s_debugSerialForced = false;

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

static const char* kButtonNames[10] = {
  "ON", "Fire", "UP", "DOWN", "BACK", "OK", "LUP", "LDN", "RUP", "RDN"
};

static const uint8_t kBrightnessLevels[11] = { 1, 2, 3, 5, 8, 13, 20, 32, 50, 79, 127 };

static bool SerialEnabled(void)
{
#ifdef DEBUG_I2C
  return s_usbConnected || s_debugSerialForced;
#else
  return s_usbConnected;
#endif
}

static void DebugEnsureSerialOnError(void)
{
#ifdef DEBUG_I2C
  if (!s_debugSerialForced)
  {
    Serial.begin(115200);
    delay(20);
    s_debugSerialForced = true;
  }
#endif
}

static void DebugLogHmiErrors(hmi_tick_result_t err)
{
#ifdef DEBUG_I2C
  if (err == HMI_TICK_OK)
  {
    return;
  }
  DebugEnsureSerialOnError();
  if (!Serial)
  {
    return;
  }
  char line[160];
  snprintf(line, sizeof(line),
           "HMI err:%04X%s%s%s%s%s",
           (unsigned)err,
           (err & HMI_TICK_ERR_NOT_INITIALIZED) ? " NOT_INIT" : "",
           (err & HMI_TICK_ERR_I2C_REQUEST) ? " I2C_REQ" : "",
           (err & HMI_TICK_ERR_I2C_READ) ? " I2C_READ" : "",
           (err & HMI_TICK_ERR_BAD_PACKET) ? " BAD_PKT" : "",
           (err & HMI_TICK_ERR_TX) ? " TX" : "");
  Serial.println(line);
#else
  (void)err;
#endif
}

static uint8_t BrightnessLevel(void)
{
  return kBrightnessLevels[s_brightnessStep];
}

static uint8_t DimmedBrightnessLevel(void)
{
  const uint8_t full = BrightnessLevel();
  return (uint8_t)max(1, (int)full / 2);
}

static bool ButtonPressed(hmi_data_idx_t idx)
{
  return hmi_get(idx) != 0U;
}

static uint16_t CurrentRawX(void)
{
  return hmi_get(HMI_DATA_JOY_X);
}

static uint16_t CurrentRawY(void)
{
  return hmi_get(HMI_DATA_JOY_Y);
}

static bool CurrentBacklightOn(void)
{
  return hmi_get(HMI_DATA_STAT_BL_ON) != 0U;
}

static bool CurrentJoyValid(void)
{
  return s_hmiDataValid;
}

static void OpenPrefs(void)
{
  if (!s_prefsOpened)
  {
    s_prefs.begin("ui", false);
    s_prefsOpened = true;
  }
}

static void LoadPrefs(void)
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

static void SaveBrightnessPrefs(void)
{
  OpenPrefs();
  s_prefs.putUChar("bl_step", s_brightnessStep);
}

static void SaveCalibrationPrefs(void)
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

static float GetNormX(void)
{
  return MapAxisRawToNorm(CurrentRawX(), s_calX);
}

static float GetNormY(void)
{
  return -MapAxisRawToNorm(CurrentRawY(), s_calY);
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
  const float t = (float)((int)raw - left) / (float)(right - left);
  return px0 + (int)lroundf(t * (float)(px1 - px0));
}

static int MapRawFullToPixel(uint16_t raw, int px0, int px1)
{
  float t = (float)raw / 4095.0f;
  if (t < 0.0f) t = 0.0f;
  if (t > 1.0f) t = 1.0f;
  return px0 + (int)lroundf(t * (float)(px1 - px0));
}

static const char* FirstPressedButtonName(void)
{
  if (ButtonPressed(HMI_DATA_BTN_ON))   return kButtonNames[0];
  if (ButtonPressed(HMI_DATA_BTN_FIRE)) return kButtonNames[1];
  if (ButtonPressed(HMI_DATA_BTN_UP))   return kButtonNames[2];
  if (ButtonPressed(HMI_DATA_BTN_DOWN)) return kButtonNames[3];
  if (ButtonPressed(HMI_DATA_BTN_BACK)) return kButtonNames[4];
  if (ButtonPressed(HMI_DATA_BTN_OK))   return kButtonNames[5];
  if (ButtonPressed(HMI_DATA_BTN_LUP))  return kButtonNames[6];
  if (ButtonPressed(HMI_DATA_BTN_LDN))  return kButtonNames[7];
  if (ButtonPressed(HMI_DATA_BTN_RUP))  return kButtonNames[8];
  if (ButtonPressed(HMI_DATA_BTN_RDN))  return kButtonNames[9];
  return "";
}

static void PrintPacketLine(void)
{
#ifdef DEBUG_I2C
  if (!SerialEnabled())
  {
    return;
  }
  char line[96];
  snprintf(line, sizeof(line), "%.3f %.3f %s", GetNormX(), GetNormY(), FirstPressedButtonName());
  Serial.println(line);
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
#ifdef DEBUG_I2C
  if (s_debugSerialForced)
  {
    s_usbConnected = usbConnected;
    hmi_cmd_lcd_set_indicator(1, usbConnected);
    return;
  }
#endif

  if (s_usbConnected == usbConnected)
  {
    return;
  }

  if (!usbConnected)
  {
    if (s_usbConnected)
    {
      Serial.end();
    }
    s_usbConnected = false;
    hmi_cmd_lcd_set_indicator(1, false);
  }
  else
  {
    s_usbConnected = true;
    Serial.begin(115200);
    delay(20);
    hmi_cmd_lcd_set_indicator(1, true);
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

static void QueueBrightnessField(void)
{
  char line[22];
  snprintf(line, sizeof(line), "Brightness %3u   ", BrightnessLevel());
  hmi_cmd_lcd_draw_text(10, 56, COLOR_CYAN, line);
}

static void QueueMenuFrame(void)
{
  hmi_cmd_lcd_clear(COLOR_BLACK);
  hmi_cmd_lcd_draw_text(10, 10, COLOR_YELLOW, "MENU");
  hmi_cmd_lcd_draw_text(10, MenuItemY(MENU_CAL_CENTER), s_menuIndex == MENU_CAL_CENTER ? COLOR_GREEN : COLOR_WHITE, MenuItemText(MENU_CAL_CENTER, s_menuIndex == MENU_CAL_CENTER));
  hmi_cmd_lcd_draw_text(10, MenuItemY(MENU_CAL_EDGE), s_menuIndex == MENU_CAL_EDGE ? COLOR_GREEN : COLOR_WHITE, MenuItemText(MENU_CAL_EDGE, s_menuIndex == MENU_CAL_EDGE));
  QueueBrightnessField();
}

static void QueueMenuSelectionUpdate(uint8_t oldIndex, uint8_t newIndex)
{
  if (oldIndex == newIndex)
  {
    return;
  }
  hmi_cmd_lcd_draw_text(10, MenuItemY(oldIndex), COLOR_WHITE, MenuItemText(oldIndex, false));
  hmi_cmd_lcd_draw_text(10, MenuItemY(newIndex), COLOR_GREEN, MenuItemText(newIndex, true));
}

static void QueueCalCenterStatic(void)
{
  hmi_cmd_lcd_clear(COLOR_BLACK);
  hmi_cmd_lcd_draw_text(10, 10, COLOR_GREEN, "c\na\nl");
  hmi_cmd_lcd_draw_text(142, 10, COLOR_CYAN, "c\ne\nn");
}

static void QueueCalEdgeStatic(void)
{
  hmi_cmd_lcd_clear(COLOR_BLACK);
  hmi_cmd_lcd_draw_text(10, 10, COLOR_GREEN, "e\nd\ng");
  hmi_cmd_lcd_draw_text(142, 10, COLOR_CYAN, "m\na\nx");
}

static void ResetCalUiState(void)
{
  s_calMarkerLastX = -1;
  s_calMarkerLastY = -1;
}

static void QueueMenuModeBrightness(void)
{
  hmi_cmd_set_brightness(BrightnessLevel());
}

static void QueueNormalModeBrightness(void)
{
  hmi_cmd_set_brightness(DimmedBrightnessLevel());
}

static void EnterMenu(void)
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
    if (hmi_cmd_lcd_draw_marker(s_markerCurrent.x, s_markerCurrent.y, 3, COLOR_BLACK))
    {
      s_markerCurrent.visible = false;
    }
  }
  QueueMenuFrame();
}

static void ExitMenu(void)
{
  if (s_uiMode == UI_NORMAL)
  {
    return;
  }
  s_uiMode = UI_NORMAL;
  s_skipMarkerUpdateOnce = true;
  s_markerCurrent.visible = false;
  QueueNormalModeBrightness();
  hmi_cmd_lcd_set_bg(COLOR_BLACK);
  hmi_cmd_lcd_clear(COLOR_BLACK);
}

static void EnterCalCenter(void)
{
  s_uiMode = UI_CAL_CENTER;
  s_tmpCalX = s_calX;
  s_tmpCalY = s_calY;
  s_tmpCalX.cMin = CurrentRawX();
  s_tmpCalX.cMax = CurrentRawX();
  s_tmpCalY.cMin = CurrentRawY();
  s_tmpCalY.cMax = CurrentRawY();
  ResetCalUiState();
  s_lastMenuActivityMs = millis();
  QueueCalCenterStatic();
}

static void EnterCalEdge(void)
{
  s_uiMode = UI_CAL_EDGE;
  s_tmpCalX = s_calX;
  s_tmpCalY = s_calY;
  s_tmpCalX.min = CurrentRawX();
  s_tmpCalX.max = CurrentRawX();
  s_tmpCalY.min = CurrentRawY();
  s_tmpCalY.max = CurrentRawY();
  ResetCalUiState();
  s_lastMenuActivityMs = millis();
  QueueCalEdgeStatic();
}

static void ReturnToMenu(void)
{
  s_uiMode = UI_MENU;
  ResetCalUiState();
  s_lastMenuActivityMs = millis();
  QueueMenuModeBrightness();
  QueueMenuFrame();
}

static void SaveCalCenterAndReturn(void)
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

static void SaveCalEdgeAndReturn(void)
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
  hmi_cmd_set_brightness(BrightnessLevel());
  if (s_uiMode == UI_MENU)
  {
    QueueBrightnessField();
  }
}

static void UpdateMarkerTargetFromAdc(void)
{
  if (s_uiMode != UI_NORMAL)
  {
    s_markerTargetVisible = false;
    return;
  }
  if (!CurrentJoyValid())
  {
    s_markerTargetVisible = false;
    return;
  }
  if (!CurrentBacklightOn())
  {
    s_markerTargetVisible = false;
    return;
  }

  s_markerTargetVisible = true;
  s_markerTargetX = NormToPixel(GetNormX(), JOY_AREA_X0, JOY_AREA_X1);
  s_markerTargetY = NormToPixel(-GetNormY(), JOY_AREA_Y0, JOY_AREA_Y1);
}

static bool QueueMarkerUpdateIfNeeded(void)
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
      if (hmi_cmd_lcd_draw_marker(s_markerCurrent.x, s_markerCurrent.y, 3, COLOR_BLACK))
      {
        s_markerCurrent.visible = false;
        return true;
      }
    }
    return false;
  }

  if (s_markerCurrent.visible &&
      s_markerCurrent.x == (uint8_t)s_markerTargetX &&
      s_markerCurrent.y == (uint8_t)s_markerTargetY)
  {
    return false;
  }

  if (s_markerCurrent.visible)
  {
    if (!hmi_cmd_lcd_draw_marker(s_markerCurrent.x, s_markerCurrent.y, 3, COLOR_BLACK))
    {
      return false;
    }
  }

  if (!hmi_cmd_lcd_draw_marker((uint8_t)s_markerTargetX, (uint8_t)s_markerTargetY, 3, COLOR_WHITE))
  {
    return false;
  }

  s_markerCurrent.visible = true;
  s_markerCurrent.x = (uint8_t)s_markerTargetX;
  s_markerCurrent.y = (uint8_t)s_markerTargetY;
  return true;
}

static void UpdateCalibrationRuntime(void)
{
  if (!CurrentJoyValid() || !CurrentBacklightOn())
  {
    return;
  }

  const uint16_t rawX = CurrentRawX();
  const uint16_t rawY = CurrentRawY();
  int px = -1;
  int py = -1;

  if (s_uiMode == UI_CAL_CENTER)
  {
    if (rawX < s_tmpCalX.cMin) s_tmpCalX.cMin = rawX;
    if (rawX > s_tmpCalX.cMax) s_tmpCalX.cMax = rawX;
    if (rawY < s_tmpCalY.cMin) s_tmpCalY.cMin = rawY;
    if (rawY > s_tmpCalY.cMax) s_tmpCalY.cMax = rawY;

    const int widthX = (int)s_calX.cMax - (int)s_calX.cMin;
    const int widthY = (int)s_calY.cMax - (int)s_calY.cMin;
    const int w = 2 * max(max(widthX, widthY), 16);
    const int cx = ((int)s_calX.cMin + (int)s_calX.cMax) / 2;
    const int cy = ((int)s_calY.cMin + (int)s_calY.cMax) / 2;
    px = MapRawWindowToPixel(rawX, cx, w, JOY_AREA_X0, JOY_AREA_X1);
    py = MapRawWindowToPixel(rawY, cy, w, JOY_AREA_Y0, JOY_AREA_Y1);
    if (px != s_calMarkerLastX || py != s_calMarkerLastY)
    {
      hmi_cmd_lcd_draw_marker((uint8_t)px, (uint8_t)py, 5, COLOR_GREEN);
      s_calMarkerLastX = px;
      s_calMarkerLastY = py;
    }
  }
  else if (s_uiMode == UI_CAL_EDGE)
  {
    if (rawX < s_tmpCalX.min) s_tmpCalX.min = rawX;
    if (rawX > s_tmpCalX.max) s_tmpCalX.max = rawX;
    if (rawY < s_tmpCalY.min) s_tmpCalY.min = rawY;
    if (rawY > s_tmpCalY.max) s_tmpCalY.max = rawY;

    px = MapRawFullToPixel(rawX, JOY_AREA_X0, JOY_AREA_X1);
    py = MapRawFullToPixel(rawY, JOY_AREA_Y0, JOY_AREA_Y1);
    if (px != s_calMarkerLastX || py != s_calMarkerLastY)
    {
      hmi_cmd_lcd_draw_marker((uint8_t)px, (uint8_t)py, 3, COLOR_BLUE);
      s_calMarkerLastX = px;
      s_calMarkerLastY = py;
    }
  }
}

static void HandleButtonEvents(const ButtonEvents& ev)
{
  if (ev.anyChange && s_uiMode != UI_NORMAL)
  {
    s_lastMenuActivityMs = millis();
  }

  if (s_uiMode != UI_NORMAL)
  {
    if (ev.lupRise)
    {
      AdjustBrightness(+1);
    }
    if (ev.ldnRise)
    {
      AdjustBrightness(-1);
    }
  }

  switch (s_uiMode)
  {
    case UI_NORMAL:
      if (ev.okRise)
      {
        EnterMenu();
      }
      break;

    case UI_MENU:
      if (ev.backRise)
      {
        ExitMenu();
        break;
      }
      if (ev.upRise)
      {
        const uint8_t oldIndex = s_menuIndex;
        s_menuIndex = (s_menuIndex == 0U) ? (MENU_ITEM_COUNT - 1U) : (uint8_t)(s_menuIndex - 1U);
        QueueMenuSelectionUpdate(oldIndex, s_menuIndex);
      }
      if (ev.downRise)
      {
        const uint8_t oldIndex = s_menuIndex;
        s_menuIndex = (uint8_t)((s_menuIndex + 1U) % MENU_ITEM_COUNT);
        QueueMenuSelectionUpdate(oldIndex, s_menuIndex);
      }
      if (ev.okRise)
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
      if (ev.backRise)
      {
        ReturnToMenu();
      }
      else if (ev.okRise)
      {
        SaveCalCenterAndReturn();
      }
      break;

    case UI_CAL_EDGE:
      if (ev.backRise)
      {
        ReturnToMenu();
      }
      else if (ev.okRise)
      {
        SaveCalEdgeAndReturn();
      }
      break;

    default:
      break;
  }
}

static void HandleMenuTimers(void)
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
    hmi_cmd_set_backlight_timeout(MENU_TIMEOUT_VALUE_MS);
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
    hmi_cmd_lcd_set_progress(index, out);
    return;
  }

  if (line[0] == 'D')
  {
    const int value = line.substring(1).toInt();
    hmi_cmd_lcd_set_indicator(0, value != 0);
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
    hmi_cmd_play_tone(divider, delayMs);
  }
}

static void PumpSerialRx(void)
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

static void ProcessHmiChanges(void)
{
  ButtonEvents ev;
  bool joyChanged = false;

  for (;;)
  {
    const hmi_data_idx_t idx = hmi_get_next_changed();
    if (idx == HMI_DATA_COUNT)
    {
      break;
    }

    switch (idx)
    {
      case HMI_DATA_STAT_USB_CONN:
        HandleUsbState(hmi_get(HMI_DATA_STAT_USB_CONN) != 0U);
        break;

      case HMI_DATA_JOY_X:
      case HMI_DATA_JOY_Y:
        joyChanged = true;
        break;

      case HMI_DATA_BTN_OK:
        ev.anyChange = true;
        ev.okRise = ButtonPressed(HMI_DATA_BTN_OK);
        break;

      case HMI_DATA_BTN_BACK:
        ev.anyChange = true;
        ev.backRise = ButtonPressed(HMI_DATA_BTN_BACK);
        break;

      case HMI_DATA_BTN_UP:
        ev.anyChange = true;
        ev.upRise = ButtonPressed(HMI_DATA_BTN_UP);
        break;

      case HMI_DATA_BTN_DOWN:
        ev.anyChange = true;
        ev.downRise = ButtonPressed(HMI_DATA_BTN_DOWN);
        break;

      case HMI_DATA_BTN_LUP:
        ev.anyChange = true;
        ev.lupRise = ButtonPressed(HMI_DATA_BTN_LUP);
        break;

      case HMI_DATA_BTN_LDN:
        ev.anyChange = true;
        ev.ldnRise = ButtonPressed(HMI_DATA_BTN_LDN);
        break;

      case HMI_DATA_BTN_ON:
      case HMI_DATA_BTN_FIRE:
      case HMI_DATA_BTN_RUP:
      case HMI_DATA_BTN_RDN:
        ev.anyChange = true;
        break;

      default:
        break;
    }
  }

  if (joyChanged)
  {
    s_hmiDataValid = true;
  }

  HandleButtonEvents(ev);
  UpdateCalibrationRuntime();
  QueueMarkerUpdateIfNeeded();
  PrintPacketLine();
}

void setup(void)
{
  LoadPrefs();
  hmi_init();
  s_nextPollMs = millis();
  s_lastMenuActivityMs = s_nextPollMs;
  s_lastMenuTimeoutRefreshMs = s_nextPollMs;
}

void loop(void)
{
  const uint32_t now = millis();
  if ((int32_t)(now - s_nextPollMs) >= 0)
  {
    s_nextPollMs += HMI_POLL_MS;

    const hmi_tick_result_t tickResult = hmi_tick();
    DebugLogHmiErrors(tickResult);

    const hmi_tick_result_t dataErrors = (hmi_tick_result_t)(tickResult &
      (HMI_TICK_ERR_NOT_INITIALIZED | HMI_TICK_ERR_I2C_REQUEST | HMI_TICK_ERR_I2C_READ | HMI_TICK_ERR_BAD_PACKET));

    if (dataErrors != HMI_TICK_OK)
    {
      s_hmiDataValid = false;
    }
    else
    {
      ProcessHmiChanges();
    }
  }

  HandleMenuTimers();
  PumpSerialRx();
}
