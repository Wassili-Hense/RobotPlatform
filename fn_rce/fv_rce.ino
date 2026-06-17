#include <Wire.h>
#include <Preferences.h>
#include <math.h>
#include <ctype.h>

static constexpr uint8_t I2C_ADDR = 20;
static constexpr int I2C_SDA = 21;
static constexpr int I2C_SCL = 22;
static constexpr uint32_t I2C_FREQ = 100000;

// I2C commands
static constexpr uint8_t CMD_TONE                 = 0x00;
static constexpr uint8_t CMD_LCD_SET_BL_LEVEL     = 0x11;
static constexpr uint8_t CMD_LCD_CLEAR            = 0x12;
static constexpr uint8_t CMD_LCD_FILL_RECT        = 0x13;
static constexpr uint8_t CMD_LCD_FILL_CIRCLE      = 0x14;
static constexpr uint8_t CMD_LCD_DRAW_TEXT        = 0x15;
static constexpr uint8_t CMD_LCD_DRAW_PROGRESSBAR = 0x16;

// Response indexes
static constexpr uint8_t IDX_STATUS      = 0;
static constexpr uint8_t IDX_ADC_X       = 1;
static constexpr uint8_t IDX_ADC_Y       = 2;
static constexpr uint8_t IDX_BUTTON_ON   = 3;
static constexpr uint8_t IDX_BUTTON_FIRE = 4;
static constexpr uint8_t IDX_BUTTON_UP   = 5;
static constexpr uint8_t IDX_BUTTON_DOWN = 6;
static constexpr uint8_t IDX_BUTTON_BACK = 7;
static constexpr uint8_t IDX_BUTTON_OK   = 8;
static constexpr uint8_t IDX_BUTTON_LUP  = 9;
static constexpr uint8_t IDX_BUTTON_LDN  = 10;
static constexpr uint8_t IDX_BUTTON_RUP  = 11;
static constexpr uint8_t IDX_BUTTON_RDN  = 12;

// Colors RGB565
static constexpr uint16_t LCD_BLACK = 0x0000;
static constexpr uint16_t LCD_WHITE = 0xFFFF;
static constexpr uint16_t LCD_GREEN = 0x07E0;
static constexpr uint16_t LCD_BLUE  = 0x001F;

// Joystick drawing area
static constexpr uint8_t JOY_X0 = 10;
static constexpr uint8_t JOY_Y0 = 10;
static constexpr uint8_t JOY_X1 = 79;
static constexpr uint8_t JOY_Y1 = 79;
static constexpr uint8_t JOY_R  = 3;

// Root menu area
static constexpr uint8_t MENU_X0 = 10;
static constexpr uint8_t MENU_Y0 = 10;
static constexpr uint8_t MENU_X1 = 140;
static constexpr uint8_t MENU_Y1 = 79;
static constexpr uint8_t MENU_W  = (uint8_t)(MENU_X1 - MENU_X0 + 1U);
static constexpr uint8_t MENU_H  = (uint8_t)(MENU_Y1 - MENU_Y0 + 1U);

// Menu text position (keep away from joystick field)
static constexpr uint8_t TXT_X = 90;
static constexpr uint8_t TXT_Y0 = 10;
static constexpr uint8_t TXT_DY = 16;

static constexpr uint32_t POLL_PERIOD_MS = 10;
static constexpr uint32_t LCD_WAIT_TIMEOUT_MS = 500;

static const uint8_t kBrightnessTable[] = { 1, 4, 9, 16, 25, 36, 49, 64, 81, 100, 121, 127 };
static constexpr uint8_t kBrightnessCount = sizeof(kBrightnessTable) / sizeof(kBrightnessTable[0]);

Preferences prefs;

struct AxisCal
{
  uint16_t min;
  uint16_t cMin;
  uint16_t cMax;
  uint16_t max;
};

struct Settings
{
  AxisCal x;
  AxisCal y;
  uint8_t brightness;
};

struct MinMaxTrack
{
  uint16_t min;
  uint16_t max;
};

static Settings s_cfg;

static uint16_t s_values[13] = {};
static uint8_t s_buttons[13] = {};
static uint8_t s_prevButtons[13] = {};

static uint16_t s_adcX = 0;
static uint16_t s_adcY = 0;

static float s_normX = 0.0f;
static float s_normY = 0.0f;

static bool s_haveFirstResponse = false;
static bool s_brightnessSent = false;
static bool s_usbConnected = false;
static bool s_serialStarted = false;
static bool s_lcdBusy = false;
static bool s_joyPrintPending = false;

static int16_t s_lastJoyX = JOY_X0;
static int16_t s_lastJoyY = JOY_Y0;
static bool s_joyVisible = false;

static String s_serialLine;

enum UiMode
{
  UI_NORMAL = 0,
  UI_MENU,
  UI_CAL_CENTER,
  UI_CAL_EDGE
};

static UiMode s_uiMode = UI_NORMAL;
static uint8_t s_menuIndex = 0;

static MinMaxTrack s_centerX = {};
static MinMaxTrack s_centerY = {};
static MinMaxTrack s_edgeX = {};
static MinMaxTrack s_edgeY = {};

static float clampNorm(float v)
{
  if (v < -1.0f) return -1.0f;
  if (v > 1.0f) return 1.0f;
  return v;
}

static uint8_t clampU8(int v)
{
  if (v < 0) return 0;
  if (v > 255) return 255;
  return (uint8_t)v;
}

static void fixAxisOrder(AxisCal& a)
{
  if (a.min > a.cMin) a.min = a.cMin;
  if (a.cMin > a.cMax) a.cMax = a.cMin;
  if (a.cMax > a.max) a.max = a.cMax;

  if (a.min == a.cMin)
  {
    if (a.min > 0U) a.min--;
    else if (a.max < 4095U) a.max++;
  }

  if (a.cMax == a.max)
  {
    if (a.max < 4095U) a.max++;
    else if (a.min > 0U) a.min--;
  }
}

static int brightnessToIndex(uint8_t value)
{
  int bestIndex = 0;
  int bestDiff = abs((int)value - (int)kBrightnessTable[0]);

  for (int i = 1; i < (int)kBrightnessCount; ++i)
  {
    int diff = abs((int)value - (int)kBrightnessTable[i]);
    if (diff < bestDiff)
    {
      bestDiff = diff;
      bestIndex = i;
    }
  }

  return bestIndex;
}

static void saveSettings()
{
  fixAxisOrder(s_cfg.x);
  fixAxisOrder(s_cfg.y);

  prefs.putUShort("x_min",  s_cfg.x.min);
  prefs.putUShort("x_cmin", s_cfg.x.cMin);
  prefs.putUShort("x_cmax", s_cfg.x.cMax);
  prefs.putUShort("x_max",  s_cfg.x.max);

  prefs.putUShort("y_min",  s_cfg.y.min);
  prefs.putUShort("y_cmin", s_cfg.y.cMin);
  prefs.putUShort("y_cmax", s_cfg.y.cMax);
  prefs.putUShort("y_max",  s_cfg.y.max);

  prefs.putUChar("bl", s_cfg.brightness);
}

static void loadSettings()
{
  prefs.begin("joylcd", false);

  s_cfg.x.min  = prefs.getUShort("x_min",  0);
  s_cfg.x.cMin = prefs.getUShort("x_cmin", 1900);
  s_cfg.x.cMax = prefs.getUShort("x_cmax", 2200);
  s_cfg.x.max  = prefs.getUShort("x_max",  4095);

  s_cfg.y.min  = prefs.getUShort("y_min",  0);
  s_cfg.y.cMin = prefs.getUShort("y_cmin", 1900);
  s_cfg.y.cMax = prefs.getUShort("y_cmax", 2200);
  s_cfg.y.max  = prefs.getUShort("y_max",  4095);

  s_cfg.brightness = prefs.getUChar("bl", 64);

  fixAxisOrder(s_cfg.x);
  fixAxisOrder(s_cfg.y);

  s_cfg.brightness = kBrightnessTable[brightnessToIndex(s_cfg.brightness)];
}

static bool isPressedEdge(uint8_t idx)
{
  return (s_buttons[idx] != 0U) && (s_prevButtons[idx] == 0U);
}

static void copyButtonsToPrev()
{
  for (uint8_t i = 0; i < 13U; ++i)
  {
    s_prevButtons[i] = s_buttons[i];
  }
}

static bool i2cWrite(const uint8_t* data, size_t len)
{
  Wire.beginTransmission(I2C_ADDR);
  size_t written = Wire.write(data, len);
  uint8_t err = Wire.endTransmission();
  return (written == len) && (err == 0U);
}

static bool i2cReadPair(uint8_t& index, uint16_t& value)
{
  uint8_t got = Wire.requestFrom((int)I2C_ADDR, 2);
  if (got != 2U)
  {
    while (Wire.available()) (void)Wire.read();
    return false;
  }

  uint8_t b0 = (uint8_t)Wire.read();
  uint8_t b1 = (uint8_t)Wire.read();

  index = (uint8_t)(b0 >> 4);
  value = (uint16_t)(((uint16_t)(b0 & 0x0FU) << 8) | b1);
  return true;
}

static void applyUsbState(bool connected)
{
  if (connected)
  {
    if (!s_serialStarted)
    {
      Serial.begin(115200);
      delay(20);
      s_serialStarted = true;
    }
  }
  else
  {
    if (s_serialStarted)
    {
      Serial.flush();
      Serial.end();
      s_serialStarted = false;
    }
  }
}

static void updateByResponse(uint8_t index, uint16_t value)
{
  if (index < 13U)
  {
    s_values[index] = value;
  }

  switch (index)
  {
    case IDX_STATUS:
      s_lcdBusy = (value & 0x0001U) != 0U;
      s_usbConnected = (value & 0x0002U) != 0U;
      applyUsbState(s_usbConnected);
      break;

    case IDX_ADC_X:
      if (s_adcX != value)
      {
        s_adcX = value;
        s_joyPrintPending = true;
      }
      break;

    case IDX_ADC_Y:
      if (s_adcY != value)
      {
        s_adcY = value;
        s_joyPrintPending = true;
      }
      break;

    case IDX_BUTTON_ON:
    case IDX_BUTTON_FIRE:
    case IDX_BUTTON_UP:
    case IDX_BUTTON_DOWN:
    case IDX_BUTTON_BACK:
    case IDX_BUTTON_OK:
    case IDX_BUTTON_LUP:
    case IDX_BUTTON_LDN:
    case IDX_BUTTON_RUP:
    case IDX_BUTTON_RDN:
      s_buttons[index] = (value != 0U) ? 1U : 0U;
      break;

    default:
      break;
  }
}

static bool pollOneResponse()
{
  uint8_t index = 0;
  uint16_t value = 0;
  if (!i2cReadPair(index, value))
  {
    return false;
  }

  s_haveFirstResponse = true;
  updateByResponse(index, value);
  return true;
}

static void pollBurst(uint8_t count)
{
  for (uint8_t i = 0; i < count; ++i)
  {
    (void)pollOneResponse();
    delay(1);
  }
}

static bool waitLcdReady(uint32_t timeoutMs)
{
  uint32_t start = millis();

  while (s_lcdBusy)
  {
    (void)pollOneResponse();

    if ((uint32_t)(millis() - start) >= timeoutMs)
    {
      return false;
    }

    delay(2);
  }

  return true;
}

static bool cmdTone(uint16_t divider, uint16_t delayMs)
{
  uint8_t buf[5];
  buf[0] = CMD_TONE;
  buf[1] = (uint8_t)(divider & 0xFFU);
  buf[2] = (uint8_t)(divider >> 8);
  buf[3] = (uint8_t)(delayMs & 0xFFU);
  buf[4] = (uint8_t)(delayMs >> 8);
  return i2cWrite(buf, sizeof(buf));
}

static bool cmdSetBacklightLevel(uint8_t level)
{
  if (!waitLcdReady(LCD_WAIT_TIMEOUT_MS)) return false;

  uint8_t buf[2];
  buf[0] = CMD_LCD_SET_BL_LEVEL;
  buf[1] = (level > 127U) ? 127U : level;
  return i2cWrite(buf, sizeof(buf));
}

static bool cmdClear(uint16_t color)
{
  if (!waitLcdReady(LCD_WAIT_TIMEOUT_MS)) return false;

  uint8_t buf[3];
  buf[0] = CMD_LCD_CLEAR;
  buf[1] = (uint8_t)(color & 0xFFU);
  buf[2] = (uint8_t)(color >> 8);
  return i2cWrite(buf, sizeof(buf));
}

static bool cmdFillRect(uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint16_t color)
{
  if (!waitLcdReady(LCD_WAIT_TIMEOUT_MS)) return false;

  uint8_t buf[7];
  buf[0] = CMD_LCD_FILL_RECT;
  buf[1] = x;
  buf[2] = y;
  buf[3] = w;
  buf[4] = h;
  buf[5] = (uint8_t)(color & 0xFFU);
  buf[6] = (uint8_t)(color >> 8);
  return i2cWrite(buf, sizeof(buf));
}

static bool cmdFillCircle(uint8_t x, uint8_t y, uint8_t r, uint16_t color)
{
  if (!waitLcdReady(LCD_WAIT_TIMEOUT_MS)) return false;

  uint8_t buf[6];
  buf[0] = CMD_LCD_FILL_CIRCLE;
  buf[1] = x;
  buf[2] = y;
  buf[3] = r;
  buf[4] = (uint8_t)(color & 0xFFU);
  buf[5] = (uint8_t)(color >> 8);
  return i2cWrite(buf, sizeof(buf));
}

static bool cmdDrawText(uint8_t x, uint8_t y, const char* text)
{
  if (!waitLcdReady(LCD_WAIT_TIMEOUT_MS)) return false;

  Wire.beginTransmission(I2C_ADDR);
  Wire.write((uint8_t)CMD_LCD_DRAW_TEXT);
  Wire.write(x);
  Wire.write(y);

  for (const char* p = text; *p != '\0'; ++p)
  {
    char c = *p;
    if (c < 32 || c > 126) c = '?';
    Wire.write((uint8_t)c);
  }

  return Wire.endTransmission() == 0U;
}

static bool cmdDrawProgressBar(uint8_t index, uint8_t value)
{
  if (!waitLcdReady(LCD_WAIT_TIMEOUT_MS)) return false;

  uint8_t buf[3];
  buf[0] = CMD_LCD_DRAW_PROGRESSBAR;
  buf[1] = index;
  buf[2] = value;
  return i2cWrite(buf, sizeof(buf));
}

static float mapAxisValue(uint16_t raw, const AxisCal& a)
{
  if (raw <= a.cMin)
  {
    int32_t den = (int32_t)a.cMin - (int32_t)a.min;
    if (den <= 0) return -1.0f;
    return clampNorm((float)((int32_t)raw - (int32_t)a.cMin) / (float)den);
  }

  if (raw >= a.cMax)
  {
    int32_t den = (int32_t)a.max - (int32_t)a.cMax;
    if (den <= 0) return 1.0f;
    return clampNorm((float)((int32_t)raw - (int32_t)a.cMax) / (float)den);
  }

  return 0.0f;
}

static void computeNorm()
{
  s_normX = mapAxisValue(s_adcX, s_cfg.x);
  s_normY = mapAxisValue(s_adcY, s_cfg.y);
}

static int16_t joyToScreenX(float x)
{
  float t = (x + 1.0f) * 0.5f;
  int16_t w = (int16_t)(JOY_X1 - JOY_X0);
  return (int16_t)(JOY_X0 + lroundf(t * (float)w));
}

static int16_t joyToScreenY(float y)
{
  float t = (y + 1.0f) * 0.5f;
  int16_t h = (int16_t)(JOY_Y1 - JOY_Y0);
  return (int16_t)(JOY_Y0 + lroundf(t * (float)h));
}

static void drawJoyMarker(uint16_t color, bool eraseOld)
{
  int16_t x = joyToScreenX(s_normX);
  int16_t y = joyToScreenY(s_normY);

  if (eraseOld && s_joyVisible)
  {
    (void)cmdFillCircle((uint8_t)s_lastJoyX, (uint8_t)s_lastJoyY, JOY_R, LCD_BLACK);
  }

  (void)cmdFillCircle((uint8_t)x, (uint8_t)y, JOY_R, color);

  s_lastJoyX = x;
  s_lastJoyY = y;
  s_joyVisible = true;
}

static void clearMenuArea()
{
  (void)cmdFillRect(MENU_X0, MENU_Y0, MENU_W, MENU_H, LCD_BLACK);
}

static void clearWholeScreen()
{
  (void)cmdClear(LCD_BLACK);
}

static void drawMenu()
{
  clearMenuArea();
  (void)cmdDrawText(TXT_X, TXT_Y0 + 0 * TXT_DY, "MENU");
  (void)cmdDrawText(TXT_X, TXT_Y0 + 1 * TXT_DY, s_menuIndex == 0U ? ">CENTER" : " CENTER");
  (void)cmdDrawText(TXT_X, TXT_Y0 + 2 * TXT_DY, s_menuIndex == 1U ? ">EDGE"   : " EDGE");
  (void)cmdDrawText(TXT_X, TXT_Y0 + 3 * TXT_DY, "LUP LDN");
}

static void enterMenu()
{
  s_uiMode = UI_MENU;
  drawMenu();
}

static void leaveMenu()
{
  s_uiMode = UI_NORMAL;
  clearMenuArea();
  s_joyVisible = false;
}

static void startCalCenter()
{
  s_uiMode = UI_CAL_CENTER;

  s_centerX.min = s_adcX;
  s_centerX.max = s_adcX;
  s_centerY.min = s_adcY;
  s_centerY.max = s_adcY;

  clearMenuArea();
  (void)cmdDrawText(TXT_X, TXT_Y0 + 0 * TXT_DY, "CENTER");
  (void)cmdDrawText(TXT_X, TXT_Y0 + 1 * TXT_DY, "OK SAVE");
  (void)cmdDrawText(TXT_X, TXT_Y0 + 2 * TXT_DY, "BACK");
}

static void startCalEdge()
{
  s_uiMode = UI_CAL_EDGE;

  s_edgeX.min = s_adcX;
  s_edgeX.max = s_adcX;
  s_edgeY.min = s_adcY;
  s_edgeY.max = s_adcY;

  clearMenuArea();
  (void)cmdDrawText(TXT_X, TXT_Y0 + 0 * TXT_DY, "EDGE");
  (void)cmdDrawText(TXT_X, TXT_Y0 + 1 * TXT_DY, "OK SAVE");
  (void)cmdDrawText(TXT_X, TXT_Y0 + 2 * TXT_DY, "BACK");
}

static void updateBrightness(int delta)
{
  int idx = brightnessToIndex(s_cfg.brightness);
  idx += delta;

  if (idx < 0) idx = 0;
  if (idx >= (int)kBrightnessCount) idx = (int)kBrightnessCount - 1;

  uint8_t newLevel = kBrightnessTable[idx];
  if (newLevel != s_cfg.brightness)
  {
    s_cfg.brightness = newLevel;
    saveSettings();
    (void)cmdSetBacklightLevel(s_cfg.brightness);
  }
}

static void processSerialCommand(const String& line)
{
  if (line.length() < 2) return;

  char c = (char)toupper((unsigned char)line[0]);

  if (c == 'A' || c == 'B' || c == 'C')
  {
    int value = line.substring(1).toInt();
    uint8_t barIndex = (c == 'A') ? 0U : (c == 'B' ? 1U : 2U);
    (void)cmdDrawProgressBar(barIndex, clampU8(value));
    return;
  }

  if (c == 'T')
  {
    int comma = line.indexOf(',');
    if (comma > 1)
    {
      int hz = line.substring(1, comma).toInt();
      int ms = line.substring(comma + 1).toInt();

      if ((hz > 0) && (ms > 0))
      {
        uint32_t divider = 1000000UL / (uint32_t)hz;
        if (divider == 0U) divider = 1U;
        if (divider > 65535UL) divider = 65535UL;
        if (ms > 65535) ms = 65535;

        (void)cmdTone((uint16_t)divider, (uint16_t)ms);
      }
    }
  }
}

static void processSerialInput()
{
  if (!s_serialStarted) return;

  while (Serial.available() > 0)
  {
    char ch = (char)Serial.read();

    if (ch == '\r')
    {
      continue;
    }
    else if (ch == '\n')
    {
      processSerialCommand(s_serialLine);
      s_serialLine = "";
    }
    else
    {
      if (s_serialLine.length() < 63)
      {
        s_serialLine += ch;
      }
    }
  }
}

static void printJoy()
{
  if (!s_serialStarted) return;
  if (!s_joyPrintPending) return;

  s_joyPrintPending = false;

  Serial.print(s_normX, 3);
  Serial.print(",");
  Serial.println(s_normY, 3);
}

static void processCalCenter()
{
  if (s_adcX < s_centerX.min) s_centerX.min = s_adcX;
  if (s_adcX > s_centerX.max) s_centerX.max = s_adcX;
  if (s_adcY < s_centerY.min) s_centerY.min = s_adcY;
  if (s_adcY > s_centerY.max) s_centerY.max = s_adcY;

  drawJoyMarker(LCD_GREEN, false);

  if (isPressedEdge(IDX_BUTTON_OK))
  {
    s_cfg.x.cMin = s_centerX.min;
    s_cfg.x.cMax = s_centerX.max;
    s_cfg.y.cMin = s_centerY.min;
    s_cfg.y.cMax = s_centerY.max;

    if (s_cfg.x.min > s_cfg.x.cMin) s_cfg.x.min = s_cfg.x.cMin;
    if (s_cfg.x.max < s_cfg.x.cMax) s_cfg.x.max = s_cfg.x.cMax;
    if (s_cfg.y.min > s_cfg.y.cMin) s_cfg.y.min = s_cfg.y.cMin;
    if (s_cfg.y.max < s_cfg.y.cMax) s_cfg.y.max = s_cfg.y.cMax;

    saveSettings();
    enterMenu();
  }
  else if (isPressedEdge(IDX_BUTTON_BACK))
  {
    enterMenu();
  }
}

static void processCalEdge()
{
  if (s_adcX < s_edgeX.min) s_edgeX.min = s_adcX;
  if (s_adcX > s_edgeX.max) s_edgeX.max = s_adcX;
  if (s_adcY < s_edgeY.min) s_edgeY.min = s_adcY;
  if (s_adcY > s_edgeY.max) s_edgeY.max = s_adcY;

  drawJoyMarker(LCD_BLUE, false);

  if (isPressedEdge(IDX_BUTTON_OK))
  {
    s_cfg.x.min = s_edgeX.min;
    s_cfg.x.max = s_edgeX.max;
    s_cfg.y.min = s_edgeY.min;
    s_cfg.y.max = s_edgeY.max;

    if (s_cfg.x.cMin < s_cfg.x.min) s_cfg.x.cMin = s_cfg.x.min;
    if (s_cfg.x.cMax > s_cfg.x.max) s_cfg.x.cMax = s_cfg.x.max;
    if (s_cfg.y.cMin < s_cfg.y.min) s_cfg.y.cMin = s_cfg.y.min;
    if (s_cfg.y.cMax > s_cfg.y.max) s_cfg.y.cMax = s_cfg.y.max;

    saveSettings();
    enterMenu();
  }
  else if (isPressedEdge(IDX_BUTTON_BACK))
  {
    enterMenu();
  }
}

static void processUi()
{
  if (s_uiMode != UI_NORMAL)
  {
    if (isPressedEdge(IDX_BUTTON_LUP)) updateBrightness(+1);
    if (isPressedEdge(IDX_BUTTON_LDN)) updateBrightness(-1);
  }

  if (s_uiMode == UI_NORMAL)
  {
    if (isPressedEdge(IDX_BUTTON_OK))
    {
      enterMenu();
      return;
    }

    drawJoyMarker(LCD_WHITE, true);
    return;
  }

  if (s_uiMode == UI_MENU)
  {
    if (isPressedEdge(IDX_BUTTON_UP))
    {
      if (s_menuIndex > 0U) s_menuIndex--;
      drawMenu();
    }

    if (isPressedEdge(IDX_BUTTON_DOWN))
    {
      if (s_menuIndex < 1U) s_menuIndex++;
      drawMenu();
    }

    if (isPressedEdge(IDX_BUTTON_OK))
    {
      if (s_menuIndex == 0U) startCalCenter();
      else startCalEdge();
      return;
    }

    if (isPressedEdge(IDX_BUTTON_BACK))
    {
      leaveMenu();
      return;
    }

    return;
  }

  if (s_uiMode == UI_CAL_CENTER)
  {
    processCalCenter();
    return;
  }

  if (s_uiMode == UI_CAL_EDGE)
  {
    processCalEdge();
    return;
  }
}

void setup()
{
  Serial.begin(115200);
  delay(50);
  s_serialStarted = true;

  Wire.begin(I2C_SDA, I2C_SCL, I2C_FREQ);

  loadSettings();

  for (uint8_t i = 0; i < 24U; ++i)
  {
    (void)pollOneResponse();
    delay(5);
  }

  if (s_haveFirstResponse && !s_brightnessSent)
  {
    (void)cmdSetBacklightLevel(s_cfg.brightness);
    s_brightnessSent = true;
  }

  clearWholeScreen();
  s_joyVisible = false;
}

void loop()
{
  static uint32_t lastPollMs = 0;
  uint32_t now = millis();

  if ((uint32_t)(now - lastPollMs) >= POLL_PERIOD_MS)
  {
    lastPollMs = now;
    copyButtonsToPrev();
    pollBurst(4);
  }

  if (s_haveFirstResponse && !s_brightnessSent)
  {
    (void)cmdSetBacklightLevel(s_cfg.brightness);
    s_brightnessSent = true;
  }

  computeNorm();
  processUi();
  processSerialInput();
  printJoy();
}