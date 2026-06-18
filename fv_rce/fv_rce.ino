#include <Wire.h>

static constexpr uint8_t  I2C_ADDR       = 0x14;
static constexpr int      SDA_PIN        = 21;
static constexpr int      SCL_PIN        = 22;
static constexpr uint32_t POLL_PERIOD_MS = 20;
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

// Write commands
static constexpr uint8_t CMD_LCD_FILL_CIRCLE = 0x12;
static constexpr uint8_t CMD_LCD_INDICATOR   = 0x20;

// LCD_Indicator params
static constexpr uint8_t LCD_INDICATOR_USB = 1;
static constexpr uint8_t LCD_INDICATOR_OFF = 0;
static constexpr uint8_t LCD_INDICATOR_ON  = 1;

// Colors RGB565
static constexpr uint16_t LCD_BLACK = 0x0000;
static constexpr uint16_t LCD_WHITE = 0xFFFF;

// Joystick marker area
static constexpr uint8_t JOY_X0 = 10;
static constexpr uint8_t JOY_Y0 = 10;
static constexpr uint8_t JOY_X1 = 79;
static constexpr uint8_t JOY_Y1 = 79;
static constexpr uint8_t JOY_R  = 3;

static uint16_t s_status = 0;
static uint16_t s_adcX = 0;
static uint16_t s_adcY = 0;
static uint8_t  s_buttons[10] = {0};

static bool s_adcXValid = false;
static bool s_adcYValid = false;

static bool s_serialActive = false;

static bool s_usbIndicatorPending = false;
static uint8_t s_usbIndicatorState = LCD_INDICATOR_OFF;

// Marker state
static bool s_markerVisible = false;
static bool s_markerPending = false;
static uint8_t s_markerDrawX = JOY_X0;
static uint8_t s_markerDrawY = JOY_Y0;
static uint8_t s_markerTargetX = JOY_X0;
static uint8_t s_markerTargetY = JOY_Y0;

static uint32_t s_nextPollMs = 0;

// Names by full protocol index [0..12]
static const char* btnNames[] =
{
  "", "", "", "ON", "Fire", "UP", "DOWN", "BACK", "OK",
  "LUP", "LDN", "RUP", "RDN"
};

static float Raw12ToNorm(uint16_t raw)
{
  return ((float)raw * (2.0f / 4095.0f)) - 1.0f;
}

static uint8_t Raw12ToRange(uint16_t raw, uint8_t outMin, uint8_t outMax)
{
  const uint32_t span = (uint32_t)outMax - (uint32_t)outMin;
  const uint32_t scaled = ((uint32_t)raw * span + 2047U) / 4095U;
  return (uint8_t)(outMin + scaled);
}

static const char* GetPressedButtonName()
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

static void UpdateMarkerTarget(void)
{
  if (!s_adcXValid || !s_adcYValid)
  {
    return;
  }

  const uint8_t newX = Raw12ToRange(s_adcX, JOY_X0, JOY_X1);
  const uint8_t newY = Raw12ToRange(s_adcY, JOY_Y0, JOY_Y1);

  if (s_markerPending)
  {
    if ((s_markerTargetX != newX) || (s_markerTargetY != newY))
    {
      s_markerTargetX = newX;
      s_markerTargetY = newY;
    }
    return;
  }

  if (!s_markerVisible || (s_markerDrawX != newX) || (s_markerDrawY != newY))
  {
    s_markerTargetX = newX;
    s_markerTargetY = newY;
    s_markerPending = true;
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
    "%1X %d %.3f %.3f %s\n",
    changedIndex & 0x0FU,
    (int)s_status,
    Raw12ToNorm(s_adcX),
    Raw12ToNorm(s_adcY),
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
      HandleUsbStateChange(oldStatus, s_status);
      break;

    case IDX_ADC_X:
      s_adcX = value;
      s_adcXValid = true;
      UpdateMarkerTarget();
      break;

    case IDX_ADC_Y:
      s_adcY = value;
      s_adcYValid = true;
      UpdateMarkerTarget();
      break;

    default:
      if ((index >= IDX_BUTTON_0) && (index <= IDX_BUTTON_9))
      {
        s_buttons[index - IDX_BUTTON_0] = (value != 0U) ? 1U : 0U;
      }
      break;
  }

  return shouldPrint;
}

static void TrySendPendingLcdCommands(void)
{
  if ((s_status & STATUS_LCD_BUSY) != 0U)
  {
    return;
  }

  if (s_usbIndicatorPending)
  {
    if (LCD_Indicator(LCD_INDICATOR_USB, s_usbIndicatorState))
    {
      s_usbIndicatorPending = false;
    }
    return;
  }

  if (!s_markerPending)
  {
    return;
  }

  if (s_markerVisible)
  {
    if (LCD_FillCircle(s_markerDrawX, s_markerDrawY, JOY_R, LCD_BLACK))
    {
      s_markerVisible = false;
    }
    return;
  }

  if (LCD_FillCircle(s_markerTargetX, s_markerTargetY, JOY_R, LCD_WHITE))
  {
    s_markerDrawX = s_markerTargetX;
    s_markerDrawY = s_markerTargetY;
    s_markerVisible = true;
    s_markerPending = false;
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
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);

  s_nextPollMs = millis();
}

void loop(void)
{
  const uint32_t now = millis();

  if ((int32_t)(now - s_nextPollMs) >= 0)
  {
    s_nextPollMs += POLL_PERIOD_MS;
    PollOnce();
  }
}
