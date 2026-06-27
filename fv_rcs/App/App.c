#include "App.h"
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"
#include "I2cSlave.h"
#include "inputs.h"
#include "st7735.h"

#define APP_I2C_PACKET_SIZE             6U
#define APP_I2C_STATUS_BUTTONS_MASK_LO  0xFFU
#define APP_I2C_STATUS_BUTTONS_MASK_HI  0x03U
#define APP_I2C_STATUS_USB_MASK         0x10U
#define APP_I2C_STATUS_LCD_BL_MASK      0x40U
#define APP_I2C_STATUS_LCD_BUSY_MASK    0x80U
#define APP_I2C_ADC_VALUE_MASK          0x0FFFU
#define APP_I2C_ADC_CHANGED_MASK        0x8000U
#define APP_POWER_OFF_COUNT          1000U

enum {
  APP_RX_CMD_LCD_SET_BL_TIMEOUT = 0x03,
  APP_RX_CMD_LCD_SET_BL_LEVEL = 0x04,
  APP_RX_CMD_TONE = 0x07,
  APP_RX_CMD_LCD_CLEAR = 0x10,
  APP_RX_CMD_LCD_FILL_RECT = 0x11,
  APP_RX_CMD_LCD_DRAW_MARKER = 0x12,
  APP_RX_CMD_LCD_DRAW_TEXT = 0x13,
  APP_RX_CMD_LCD_SET_BG_COLOR = 0x14,
  APP_RX_CMD_LCD_DRAW_INDICATOR = 0x20,
  APP_RX_CMD_LCD_DRAW_PROGRESS_BAR = 0x21
};

/* -------------------------------------------------------------------------- */
/* App state                                                                  */
/* -------------------------------------------------------------------------- */
static volatile uint8_t s_toneBusy = 0U;
static volatile uint32_t s_toneStopTick = 0U;
static uint32_t s_lastAppTick = 0U;
static uint8_t s_indicatorValue[2] = { 0U, 0U };
static volatile uint8_t s_i2cPacket[APP_I2C_PACKET_SIZE] = { 0U, 0U, 0U, 0U, 0U, 0U };

/* -------------------------------------------------------------------------- */
/* I2C packet helpers                                                         */
/* -------------------------------------------------------------------------- */
static void App_I2cPacketSetWord(uint8_t offset, uint16_t value) {
  s_i2cPacket[offset] = (uint8_t) (value & 0xFFU);
  s_i2cPacket[offset + 1U] = (uint8_t) (value >> 8);
}

static void App_SetButtonsForI2c(uint16_t buttons) {
  s_i2cPacket[0] = (uint8_t) (buttons & APP_I2C_STATUS_BUTTONS_MASK_LO);
  s_i2cPacket[1] = (uint8_t) ((s_i2cPacket[1] & (uint8_t) ~APP_I2C_STATUS_BUTTONS_MASK_HI) |
      ((buttons >> 8) & APP_I2C_STATUS_BUTTONS_MASK_HI));
}

static void App_SetUsbConnectedForI2c(uint8_t connected) {
  if (connected != 0U) {
    s_i2cPacket[1] |= APP_I2C_STATUS_USB_MASK;
  } else {
    s_i2cPacket[1] &= (uint8_t) ~APP_I2C_STATUS_USB_MASK;
  }
}

static void App_SetLcdFlagsForI2c(uint8_t value) {
  s_i2cPacket[1] = (uint8_t) ((s_i2cPacket[1] & (uint8_t) ~(APP_I2C_STATUS_LCD_BL_MASK | APP_I2C_STATUS_LCD_BUSY_MASK)) |
      (value & (APP_I2C_STATUS_LCD_BL_MASK | APP_I2C_STATUS_LCD_BUSY_MASK)));
}

static uint8_t App_UpdateAdcWordForI2c(uint8_t adcChannel, uint8_t packetOffset) {
  uint16_t value = (uint16_t) Inp_AiGet(adcChannel) & APP_I2C_ADC_VALUE_MASK;
  uint8_t ch = Inp_AdcisChanged(adcChannel)!= 0U;
  if (ch ) {
    value |= APP_I2C_ADC_CHANGED_MASK;
  }
  App_I2cPacketSetWord(packetOffset, value);
  return ch;
}

/* -------------------------------------------------------------------------- */
/* I2C callbacks                                                              */
/* -------------------------------------------------------------------------- */
static uint8_t App_DrawIndicator(uint8_t index) {
  if (index > 1U) return 2U;
  return LCD_DrawIndicator(index, s_indicatorValue[index]);
}

static void App_LcdQueueCallback(uint8_t value) {
  App_SetLcdFlagsForI2c(value);
}

/* outData length 32 */
static uint8_t App_I2cRequestCallback(uint8_t *outData) {
  if (outData == 0) return 0U;
  outData[0] = s_i2cPacket[0];
  outData[1] = s_i2cPacket[1];
  outData[2] = s_i2cPacket[2];
  outData[3] = s_i2cPacket[3];
  outData[4] = s_i2cPacket[4];
  outData[5] = s_i2cPacket[5];
  return APP_I2C_PACKET_SIZE;
}

static void App_I2cOnReceive(uint8_t *data, uint16_t size) {
  if ((data == 0) || (size == 0U)) return;

  switch (data[0]) {
  case APP_RX_CMD_TONE:
    if (size >= 5U) {
      uint16_t divider = (uint16_t) data[1] | ((uint16_t) data[2] << 8);
      uint16_t delayMs = (uint16_t) data[3] | ((uint16_t) data[4] << 8);
      Tone(divider, delayMs);
    }
    break;

  case APP_RX_CMD_LCD_SET_BL_TIMEOUT:
    if (size >= 5U) {
      uint32_t timeoutMs = (uint32_t) data[1] | ((uint32_t) data[2] << 8) |
          ((uint32_t) data[3] << 16) | ((uint32_t) data[4] << 24);
      LCD_SetBacklightTimeout(timeoutMs);
    }
    break;

  case APP_RX_CMD_LCD_SET_BL_LEVEL:
    if (size >= 2U) {
      LCD_SetBacklightLevel(data[1]);
    }
    break;

  case APP_RX_CMD_LCD_CLEAR:
    if (size >= 3U) {
      uint16_t color = (uint16_t) data[1] | ((uint16_t) data[2] << 8);
      //(void) LCD_Clear(color);
    }
    break;

  case APP_RX_CMD_LCD_FILL_RECT:
    if (size >= 7U) {
      uint16_t color = (uint16_t) data[5] | ((uint16_t) data[6] << 8);
      //(void) LCD_FillRect(data[1], data[2], data[3], data[4], color);
    }
    break;

  case APP_RX_CMD_LCD_DRAW_MARKER:
    if (size >= 6U) {
      uint16_t color = (uint16_t) data[4] | ((uint16_t) data[5] << 8);
      //(void) LCD_DrawMarker(data[1], data[2], data[3], color);
    }
    break;

  case APP_RX_CMD_LCD_DRAW_TEXT:
    if (size >= 6U) {
      uint16_t color = (uint16_t) data[3] | ((uint16_t) data[4] << 8);
      if (data[size - 1U] != 0U) data[size - 1U] = 0U;
      //(void) LCD_DrawText(data[1], data[2], (const char*) &data[5], color);
    }
    break;

  case APP_RX_CMD_LCD_SET_BG_COLOR:
    if (size >= 3U) {
      uint16_t color = (uint16_t) data[1] | ((uint16_t) data[2] << 8);
      LCD_SetBackgroundColor(color);
    }
    break;

  case APP_RX_CMD_LCD_DRAW_INDICATOR:
    if ((size >= 3U) && (data[1] <= 1U)) {
      s_indicatorValue[data[1]] =
          (uint8_t) ((data[1] == 1U ? (s_indicatorValue[data[1]] & 2U) : 0U) | (data[2] ? 1U : 0U));
      (void) App_DrawIndicator(data[1]);
    }
    break;

  case APP_RX_CMD_LCD_DRAW_PROGRESS_BAR:
    if ((size >= 3U) && (data[1] <= 2U)) {
      //(void) LCD_DrawProgressBar(data[1], data[2]);
    }
    break;

  default:
    break;
  }
}

static void App_ResetI2cState(void) {
  s_i2cPacket[0] = 0U;
  s_i2cPacket[1] = 0U;
  s_i2cPacket[2] = 0U;
  s_i2cPacket[3] = 0U;
  s_i2cPacket[4] = 0U;
  s_i2cPacket[5] = 0U;
}

/* -------------------------------------------------------------------------- */
/* Tone                                                                       */
/* divider = 1000000 / Freq                                                   */
/* -------------------------------------------------------------------------- */
void Tone(uint16_t divider, uint16_t delay_ms) {
  if (htim1.State == HAL_TIM_STATE_RESET) return;
  HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
  if (delay_ms == 0U) {
    s_toneBusy = 0U;
    return;
  }
  s_toneStopTick = HAL_GetTick() + delay_ms;
  s_toneBusy = 1U;
  if (divider == 0U) return;
  __HAL_TIM_SET_AUTORELOAD(&htim1, divider);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, divider / 16U);
  __HAL_TIM_SET_COUNTER(&htim1, 0U);
  HAL_TIM_GenerateEvent(&htim1, TIM_EVENTSOURCE_UPDATE);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
}

static void App_ProcessTone(void) {
  if (s_toneBusy == 0U) return;
  if ((int32_t) (HAL_GetTick() - s_toneStopTick) >= 0) {
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
    s_toneBusy = 0U;
  }
}

uint8_t Tone_IsBusy(void) {
  return s_toneBusy;
}

/* -------------------------------------------------------------------------- */
/* Power                                                                      */
/* -------------------------------------------------------------------------- */
static void App_ProcessPower(void) {
  static uint16_t offCounter = 0U;

  if (Inp_AiGet(ADC_INPUT_CH_V) < 850U) {
    if (offCounter >= APP_POWER_OFF_COUNT) {
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);
    } else {
      offCounter++;
      if (offCounter == (APP_POWER_OFF_COUNT / 2U)) {
        Tone(500U, 60U);
      }
    }
  } else if ((s_i2cPacket[0] & 0x01U) != 0U) {
    offCounter++;
    if (offCounter > APP_POWER_OFF_COUNT) {
      if (offCounter == (APP_POWER_OFF_COUNT + 2U)) {
        Tone(500U, 10U);
      }
      if ((offCounter * 3U) > (APP_POWER_OFF_COUNT * 4U)) {
        offCounter = APP_POWER_OFF_COUNT + 1U;
      }
    }
  } else if (offCounter > 0U) {
    if (offCounter >= APP_POWER_OFF_COUNT) {
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);
    } else {
      offCounter--;
    }
  }
}

/* -------------------------------------------------------------------------- */
/* Progress bar / battery SOC                                                 */
/* -------------------------------------------------------------------------- */
/* Ubat ≈ ADC_V * 0.00399446 - 0.19284 */
static const uint16_t ocv_adc[] =
  { 850, 874, 919, 940, 965, 975, 982, 990, 1002, 1010, 1015, 1022, 1030 };
/* SOC * 0.7 for progress bar (0..70) */
static const uint8_t ocv_soc[] =
  { 0, 4, 14, 21, 31, 35, 39, 42, 49, 53, 56, 63, 69 };
#define OCV_POINTS (sizeof(ocv_adc) / sizeof(ocv_adc[0]))

static inline uint8_t interp_fast(uint16_t x, uint16_t x1, uint16_t x2, uint8_t y1, uint8_t y2) {
  uint16_t dx = (uint16_t) (x2 - x1);
  uint16_t num = (uint16_t) (x - x1);
  uint16_t t = (uint16_t) ((num << 8) / dx);
  return (uint8_t) (y1 + ((((uint16_t) (y2 - y1)) * t) >> 8));
}

static uint8_t adc_to_soc(uint16_t adc) {
  int low;
  int high;

  if (adc <= ocv_adc[0]) return ocv_soc[0];
  if (adc >= ocv_adc[OCV_POINTS - 1U]) return ocv_soc[OCV_POINTS - 1U];

  low = 0;
  high = (int) OCV_POINTS - 1;
  while ((high - low) > 1) {
    int mid = (low + high) >> 1;
    if (adc < ocv_adc[mid]) {
      high = mid;
    } else {
      low = mid;
    }
  }
  return interp_fast(adc, ocv_adc[low], ocv_adc[high], ocv_soc[low], ocv_soc[high]);
}

/* -------------------------------------------------------------------------- */
/* ADC service                                                                */
/* -------------------------------------------------------------------------- */
static void App_ProcessAdc(void) {
  if (Inp_AdcEnsureStarted()) return;

  uint8_t ch_x = App_UpdateAdcWordForI2c(ADC_INPUT_CH_X, 2U);
  if(App_UpdateAdcWordForI2c(ADC_INPUT_CH_Y, 4U) || ch_x) return;

  if (Inp_AdcisChanged(ADC_INPUT_CH_U) != 0U) {
    uint8_t usbConnected = (Inp_AiGet(ADC_INPUT_CH_U) > 1000U) ? 1U : 0U;
    App_SetUsbConnectedForI2c(usbConnected);
    s_indicatorValue[1] = (uint8_t) ((usbConnected ? 2U : 0U) | (s_indicatorValue[1] & 1U));
    (void) App_DrawIndicator(1U);
  } else if (Inp_AdcisChanged(ADC_INPUT_CH_V) != 0U) {
    (void) LCD_DrawProgressBar(3U, adc_to_soc(Inp_AiGet(ADC_INPUT_CH_V)));
  }
}

/* -------------------------------------------------------------------------- */
/* App lifecycle                                                              */
/* -------------------------------------------------------------------------- */
void App_Init(void) {
  App_ResetI2cState();
  Inp_Init();
  I2cSlave_Init(&hi2c1, App_I2cRequestCallback, App_I2cOnReceive);
  LCD_Init(App_LcdQueueCallback);

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET); /* Power ON latch */
  LCD_SetBacklightTimeout(5000U);
  (void) Inp_AdcEnsureStarted();
  (void) App_DrawIndicator(0U);
  (void) App_DrawIndicator(1U);
  // !!!!!!!!!
  (void) LCD_FillRect(0, 8, 1, LCD_HEIGHT - 8, LCD_RED);
  (void) LCD_FillRect(LCD_WIDTH - 1, 8, 1, LCD_HEIGHT - 8, LCD_BLUE);
  (void) LCD_FillRect(0, LCD_HEIGHT-1, LCD_WIDTH, 1, LCD_GREEN);

  // !!!!!!!!!

  Tone(757U, 45U);
  HAL_Delay(65U);
  while (Inp_DiGet(0U) != 0U) {
    App_ProcessTone();
    (void) LCD_Process();
    __WFI();
  }
  Tone(636U, 60U);
  HAL_Delay(90U);
  Tone(476U, 90U);
  s_lastAppTick = HAL_GetTick();
}

void App_Run(void) {
  uint32_t now = HAL_GetTick();
  if (now != s_lastAppTick) {
    uint16_t buttons = 0U;
    uint8_t i;

    s_lastAppTick++;
    for (i = 0U; i < INPUT_BUTTON_COUNT; i++) {
      if (Inp_DiGet(i) != 0U) {
        buttons |= (uint16_t) (1U << i);
      }
    }
    App_SetButtonsForI2c(buttons);
    if (buttons != 0U) {
      LCD_SetBacklightTimeout(15000U);
    }
    App_ProcessTone();
    App_ProcessAdc();
    App_ProcessPower();
  }
  if ((LCD_Process() == 0U) && (HAL_GetTick() == s_lastAppTick)) {
    __WFI();
  }
}
