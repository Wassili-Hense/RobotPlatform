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

#define APP_POWER_OFF_COUNT            1000U
#define APP_ADC_START_PERIOD_MS         100U
#define APP_ADC_FORCE_SEND_MS          2500U
#define APP_ADC_MIN_SEND_PERIOD_MS      100U
#define APP_ADC_DELTA                    15U
#define APP_I2C_ITEM_COUNT              13U

#define APP_ABS_DIFF_U16(a, b) (((a) >= (b)) ? ((a) - (b)) : ((b) - (a)))

enum {
  APP_I2C_INDEX_STATUS = 0,
  APP_I2C_INDEX_ADC_X = 1,
  APP_I2C_INDEX_ADC_Y = 2,
  APP_I2C_INDEX_BUTTON_0 = 3,
  APP_I2C_INDEX_BUTTON_1 = 4,
  APP_I2C_INDEX_BUTTON_2 = 5,
  APP_I2C_INDEX_BUTTON_3 = 6,
  APP_I2C_INDEX_BUTTON_4 = 7,
  APP_I2C_INDEX_BUTTON_5 = 8,
  APP_I2C_INDEX_BUTTON_6 = 9,
  APP_I2C_INDEX_BUTTON_7 = 10,
  APP_I2C_INDEX_BUTTON_8 = 11,
  APP_I2C_INDEX_BUTTON_9 = 12
};

enum {
  APP_RX_CMD_TONE = 0x00,
  APP_RX_CMD_LCD_SET_BL_TIMEOUT = 0x10,
  APP_RX_CMD_LCD_SET_BL_LEVEL = 0x11,
  APP_RX_CMD_LCD_CLEAR = 0x12,
  APP_RX_CMD_LCD_FILL_RECT = 0x13,
  APP_RX_CMD_LCD_FILL_CIRCLE = 0x14,
  APP_RX_CMD_LCD_DRAW_TEXT = 0x15,
  APP_RX_CMD_LCD_DRAW_PROGRESS_BAR = 0x16
};

/* -------------------------------------------------------------------------- */
/* I2C state                                                                  */
/* -------------------------------------------------------------------------- */
static volatile uint8_t s_i2cDirty[APP_I2C_ITEM_COUNT];
static volatile uint16_t s_i2cActualValue[APP_I2C_ITEM_COUNT];
static volatile uint16_t s_i2cSentValue[APP_I2C_ITEM_COUNT];
static volatile uint32_t s_i2cTick[APP_I2C_ITEM_COUNT];

/* -------------------------------------------------------------------------- */
/* App state                                                                  */
/* -------------------------------------------------------------------------- */
static volatile uint8_t s_toneBusy = 0U;
static volatile uint32_t s_toneStopTick = 0U;
static uint32_t s_lastAppTick = 0U;
static uint32_t s_adcStartTick = 0U;

/* -------------------------------------------------------------------------- */
/* I2C callbacks and data preparation                                         */
/* -------------------------------------------------------------------------- */
static void App_PrepareDigitalForI2c(uint8_t index, uint16_t value) {
  if (index >= APP_I2C_ITEM_COUNT) return;

  s_i2cActualValue[index] = value;
  if (s_i2cSentValue[index] != value) s_i2cDirty[index] = 1U;
}

static uint8_t App_ProcessAnalogForI2c(uint8_t adcChannel, uint8_t index, uint16_t value) {
  uint8_t adcChanged;

  if (index >= APP_I2C_ITEM_COUNT) return 0U;

  adcChanged = Inp_AdcisChanged(adcChannel);

  if (adcChanged) {
    s_i2cActualValue[index] = value;

    if (!s_i2cDirty[index] && (APP_ABS_DIFF_U16(s_i2cActualValue[index], s_i2cSentValue[index]) > APP_ADC_DELTA)) {
      s_i2cTick[index] -= APP_ADC_FORCE_SEND_MS - APP_ADC_MIN_SEND_PERIOD_MS;
    }
  }

  if ((int32_t) (HAL_GetTick() - s_i2cTick[index]) >= 0) s_i2cDirty[index] = 1U;

  return adcChanged;
}

static void App_LcdQueueCallback(uint8_t value) {
  uint16_t status = s_i2cActualValue[APP_I2C_INDEX_STATUS] & 0x0FFE;

  if (value != 0U) {
    status |= 1U;
  }
  App_PrepareDigitalForI2c(APP_I2C_INDEX_STATUS, status);
}

static uint8_t App_I2cRequestCallback(uint8_t *outData) {
  uint8_t i;
  uint8_t index = APP_I2C_INDEX_STATUS;
  uint16_t value;

  if (outData == 0) return 0U;

  for (i = 0U; i < APP_I2C_ITEM_COUNT; i++) {
    if (s_i2cDirty[i] != 0U) {
      index = i;
      break;
    }
  }

  value = s_i2cActualValue[index];
  outData[0] = (uint8_t) ((index << 4) | ((value >> 8) & 0x0FU));
  outData[1] = (uint8_t) value;

  s_i2cDirty[index] = 0U;
  s_i2cSentValue[index] = value;
  s_i2cTick[index] = HAL_GetTick() + APP_ADC_FORCE_SEND_MS;

  return 2U;
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
      uint32_t timeoutMs = (uint32_t) data[1] | ((uint32_t) data[2] << 8) | ((uint32_t) data[3] << 16) | ((uint32_t) data[4] << 24);
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
      (void) LCD_Clear(color);
    }
    break;

  case APP_RX_CMD_LCD_FILL_RECT:
    if (size >= 7U) {
      uint16_t color = (uint16_t) data[5] | ((uint16_t) data[6] << 8);
      (void) LCD_FillRect(data[1], data[2], data[3], data[4], color);
    }
    break;

  case APP_RX_CMD_LCD_FILL_CIRCLE:
    if (size >= 6U) {
      uint16_t color = (uint16_t) data[4] | ((uint16_t) data[5] << 8);
      (void) LCD_FillCircle(data[1], data[2], data[3], color);
    }
    break;

  case APP_RX_CMD_LCD_DRAW_TEXT:
    if (size >= 4U) {
      if ((data[size - 1U] != 0U)) data[size - 1U] = 0U;
      (void) LCD_DrawText(data[1], data[2], (const char*) &data[3],
      LCD_WHITE, LCD_BLACK);
    }
    break;

  case APP_RX_CMD_LCD_DRAW_PROGRESS_BAR:
    if ((size >= 3U) && (data[1] <= 2U)) {
      (void) LCD_DrawProgressBar(data[1], data[2]);
    }
    break;

  default:
    break;
  }
}

static void App_ResetI2cState(void) {
  uint8_t i;

  for (i = 0U; i < APP_I2C_ITEM_COUNT; i++) {
    s_i2cDirty[i] = 0U;
    s_i2cActualValue[i] = 0U;
    s_i2cSentValue[i] = 0U;
    s_i2cTick[i] = 0U;
  }
}

/* -------------------------------------------------------------------------- */
/* Tone                                                                       */
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
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, divider / 2U);
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

  if (ADC_V < 850U) {
    if (offCounter >= APP_POWER_OFF_COUNT) {
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);
    } else {
      offCounter++;
      if (offCounter == (APP_POWER_OFF_COUNT / 2U)) {
        Tone(500U, 60U);
      }
    }
  } else if (s_i2cActualValue[APP_I2C_INDEX_BUTTON_0]) {
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
  uint32_t tick = HAL_GetTick();

  if ((tick - s_adcStartTick) >= APP_ADC_START_PERIOD_MS) {
    s_adcStartTick = tick;
    Inp_AdcStart();
  } else
    if (App_ProcessAnalogForI2c(ADC_INPUT_CH_X, APP_I2C_INDEX_ADC_X, ADC_X)) {

  } else if (App_ProcessAnalogForI2c(ADC_INPUT_CH_Y, APP_I2C_INDEX_ADC_Y, ADC_Y)) {

  } else if (Inp_AdcisChanged(ADC_INPUT_CH_U) != 0U) {
    App_PrepareDigitalForI2c(APP_I2C_INDEX_STATUS, ((ADC_U > 1000U) ? 2U : 0U) | (s_i2cActualValue[APP_I2C_INDEX_STATUS] & 0x0FFD));
  } else if (Inp_AdcisChanged(ADC_INPUT_CH_V) != 0U) {
    (void) LCD_DrawProgressBar(3U, adc_to_soc(ADC_V));
  }
}

/* -------------------------------------------------------------------------- */
/* App lifecycle                                                              */
/* -------------------------------------------------------------------------- */
void App_Init(void) {
  s_lastAppTick = HAL_GetTick();
  s_adcStartTick = s_lastAppTick - APP_ADC_START_PERIOD_MS;

  App_ResetI2cState();
  Inp_Init();
  I2cSlave_Init(&hi2c1, App_I2cRequestCallback, App_I2cOnReceive);
  LCD_Init(App_LcdQueueCallback);
  Inp_AdcStart();

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET); /* Power ON latch */
  LCD_SetBacklightTimeout(5000U);

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
}

void App_Run(void) {
  uint32_t now = HAL_GetTick();

  if (now != s_lastAppTick) {
    uint8_t anyButtonPressed = 0U;
    uint8_t i;

    s_lastAppTick = now;

    for (i = 0U; i < 10U; i++) {
      uint8_t button = Inp_DiGet(i);
      App_PrepareDigitalForI2c((uint8_t) (APP_I2C_INDEX_BUTTON_0 + i), button);
      if (button != 0U) {
        anyButtonPressed = 1U;
      }
    }

    if (anyButtonPressed != 0U) {
      LCD_SetBacklightTimeout(15000U);
    }

    App_ProcessPower();
    App_ProcessTone();
    App_ProcessAdc();
  }

  if ((LCD_Process() == 0U) && (HAL_GetTick() == s_lastAppTick)) {
    __WFI();
  }
}
