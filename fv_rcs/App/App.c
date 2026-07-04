#include "App.h"
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "gpio.h"
#include "I2cSlave.h"
#include "inputs.h"

#define APP_I2C_PACKET_SIZE             8U
#define APP_I2C_STATUS_BUTTONS_MASK_LO  0xFFU
#define APP_I2C_STATUS_BUTTONS_MASK_HI  0x03U
#define APP_I2C_STATUS_USB_MASK         0x10U
#define APP_I2C_STATUS_LCD_BL_MASK      0x40U
#define APP_I2C_ADC_VALUE_MASK          0x0FFFU
#define APP_I2C_ADC_CHANGED_MASK        0x8000U
#define APP_POWER_OFF_COUNT          1000U
#define APP_MELODY_POWER_ON          1U
#define APP_MELODY_CONNECTED         2U
#define APP_MELODY_DISCONNECTED      3U

enum {
  APP_RX_CMD_LCD_SET_BL_TIMEOUT = 0x03,
  APP_RX_CMD_LCD_SET_BL_LEVEL = 0x04,
  APP_RX_CMD_TONE = 0x07,
  APP_RX_CMD_MELODY = 0x08,
  APP_RX_CMD_POWER_OFF = 0x0F
};

/* -------------------------------------------------------------------------- */
/* App state                                                                  */
/* -------------------------------------------------------------------------- */
static volatile uint8_t s_toneBusy = 0U;
static volatile uint32_t s_toneStopTick = 0U;
static const uint16_t *s_melodyData = 0;
static uint8_t s_melodyLength = 0U;
static uint8_t s_melodyIndex = 0U;
static uint32_t s_lastAppTick = 0U;
static volatile uint8_t s_i2cPacket[APP_I2C_PACKET_SIZE] = { 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U };

void Tone(uint16_t divider, uint16_t delay_ms);
void App_PlayMelody(uint8_t melody);
static void App_StartTone(uint16_t divider, uint16_t delay_ms);


/* -------------------------------------------------------------------------- */
/* I2C packet helpers                                                         */
/* -------------------------------------------------------------------------- */
static void App_I2cPacketSetWord(uint8_t offset, uint16_t value) {
  s_i2cPacket[offset] = (uint8_t) (value & 0xFFU);
  s_i2cPacket[offset + 1U] = (uint8_t) (value >> 8);
}

static void App_SetButtonsForI2c(uint16_t buttons) {
  s_i2cPacket[0] = (uint8_t) (buttons & APP_I2C_STATUS_BUTTONS_MASK_LO);
  s_i2cPacket[1] = (uint8_t) ((s_i2cPacket[1] & (uint8_t) ~APP_I2C_STATUS_BUTTONS_MASK_HI)
      | ((buttons >> 8) & APP_I2C_STATUS_BUTTONS_MASK_HI));
}
static void App_SetFlagForI2c(uint8_t mask, uint8_t value){
  s_i2cPacket[1] = (uint8_t) ((s_i2cPacket[1] & (uint8_t) ~mask) | (value ? mask : 0));
}
static uint8_t App_UpdateAdcWordForI2c(uint8_t adcChannel, uint8_t packetOffset) {
  uint16_t value = (uint16_t) Inp_AiGet(adcChannel) & APP_I2C_ADC_VALUE_MASK;
  uint8_t ch = Inp_AdcisChanged(adcChannel) != 0U;
  if (ch) {
    value |= APP_I2C_ADC_CHANGED_MASK;
  }
  App_I2cPacketSetWord(packetOffset, value);
  return ch;
}
/*---------------------- Backlight -------------------------*/
static uint8_t s_backlightLevel = 32U;
static uint8_t s_backlightApplied = 0U;
static uint32_t s_backlightOffTick = 0U;


static void ST7735_UpdateBacklightPwm(void) {
  uint32_t now = HAL_GetTick();
  uint32_t remaining = 0U;
  uint8_t pwm;
  uint8_t prevApplied = s_backlightApplied;

  if ((int32_t) (s_backlightOffTick - now) > 0) {
    remaining = s_backlightOffTick - now;
  }
  pwm = (uint8_t) (((remaining >> 3) < s_backlightLevel) ? (remaining >> 3) : s_backlightLevel);
  if ((pwm != s_backlightApplied) && (htim14.State != HAL_TIM_STATE_RESET)) {
    __HAL_TIM_SET_COMPARE(&htim14, TIM_CHANNEL_1, pwm);
    s_backlightApplied = pwm;
  }
  if (prevApplied == 0U && s_backlightApplied > 0U){
	  App_SetFlagForI2c(APP_I2C_STATUS_LCD_BL_MASK, 1);
  } else if(prevApplied > 0U && s_backlightApplied == 0U) {
	  App_SetFlagForI2c(APP_I2C_STATUS_LCD_BL_MASK, 0);
  }
}
void LCD_SetBacklightTimeout(uint32_t timeout_ms) {
  s_backlightOffTick = HAL_GetTick() + timeout_ms;
}

void LCD_SetBacklightLevel(uint8_t level_0_127) {
  if (level_0_127 > 127U) {
    level_0_127 = 127U;
  }
  s_backlightLevel = level_0_127;
}
/* -------------------------------------------------------------------------- */
/* I2C callbacks                                                              */
/* -------------------------------------------------------------------------- */
/* outData length 32 */
static uint8_t App_I2cRequestCallback(uint8_t *outData) {
  if (outData == 0) return 0U;
  outData[0] = s_i2cPacket[0];
  outData[1] = s_i2cPacket[1];
  outData[2] = s_i2cPacket[2];
  outData[3] = s_i2cPacket[3];
  outData[4] = s_i2cPacket[4];
  outData[5] = s_i2cPacket[5];
  outData[6] = s_i2cPacket[6];
  outData[7] = s_i2cPacket[7];
  return APP_I2C_PACKET_SIZE;
}

//static void App_I2cOnReceive(uint8_t *data, uint16_t size) {
static void App_ProcessI2cRx(void) {
  uint8_t data[I2C_SLAVE_RX_SIZE];
  uint8_t size;

  if (I2cSlave_GetNextRxPacket(data, &size) == 0U || size == 0U) return;

  switch (data[0]) {
  case APP_RX_CMD_TONE:
    if (size >= 5U) {
      uint16_t divider = (uint16_t) data[1] | ((uint16_t) data[2] << 8);
      uint16_t delayMs = (uint16_t) data[3] | ((uint16_t) data[4] << 8);
      Tone(divider, delayMs);
    }
    break;
  case APP_RX_CMD_MELODY:
    if (size >= 2U) {
      App_PlayMelody(data[1]);
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

  case APP_RX_CMD_POWER_OFF:
    if (size >= 2 && data[1] == 0xAA) {
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);
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
static const uint16_t s_melodyPowerOn[] =
  { 1515U, 55U, 1276U, 55U, 1012U, 80U };

static const uint16_t s_melodyConnected[] =
  { 1276U, 45U, 1012U, 45U, 851U, 70U };

static const uint16_t s_melodyDisconnected[] =
  { 851U, 55U, 1012U, 55U, 1515U, 90U };

static void App_StartMelody(const uint16_t *data, uint8_t length) {
  s_melodyData = data;
  s_melodyLength = length;
  s_melodyIndex = 0U;
  if ((s_melodyData == 0) || (s_melodyLength == 0U)) {
    App_StartTone(0U, 0U);
    return;
  }
  App_StartTone(s_melodyData[0], s_melodyData[1]);
}

void App_PlayMelody(uint8_t melody) {
  switch (melody) {
  case APP_MELODY_POWER_ON:
    App_StartMelody(s_melodyPowerOn, (uint8_t) (sizeof(s_melodyPowerOn) / sizeof(s_melodyPowerOn[0])));
    break;
  case APP_MELODY_CONNECTED:
    App_StartMelody(s_melodyConnected, (uint8_t) (sizeof(s_melodyConnected) / sizeof(s_melodyConnected[0])));
    break;
  case APP_MELODY_DISCONNECTED:
    App_StartMelody(s_melodyDisconnected, (uint8_t) (sizeof(s_melodyDisconnected) / sizeof(s_melodyDisconnected[0])));
    break;
  default:
    App_StartMelody(0, 0U);
    break;
  }
}

static void App_StartTone(uint16_t divider, uint16_t delay_ms) {
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

void Tone(uint16_t divider, uint16_t delay_ms) {
  s_melodyData = 0;
  s_melodyLength = 0U;
  s_melodyIndex = 0U;
  App_StartTone(divider, delay_ms);
}

static void App_ProcessTone(void) {
  if (s_toneBusy == 0U) return;
  if ((int32_t) (HAL_GetTick() - s_toneStopTick) < 0) return;

  HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
  s_toneBusy = 0U;

  if (s_melodyData == 0) return;
  s_melodyIndex = (uint8_t) (s_melodyIndex + 2U);
  if ((uint8_t) (s_melodyIndex + 1U) >= s_melodyLength) {
    s_melodyData = 0;
    s_melodyLength = 0U;
    s_melodyIndex = 0U;
    return;
  }
  App_StartTone(s_melodyData[s_melodyIndex], s_melodyData[s_melodyIndex + 1U]);
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
/* ADC service                                                                */
/* -------------------------------------------------------------------------- */
static void App_ProcessAdc(void) {
  if (Inp_AdcEnsureStarted()) return;

  App_UpdateAdcWordForI2c(ADC_INPUT_CH_X, 2U);
  App_UpdateAdcWordForI2c(ADC_INPUT_CH_Y, 4U);
  App_UpdateAdcWordForI2c(ADC_INPUT_CH_V, 6U);
  if (Inp_AdcisChanged(ADC_INPUT_CH_U) != 0U) {
    App_SetFlagForI2c(APP_I2C_STATUS_USB_MASK, (Inp_AiGet(ADC_INPUT_CH_U) > 1000U));
  }
}

/* -------------------------------------------------------------------------- */
/* App lifecycle                                                              */
/* -------------------------------------------------------------------------- */
void App_Init(void) {
  App_ResetI2cState();
  Inp_Init();

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET); /* Power ON latch */

  (void) Inp_AdcEnsureStarted();
  I2cSlave_Init(&hi2c1, App_I2cRequestCallback);

  LCD_SetBacklightTimeout(55000U);
  Tone(757U, 45U);
  while (Inp_DiGet(0U) != 0U) {
    App_ProcessTone();
    App_ProcessI2cRx();
    __WFI();
  }
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
      LCD_SetBacklightTimeout(55000U);
    }
    App_ProcessTone();
    App_ProcessAdc();
    App_ProcessPower();
    ST7735_UpdateBacklightPwm();
  }
  App_ProcessI2cRx();
  if (HAL_GetTick() == s_lastAppTick) {
    __WFI();
  }
}
