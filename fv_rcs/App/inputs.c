#include "inputs.h"
#include "gpio.h"
#include <string.h>

extern ADC_HandleTypeDef hadc;

volatile uint16_t ADC_V = 0;
volatile uint16_t ADC_X = 0;
volatile uint16_t ADC_Y = 0;
volatile uint16_t ADC_U = 0;

/* DMA буфер */
static uint16_t s_dma[ADC_INPUT_COUNT];

/* История 5 значений на канал */
static uint16_t s_hist_v[5];
static uint16_t s_hist_x[5];
static uint16_t s_hist_y[5];
static uint16_t s_hist_u[5];

static volatile uint8_t s_adcBusy = 0U;
static volatile uint32_t s_adcStartTick = 0U;
static uint32_t s_adcLastStartTick = 0U;

static uint8_t s_initialized = 0U;
static volatile uint8_t s_changedMask = 0U;

/* ---------------- Digital Inputs ---------------- */
#define BUTTON_COUNT         10U
#define DEBOUNCE_TICKS       20U  /* 20 ms при вызове Buttons_Get(ch) каждые 1 мс для данного канала */
#define ADC_TIMEOUT_MS        2U
#define ADC_START_PERIOD_MS  25U

typedef struct {
  GPIO_TypeDef *port;
  uint16_t pin;
} ButtonPin;

static const ButtonPin s_buttonPins[BUTTON_COUNT] = {
    { GPIOA, GPIO_PIN_12 }, /* Button ON */
    { GPIOB, GPIO_PIN_10 }, /* Button Fire */
    { GPIOA, GPIO_PIN_6  }, /* Button A */
    { GPIOF, GPIO_PIN_1  }, /* Button B */
    { GPIOA, GPIO_PIN_5  }, /* Button C */
    { GPIOC, GPIO_PIN_15 }, /* Button D */
    { GPIOB, GPIO_PIN_0  }, /* Button UP */
    { GPIOA, GPIO_PIN_7  }, /* Button Down */
    { GPIOF, GPIO_PIN_7  }, /* Button Ok */
    { GPIOC, GPIO_PIN_13 }  /* Button Back */
};

static uint8_t s_integrator[BUTTON_COUNT];
static uint8_t s_state[BUTTON_COUNT];

/* ---------------- helpers ---------------- */
static void HistShiftAppend(uint16_t hist[5], uint16_t value) {
  hist[0] = hist[1];
  hist[1] = hist[2];
  hist[2] = hist[3];
  hist[3] = hist[4];
  hist[4] = value;
}

static uint16_t Median5(const uint16_t in[5]) {
  uint16_t a0 = in[0];
  uint16_t a1 = in[1];
  uint16_t a2 = in[2];
  uint16_t a3 = in[3];
  uint16_t a4 = in[4];
  uint16_t t;

  if (a0 > a1) { t = a0; a0 = a1; a1 = t; }
  if (a3 > a4) { t = a3; a3 = a4; a4 = t; }
  if (a0 > a3) { t = a0; a0 = a3; a3 = t; }
  if (a1 > a4) { t = a1; a1 = a4; a4 = t; }
  if (a1 > a2) { t = a1; a1 = a2; a2 = t; }
  if (a2 > a3) { t = a2; a2 = a3; a3 = t; }
  if (a1 > a2) { t = a1; a1 = a2; a2 = t; }

  return a2;
}

static void InitHistoryFromFirstSample(void) {
  uint8_t i;

  for (i = 0U; i < 5U; i++) {
    s_hist_v[i] = s_dma[ADC_INPUT_CH_V];
    s_hist_x[i] = s_dma[ADC_INPUT_CH_X];
    s_hist_y[i] = s_dma[ADC_INPUT_CH_Y];
    s_hist_u[i] = s_dma[ADC_INPUT_CH_U];
  }

  ADC_V = s_dma[ADC_INPUT_CH_V];
  ADC_X = s_dma[ADC_INPUT_CH_X];
  ADC_Y = s_dma[ADC_INPUT_CH_Y];
  ADC_U = s_dma[ADC_INPUT_CH_U];

  s_changedMask = (1U << ADC_INPUT_CH_V)
                | (1U << ADC_INPUT_CH_X)
                | (1U << ADC_INPUT_CH_Y)
                | (1U << ADC_INPUT_CH_U);
  s_initialized = 1U;
}

static void UpdateFilteredValues(void) {
  uint16_t newV;
  uint16_t newX;
  uint16_t newY;
  uint16_t newU;
  uint8_t changedMask = 0U;

  if (s_initialized == 0U) {
    InitHistoryFromFirstSample();
    return;
  }

  HistShiftAppend(s_hist_v, s_dma[ADC_INPUT_CH_V]);
  HistShiftAppend(s_hist_x, s_dma[ADC_INPUT_CH_X]);
  HistShiftAppend(s_hist_y, s_dma[ADC_INPUT_CH_Y]);
  HistShiftAppend(s_hist_u, s_dma[ADC_INPUT_CH_U]);

  newV = Median5(s_hist_v);
  newX = Median5(s_hist_x);
  newY = Median5(s_hist_y);
  newU = Median5(s_hist_u);

  if (ADC_V != newV) {
    ADC_V = newV;
    changedMask |= (1U << ADC_INPUT_CH_V);
  }
  if (ADC_X != newX) {
    ADC_X = newX;
    changedMask |= (1U << ADC_INPUT_CH_X);
  }
  if (ADC_Y != newY) {
    ADC_Y = newY;
    changedMask |= (1U << ADC_INPUT_CH_Y);
  }
  if (ADC_U != newU) {
    ADC_U = newU;
    changedMask |= (1U << ADC_INPUT_CH_U);
  }

  s_changedMask |= changedMask;
}

static uint8_t Buttons_ReadRaw(uint8_t ch) {
  GPIO_PinState pinState;

  pinState = HAL_GPIO_ReadPin(s_buttonPins[ch].port, s_buttonPins[ch].pin);
  return (pinState == GPIO_PIN_RESET) ? 1U : 0U; /* active low */
}

/* ---------------- public ---------------- */
void Inp_Init(void) {
  uint8_t i;
  uint8_t raw;

  memset((void*)s_dma, 0, sizeof(s_dma));
  memset((void*)s_hist_v, 0, sizeof(s_hist_v));
  memset((void*)s_hist_x, 0, sizeof(s_hist_x));
  memset((void*)s_hist_y, 0, sizeof(s_hist_y));
  memset((void*)s_hist_u, 0, sizeof(s_hist_u));

  ADC_V = 0U;
  ADC_X = 0U;
  ADC_Y = 0U;
  ADC_U = 0U;

  s_adcBusy = 0U;
  s_adcStartTick = 0U;
  s_adcLastStartTick = 0U;
  s_initialized = 0U;
  s_changedMask = 0U;

  for (i = 0U; i < BUTTON_COUNT; i++) {
    raw = Buttons_ReadRaw(i);
    s_state[i] = raw;
    s_integrator[i] = raw ? DEBOUNCE_TICKS : 0U;
  }
}

uint8_t Inp_AdcEnsureStarted(void) {
  uint32_t now = HAL_GetTick();

  if (s_adcBusy != 0U) {
    if ((uint32_t)(now - s_adcStartTick) <= ADC_TIMEOUT_MS) {
      return s_adcBusy;
    }

    HAL_ADC_Stop_DMA(&hadc);
    s_adcBusy = 0U;
  }

  if ((uint32_t)(now - s_adcLastStartTick) < ADC_START_PERIOD_MS) {
    return s_adcBusy;
  }

  s_adcStartTick = now;
  s_adcLastStartTick = now;
  s_adcBusy = 1U;
  HAL_ADC_Start_DMA(&hadc, (uint32_t*)s_dma, ADC_INPUT_COUNT);

  return s_adcBusy;
}

uint8_t Inp_AdcisChanged(uint8_t channel) {
  uint8_t mask;
  uint8_t changed;

  if (channel >= ADC_INPUT_COUNT) {
    return 0U;
  }

  mask = (uint8_t)(1U << channel);
  changed = (s_changedMask & mask) ? 1U : 0U;
  if (changed != 0U) {
    s_changedMask &= (uint8_t)(~mask);
  }

  return changed;
}

uint8_t Inp_DiGet(uint8_t ch) {
  uint8_t raw;

  if (ch >= BUTTON_COUNT) {
    return 0U;
  }

  raw = Buttons_ReadRaw(ch);
  if (raw != 0U) {
    if (s_integrator[ch] < DEBOUNCE_TICKS) {
      s_integrator[ch]++;
    }
  } else {
    if (s_integrator[ch] > 0U) {
      s_integrator[ch]--;
    }
  }

  if (s_integrator[ch] == 0U) {
    s_state[ch] = 0U;
  } else if (s_integrator[ch] >= DEBOUNCE_TICKS) {
    s_integrator[ch] = DEBOUNCE_TICKS;
    s_state[ch] = 1U;
  }

  return s_state[ch];
}

/* ---------------- HAL callbacks ---------------- */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadcHandle) {
  if (hadcHandle->Instance == ADC1) {
    HAL_ADC_Stop_DMA(hadcHandle);
    UpdateFilteredValues();
    s_adcBusy = 0U;
  }
}

void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadcHandle) {
  if (hadcHandle->Instance == ADC1) {
    HAL_ADC_Stop_DMA(hadcHandle);
    s_adcBusy = 0U;
  }
}
