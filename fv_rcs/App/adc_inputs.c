#include "adc_inputs.h"
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
static uint8_t s_initialized = 0U;
static volatile uint8_t s_changedMask = 0U;

/* ---------------- helpers ---------------- */
static void HistShiftAppend(uint16_t hist[5], uint16_t value)
{
    hist[0] = hist[1];
    hist[1] = hist[2];
    hist[2] = hist[3];
    hist[3] = hist[4];
    hist[4] = value;
}

static uint16_t Median5(const uint16_t in[5])
{
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

static void InitHistoryFromFirstSample(void)
{
    uint8_t i;

    for (i = 0U; i < 5U; i++)
    {
        s_hist_v[i] = s_dma[ADC_INPUT_CH_V];
        s_hist_x[i] = s_dma[ADC_INPUT_CH_X];
        s_hist_y[i] = s_dma[ADC_INPUT_CH_Y];
        s_hist_u[i] = s_dma[ADC_INPUT_CH_U];
    }

    ADC_V = s_dma[ADC_INPUT_CH_V];
    ADC_X = s_dma[ADC_INPUT_CH_X];
    ADC_Y = s_dma[ADC_INPUT_CH_Y];
    ADC_U = s_dma[ADC_INPUT_CH_U];

    s_changedMask = (1U << ADC_INPUT_CH_V) |
                    (1U << ADC_INPUT_CH_X) |
                    (1U << ADC_INPUT_CH_Y) |
                    (1U << ADC_INPUT_CH_U);

    s_initialized = 1U;
}

static void UpdateFilteredValues(void)
{
    uint16_t newV;
    uint16_t newX;
    uint16_t newY;
    uint16_t newU;
    uint8_t changedMask = 0U;

    if (s_initialized == 0U)
    {
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

    if (ADC_V != newV)
    {
        ADC_V = newV;
        changedMask |= (1U << ADC_INPUT_CH_V);
    }

    if (ADC_X != newX)
    {
        ADC_X = newX;
        changedMask |= (1U << ADC_INPUT_CH_X);
    }

    if (ADC_Y != newY)
    {
        ADC_Y = newY;
        changedMask |= (1U << ADC_INPUT_CH_Y);
    }

    if (ADC_U != newU)
    {
        ADC_U = newU;
        changedMask |= (1U << ADC_INPUT_CH_U);
    }

    s_changedMask |= changedMask;
}

/* ---------------- public ---------------- */
void AdcInputs_Init(void)
{
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
    s_initialized = 0U;
    s_changedMask = 0U;
}

void AdcInputs_Start(void)
{
    if (s_adcBusy != 0U)
    {
        return;
    }

    s_adcBusy = 1U;
    HAL_ADC_Start_DMA(&hadc, (uint32_t*)s_dma, ADC_INPUT_COUNT);
}

uint8_t ADC_isChanged(uint8_t channel)
{
    uint8_t mask;
    uint8_t changed;

    if (channel >= ADC_INPUT_COUNT)
    {
        return 0U;
    }

    mask = (uint8_t)(1U << channel);
    changed = (s_changedMask & mask) ? 1U : 0U;

    if (changed != 0U)
    {
        s_changedMask &= (uint8_t)(~mask);
    }

    return changed;
}

/* ---------------- HAL callbacks ---------------- */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadcHandle)
{
    if (hadcHandle->Instance == ADC1)
    {
        HAL_ADC_Stop_DMA(hadcHandle);
        UpdateFilteredValues();
        s_adcBusy = 0U;
    }
}

void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadcHandle)
{
    if (hadcHandle->Instance == ADC1)
    {
        HAL_ADC_Stop_DMA(hadcHandle);
        s_adcBusy = 0U;
    }
}
