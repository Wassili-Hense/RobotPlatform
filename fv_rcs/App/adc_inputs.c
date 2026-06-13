#include "adc_inputs.h"
#include <string.h>

extern ADC_HandleTypeDef hadc;
extern TIM_HandleTypeDef htim17;

volatile uint16_t ADC_V = 0;
volatile uint16_t ADC_X = 0;
volatile uint16_t ADC_Y = 0;
volatile uint16_t ADC_U = 0;

/* DMA буфер */
static uint16_t s_dma[4];

/* История 5 значений на канал */
static uint16_t s_hist_v[5];
static uint16_t s_hist_x[5];
static uint16_t s_hist_y[5];
static uint16_t s_hist_u[5];

static volatile uint8_t s_adcBusy = 0;
static uint8_t s_initialized = 0;

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

    for (i = 0; i < 5; i++)
    {
        s_hist_v[i] = s_dma[0];
        s_hist_x[i] = s_dma[1];
        s_hist_y[i] = s_dma[2];
        s_hist_u[i] = s_dma[3];
    }

    ADC_V = s_dma[0];
    ADC_X = s_dma[1];
    ADC_Y = s_dma[2];
    ADC_U = s_dma[3];

    s_initialized = 1;
}

static void UpdateFilteredValues(void)
{
    if (!s_initialized)
    {
        InitHistoryFromFirstSample();
        return;
    }

    HistShiftAppend(s_hist_v, s_dma[0]);
    HistShiftAppend(s_hist_x, s_dma[1]);
    HistShiftAppend(s_hist_y, s_dma[2]);
    HistShiftAppend(s_hist_u, s_dma[3]);

    ADC_V = Median5(s_hist_v);
    ADC_X = Median5(s_hist_x);
    ADC_Y = Median5(s_hist_y);
    ADC_U = Median5(s_hist_u);
}

/* ---------------- public ---------------- */

void AdcInputs_Init(void)
{
    memset((void*)s_dma, 0, sizeof(s_dma));
    memset((void*)s_hist_v, 0, sizeof(s_hist_v));
    memset((void*)s_hist_x, 0, sizeof(s_hist_x));
    memset((void*)s_hist_y, 0, sizeof(s_hist_y));
    memset((void*)s_hist_u, 0, sizeof(s_hist_u));

    ADC_V = 0;
    ADC_X = 0;
    ADC_Y = 0;
    ADC_U = 0;

    s_adcBusy = 0;
    s_initialized = 0;

    HAL_TIM_Base_Start_IT(&htim17);
}

/* ---------------- HAL callbacks ---------------- */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM17)
    {
        if (s_adcBusy == 0U)
        {
            s_adcBusy = 1U;
            HAL_ADC_Start_DMA(&hadc, (uint32_t*)s_dma, 4);
        }
    }
}

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
