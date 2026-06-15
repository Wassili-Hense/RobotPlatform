#include "App.h"
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"
#include "adc_inputs.h"
#include "buttons.h"
#include "I2cSlave.h"
#include "st7735.h"

#define APP_POWER_OFF_COUNT          1000U
#define APP_ADC_START_PERIOD_MS       100U
#define APP_ADC_FORCE_SEND_MS        2500U
#define APP_ADC_MIN_SEND_PERIOD_MS    100U
#define APP_ADC_DELTA                 15U
#define APP_I2C_ITEM_COUNT            14U

typedef enum
{
    AppI2cType_Digital = 0U,
    AppI2cType_Analog  = 1U
} AppI2cType;

typedef struct
{
    uint8_t type;
    uint8_t channel;
} AppI2cItem;

enum
{
    APP_I2C_INDEX_ST7735_OVERFLOW = 0,
    APP_I2C_INDEX_ADC_X           = 1,
    APP_I2C_INDEX_ADC_Y           = 2,
    APP_I2C_INDEX_BUTTON_0        = 3,
    APP_I2C_INDEX_BUTTON_1        = 4,
    APP_I2C_INDEX_BUTTON_2        = 5,
    APP_I2C_INDEX_BUTTON_3        = 6,
    APP_I2C_INDEX_BUTTON_4        = 7,
    APP_I2C_INDEX_BUTTON_5        = 8,
    APP_I2C_INDEX_BUTTON_6        = 9,
    APP_I2C_INDEX_BUTTON_7        = 10,
    APP_I2C_INDEX_BUTTON_8        = 11,
    APP_I2C_INDEX_BUTTON_9        = 12,
    APP_I2C_INDEX_USB_CONN        = 13
};

/* -------------------------------------------------------------------------- */
/* I2C state                                                                  */
/* -------------------------------------------------------------------------- */

static const AppI2cItem s_i2cMap[APP_I2C_ITEM_COUNT] =
{
    { AppI2cType_Digital, 11U },
    { AppI2cType_Analog,   1U },
    { AppI2cType_Analog,   2U },
    { AppI2cType_Digital,  0U },
    { AppI2cType_Digital,  1U },
    { AppI2cType_Digital,  2U },
    { AppI2cType_Digital,  3U },
    { AppI2cType_Digital,  4U },
    { AppI2cType_Digital,  5U },
    { AppI2cType_Digital,  6U },
    { AppI2cType_Digital,  7U },
    { AppI2cType_Digital,  8U },
    { AppI2cType_Digital,  9U },
    { AppI2cType_Digital, 12U }
};

static volatile uint8_t  s_i2cDirty[APP_I2C_ITEM_COUNT];
static volatile uint16_t s_i2cActualValue[APP_I2C_ITEM_COUNT];
static volatile uint16_t s_i2cSentValue[APP_I2C_ITEM_COUNT];
static volatile uint32_t s_i2cTick[APP_I2C_ITEM_COUNT];

/* -------------------------------------------------------------------------- */
/* App state                                                                  */
/* -------------------------------------------------------------------------- */

static uint16_t s_backlightCounter = 5000U;
static volatile uint8_t s_toneBusy = 0U;
static volatile uint32_t s_toneStopTick = 0U;
static uint32_t s_lastAppTick = 0U;
static uint32_t s_adcStartTick = 0U;

/* -------------------------------------------------------------------------- */
/* I2C callbacks and data preparation                                         */
/* -------------------------------------------------------------------------- */

static void App_PrepareDigitalForI2c(uint8_t index, uint16_t value)
{
    if (index >= APP_I2C_ITEM_COUNT)
    {
        return;
    }

    s_i2cActualValue[index] = value;
    if (s_i2cSentValue[index] != value)
    {
        s_i2cDirty[index] = 1U;
    }
}

static void App_ProcessAnalogForI2c(uint8_t adcChannel, uint16_t value, uint8_t index)
{
    if (index >= APP_I2C_ITEM_COUNT)
    {
        return;
    }

    if (ADC_isChanged(adcChannel) != 0U)
    {
        s_i2cActualValue[index] = value;
    }

    if (s_i2cDirty[index] == 0U)
    {
        uint16_t prev = s_i2cSentValue[index];
        uint16_t diff = (s_i2cActualValue[index] >= prev)
                      ? (uint16_t)(s_i2cActualValue[index] - prev)
                      : (uint16_t)(prev - s_i2cActualValue[index]);
        uint32_t dt = HAL_GetTick() - s_i2cTick[index];

        if (((dt > APP_ADC_MIN_SEND_PERIOD_MS) && (diff > APP_ADC_DELTA)) ||
            (dt > APP_ADC_FORCE_SEND_MS))
        {
            s_i2cDirty[index] = 1U;
        }
    }
}

static uint8_t App_I2cRequestCallback(uint8_t *outData)
{
    uint8_t i;
    uint16_t value;
    uint8_t channel;

    if (outData == 0)
    {
        return 0U;
    }

    for (i = 0U; i < APP_I2C_ITEM_COUNT; i++)
    {
        if (s_i2cDirty[i] == 0U)
        {
            continue;
        }

        value = s_i2cActualValue[i];
        channel = s_i2cMap[i].channel;

        if (s_i2cMap[i].type == AppI2cType_Digital)
        {
            outData[0] = (value != 0U) ? 0xF0U : 0xE0U;
            outData[1] = channel;
        }
        else
        {
            outData[0] = (uint8_t)((channel << 4) | ((value >> 8) & 0x0FU));
            outData[1] = (uint8_t)value;
        }

        s_i2cDirty[i] = 0U;
        s_i2cSentValue[i] = value;
        s_i2cTick[i] = HAL_GetTick();
        return 2U;
    }

    outData[0] = 0U;
    outData[1] = 0U;
    return 2U;
}

static void App_I2cOnReceive(const uint8_t *data, uint16_t size)
{
    (void)data;
    (void)size;
}

static void App_ResetI2cState(void)
{
    uint8_t i;

    for (i = 0U; i < APP_I2C_ITEM_COUNT; i++)
    {
        s_i2cDirty[i] = 0U;
        s_i2cActualValue[i] = 0U;
        s_i2cSentValue[i] = 0U;
        s_i2cTick[i] = 0U;
    }
}

/* -------------------------------------------------------------------------- */
/* Backlight                                                                  */
/* -------------------------------------------------------------------------- */

static void App_SetBacklightLevel(uint16_t pulse)
{
    if (htim14.State == HAL_TIM_STATE_RESET)
    {
        return;
    }

    __HAL_TIM_SET_COMPARE(&htim14, TIM_CHANNEL_1, pulse);
}

static void App_ProcessBacklight(void)
{
    if (s_backlightCounter > 0U)
    {
        App_SetBacklightLevel((s_backlightCounter < 1024U) ? (s_backlightCounter >> 3) : 127U);
        s_backlightCounter--;
    }
}

/* -------------------------------------------------------------------------- */
/* Tone                                                                       */
/* -------------------------------------------------------------------------- */

void Tone(uint16_t divider, uint16_t delay_ms)
{
    if (htim1.State == HAL_TIM_STATE_RESET)
    {
        return;
    }

    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);

    if (delay_ms == 0U)
    {
        s_toneBusy = 0U;
        return;
    }

    s_toneStopTick = HAL_GetTick() + delay_ms;
    s_toneBusy = 1U;

    if (divider == 0U)
    {
        return;
    }

    __HAL_TIM_SET_AUTORELOAD(&htim1, divider);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, divider / 2U);
    __HAL_TIM_SET_COUNTER(&htim1, 0U);
    HAL_TIM_GenerateEvent(&htim1, TIM_EVENTSOURCE_UPDATE);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
}

static void App_ProcessTone(void)
{
    if (s_toneBusy == 0U)
    {
        return;
    }

    if ((int32_t)(HAL_GetTick() - s_toneStopTick) >= 0)
    {
        HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
        s_toneBusy = 0U;
    }
}

uint8_t Tone_IsBusy(void)
{
    return s_toneBusy;
}

/* -------------------------------------------------------------------------- */
/* Power                                                                      */
/* -------------------------------------------------------------------------- */

static void App_ProcessPower(void)
{
    static uint16_t offCounter = 0U;

    if (ADC_V < 950U)
    {
        if (offCounter >= APP_POWER_OFF_COUNT)
        {
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);
        }
        else
        {
            offCounter++;
            if (offCounter == (APP_POWER_OFF_COUNT / 2U))
            {
                Tone(500U, 60U);
            }
        }
    }
    else if (s_i2cActualValue[APP_I2C_INDEX_BUTTON_0] != 0U)
    {
        offCounter++;
        if (offCounter > APP_POWER_OFF_COUNT)
        {
            if (offCounter == (APP_POWER_OFF_COUNT + 2U))
            {
                Tone(500U, 10U);
            }

            if ((offCounter * 3U) > (APP_POWER_OFF_COUNT * 4U))
            {
                offCounter = APP_POWER_OFF_COUNT + 1U;
            }
        }
        else
        {
            s_backlightCounter = 15000U;
        }
    }
    else if (offCounter > 0U)
    {
        if (offCounter >= APP_POWER_OFF_COUNT)
        {
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);
        }
        else
        {
            offCounter--;
        }
    }
}

/* -------------------------------------------------------------------------- */
/* ADC service                                                                */
/* -------------------------------------------------------------------------- */

static void App_ProcessAdc(void)
{
    uint32_t tick = HAL_GetTick();

    if ((tick - s_adcStartTick) >= APP_ADC_START_PERIOD_MS)
    {
        s_adcStartTick = tick;
        AdcInputs_Start();
    }
    else
    {
        App_ProcessAnalogForI2c(ADC_INPUT_CH_X, ADC_X, APP_I2C_INDEX_ADC_X);
        App_ProcessAnalogForI2c(ADC_INPUT_CH_Y, ADC_Y, APP_I2C_INDEX_ADC_Y);

        if (ADC_isChanged(ADC_INPUT_CH_U) != 0U)
        {
            App_PrepareDigitalForI2c(APP_I2C_INDEX_USB_CONN, (ADC_U > 1000U) ? 1U : 0U);
        }
    }
}


/* -------------------------------------------------------------------------- */
/* App lifecycle                                                              */
/* -------------------------------------------------------------------------- */

void App_Init(void)
{
    s_lastAppTick = HAL_GetTick();
    s_adcStartTick = s_lastAppTick - APP_ADC_START_PERIOD_MS;
    App_ResetI2cState();

    AdcInputs_Init();
    Buttons_Init();
    I2cSlave_Init(&hi2c1, App_I2cRequestCallback, App_I2cOnReceive);
    ST7735_Init();

    AdcInputs_Start();
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);  /* Power ON latch */
    App_SetBacklightLevel(63U);

    Tone(757U, 45U);
    HAL_Delay(65U);

    ST7735_Clear(ST7735_BLACK);
    while (Buttons_Get(0U) != 0U)
    {
        App_ProcessTone();
        if (ST7735_NeedsProcess() != 0U)
        {
            ST7735_Process();
        }
        __WFI();
    }

    Tone(636U, 60U);
    HAL_Delay(90U);
    Tone(476U, 90U);
}

void App_Run(void)
{
    uint32_t now = HAL_GetTick();

    if (now != s_lastAppTick)
    {
        s_lastAppTick = now;
        uint8_t st_busy = (ST7735_GetQueueFill() > (ST7735_QUEUE_SIZE - 2U)) ? 1U : 0U;
        /* 1 ms tasks */
        App_PrepareDigitalForI2c(APP_I2C_INDEX_ST7735_OVERFLOW, st_busy);
        App_PrepareDigitalForI2c(APP_I2C_INDEX_BUTTON_0, Buttons_Get(0U));
        App_PrepareDigitalForI2c(APP_I2C_INDEX_BUTTON_1, Buttons_Get(1U));
        App_PrepareDigitalForI2c(APP_I2C_INDEX_BUTTON_2, Buttons_Get(2U));
        App_PrepareDigitalForI2c(APP_I2C_INDEX_BUTTON_3, Buttons_Get(3U));
        App_PrepareDigitalForI2c(APP_I2C_INDEX_BUTTON_4, Buttons_Get(4U));
        App_PrepareDigitalForI2c(APP_I2C_INDEX_BUTTON_5, Buttons_Get(5U));
        App_PrepareDigitalForI2c(APP_I2C_INDEX_BUTTON_6, Buttons_Get(6U));
        App_PrepareDigitalForI2c(APP_I2C_INDEX_BUTTON_7, Buttons_Get(7U));
        App_PrepareDigitalForI2c(APP_I2C_INDEX_BUTTON_8, Buttons_Get(8U));
        App_PrepareDigitalForI2c(APP_I2C_INDEX_BUTTON_9, Buttons_Get(9U));
        App_ProcessPower();
        App_ProcessTone();
        App_ProcessBacklight();
        App_ProcessAdc();

        if(st_busy){  // for test
            Tone(636U, 60U);
        }
    }

    if (ST7735_NeedsProcess() != 0U)
    {
        ST7735_Process();
    }
    else
    {
        __WFI();
    }
}
