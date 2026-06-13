#include "buttons.h"
#include "gpio.h"

#define BUTTON_COUNT        10U
#define DEBOUNCE_TICKS      20U   /* 20 ms */

typedef struct
{
    GPIO_TypeDef* port;
    uint16_t pin;
} ButtonPin;

static const ButtonPin s_buttonPins[BUTTON_COUNT] =
{
    {GPIOA, GPIO_PIN_12}, // Button ON
	{GPIOB, GPIO_PIN_10}, // Button Fire
	{GPIOA, GPIO_PIN_6 }, // Button A
	{GPIOF, GPIO_PIN_1 }, // Button B
	{GPIOA, GPIO_PIN_5 }, // Button C
	{GPIOC, GPIO_PIN_15}, // Button D
	{GPIOB, GPIO_PIN_0 }, // Button UP
	{GPIOA, GPIO_PIN_7 }, // Button Down
	{GPIOF, GPIO_PIN_7 }, // Button Ok
	{GPIOC, GPIO_PIN_13}  // Button Back
};

static uint8_t s_integrator[BUTTON_COUNT];
static uint8_t s_state[BUTTON_COUNT];

static uint8_t Buttons_ReadRaw(uint8_t ch)
{
    GPIO_PinState pinState;

    pinState = HAL_GPIO_ReadPin(s_buttonPins[ch].port, s_buttonPins[ch].pin);

    /* active low */
    return (pinState == GPIO_PIN_RESET) ? 1U : 0U;
}

void Buttons_Init(void)
{
    uint8_t i;
    uint8_t raw;

    for (i = 0U; i < BUTTON_COUNT; i++)
    {
        raw = Buttons_ReadRaw(i);

        s_state[i] = raw;
        s_integrator[i] = raw ? DEBOUNCE_TICKS : 0U;
    }
}

void Buttons_Tick1ms(void)
{
    uint8_t i;
    uint8_t raw;
    uint8_t newState;

    for (i = 0U; i < BUTTON_COUNT; i++)
    {
        raw = Buttons_ReadRaw(i);

        if (raw != 0U)
        {
            if (s_integrator[i] < DEBOUNCE_TICKS)
            {
                s_integrator[i]++;
            }
        }
        else
        {
            if (s_integrator[i] > 0U)
            {
                s_integrator[i]--;
            }
        }

        newState = s_state[i];

        if (s_integrator[i] == 0U)
        {
            newState = 0U;
        }
        else if (s_integrator[i] >= DEBOUNCE_TICKS)
        {
            newState = 1U;
            s_integrator[i] = DEBOUNCE_TICKS;
        }

        s_state[i] = newState;
    }
}

uint8_t Buttons_Get(uint8_t ch)
{
    if (ch >= BUTTON_COUNT)
    {
        return 0U;
    }

    return s_state[ch];
}
