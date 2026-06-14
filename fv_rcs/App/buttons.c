#include "buttons.h"
#include "gpio.h"

#define BUTTON_COUNT        10U
#define DEBOUNCE_TICKS      20U   /* 20 ms при вызове Buttons_Get(ch) каждые 1 мс для данного канала */

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

uint8_t Buttons_Get(uint8_t ch)
{
    uint8_t raw;

    if (ch >= BUTTON_COUNT)
    {
        return 0U;
    }

    raw = Buttons_ReadRaw(ch);

    if (raw != 0U)
    {
        if (s_integrator[ch] < DEBOUNCE_TICKS)
        {
            s_integrator[ch]++;
        }
    }
    else
    {
        if (s_integrator[ch] > 0U)
        {
            s_integrator[ch]--;
        }
    }

    if (s_integrator[ch] == 0U)
    {
        s_state[ch] = 0U;
    }
    else if (s_integrator[ch] >= DEBOUNCE_TICKS)
    {
        s_integrator[ch] = DEBOUNCE_TICKS;
        s_state[ch] = 1U;
    }

    return s_state[ch];
}
