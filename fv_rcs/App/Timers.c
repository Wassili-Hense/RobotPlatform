#include "Timers.h"
#include "tim.h"

inline void SetBackLight(uint16_t pulse)
{
	if (htim14.State == HAL_TIM_STATE_RESET) return;
    __HAL_TIM_SET_COMPARE(&htim14, TIM_CHANNEL_1, pulse);
}

static volatile uint8_t s_toneBusy = 0U;
static volatile uint32_t s_toneStopTick = 0U;

void Tone(uint16_t divider, uint16_t delay_ms)
{
    if (htim1.State == HAL_TIM_STATE_RESET)
    {
        return;
    }

    /* Останавливаем предыдущий тон/паузу и запускаем новый */
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);

    if (delay_ms == 0U)
    {
        s_toneBusy = 0U;
        return;
    }

    s_toneStopTick = HAL_GetTick() + delay_ms;
    s_toneBusy = 1U;

    /* divider == 0 -> пауза (тишина), просто ждём время без PWM */
    if (divider == 0U)
    {
        return;
    }

    __HAL_TIM_SET_AUTORELOAD(&htim1, divider);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, divider / 2U); /* ~50% duty */
    __HAL_TIM_SET_COUNTER(&htim1, 0U);
    HAL_TIM_GenerateEvent(&htim1, TIM_EVENTSOURCE_UPDATE);

    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
}

void Tone_Process(void)
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
