#ifndef ADC_INPUTS_H
#define ADC_INPUTS_H

#include "main.h"
#include <stdint.h>

#define ADC_INPUT_CH_V   0U
#define ADC_INPUT_CH_X   1U
#define ADC_INPUT_CH_Y   2U
#define ADC_INPUT_CH_U   3U
#define ADC_INPUT_COUNT  4U

extern volatile uint16_t ADC_V;
extern volatile uint16_t ADC_X;
extern volatile uint16_t ADC_Y;
extern volatile uint16_t ADC_U;

void AdcInputs_Init(void);
void AdcInputs_Start(void);
uint8_t ADC_isChanged(uint8_t channel);

#endif // ADC_INPUTS_H
