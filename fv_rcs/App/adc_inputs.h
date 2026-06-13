#ifndef ADC_INPUTS_H
#define ADC_INPUTS_H

#include "main.h"
#include <stdint.h>

extern volatile uint16_t ADC_V;
extern volatile uint16_t ADC_X;
extern volatile uint16_t ADC_Y;
extern volatile uint16_t ADC_U;

void AdcInputs_Init(void);

#endif // ADC_INPUTS_H
