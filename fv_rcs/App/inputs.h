#ifndef INPUTS_H_
#define INPUTS_H_

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

void Inp_Init(void);
void Inp_AdcStart(void);
uint8_t Inp_AdcisChanged(uint8_t channel);
uint8_t Inp_DiGet(uint8_t ch);

#endif /* INPUTS_H_ */
