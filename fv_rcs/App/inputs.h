#ifndef INPUTS_H
#define INPUTS_H

#include "main.h"
#include "adc.h"
#include <stdint.h>

#define ADC_INPUT_CH_V   0U
#define ADC_INPUT_CH_X   1U
#define ADC_INPUT_CH_Y   2U
#define ADC_INPUT_CH_U   3U
#define ADC_INPUT_COUNT  4U
#define INPUT_BUTTON_COUNT 10U

void Inp_Init(void);
uint8_t Inp_AdcEnsureStarted(void);
uint8_t Inp_AdcisChanged(uint8_t channel);
uint16_t Inp_AiGet(uint8_t channel);
uint8_t Inp_DiGet(uint8_t ch);

#endif /* INPUTS_H */
