
#ifndef TIMERS_H_
#define TIMERS_H_

#include "main.h"
#include <stdint.h>

void SetBackLight(uint16_t pulse);

void Tone(uint16_t divider, uint16_t delay_ms);
void Tone_Process(void);
uint8_t Tone_IsBusy(void);

#endif /* TIMERS_H_ */
