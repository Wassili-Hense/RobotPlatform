/*
 * App.h
 *
 *  Created on: 13.06.2026
 *      Author: hensew
 */

#ifndef APP_H_
#define APP_H_

#include <stdint.h>

void App_Init(void);
void App_Run(void);
void Tone(uint16_t divider, uint16_t delay_ms);


#endif /* APP_H_ */
