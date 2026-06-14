#ifndef BUTTONS_H
#define BUTTONS_H

#include "main.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void Buttons_Init(void);
uint8_t Buttons_Get(uint8_t ch);

#ifdef __cplusplus
}
#endif

#endif /* BUTTONS_H */
