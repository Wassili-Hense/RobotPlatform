#ifndef ST7735_H
#define ST7735_H

#include "main.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Реальный размер видимой области для текущего модуля */
#define ST7735_WIDTH   160U
#define ST7735_HEIGHT   80U

/* Размер очереди команд */
#define ST7735_QUEUE_SIZE 24U

/* RGB565 colors */
#define ST7735_BLACK   0x0000U
#define ST7735_WHITE   0xFFFFU
#define ST7735_RED     0xF800U
#define ST7735_GREEN   0x07E0U
#define ST7735_BLUE    0x001FU
#define ST7735_YELLOW  0xFFE0U
#define ST7735_CYAN    0x07FFU
#define ST7735_MAGENTA 0xF81FU

void ST7735_Init(void);

/* Вызывать в главном цикле */
void ST7735_Process(void);

/*
 * Команды ставятся в очередь и сразу возвращают управление.
 * Возврат:
 *   1 - команда поставлена в очередь
 *   0 - очередь заполнена / некорректные параметры
 */
uint8_t ST7735_Clear(uint16_t color);
uint8_t ST7735_FillRect(uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint16_t color);
uint8_t ST7735_FillCircle(uint8_t x0, uint8_t y0, uint8_t r, uint16_t color);
uint8_t ST7735_DrawText(uint8_t x, uint8_t y, const char* text, uint16_t color, uint16_t bgColor);

/*
 * Возвращает количество команд, ожидающих в очереди.
 * Активная выполняемая команда в это число не входит.
 */
uint8_t ST7735_GetQueueFill(void);

#ifdef __cplusplus
}
#endif

#endif /* ST7735_H */
