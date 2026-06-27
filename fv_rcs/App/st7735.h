#ifndef ST7735_H
#define ST7735_H

#include "main.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define LCD_WIDTH   160U
#define LCD_HEIGHT   80U

/* RGB565 colors */
#define LCD_BLACK   0x0000U
#define LCD_GRAY    0x4208U
#define LCD_WHITE   0xFFFFU
#define LCD_RED     0xF800U
#define LCD_ORANGE  0xFD20U
#define LCD_YELLOW  0xFFE0U
#define LCD_GREEN   0x07E0U
#define LCD_CYAN    0x07FFU
#define LCD_BLUE    0x001FU
#define LCD_MAGENTA 0xF81FU

/* Queue/status callback value bits:
 * bit7 = 1 -> after push only 2 free slots left
 * bit7 = 0 -> after pop 3 free slots available or queue reset
 * bit6 = current backlight status (1 = on, 0 = off)
 */
typedef void (*LCD_QueueCallback)(uint8_t value);

void LCD_Init(LCD_QueueCallback cb);

/*
 * Возвращает 1, если есть незавершённая команда и можно сразу вызвать LCD_Process().
 * Если DMA ещё занята, возвращает 0.
 */
uint8_t LCD_Process(void);

void LCD_SetBacklightTimeout(uint32_t timeout_ms);
void LCD_SetBacklightLevel(uint8_t level_0_127);
void LCD_SetBackgroundColor(uint16_t color);

/*
 * Команды ставятся в очередь и сразу возвращают управление.
 * Возврат:
 *   1 - команда поставлена в очередь
 *   0 - очередь заполнена / некорректные параметры
 */
/* Сбрасывает очередь и очищает только центральную область, не затрагивая Progress Bars. */
uint8_t LCD_Clear(uint16_t color);
uint8_t LCD_FillRect(uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint16_t color);
uint8_t LCD_DrawText(uint8_t x, uint8_t y, const char *text, uint16_t color);
/* x,y - центр маркера; idx - индекс маркера */
uint8_t LCD_DrawMarker(uint8_t x, uint8_t y, uint8_t idx, uint16_t color);

/* index: 0 - antenna, 1 - connect, value: 0..3 */
uint8_t LCD_DrawIndicator(uint8_t index, uint8_t value);

/* index: [0 .. 3], value: 0..64 */
uint8_t LCD_DrawProgressBar(uint8_t index, uint8_t value);

#ifdef __cplusplus
}
#endif

#endif
