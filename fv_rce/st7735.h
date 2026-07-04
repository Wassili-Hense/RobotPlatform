#ifndef ST7735_H
#define ST7735_H

#include <Arduino.h>
#include <stdint.h>
#include "driver/spi_master.h"

#ifdef __cplusplus
extern "C" {
#endif
// Hardware
#define LCD_SPI_HOST VSPI_HOST
#define LCD_PIN_MOSI 23
#define LCD_PIN_MISO -1
#define LCD_PIN_SCLK 18
#define LCD_PIN_CS   5
#define LCD_PIN_DC   16
#define LCD_PIN_RST  17
#define LCD_SPI_CLOCK_HZ 24000000UL

#define LCD_WIDTH   160U
#define LCD_HEIGHT  80U

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


void LCD_Init(void);
uint8_t LCD_Process(void);
void LCD_SetBackgroundColor(uint16_t color);
void LCD_Clear(uint16_t color);
void LCD_FillRect(uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint16_t color);
void LCD_DrawText(uint8_t x, uint8_t y, uint16_t color, const char *text);
void LCD_DrawMarker(uint8_t x, uint8_t y, uint8_t idx, uint16_t color);
void LCD_DrawIndicator(uint8_t index, uint8_t value);
void LCD_DrawProgressBar(uint8_t index, uint8_t value);

#ifdef __cplusplus
}
#endif

#endif
