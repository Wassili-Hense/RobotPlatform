#ifndef __ST7735_FONT_7X10_H__
#define __ST7735_FONT_7X10_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ST7735_FONT_7X10_WIDTH       7U
#define ST7735_FONT_7X10_HEIGHT      10U
#define ST7735_FONT_7X10_SPACING     1U
#define ST7735_FONT_7X10_FIRST_CHAR  32U
#define ST7735_FONT_7X10_LAST_CHAR   126U
#define ST7735_FONT_7X10_GLYPH_ROWS  10U
#define ST7735_FONT_7X10_CHAR_COUNT  (ST7735_FONT_7X10_LAST_CHAR - ST7735_FONT_7X10_FIRST_CHAR + 1U)

extern const uint8_t Font7x10[];

#ifdef __cplusplus
}
#endif

#endif
