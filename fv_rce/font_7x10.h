#ifndef __FONT_7X10_H__
#define __FONT_7X10_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define FONT_7X10_WIDTH       7U
#define FONT_7X10_HEIGHT      10U
#define FONT_7X10_SPACING     1U
#define FONT_7X10_GLYPH_ROWS  FONT_7X10_HEIGHT
#define FONT_FIRST_CHAR  32U
#define FONT_LAST_CHAR   126U
#define FONT_CHAR_COUNT  (FONT_LAST_CHAR - FONT_FIRST_CHAR + 1U)

extern const uint8_t Font7x10[];

#ifdef __cplusplus
}
#endif

#endif
