#include <font_7x10.h>
#include "st7735.h"
#include "spi.h"
#include "gpio.h"
#include "tim.h"
#include <string.h>

#define MAX(a,b) (((a)>(b))?(a):(b))

#define LCD_QUEUE_SIZE 24U
#define ST7735_FONT_GLYPH_INDEX(ch) ((((uint8_t)(ch)) - FONT_FIRST_CHAR) * FONT_7X10_GLYPH_ROWS)
#define ST7735_TEXT_CELL_WIDTH   (FONT_7X10_WIDTH + FONT_7X10_SPACING)
#define ST7735_MAX_TEXT_LEN      21U
#define ST7735_TEXT_STORAGE_LEN  (ST7735_MAX_TEXT_LEN + 1U)
#define ST7735_MARKER_COUNT      11U
#define ST7735_X_OFFSET 0U
#define ST7735_Y_OFFSET 24U
#define ST7735_IO_BUFFER_SIZE     MAX((ST7735_TEXT_CELL_WIDTH * FONT_7X10_HEIGHT * 2U), LCD_HEIGHT*8)
#define ST7735_COLOR_CHUNK_PIXELS  (ST7735_IO_BUFFER_SIZE / 2U)
#define ProgressBar_PB_LEN   64U
#define ProgressBar_PB_TH     3U

typedef enum {
  ST7735_CMD_NONE = 0, ST7735_CMD_FILL_RECT, ST7735_CMD_DRAW_TEXT, ST7735_CMD_DRAW_MARKER
} ST7735_CommandType;

typedef enum {
  ST7735_TX_NONE = 0, ST7735_TX_COLOR_REPEAT, ST7735_TX_BITMAP
} ST7735_TxType;

typedef enum {
  ProgressBar_DIR_RIGHT = 0, ProgressBar_DIR_LEFT, ProgressBar_DIR_UP
} ProgressBar_Dir;

typedef struct {
  uint8_t x0;
  uint8_t y0;
  ProgressBar_Dir dir;
} ProgressBar_Spec;

typedef struct {
  uint8_t x;
  uint8_t y;
  uint8_t w;
  uint8_t h;
  uint16_t color;
} ST7735_FillRectCmd;

typedef struct {
  uint8_t x;
  uint8_t y;
  uint16_t color;
  char text[ST7735_TEXT_STORAGE_LEN];
} ST7735_DrawTextCmd;

typedef struct {
  uint8_t x;
  uint8_t y;
  uint8_t idx;
  uint16_t color;
} ST7735_DrawMarkerCmd;

typedef struct {
  uint8_t type;
  union {
    ST7735_FillRectCmd fillRect;
    ST7735_DrawTextCmd drawText;
    ST7735_DrawMarkerCmd drawMarker;
  } data;
} ST7735_Command;

typedef struct {
  uint8_t busy;
  ST7735_Command cmd;
  struct {
    uint8_t index;
    uint8_t cursorX;
    uint8_t cursorY;
  } text;
} ST7735_ActiveJob;

typedef struct {
  uint8_t type;
  uint16_t remainingPixels;
  uint16_t bitmapBytes;
} ST7735_TxState;

static ST7735_Command s_queue[LCD_QUEUE_SIZE];
static uint8_t s_queueHead = 0U;
static uint8_t s_queueTail = 0U;
static uint8_t s_queueCount = 0U;
static uint8_t s_queueWarn = 0U;
static LCD_QueueCallback s_queueCb = NULL;
static ST7735_ActiveJob s_active;
static ST7735_TxState s_tx;
static uint8_t s_ioBuffer[ST7735_IO_BUFFER_SIZE];
static volatile uint8_t s_spiDmaBusy = 0U;
static volatile uint8_t s_spiDmaError = 0U;
static uint8_t s_backlightLevel = 64U;
static uint8_t s_backlightApplied = 0U;
static uint32_t s_backlightOffTick = 0U;
static uint16_t s_bgColor = LCD_BLACK;
static uint8_t s_InitSeq = 1;
static const ProgressBar_Spec s_progressBars[4] =
  {
    { .x0 = 10U, .y0 = 0U, .dir = ProgressBar_DIR_RIGHT },
    { .x0 = 10U, .y0 = 4U, .dir = ProgressBar_DIR_RIGHT },
    { .x0 = (uint8_t) (LCD_WIDTH / 2U + 6U), .y0 = 0U, .dir = ProgressBar_DIR_RIGHT },
    { .x0 = (uint8_t) (LCD_WIDTH / 2U + 6U), .y0 = 4U, .dir = ProgressBar_DIR_RIGHT }
  };
static uint8_t s_progressBarInitMask = 0U;
static uint8_t s_progressBarPrev[4] = { 0U, 0U, 0U, 0U };

static const uint8_t s_circle_1[] = {1U, 0x01U };
static const uint8_t s_circle_2[] = {2U, 0x03U, 0x03U };
static const uint8_t s_circle_3[] = {3U, 0x07U, 0x07U, 0x07U };
static const uint8_t s_circle_4[] = {4U, 0x06U, 0x0FU, 0x0FU, 0x06U };
static const uint8_t s_circle_5[] = {5U, 0x0EU, 0x1FU, 0x1FU, 0x1FU, 0x0EU };
static const uint8_t s_circle_6[] = {6U, 0x1EU, 0x3FU, 0x3FU, 0x3FU, 0x3FU, 0x1EU };
static const uint8_t s_circle_7[] = {7U, 0x1CU, 0x3EU, 0x7FU, 0x7FU, 0x7FU, 0x3EU, 0x1CU };
static const uint8_t s_circle_8[] = {8U, 0x3CU, 0x7EU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0x7EU, 0x3CU };
static const uint8_t s_antena[]   = {8U, 0xE0U, 0x18U, 0x04U, 0xC2U, 0x32U, 0x11U, 0xC9U, 0xC9U };
static const uint8_t s_battery[]  = {8U, 0x18U, 0x3CU, 0x42U, 0x5AU, 0x5AU, 0x5AU, 0x42U, 0x3CU };
static const uint8_t s_connect[]  = {8U, 0x14U, 0x16U, 0x17U, 0x14U, 0x14U, 0x74U, 0x34U, 0x14U };
static const uint8_t * const s_markers[ST7735_MARKER_COUNT] =
  {
  s_circle_1,
  s_circle_2,
  s_circle_3,
  s_circle_4,
  s_circle_5,
  s_circle_6,
  s_circle_7,
  s_circle_8,
  s_antena,
  s_battery,
  s_connect
  };

static void ST7735_NotifyStatus(void) {
  if (s_queueCb != NULL) {
    uint8_t value = (uint8_t) (s_queueWarn ? 0x80U : 0U);
    if (s_backlightApplied > 0U) value |= 0x40U;
    s_queueCb(value);
  }
}

static const uint8_t *ST7735_GetMarkerGlyph(uint8_t idx) {
  if (idx >= ST7735_MARKER_COUNT) return NULL;
  return s_markers[idx];
}

static void ST7735_Select(void) {
  HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_RESET);
}

static void ST7735_Unselect(void) {
  HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_SET);
}

static void ST7735_DC_Command(void) {
  HAL_GPIO_WritePin(LCD_DC_GPIO_Port, LCD_DC_Pin, GPIO_PIN_RESET);
}

static void ST7735_DC_Data(void) {
  HAL_GPIO_WritePin(LCD_DC_GPIO_Port, LCD_DC_Pin, GPIO_PIN_SET);
}

static void ST7735_Reset(void) {
  HAL_GPIO_WritePin(LCD_RST_GPIO_Port, LCD_RST_Pin, GPIO_PIN_SET);
  HAL_Delay(5U);
  HAL_GPIO_WritePin(LCD_RST_GPIO_Port, LCD_RST_Pin, GPIO_PIN_RESET);
  HAL_Delay(20U);
  HAL_GPIO_WritePin(LCD_RST_GPIO_Port, LCD_RST_Pin, GPIO_PIN_SET);
  HAL_Delay(120U);
}

static void ST7735_WriteCommand(uint8_t cmd) {
  ST7735_Select();
  ST7735_DC_Command();
  (void) HAL_SPI_Transmit(&hspi2, &cmd, 1U, HAL_MAX_DELAY);
  ST7735_Unselect();
}

static void ST7735_WriteData(const uint8_t *data, uint16_t size) {
  ST7735_Select();
  ST7735_DC_Data();
  (void) HAL_SPI_Transmit(&hspi2, (uint8_t*) data, size, HAL_MAX_DELAY);
  ST7735_Unselect();
}

static void ST7735_WriteCommandWithData(uint8_t cmd, const uint8_t *data, uint16_t size) {
  ST7735_Select();
  ST7735_DC_Command();
  (void) HAL_SPI_Transmit(&hspi2, &cmd, 1U, HAL_MAX_DELAY);
  if ((data != NULL) && (size != 0U)) {
    ST7735_DC_Data();
    (void) HAL_SPI_Transmit(&hspi2, (uint8_t*) data, size, HAL_MAX_DELAY);
  }
  ST7735_Unselect();
}

static void ST7735_SetAddressWindow(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1) {
  uint8_t data[4];
  uint16_t ax0 = (uint16_t) (x0 + ST7735_X_OFFSET);
  uint16_t ax1 = (uint16_t) (x1 + ST7735_X_OFFSET);
  uint16_t ay0 = (uint16_t) (y0 + ST7735_Y_OFFSET);
  uint16_t ay1 = (uint16_t) (y1 + ST7735_Y_OFFSET);

  ST7735_WriteCommand(0x2A);
  data[0] = (uint8_t) (ax0 >> 8);
  data[1] = (uint8_t) (ax0 & 0xFFU);
  data[2] = (uint8_t) (ax1 >> 8);
  data[3] = (uint8_t) (ax1 & 0xFFU);
  ST7735_WriteData(data, 4U);

  ST7735_WriteCommand(0x2B);
  data[0] = (uint8_t) (ay0 >> 8);
  data[1] = (uint8_t) (ay0 & 0xFFU);
  data[2] = (uint8_t) (ay1 >> 8);
  data[3] = (uint8_t) (ay1 & 0xFFU);
  ST7735_WriteData(data, 4U);

  ST7735_WriteCommand(0x2C);
}

static uint8_t ST7735_StartDma(uint8_t *data, uint16_t size) {
  s_spiDmaError = 0U;
  if (HAL_SPI_Transmit_DMA(&hspi2, data, size) != HAL_OK) return 0U;
  s_spiDmaBusy = 1U;
  return 1U;
}
/*
 * Пытается восстановить локальный флаг s_spiDmaBusy, если DMA-передача по SPI2 уже фактически завершилась,
 * но HAL_SPI_TxCpltCallback() по какой-то причине не сбросил его.
 *
 * Что делает:
 * - если s_spiDmaBusy == 0, сразу возвращает 1;
 * - если у SPI2 нет DMA TX handle, восстановление невозможно -> 0;
 * - если DMA ещё не в состоянии READY, передача не завершена -> 0;
 * - если SPI всё ещё держит флаг BSY, последние биты ещё передаются -> 0;
 * - если DMA уже READY и SPI не busy, принудительно сбрасывает s_spiDmaBusy
 *   и возвращает 1.
 *
 * Какую проблему решает:
 * В текущем драйвере ST7735_NeedsProcess() возвращает 0, пока s_spiDmaBusy != 0, а LCD_Process() не продвигает очередь/активную команду.
 * Если completion-callback не дошёл, экран может "зависнуть" на последнем кадре, хотя DMA и SPI уже реально освободились.
 * Эта функция устраняет такое ложное busy-состояние.
 */

static uint8_t ST7735_TryRecoverDmaBusy(void) {
  if (s_spiDmaBusy == 0U) return 1U;
  if (hspi2.hdmatx == NULL) return 0U;
  if (HAL_DMA_GetState(hspi2.hdmatx) != HAL_DMA_STATE_READY) return 0U;
  if (__HAL_SPI_GET_FLAG(&hspi2, SPI_FLAG_BSY) != RESET) return 0U;
  s_spiDmaBusy = 0U;
  return 1U;
}

static void ST7735_FillColorBuffer(uint16_t color) {
  uint8_t hi = (uint8_t) (color >> 8);
  uint8_t lo = (uint8_t) (color & 0xFFU);
  uint16_t i;

  for (i = 0U; i < sizeof(s_ioBuffer); i += 2U) {
    s_ioBuffer[i] = hi;
    s_ioBuffer[i + 1U] = lo;
  }
}

static uint8_t ST7735_BeginColorRepeat(uint16_t color, uint16_t pixelCount) {
  uint16_t chunkPixels;

  ST7735_FillColorBuffer(color);
  ST7735_Select();
  ST7735_DC_Data();
  s_tx.type = ST7735_TX_COLOR_REPEAT;
  s_tx.remainingPixels = pixelCount;
  s_tx.bitmapBytes = 0U;

  chunkPixels = (s_tx.remainingPixels > ST7735_COLOR_CHUNK_PIXELS) ? ST7735_COLOR_CHUNK_PIXELS : s_tx.remainingPixels;
  if (chunkPixels == 0U) {
    ST7735_Unselect();
    s_tx.type = ST7735_TX_NONE;
    return 1U;
  }
  if (!ST7735_StartDma(s_ioBuffer, (uint16_t) (chunkPixels * 2U))) {
    (void) HAL_SPI_Transmit(&hspi2, s_ioBuffer, (uint16_t) (chunkPixels * 2U), HAL_MAX_DELAY);
    s_tx.remainingPixels = (uint16_t) (s_tx.remainingPixels - chunkPixels);
  } else {
    s_tx.remainingPixels = (uint16_t) (s_tx.remainingPixels - chunkPixels);
    return 0U;
  }
  while (s_tx.remainingPixels > 0U) {
    chunkPixels = (s_tx.remainingPixels > ST7735_COLOR_CHUNK_PIXELS) ? ST7735_COLOR_CHUNK_PIXELS : s_tx.remainingPixels;
    (void) HAL_SPI_Transmit(&hspi2, s_ioBuffer, (uint16_t) (chunkPixels * 2U), HAL_MAX_DELAY);
    s_tx.remainingPixels = (uint16_t) (s_tx.remainingPixels - chunkPixels);
  }
  ST7735_Unselect();
  s_tx.type = ST7735_TX_NONE;
  return 1U;
}

static uint8_t ST7735_ContinueColorRepeat(void) {
  uint16_t chunkPixels;

  if (s_tx.type != ST7735_TX_COLOR_REPEAT) return 1U;
  if (s_spiDmaBusy != 0U) return 0U;
  if (s_spiDmaError != 0U) {
    ST7735_Unselect();
    s_tx.type = ST7735_TX_NONE;
    return 1U;
  }
  if (s_tx.remainingPixels == 0U) {
    ST7735_Unselect();
    s_tx.type = ST7735_TX_NONE;
    return 1U;
  }

  chunkPixels = (s_tx.remainingPixels > ST7735_COLOR_CHUNK_PIXELS) ? ST7735_COLOR_CHUNK_PIXELS : s_tx.remainingPixels;
  if (ST7735_StartDma(s_ioBuffer, (uint16_t) (chunkPixels * 2U)) == 0U) {
    (void) HAL_SPI_Transmit(&hspi2, s_ioBuffer, (uint16_t) (chunkPixels * 2U), HAL_MAX_DELAY);
    s_tx.remainingPixels = (uint16_t) (s_tx.remainingPixels - chunkPixels);
    while (s_tx.remainingPixels > 0U) {
      chunkPixels = (s_tx.remainingPixels > ST7735_COLOR_CHUNK_PIXELS) ? ST7735_COLOR_CHUNK_PIXELS : s_tx.remainingPixels;
      (void) HAL_SPI_Transmit(&hspi2, s_ioBuffer, (uint16_t) (chunkPixels * 2U), HAL_MAX_DELAY);
      s_tx.remainingPixels = (uint16_t) (s_tx.remainingPixels - chunkPixels);
    }
    ST7735_Unselect();
    s_tx.type = ST7735_TX_NONE;
    return 1U;
  }
  s_tx.remainingPixels = (uint16_t) (s_tx.remainingPixels - chunkPixels);
  return 0U;
}

static uint8_t ST7735_BeginBitmapTransfer(uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint16_t bytes) {
  ST7735_SetAddressWindow(x, y, (uint8_t) (x + w - 1U), (uint8_t) (y + h - 1U));
  ST7735_Select();
  ST7735_DC_Data();
  s_tx.type = ST7735_TX_BITMAP;
  s_tx.remainingPixels = 0U;
  s_tx.bitmapBytes = bytes;
  if (!ST7735_StartDma(s_ioBuffer, bytes)) {
    (void) HAL_SPI_Transmit(&hspi2, s_ioBuffer, bytes, HAL_MAX_DELAY);
    ST7735_Unselect();
    s_tx.type = ST7735_TX_NONE;
    s_tx.bitmapBytes = 0U;
    return 1U;
  }
  return 0U;
}

static uint8_t ST7735_ContinueBitmapTransfer(void) {
  if (s_tx.type != ST7735_TX_BITMAP) return 1U;
  if (s_spiDmaBusy != 0U) return 0U;
  ST7735_Unselect();
  s_tx.type = ST7735_TX_NONE;
  s_tx.bitmapBytes = 0U;
  return 1U;
}

static uint16_t ST7735_BuildCharBitmap(char ch, uint16_t color, uint16_t bgColor) {
  uint16_t p = 0U;
  const uint8_t *glyph;
  uint8_t row;
  uint8_t col;

  if (((uint8_t) ch < FONT_FIRST_CHAR) || ((uint8_t) ch > FONT_LAST_CHAR)) ch = '?';
  glyph = &Font7x10[ST7735_FONT_GLYPH_INDEX(ch)];
  for (row = 0U; row < FONT_7X10_HEIGHT; row++) {
    uint8_t rowBits = glyph[row];
    for (col = 0U; col < FONT_7X10_WIDTH; col++) {
      uint16_t drawColor = (rowBits & (uint8_t) (0x80U >> col)) ? color : bgColor;
      s_ioBuffer[p++] = (uint8_t) (drawColor >> 8);
      s_ioBuffer[p++] = (uint8_t) (drawColor & 0xFFU);
    }
    s_ioBuffer[p++] = (uint8_t) (bgColor >> 8);
    s_ioBuffer[p++] = (uint8_t) (bgColor & 0xFFU);
  }
  return p;
}

static uint16_t ST7735_BuildMarkerBitmap(const uint8_t *glyph, uint16_t color, uint16_t bgColor) {
  uint16_t p = 0U;
  uint8_t size;
  uint8_t row;
  uint8_t col;

  if (glyph == NULL) return 0U;
  size = glyph[0];
  for (row = 0U; row < size; row++) {
    uint8_t rowBits = glyph[1U + row];
    for (col = 0U; col < size; col++) {
      uint16_t drawColor = (rowBits & (uint8_t) (1U << (size - 1U - col))) ? color : bgColor;
      s_ioBuffer[p++] = (uint8_t) (drawColor >> 8);
      s_ioBuffer[p++] = (uint8_t) (drawColor & 0xFFU);
    }
  }
  return p;
}

static void ST7735_QueueReset(void) {
  s_queueHead = 0U;
  s_queueTail = 0U;
  s_queueCount = 0U;
  s_queueWarn = 0U;
  ST7735_NotifyStatus();
}

static uint8_t ST7735_QueuePush(const ST7735_Command *cmd) {
  if (s_queueCount >= LCD_QUEUE_SIZE) return 0U;
  s_queue[s_queueTail] = *cmd;
  s_queueTail = (uint8_t) ((s_queueTail + 1U) % LCD_QUEUE_SIZE);
  s_queueCount++;
  if (((uint8_t) (LCD_QUEUE_SIZE - s_queueCount) == 2U) && (s_queueWarn == 0U)) {
    s_queueWarn = 1U;
    ST7735_NotifyStatus();
  }
  return 1U;
}

static uint8_t ST7735_QueuePop(ST7735_Command *cmd) {
  if (s_queueCount == 0U) return 0U;
  *cmd = s_queue[s_queueHead];
  s_queueHead = (uint8_t) ((s_queueHead + 1U) % LCD_QUEUE_SIZE);
  s_queueCount--;
  if (((uint8_t) (LCD_QUEUE_SIZE - s_queueCount) == 3U) && (s_queueWarn != 0U)) {
    s_queueWarn = 0U;
    ST7735_NotifyStatus();
  }
	if(s_InitSeq && s_queueCount == 0U){
	 s_InitSeq = 0;
	}
  return 1U;
}

static uint8_t ST7735_StartNextCommand(void) {
  if (s_active.busy != 0U) return 1U;
  if (!ST7735_QueuePop(&s_active.cmd)) return 0U;
  s_active.busy = 1U;
  switch ((ST7735_CommandType) s_active.cmd.type) {
  case ST7735_CMD_FILL_RECT:
    break;
  case ST7735_CMD_DRAW_TEXT:
    s_active.text.index = 0U;
    s_active.text.cursorX = s_active.cmd.data.drawText.x;
    s_active.text.cursorY = s_active.cmd.data.drawText.y;
    break;
  case ST7735_CMD_DRAW_MARKER:
    break;
  default:
    s_active.busy = 0U;
    s_active.cmd.type = ST7735_CMD_NONE;
    return 0U;
  }
  return 1U;
}

static uint8_t ST7735_ProcessRectLike(uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint16_t color) {
  if (s_tx.type == ST7735_TX_COLOR_REPEAT) {
    return ST7735_ContinueColorRepeat();
  }
  if ((w == 0U) || (h == 0U)) return 1U;
  ST7735_SetAddressWindow(x, y, (uint8_t) (x + w - 1U), (uint8_t) (y + h - 1U));
  return ST7735_BeginColorRepeat(color, (uint16_t) ((uint16_t) w * (uint16_t) h));
}

static uint8_t ST7735_ProcessDrawTextStep(void) {
  ST7735_DrawTextCmd *t = &s_active.cmd.data.drawText;
  char ch;

  if (s_tx.type == ST7735_TX_BITMAP) {
    if (!ST7735_ContinueBitmapTransfer()) return 0U;
    s_active.text.cursorX = (uint8_t) (s_active.text.cursorX + ST7735_TEXT_CELL_WIDTH);
    s_active.text.index++;
    return 0U;
  }
  ch = t->text[s_active.text.index];
  if (ch == '\0') {
    return 1U;
  }
  if (ch == '\n') {
    s_active.text.cursorX = t->x;
    s_active.text.cursorY = (uint8_t) (s_active.text.cursorY + FONT_7X10_HEIGHT + FONT_7X10_HEIGHT / 2);
    s_active.text.index++;
    return 0U;
  }
  if ((uint16_t) s_active.text.cursorX + ST7735_TEXT_CELL_WIDTH > LCD_WIDTH) {
    s_active.text.cursorX = t->x;
    s_active.text.cursorY = (uint8_t) (s_active.text.cursorY + FONT_7X10_HEIGHT);
  }
  if ((uint16_t) s_active.text.cursorY + FONT_7X10_HEIGHT > LCD_HEIGHT) return 1U;
  if (ST7735_BeginBitmapTransfer(s_active.text.cursorX, s_active.text.cursorY,
      ST7735_TEXT_CELL_WIDTH, FONT_7X10_HEIGHT, ST7735_BuildCharBitmap(ch, t->color, s_bgColor)) != 0U) {
    s_active.text.cursorX = (uint8_t) (s_active.text.cursorX + ST7735_TEXT_CELL_WIDTH);
    s_active.text.index++;
  }
  return 0U;
}

static uint8_t ST7735_ProcessDrawMarkerStep(void) {
  ST7735_DrawMarkerCmd *m = &s_active.cmd.data.drawMarker;
  const uint8_t *glyph = ST7735_GetMarkerGlyph(m->idx);
  uint8_t size;
  uint16_t bytes;

  if (s_tx.type == ST7735_TX_BITMAP) {
    return ST7735_ContinueBitmapTransfer();
  }
  if (glyph == NULL) {
    return 1U;
  }
  size = glyph[0];
  bytes = ST7735_BuildMarkerBitmap(glyph, m->color, s_bgColor);
  return ST7735_BeginBitmapTransfer((uint8_t) m->x, (uint8_t) m->y, size, size, bytes);
}

static void ST7735_UpdateBacklightPwm(void) {
  uint32_t now = HAL_GetTick();
  uint32_t remaining = 0U;
  uint8_t pwm;
  uint8_t prevApplied = s_backlightApplied;

  if ((int32_t) (s_backlightOffTick - now) > 0) {
    remaining = s_backlightOffTick - now;
  }
  pwm = (uint8_t) (((remaining >> 3) < s_backlightLevel) ? (remaining >> 3) : s_backlightLevel);
  if (pwm > 127U) {
    pwm = 127U;
  }
  if ((pwm != s_backlightApplied) && (htim14.State != HAL_TIM_STATE_RESET)) {
    __HAL_TIM_SET_COMPARE(&htim14, TIM_CHANNEL_1, pwm);
    s_backlightApplied = pwm;
  }
  if (((prevApplied == 0U) && (s_backlightApplied > 0U)) || ((prevApplied > 0U) && (s_backlightApplied == 0U))) {
    ST7735_NotifyStatus();
  }
}

static inline uint8_t ProgressBar_clamp(uint8_t v) {
  return (v > ProgressBar_PB_LEN) ? ProgressBar_PB_LEN : v;
}

static inline uint16_t ProgressBar_color_for_len(uint8_t v) {
  if (v == 0U) return LCD_RED;
  if (v >= ProgressBar_PB_LEN) return LCD_GREEN;
  {
    uint16_t t = (uint16_t) v;
    uint16_t r5 = (uint16_t) ((ProgressBar_PB_LEN - 1U - t) / 2U);
    return (uint16_t) ((r5 << 11) | (t << 5));   // RGB565
  }
}

static void ProgressBar_GetBounds(const ProgressBar_Spec *spec, uint8_t *x, uint8_t *y, uint8_t *w, uint8_t *h) {
  switch (spec->dir) {
  case ProgressBar_DIR_RIGHT:
    *x = spec->x0;
    *y = spec->y0;
    *w = ProgressBar_PB_LEN;
    *h = ProgressBar_PB_TH;
    break;
  case ProgressBar_DIR_LEFT:
    *x = (uint8_t) (spec->x0 - (ProgressBar_PB_LEN - 1U));
    *y = spec->y0;
    *w = ProgressBar_PB_LEN;
    *h = ProgressBar_PB_TH;
    break;
  case ProgressBar_DIR_UP:
  default:
    *x = spec->x0;
    *y = (uint8_t) (spec->y0 - (ProgressBar_PB_LEN - 1U));
    *w = ProgressBar_PB_TH;
    *h = ProgressBar_PB_LEN;
    break;
  }
}

void LCD_Init(LCD_QueueCallback cb) {
  s_queueCb = cb;
  ST7735_Reset();
  ST7735_WriteCommand(0x11);
  HAL_Delay(120U);
  {
    const uint8_t data[] =
      { 0x05, 0x3A, 0x3A };
    ST7735_WriteCommandWithData(0xB1, data, sizeof(data));
  }
  {
    const uint8_t data[] =
      { 0x05, 0x3A, 0x3A };
    ST7735_WriteCommandWithData(0xB2, data, sizeof(data));
  }
  {
    const uint8_t data[] =
      { 0x05, 0x3A, 0x3A, 0x05, 0x3A, 0x3A };
    ST7735_WriteCommandWithData(0xB3, data, sizeof(data));
  }
  {
    const uint8_t data[] =
      { 0x03 };
    ST7735_WriteCommandWithData(0xB4, data, sizeof(data));
  }
  {
    const uint8_t data[] =
      { 0x64, 0x04, 0x84 };
    ST7735_WriteCommandWithData(0xC0, data, sizeof(data));
  }
  {
    const uint8_t data[] =
      { 0xC5 };
    ST7735_WriteCommandWithData(0xC1, data, sizeof(data));
  }
  {
    const uint8_t data[] =
      { 0x0D, 0x00 };
    ST7735_WriteCommandWithData(0xC2, data, sizeof(data));
  }
  {
    const uint8_t data[] =
      { 0x8D, 0x2A };
    ST7735_WriteCommandWithData(0xC3, data, sizeof(data));
  }
  {
    const uint8_t data[] =
      { 0x8D, 0xEE };
    ST7735_WriteCommandWithData(0xC4, data, sizeof(data));
  }
  {
    const uint8_t data[] =
      { 0x0E };
    ST7735_WriteCommandWithData(0xC5, data, sizeof(data));
  }
  {
    const uint8_t data[] =
      { 0xA8 };
    ST7735_WriteCommandWithData(0x36, data, sizeof(data));
  }
  {
    const uint8_t data[] =
      { 0x15, 0x0B, 0x02, 0x00, 0x08, 0x00, 0x00, 0x00, 0x00, 0x05, 0x11, 0x35, 0x10, 0x12, 0x05, 0x3F };
    ST7735_WriteCommandWithData(0xE0, data, sizeof(data));
  }
  {
    const uint8_t data[] =
      { 0x0E, 0x0E, 0x03, 0x00, 0x06, 0x00, 0x00, 0x00, 0x00, 0x06, 0x12, 0x37, 0x10, 0x10, 0x06, 0x3F };
    ST7735_WriteCommandWithData(0xE1, data, sizeof(data));
  }
  {
    const uint8_t data[] =
      { 0x05 };
    ST7735_WriteCommandWithData(0x3A, data, sizeof(data));
  }
  ST7735_WriteCommand(0x29);
  HAL_Delay(20U);

  memset(&s_active, 0, sizeof(s_active));
  memset(&s_tx, 0, sizeof(s_tx));
  s_spiDmaBusy = 0U;
  s_spiDmaError = 0U;
  s_backlightLevel = 64U;
  s_backlightApplied = 0U;
  s_backlightOffTick = HAL_GetTick();
  s_bgColor = LCD_BLACK;
  s_progressBarInitMask = 0U;
  memset(s_progressBarPrev, 0, sizeof(s_progressBarPrev));
  ST7735_QueueReset();
}

uint8_t LCD_Clear(uint16_t color) {
  if(!s_InitSeq){
    ST7735_QueueReset();
  }
  return LCD_FillRect(0U ,8U , LCD_WIDTH, (uint8_t) (LCD_HEIGHT - 8U), color);
}

uint8_t LCD_FillRect(uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint16_t color) {
  ST7735_Command cmd;

  if ((x >= LCD_WIDTH) || (y >= LCD_HEIGHT) || (w == 0U) || (h == 0U)) return 0U;
  if ((uint16_t) x + w > LCD_WIDTH) w = (uint8_t) (LCD_WIDTH - x);
  if ((uint16_t) y + h > LCD_HEIGHT) h = (uint8_t) (LCD_HEIGHT - y);
  memset(&cmd, 0, sizeof(cmd));
  cmd.type = ST7735_CMD_FILL_RECT;
  cmd.data.fillRect.x = x;
  cmd.data.fillRect.y = y;
  cmd.data.fillRect.w = w;
  cmd.data.fillRect.h = h;
  cmd.data.fillRect.color = color;
  return ST7735_QueuePush(&cmd);
}

uint8_t LCD_DrawText(uint8_t x, uint8_t y, const char *text, uint16_t color) {
  ST7735_Command cmd;
  uint8_t i = 0U;

  if (text == NULL) return 0U;
  memset(&cmd, 0, sizeof(cmd));
  cmd.type = ST7735_CMD_DRAW_TEXT;
  cmd.data.drawText.x = x;
  cmd.data.drawText.y = y;
  cmd.data.drawText.color = color;
  while ((i < ST7735_MAX_TEXT_LEN) && (text[i] != '\0')) {
    cmd.data.drawText.text[i] = text[i];
    i++;
  }
  cmd.data.drawText.text[i] = '\0';
  return ST7735_QueuePush(&cmd);
}

uint8_t LCD_DrawMarker(uint8_t x, uint8_t y, uint8_t idx, uint16_t color) {
  ST7735_Command cmd;
  const uint8_t *glyph = ST7735_GetMarkerGlyph(idx);
  uint8_t size;
  int16_t x0;
  int16_t y0;

  if (glyph == NULL) return 0U;
  size = glyph[0];
  x0 = (int16_t) x - (int16_t) (size / 2U);
  y0 = (int16_t) y - (int16_t) (size / 2U);
  if ((x0 < 0) || (y0 < 0)) return 0U;
  if ((x0 + size) > (int16_t) LCD_WIDTH) return 0U;
  if ((y0 + size) > (int16_t) LCD_HEIGHT) return 0U;

  memset(&cmd, 0, sizeof(cmd));
  cmd.type = ST7735_CMD_DRAW_MARKER;
  cmd.data.drawMarker.x = x0;
  cmd.data.drawMarker.y = y0;
  cmd.data.drawMarker.idx = idx;
  cmd.data.drawMarker.color = color;
  return ST7735_QueuePush(&cmd);
}

uint8_t LCD_DrawIndicator(uint8_t index, uint8_t value) {
  if (index == 0U) {
    return LCD_DrawMarker(4U, 4U, 8U, value ? LCD_BLUE : LCD_GRAY);
  } else if (index == 1U) {
    uint16_t color = (value == 3U) ? LCD_BLUE : ((value == 0U) ? LCD_GRAY : LCD_ORANGE);
    return LCD_DrawMarker((uint8_t) (LCD_WIDTH - 4U), 4U, 10U, color);
  }
  return 0U;
}

uint8_t LCD_DrawProgressBar(uint8_t index, uint8_t value_pixels) {
  const ProgressBar_Spec *spec;
  uint8_t v;
  uint8_t prev;
  uint8_t bx;
  uint8_t by;
  uint8_t bw;
  uint8_t bh;
  uint16_t fg;
  uint16_t prevFg;

  if (index >= 4U || LCD_QUEUE_SIZE - s_queueCount < 2U) return 1U;
  /*
   * Первый вызов:
   *   фон + активная часть = до 2 команд
   * Изменение цвета + дельта:
   *   до 2 команд
   */
  spec = &s_progressBars[index];
  v = ProgressBar_clamp(value_pixels);
  prev = s_progressBarPrev[index];
  fg = ProgressBar_color_for_len(v);
  prevFg = ProgressBar_color_for_len(prev);
  ProgressBar_GetBounds(spec, &bx, &by, &bw, &bh);
  /* Первый вызов: рисуем весь фон и текущее значение */
  if ((s_progressBarInitMask & (1U << index)) == 0U) {
    (void) LCD_FillRect(bx, by, bw, bh, LCD_GRAY);
    if (v > 0U) {
      switch (spec->dir) {
      case ProgressBar_DIR_RIGHT:
        (void) LCD_FillRect(bx, by, v, bh, fg);
        break;
      case ProgressBar_DIR_LEFT:
        (void) LCD_FillRect((uint8_t) (spec->x0 - (v - 1U)), by, v, bh, fg);
        break;
      case ProgressBar_DIR_UP:
      default:
        (void) LCD_FillRect(bx, (uint8_t) (spec->y0 - (v - 1U)), bw, v, fg);
        break;
      }
    }
    s_progressBarPrev[index] = v;
    s_progressBarInitMask |= (1U << index);
    return 0U;
  }
  if (v == prev) {
    return 0U;
  }
  /* Если изменился цвет уже закрашенной части — перерисуем оставшуюся активную область */
  if ((v > 0U) && (prevFg != fg)) {
    uint8_t common = (v < prev) ? v : prev;
    switch (spec->dir) {
    case ProgressBar_DIR_RIGHT:
      (void) LCD_FillRect(bx, by, common, bh, fg);
      break;
    case ProgressBar_DIR_LEFT:
      (void) LCD_FillRect((uint8_t) (spec->x0 - (common - 1U)), by, common, bh, fg);
      break;
    case ProgressBar_DIR_UP:
    default:
      (void) LCD_FillRect(bx, (uint8_t) (spec->y0 - (common - 1U)), bw, common, fg);
      break;
    }
  }
  if (v > prev) {
    uint8_t delta = (uint8_t) (v - prev);
    switch (spec->dir) {
    case ProgressBar_DIR_RIGHT:
      (void) LCD_FillRect((uint8_t) (bx + prev), by, delta, bh, fg);
      break;
    case ProgressBar_DIR_LEFT:
      (void) LCD_FillRect((uint8_t) (spec->x0 - (v - 1U)), by, delta, bh, fg);
      break;
    case ProgressBar_DIR_UP:
    default:
      (void) LCD_FillRect(bx, (uint8_t) (spec->y0 - (v - 1U)), bw, delta, fg);
      break;
    }
  } else {
    uint8_t delta = (uint8_t) (prev - v);
    switch (spec->dir) {
    case ProgressBar_DIR_RIGHT:
      (void) LCD_FillRect((uint8_t) (bx + v), by, delta, bh, LCD_GRAY);
      break;
    case ProgressBar_DIR_LEFT:
      (void) LCD_FillRect((uint8_t) (spec->x0 - (prev - 1U)), by, delta, bh, LCD_GRAY);
      break;
    case ProgressBar_DIR_UP:
    default:
      (void) LCD_FillRect(bx, (uint8_t) (spec->y0 - (prev - 1U)), bw, delta, LCD_GRAY);
      break;
    }
  }
  s_progressBarPrev[index] = v;
  return 0U;
}

uint8_t LCD_Process(void) {
  uint8_t done = 0U;

  ST7735_UpdateBacklightPwm();
  if (s_spiDmaBusy != 0U) {
    if (ST7735_TryRecoverDmaBusy() == 0U) {
      return 0U;
    }
  }
  if ((s_active.busy == 0U) && (s_queueCount == 0U)) {
    return 0U;
  }
  if (s_active.busy == 0U) {
    if (ST7735_StartNextCommand() == 0U) {
      return 0U;
    }
  }
  switch (s_active.cmd.type) {
  case ST7735_CMD_FILL_RECT:
    done = ST7735_ProcessRectLike(s_active.cmd.data.fillRect.x, s_active.cmd.data.fillRect.y, s_active.cmd.data.fillRect.w,
        s_active.cmd.data.fillRect.h, s_active.cmd.data.fillRect.color);
    break;
  case ST7735_CMD_DRAW_TEXT:
    done = ST7735_ProcessDrawTextStep();
    break;
  case ST7735_CMD_DRAW_MARKER:
    done = ST7735_ProcessDrawMarkerStep();
    break;
  default:
    done = 1U;
    break;
  }
  if (done != 0U) {
    s_active.busy = 0U;
    s_active.cmd.type = ST7735_CMD_NONE;
    memset(&s_active.text, 0, sizeof(s_active.text));
    s_tx.type = ST7735_TX_NONE;
  }
  return 1U;
}

void LCD_SetBacklightTimeout(uint32_t timeout_ms) {
  s_backlightOffTick = HAL_GetTick() + timeout_ms;
}

void LCD_SetBacklightLevel(uint8_t level_0_127) {
  if (level_0_127 > 127U) {
    level_0_127 = 127U;
  }
  s_backlightLevel = level_0_127;
}

void LCD_SetBackgroundColor(uint16_t color) {
  s_bgColor = color;
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi) {
  if (hspi->Instance == SPI2) {
    s_spiDmaBusy = 0U;
  }
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi) {
  if (hspi->Instance == SPI2) {
    s_spiDmaError = 1U;
    s_spiDmaBusy = 0U;
  }
}
