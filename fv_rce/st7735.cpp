#include "st7735.h"
#include "font_7x10.h"
#include <string.h>
#include "esp_heap_caps.h"
#include "esp_err.h"

#if __has_include(<esp_arduino_version.h>)
#include <esp_arduino_version.h>
#endif

#define LCD_SPI_HOST VSPI_HOST
#define LCD_PIN_MOSI 23
#define LCD_PIN_MISO -1
#define LCD_PIN_SCLK 18
#define LCD_PIN_CS   5
#define LCD_PIN_DC   16
#define LCD_PIN_RST  17
#define LCD_SPI_CLOCK_HZ 24000000UL

#define ST7735_FONT_GLYPH_INDEX(ch) ((((uint8_t)(ch)) - FONT_FIRST_CHAR) * FONT_7X10_GLYPH_ROWS)
#define ST7735_TEXT_CELL_WIDTH   (FONT_7X10_WIDTH + FONT_7X10_SPACING)
#define ST7735_MARKER_COUNT      11U
#define ST7735_X_OFFSET 0U
#define ST7735_Y_OFFSET 24U
#define ST7735_DMA_ROWS 8U
#define ST7735_DMA_PIXELS (LCD_WIDTH * ST7735_DMA_ROWS)
#define ST7735_DIRTY_CLEAN 0xFFU
#define ProgressBar_PB_LEN   64U
#define ProgressBar_PB_TH     3U

typedef enum { FLUSH_IDLE = 0, FLUSH_CASET_CMD, FLUSH_CASET_DATA, FLUSH_RASET_CMD, FLUSH_RASET_DATA, FLUSH_RAMWR_CMD, FLUSH_PIXELS } ST7735_FlushStep;
typedef struct { uint8_t x0; uint8_t y0; } ProgressBar_Spec;
typedef struct { uint8_t active; uint8_t x; uint8_t y; uint8_t w; uint8_t h; uint8_t nextY; ST7735_FlushStep step; } ST7735_FlushState;

static spi_device_handle_t s_lcd = NULL;
static spi_transaction_t s_trans;
static volatile uint8_t s_spiBusy = 0U;
static uint8_t s_txSmall[4];
static uint8_t *s_dmaBuf = NULL;
static uint16_t s_framebuffer[LCD_WIDTH * LCD_HEIGHT];
static uint8_t s_dirtyX0[LCD_HEIGHT];
static uint8_t s_dirtyX1[LCD_HEIGHT];
static ST7735_FlushState s_flush;
static uint16_t s_bgColor = LCD_BLACK;
static uint8_t s_progressBarPrev[4] = {0U, 0U, 0U, 0U};

static const ProgressBar_Spec s_progressBars[4] = {
  {10U, 0U},
  {10U, 4U},
  {(uint8_t)(LCD_WIDTH / 2U + 6U), 0U},
  {(uint8_t)(LCD_WIDTH / 2U + 6U), 4U}
};

static const uint8_t s_circle_1[] = {1U, 0x01U};
static const uint8_t s_circle_2[] = {2U, 0x03U, 0x03U};
static const uint8_t s_circle_3[] = {3U, 0x07U, 0x07U, 0x07U};
static const uint8_t s_circle_4[] = {4U, 0x06U, 0x0FU, 0x0FU, 0x06U};
static const uint8_t s_circle_5[] = {5U, 0x0EU, 0x1FU, 0x1FU, 0x1FU, 0x0EU};
static const uint8_t s_circle_6[] = {6U, 0x1EU, 0x3FU, 0x3FU, 0x3FU, 0x3FU, 0x1EU};
static const uint8_t s_circle_7[] = {7U, 0x1CU, 0x3EU, 0x7FU, 0x7FU, 0x7FU, 0x3EU, 0x1CU};
static const uint8_t s_circle_8[] = {8U, 0x3CU, 0x7EU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0x7EU, 0x3CU};
static const uint8_t s_antena[]   = {8U, 0xE0U, 0x18U, 0x04U, 0xC2U, 0x32U, 0x11U, 0xC9U, 0xC9U};
static const uint8_t s_battery[]  = {8U, 0x18U, 0x3CU, 0x42U, 0x5AU, 0x5AU, 0x5AU, 0x42U, 0x3CU};
static const uint8_t s_connect[]  = {8U, 0x14U, 0x16U, 0x17U, 0x14U, 0x14U, 0x74U, 0x34U, 0x14U};
static const uint8_t* const s_markers[ST7735_MARKER_COUNT] = {s_circle_1,s_circle_2,s_circle_3,s_circle_4,s_circle_5,s_circle_6,s_circle_7,s_circle_8,s_antena,s_battery,s_connect};

static void ST7735_PreTransfer(spi_transaction_t *t) { digitalWrite(LCD_PIN_DC, ((uintptr_t)t->user) ? HIGH : LOW); }
static uint16_t ST7735_Idx(uint8_t x, uint8_t y) { return (uint16_t)y * LCD_WIDTH + x; }
static uint16_t ST7735_Swap565(uint16_t c) { return (uint16_t)((c << 8) | (c >> 8)); }
static const uint8_t *ST7735_GetMarkerGlyph(uint8_t idx) { return (idx >= ST7735_MARKER_COUNT) ? NULL : s_markers[idx]; }
static void ST7735_DirtyClear(void) { memset(s_dirtyX0, ST7735_DIRTY_CLEAN, sizeof(s_dirtyX0)); memset(s_dirtyX1, 0, sizeof(s_dirtyX1)); }
static uint8_t ST7735_DirtyRow(uint8_t y) { return s_dirtyX0[y] != ST7735_DIRTY_CLEAN; }
static void ST7735_DirtyAdd(uint8_t x, uint8_t y, uint8_t w, uint8_t h) {
  if ((w == 0U) || (h == 0U) || (x >= LCD_WIDTH) || (y >= LCD_HEIGHT)) return;
  uint8_t x1 = (uint8_t)(x + w - 1U);
  if ((uint16_t)x + w > LCD_WIDTH) x1 = LCD_WIDTH - 1U;
  uint8_t y1 = (uint8_t)(y + h - 1U);
  if ((uint16_t)y + h > LCD_HEIGHT) y1 = LCD_HEIGHT - 1U;
  for (uint8_t yy = y; yy <= y1; yy++) {
    if (s_dirtyX0[yy] == ST7735_DIRTY_CLEAN) { s_dirtyX0[yy] = x; s_dirtyX1[yy] = x1; }
    else { if (x < s_dirtyX0[yy]) s_dirtyX0[yy] = x; if (x1 > s_dirtyX1[yy]) s_dirtyX1[yy] = x1; }
  }
}
static uint8_t ST7735_WaitSpiDone(uint8_t block) {
  if (s_spiBusy == 0U) return 1U;
  spi_transaction_t *ret = NULL;
  esp_err_t err = spi_device_get_trans_result(s_lcd, &ret, block ? portMAX_DELAY : 0);
  if (err == ESP_OK) { s_spiBusy = 0U; return 1U; }
  return 0U;
}

static uint8_t ST7735_QueueSpi(const uint8_t *data, uint32_t len, uint8_t isData) {
  if ((s_lcd == NULL) || (data == NULL) || (len == 0U) || (s_spiBusy != 0U)) return 0U;
  memset(&s_trans, 0, sizeof(s_trans));
  s_trans.length = len * 8U;
  s_trans.tx_buffer = data;
  s_trans.user = (void *)(uintptr_t)(isData ? 1U : 0U);
  if (spi_device_queue_trans(s_lcd, &s_trans, 0) != ESP_OK) return 0U;
  s_spiBusy = 1U;
  return 1U;
}

static void ST7735_WriteCommandBlocking(uint8_t cmd) { (void)ST7735_WaitSpiDone(1U); (void)ST7735_QueueSpi(&cmd, 1U, 0U); (void)ST7735_WaitSpiDone(1U); }
static void ST7735_WriteDataBlocking(const uint8_t *data, uint16_t size) { (void)ST7735_WaitSpiDone(1U); (void)ST7735_QueueSpi(data, size, 1U); (void)ST7735_WaitSpiDone(1U); }
static void ST7735_WriteCommandWithDataBlocking(uint8_t cmd, const uint8_t *data, uint16_t size) { ST7735_WriteCommandBlocking(cmd); if ((data != NULL) && (size != 0U)) ST7735_WriteDataBlocking(data, size); }
static void ST7735_Reset(void) { digitalWrite(LCD_PIN_RST, HIGH); delay(5); digitalWrite(LCD_PIN_RST, LOW); delay(20); digitalWrite(LCD_PIN_RST, HIGH); delay(120); }

static void ST7735_FbFillRect(uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint16_t color) {
  uint16_t c = ST7735_Swap565(color);
  for (uint8_t yy = 0U; yy < h; yy++) {
    uint16_t *p = &s_framebuffer[ST7735_Idx(x, (uint8_t)(y + yy))];
    for (uint8_t xx = 0U; xx < w; xx++) p[xx] = c;
  }
}

static void ST7735_FbDrawChar(uint8_t x, uint8_t y, char ch, uint16_t color) {
    if (((uint8_t)ch < FONT_FIRST_CHAR) || ((uint8_t)ch > FONT_LAST_CHAR)) {
        ch = '?';
    }
    const uint8_t *glyph = &Font7x10[ST7735_FONT_GLYPH_INDEX(ch)];
    uint16_t fg = ST7735_Swap565(color);
    uint16_t bg = ST7735_Swap565(s_bgColor);
    for (uint8_t row = 0; row < FONT_7X10_HEIGHT; row++) {
        if ((uint16_t)y + row >= LCD_HEIGHT) break;
        uint8_t rowBits = glyph[row];
        for (uint8_t col = 0; col < FONT_7X10_WIDTH; col++) {
            if ((uint16_t)x + col >= LCD_WIDTH) break;
            s_framebuffer[ST7735_Idx(x + col, y + row)] = (rowBits & (0x80U >> col)) ? fg : bg;
        }
    }
}

static void ST7735_FbDrawMarker(uint8_t x, uint8_t y, const uint8_t *glyph, uint16_t color) {
  uint8_t size = glyph[0];
  uint16_t c = ST7735_Swap565(color);
  for (uint8_t row = 0U; row < size; row++) {
    uint8_t rowBits = glyph[1U + row];
    for (uint8_t col = 0U; col < size; col++) {
      if (rowBits & (uint8_t)(1U << (size - 1U - col))) s_framebuffer[ST7735_Idx((uint8_t)(x + col), (uint8_t)(y + row))] = c;
    }
  }
}

static uint8_t ST7735_FlushStartFromDirty(void) {
  if (s_flush.active != 0U) return 0U;
  uint8_t y = 0U;
  while ((y < LCD_HEIGHT) && (ST7735_DirtyRow(y) == 0U)) y++;
  if (y >= LCD_HEIGHT) return 0U;
  uint8_t x0 = s_dirtyX0[y];
  uint8_t x1 = s_dirtyX1[y];
  uint8_t h = 1U;
  uint8_t yy = (uint8_t)(y + 1U);
  while ((yy < LCD_HEIGHT) && ST7735_DirtyRow(yy) && (s_dirtyX0[yy] == x0) && (s_dirtyX1[yy] == x1)) { h++; yy++; }
  for (uint8_t r = y; r < (uint8_t)(y + h); r++) { s_dirtyX0[r] = ST7735_DIRTY_CLEAN; s_dirtyX1[r] = 0U; }
  s_flush.x = x0;
  s_flush.y = y;
  s_flush.w = (uint8_t)(x1 - x0 + 1U);
  s_flush.h = h;
  s_flush.nextY = y;
  s_flush.step = FLUSH_CASET_CMD;
  s_flush.active = 1U;
  return 1U;
}
static uint8_t ST7735_FlushProcess(void) {
  if (s_flush.active == 0U) return 1U;
  if (s_dmaBuf == NULL) return 0U;
  if (ST7735_WaitSpiDone(0U) == 0U) return 0U;
  switch (s_flush.step) {
  case FLUSH_CASET_CMD:
    s_txSmall[0] = 0x2A; if (!ST7735_QueueSpi(s_txSmall, 1U, 0U)) return 0U; s_flush.step = FLUSH_CASET_DATA; return 0U;
  case FLUSH_CASET_DATA: {
    uint16_t ax0 = (uint16_t)(s_flush.x + ST7735_X_OFFSET), ax1 = (uint16_t)(s_flush.x + s_flush.w - 1U + ST7735_X_OFFSET);
    s_txSmall[0] = ax0 >> 8; s_txSmall[1] = ax0 & 0xFFU; s_txSmall[2] = ax1 >> 8; s_txSmall[3] = ax1 & 0xFFU;
    if (!ST7735_QueueSpi(s_txSmall, 4U, 1U)) return 0U; s_flush.step = FLUSH_RASET_CMD; return 0U;
  }
  case FLUSH_RASET_CMD:
    s_txSmall[0] = 0x2B; if (!ST7735_QueueSpi(s_txSmall, 1U, 0U)) return 0U; s_flush.step = FLUSH_RASET_DATA; return 0U;
  case FLUSH_RASET_DATA: {
    uint16_t ay0 = (uint16_t)(s_flush.y + ST7735_Y_OFFSET), ay1 = (uint16_t)(s_flush.y + s_flush.h - 1U + ST7735_Y_OFFSET);
    s_txSmall[0] = ay0 >> 8; s_txSmall[1] = ay0 & 0xFFU; s_txSmall[2] = ay1 >> 8; s_txSmall[3] = ay1 & 0xFFU;
    if (!ST7735_QueueSpi(s_txSmall, 4U, 1U)) return 0U; s_flush.step = FLUSH_RAMWR_CMD; return 0U;
  }
  case FLUSH_RAMWR_CMD:
    s_txSmall[0] = 0x2C; if (!ST7735_QueueSpi(s_txSmall, 1U, 0U)) return 0U; s_flush.step = FLUSH_PIXELS; return 0U;
  case FLUSH_PIXELS: {
    uint8_t endY = (uint8_t)(s_flush.y + s_flush.h);
    if (s_flush.nextY >= endY) { s_flush.active = 0U; s_flush.step = FLUSH_IDLE; return 1U; }
    uint8_t rows = (uint8_t)(endY - s_flush.nextY);
    uint32_t maxRows = ST7735_DMA_PIXELS / s_flush.w;
    if (maxRows == 0U) maxRows = 1U;
    if (rows > maxRows) rows = (uint8_t)maxRows;
    if (rows > ST7735_DMA_ROWS) rows = ST7735_DMA_ROWS;
    uint32_t p = 0U;
    for (uint8_t r = 0U; r < rows; r++) {
      const uint16_t *src = &s_framebuffer[ST7735_Idx(s_flush.x, (uint8_t)(s_flush.nextY + r))];
      memcpy(&s_dmaBuf[p], src, (uint32_t)s_flush.w * 2U);
      p += (uint32_t)s_flush.w * 2U;
    }
    if (!ST7735_QueueSpi(s_dmaBuf, p, 1U)) return 0U;
    s_flush.nextY = (uint8_t)(s_flush.nextY + rows);
    return 0U;
  }
  default:
    s_flush.active = 0U; s_flush.step = FLUSH_IDLE; return 1U;
  }
}

static inline uint16_t ProgressBar_color_for_len(uint8_t v) {
  if (v < (uint8_t)(ProgressBar_PB_LEN / 3)) return LCD_RED;
  if (v < (uint8_t)(ProgressBar_PB_LEN * 2 / 3)) return LCD_YELLOW;
  return LCD_GREEN;
}


void LCD_Init(void) {
  pinMode(LCD_PIN_DC, OUTPUT); pinMode(LCD_PIN_RST, OUTPUT); digitalWrite(LCD_PIN_DC, HIGH); digitalWrite(LCD_PIN_RST, HIGH);
  if (s_dmaBuf == NULL) s_dmaBuf = (uint8_t *)heap_caps_malloc(ST7735_DMA_PIXELS * 2U, MALLOC_CAP_DMA);
  spi_bus_config_t buscfg = {};
  buscfg.mosi_io_num = LCD_PIN_MOSI; buscfg.miso_io_num = LCD_PIN_MISO; buscfg.sclk_io_num = LCD_PIN_SCLK;
  buscfg.quadwp_io_num = -1; buscfg.quadhd_io_num = -1; buscfg.max_transfer_sz = ST7735_DMA_PIXELS * 2U;
  (void)spi_bus_initialize((spi_host_device_t)LCD_SPI_HOST, &buscfg, SPI_DMA_CH_AUTO);
  spi_device_interface_config_t devcfg = {};
  devcfg.clock_speed_hz = LCD_SPI_CLOCK_HZ; devcfg.mode = 0; devcfg.spics_io_num = LCD_PIN_CS; devcfg.queue_size = 1; devcfg.pre_cb = ST7735_PreTransfer;
  if (s_lcd == NULL) (void)spi_bus_add_device((spi_host_device_t)LCD_SPI_HOST, &devcfg, &s_lcd);
  ST7735_Reset();
  ST7735_WriteCommandBlocking(0x11); delay(120);
  { const uint8_t d[] = {0x05,0x3A,0x3A}; ST7735_WriteCommandWithDataBlocking(0xB1,d,sizeof(d)); ST7735_WriteCommandWithDataBlocking(0xB2,d,sizeof(d)); }
  { const uint8_t d[] = {0x05,0x3A,0x3A,0x05,0x3A,0x3A}; ST7735_WriteCommandWithDataBlocking(0xB3,d,sizeof(d)); }
  { const uint8_t d[] = {0x03}; ST7735_WriteCommandWithDataBlocking(0xB4,d,sizeof(d)); }
  { const uint8_t d[] = {0x64,0x04,0x84}; ST7735_WriteCommandWithDataBlocking(0xC0,d,sizeof(d)); }
  { const uint8_t d[] = {0xC5}; ST7735_WriteCommandWithDataBlocking(0xC1,d,sizeof(d)); }
  { const uint8_t d[] = {0x0D,0x00}; ST7735_WriteCommandWithDataBlocking(0xC2,d,sizeof(d)); }
  { const uint8_t d[] = {0x8D,0x2A}; ST7735_WriteCommandWithDataBlocking(0xC3,d,sizeof(d)); }
  { const uint8_t d[] = {0x8D,0xEE}; ST7735_WriteCommandWithDataBlocking(0xC4,d,sizeof(d)); }
  { const uint8_t d[] = {0x0E}; ST7735_WriteCommandWithDataBlocking(0xC5,d,sizeof(d)); }
  { const uint8_t d[] = {0xA8}; ST7735_WriteCommandWithDataBlocking(0x36,d,sizeof(d)); }
  { const uint8_t d[] = {0x15,0x0B,0x02,0x00,0x08,0x00,0x00,0x00,0x00,0x05,0x11,0x35,0x10,0x12,0x05,0x3F}; ST7735_WriteCommandWithDataBlocking(0xE0,d,sizeof(d)); }
  { const uint8_t d[] = {0x0E,0x0E,0x03,0x00,0x06,0x00,0x00,0x00,0x00,0x06,0x12,0x37,0x10,0x10,0x06,0x3F}; ST7735_WriteCommandWithDataBlocking(0xE1,d,sizeof(d)); }
  { const uint8_t d[] = {0x05}; ST7735_WriteCommandWithDataBlocking(0x3A,d,sizeof(d)); }
  ST7735_WriteCommandBlocking(0x29); delay(20);
  s_bgColor = LCD_BLACK;
  memset(s_framebuffer, 0, sizeof(s_framebuffer)); memset(&s_flush, 0, sizeof(s_flush)); ST7735_DirtyClear();
  s_spiBusy = 0U;
  memset(s_progressBarPrev, 0, sizeof(s_progressBarPrev));
}

uint8_t LCD_Process(void) {
  if (s_flush.active != 0U) return ST7735_FlushProcess() ? 1U : 0U;
  if (ST7735_WaitSpiDone(0U) == 0U) return 0U;
  if (ST7735_FlushStartFromDirty() == 0U) return 0U;
  return ST7735_FlushProcess() ? 1U : 0U;
}

void LCD_SetBackgroundColor(uint16_t color) { s_bgColor = color; }
void LCD_Clear(uint16_t color) {
  LCD_SetBackgroundColor(color);
  ST7735_FbFillRect(0U, 8U, LCD_WIDTH, (uint8_t)(LCD_HEIGHT - 8U), s_bgColor);
  ST7735_DirtyAdd(0U, 8U, LCD_WIDTH, (uint8_t)(LCD_HEIGHT - 8U));
}
void LCD_FillRect(uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint16_t color) {
  if ((x >= LCD_WIDTH) || (y >= LCD_HEIGHT) || (w == 0U) || (h == 0U)) return;
  if ((uint16_t)x + w > LCD_WIDTH) w = (uint8_t)(LCD_WIDTH - x);
  if ((uint16_t)y + h > LCD_HEIGHT) h = (uint8_t)(LCD_HEIGHT - y);
  ST7735_FbFillRect(x, y, w, h, color);
  ST7735_DirtyAdd(x, y, w, h);
}
void LCD_DrawText(uint8_t x, uint8_t y, uint16_t color, const char *text) {
  if (text == NULL) return;
  uint8_t cx = x, cy = y, minX = LCD_WIDTH, minY = LCD_HEIGHT, maxX = 0U, maxY = 0U, any = 0U;
  for (uint8_t i = 0U; (i < LCD_MAX_TEXT_LEN) && (text[i] != '\0'); i++) {
    char ch = text[i];
    if (ch == '\n') { cx = x; cy = (uint8_t)(cy + FONT_7X10_HEIGHT + FONT_7X10_HEIGHT / 2); continue; }
    if ((uint16_t)cx + ST7735_TEXT_CELL_WIDTH > LCD_WIDTH) { cx = x; cy = (uint8_t)(cy + FONT_7X10_HEIGHT); }
    if ((uint16_t)cy + FONT_7X10_HEIGHT > LCD_HEIGHT) break;
    uint8_t cellW = ST7735_TEXT_CELL_WIDTH;
    if ((uint16_t)cx + cellW > LCD_WIDTH) cellW = (uint8_t)(LCD_WIDTH - cx);
    ST7735_FbDrawChar(cx, cy, ch, color);
    if (cx < minX) minX = cx; if (cy < minY) minY = cy;
    if ((uint8_t)(cx + cellW - 1U) > maxX) maxX = (uint8_t)(cx + cellW - 1U);
    if ((uint8_t)(cy + FONT_7X10_HEIGHT - 1U) > maxY) maxY = (uint8_t)(cy + FONT_7X10_HEIGHT - 1U);
    any = 1U;
    cx = (uint8_t)(cx + ST7735_TEXT_CELL_WIDTH);
  }
  if (any != 0U) ST7735_DirtyAdd(minX, minY, (uint8_t)(maxX - minX + 1U), (uint8_t)(maxY - minY + 1U));
}
void LCD_DrawMarker(uint8_t x, uint8_t y, uint8_t idx, uint16_t color) {
  const uint8_t *glyph = ST7735_GetMarkerGlyph(idx);
  if (glyph == NULL) return;
  uint8_t size = glyph[0];
  int16_t x0 = (int16_t)x - (int16_t)(size / 2U), y0 = (int16_t)y - (int16_t)(size / 2U);
  if ((x0 < 0) || (y0 < 0) || ((x0 + size) > (int16_t)LCD_WIDTH) || ((y0 + size) > (int16_t)LCD_HEIGHT)) return;
  ST7735_FbDrawMarker((uint8_t)x0, (uint8_t)y0, glyph, color);
  ST7735_DirtyAdd((uint8_t)x0, (uint8_t)y0, size, size);
}
void LCD_DrawIndicator(uint8_t index, uint8_t value) {
  if (index == 0U) return LCD_DrawMarker(4U, 4U, 8U, value ? LCD_BLUE : LCD_GRAY);
  if (index == 1U) return LCD_DrawMarker((uint8_t)(LCD_WIDTH - 4U), 4U, 10U, value==0 ? LCD_GRAY : (value==1?LCD_BLUE:LCD_ORANGE));
}
void LCD_DrawProgressBar(uint8_t index, uint8_t value) {
  if (index >= 4U) return;
  if (value == s_progressBarPrev[index]) return;
  if (value > ProgressBar_PB_LEN){
    value = ProgressBar_PB_LEN;
    if (value == s_progressBarPrev[index]) return;
  }

  uint8_t x0 = s_progressBars[index].x0, y0 = s_progressBars[index].y0;
  uint16_t C1, C2;
  if(value >= (ProgressBar_PB_LEN / 2U)){
    C1 = ProgressBar_color_for_len(value);
    C2 = LCD_GRAY;
  } else {
    C1 = LCD_GRAY;
    C2 = ProgressBar_color_for_len(value);
  }
  if (value > 0U){ 
    LCD_FillRect(x0, y0, value, ProgressBar_PB_TH, C1);
  }
  if (value < ProgressBar_PB_LEN){ 
    LCD_FillRect(x0 + value, y0, (uint8_t)(ProgressBar_PB_LEN - value), ProgressBar_PB_TH, C2);
  }
  s_progressBarPrev[index] = value;
}
