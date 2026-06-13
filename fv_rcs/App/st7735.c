#include "st7735.h"
#include "spi.h"
#include "gpio.h"
#include "st7735_font_7x10.h"
#include <string.h>

#define ST7735_FONT_GLYPH_INDEX(ch) \
    ((((uint8_t)(ch)) - ST7735_FONT_7X10_FIRST_CHAR) * ST7735_FONT_7X10_GLYPH_ROWS)

#define ST7735_TEXT_CELL_WIDTH (ST7735_FONT_7X10_WIDTH + ST7735_FONT_7X10_SPACING)

#define ST7735_MAX_TEXT_LEN      21U
#define ST7735_TEXT_STORAGE_LEN  (ST7735_MAX_TEXT_LEN + 1U)

/*
 * Смещение видимой области дисплея.
 * Для текущего модуля логические координаты 160x80 отображаются корректно
 * при смещении по Y на 24.
 */
#define ST7735_X_OFFSET 0U
#define ST7735_Y_OFFSET 24U

/*
 * Один общий буфер:
 * - для заливки одним цветом
 * - для рендера одного символа 7x10 + spacing
 */
#define ST7735_IO_BUFFER_SIZE (ST7735_TEXT_CELL_WIDTH * ST7735_FONT_7X10_HEIGHT * 2U)
#define ST7735_COLOR_CHUNK_PIXELS (ST7735_IO_BUFFER_SIZE / 2U)

typedef enum
{
    ST7735_CMD_NONE = 0,
    ST7735_CMD_FILL_RECT,
    ST7735_CMD_FILL_CIRCLE,
    ST7735_CMD_DRAW_TEXT
} ST7735_CommandType;

typedef enum
{
    ST7735_TX_NONE = 0,
    ST7735_TX_COLOR_REPEAT,
    ST7735_TX_CHAR
} ST7735_TxType;

typedef struct
{
    uint8_t x;
    uint8_t y;
    uint8_t w;
    uint8_t h;
    uint16_t color;
} ST7735_FillRectCmd;

typedef struct
{
    uint8_t x0;
    uint8_t y0;
    uint8_t r;
    uint16_t color;
} ST7735_FillCircleCmd;

typedef struct
{
    uint8_t x;
    uint8_t y;
    uint16_t color;
    uint16_t bgColor;
    char text[ST7735_TEXT_STORAGE_LEN];
} ST7735_DrawTextCmd;

typedef struct
{
    uint8_t type;
    union
    {
        ST7735_FillRectCmd fillRect;
        ST7735_FillCircleCmd fillCircle;
        ST7735_DrawTextCmd drawText;
    } data;
} ST7735_Command;

typedef struct
{
    uint8_t busy;
    ST7735_Command cmd;

    union
    {
        struct
        {
            uint8_t currentRow;
        } rect;

        struct
        {
            int16_t x;
            int16_t y;
            int16_t d;
            uint8_t firstStepDone;

            uint8_t segCount;
            uint8_t segIndex;
            uint8_t segX[4];
            uint8_t segY[4];
            uint8_t segH[4];
        } circle;

        struct
        {
            uint8_t index;
            uint8_t cursorX;
            uint8_t cursorY;
        } text;
    } state;
} ST7735_ActiveJob;

typedef struct
{
    uint8_t type;
    uint16_t remainingPixels;
    uint16_t charBytes;
} ST7735_TxState;

/* ---------- Runtime state ---------- */

static ST7735_Command s_queue[ST7735_QUEUE_SIZE];
static uint8_t s_queueHead = 0U;
static uint8_t s_queueTail = 0U;
static uint8_t s_queueCount = 0U;

static ST7735_ActiveJob s_active;
static ST7735_TxState s_tx;
static uint8_t s_ioBuffer[ST7735_IO_BUFFER_SIZE];

static volatile uint8_t s_spiDmaBusy = 0U;
static volatile uint8_t s_spiDmaError = 0U;

/* ---------- Low-level helpers ---------- */

static void ST7735_Select(void)
{
    HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_RESET);
}

static void ST7735_Unselect(void)
{
    HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_SET);
}

static void ST7735_DC_Command(void)
{
    HAL_GPIO_WritePin(LCD_DC_GPIO_Port, LCD_DC_Pin, GPIO_PIN_RESET);
}

static void ST7735_DC_Data(void)
{
    HAL_GPIO_WritePin(LCD_DC_GPIO_Port, LCD_DC_Pin, GPIO_PIN_SET);
}

static void ST7735_Reset(void)
{
    HAL_GPIO_WritePin(LCD_RST_GPIO_Port, LCD_RST_Pin, GPIO_PIN_SET);
    HAL_Delay(5);
    HAL_GPIO_WritePin(LCD_RST_GPIO_Port, LCD_RST_Pin, GPIO_PIN_RESET);
    HAL_Delay(20);
    HAL_GPIO_WritePin(LCD_RST_GPIO_Port, LCD_RST_Pin, GPIO_PIN_SET);
    HAL_Delay(120);
}

static void ST7735_WriteCommand(uint8_t cmd)
{
    ST7735_Select();
    ST7735_DC_Command();
    HAL_SPI_Transmit(&hspi2, &cmd, 1U, HAL_MAX_DELAY);
    ST7735_Unselect();
}

static void ST7735_WriteData(const uint8_t* data, uint16_t size)
{
    ST7735_Select();
    ST7735_DC_Data();
    HAL_SPI_Transmit(&hspi2, (uint8_t*)data, size, HAL_MAX_DELAY);
    ST7735_Unselect();
}

static void ST7735_WriteCommandWithData(uint8_t cmd, const uint8_t* data, uint16_t size)
{
    ST7735_Select();
    ST7735_DC_Command();
    HAL_SPI_Transmit(&hspi2, &cmd, 1U, HAL_MAX_DELAY);

    if ((data != NULL) && (size > 0U))
    {
        ST7735_DC_Data();
        HAL_SPI_Transmit(&hspi2, (uint8_t*)data, size, HAL_MAX_DELAY);
    }

    ST7735_Unselect();
}

static void ST7735_SetAddressWindow(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1)
{
    uint8_t data[4];
    uint16_t ax0 = (uint16_t)(x0 + ST7735_X_OFFSET);
    uint16_t ax1 = (uint16_t)(x1 + ST7735_X_OFFSET);
    uint16_t ay0 = (uint16_t)(y0 + ST7735_Y_OFFSET);
    uint16_t ay1 = (uint16_t)(y1 + ST7735_Y_OFFSET);

    ST7735_WriteCommand(0x2A);
    data[0] = (uint8_t)(ax0 >> 8);
    data[1] = (uint8_t)(ax0 & 0xFFU);
    data[2] = (uint8_t)(ax1 >> 8);
    data[3] = (uint8_t)(ax1 & 0xFFU);
    ST7735_WriteData(data, 4U);

    ST7735_WriteCommand(0x2B);
    data[0] = (uint8_t)(ay0 >> 8);
    data[1] = (uint8_t)(ay0 & 0xFFU);
    data[2] = (uint8_t)(ay1 >> 8);
    data[3] = (uint8_t)(ay1 & 0xFFU);
    ST7735_WriteData(data, 4U);

    ST7735_WriteCommand(0x2C);
}

/* ---------- DMA helpers ---------- */

static uint8_t ST7735_StartDma(uint8_t* data, uint16_t size)
{
    s_spiDmaError = 0U;

    if (HAL_SPI_Transmit_DMA(&hspi2, data, size) != HAL_OK)
        return 0U;

    s_spiDmaBusy = 1U;
    return 1U;
}

static void ST7735_FillColorBuffer(uint16_t color)
{
    uint8_t hi = (uint8_t)(color >> 8);
    uint8_t lo = (uint8_t)(color & 0xFFU);
    uint8_t i;

    for (i = 0U; i < sizeof(s_ioBuffer); i += 2U)
    {
        s_ioBuffer[i] = hi;
        s_ioBuffer[i + 1U] = lo;
    }
}

static uint8_t ST7735_BeginColorRepeat(uint16_t color, uint16_t pixelCount)
{
    uint16_t chunkPixels;

    ST7735_FillColorBuffer(color);

    ST7735_Select();
    ST7735_DC_Data();

    s_tx.type = ST7735_TX_COLOR_REPEAT;
    s_tx.remainingPixels = pixelCount;
    s_tx.charBytes = 0U;

    chunkPixels = (s_tx.remainingPixels > ST7735_COLOR_CHUNK_PIXELS)
                ? ST7735_COLOR_CHUNK_PIXELS
                : s_tx.remainingPixels;

    if (chunkPixels == 0U)
    {
        ST7735_Unselect();
        s_tx.type = ST7735_TX_NONE;
        return 1U;
    }

    if (!ST7735_StartDma(s_ioBuffer, (uint16_t)(chunkPixels * 2U)))
    {
        HAL_SPI_Transmit(&hspi2, s_ioBuffer, (uint16_t)(chunkPixels * 2U), HAL_MAX_DELAY);
        s_tx.remainingPixels = (uint16_t)(s_tx.remainingPixels - chunkPixels);
    }
    else
    {
        s_tx.remainingPixels = (uint16_t)(s_tx.remainingPixels - chunkPixels);
        return 0U;
    }

    while (s_tx.remainingPixels > 0U)
    {
        chunkPixels = (s_tx.remainingPixels > ST7735_COLOR_CHUNK_PIXELS)
                    ? ST7735_COLOR_CHUNK_PIXELS
                    : s_tx.remainingPixels;
        HAL_SPI_Transmit(&hspi2, s_ioBuffer, (uint16_t)(chunkPixels * 2U), HAL_MAX_DELAY);
        s_tx.remainingPixels = (uint16_t)(s_tx.remainingPixels - chunkPixels);
    }

    ST7735_Unselect();
    s_tx.type = ST7735_TX_NONE;
    return 1U;
}

static uint8_t ST7735_ContinueColorRepeat(void)
{
    uint16_t chunkPixels;

    if (s_tx.type != ST7735_TX_COLOR_REPEAT)
        return 1U;

    if (s_spiDmaBusy != 0U)
        return 0U;

    if (s_spiDmaError != 0U)
    {
        ST7735_Unselect();
        s_tx.type = ST7735_TX_NONE;
        return 1U;
    }

    if (s_tx.remainingPixels == 0U)
    {
        ST7735_Unselect();
        s_tx.type = ST7735_TX_NONE;
        return 1U;
    }

    chunkPixels = (s_tx.remainingPixels > ST7735_COLOR_CHUNK_PIXELS)
                ? ST7735_COLOR_CHUNK_PIXELS
                : s_tx.remainingPixels;

    if (!ST7735_StartDma(s_ioBuffer, (uint16_t)(chunkPixels * 2U)))
    {
        HAL_SPI_Transmit(&hspi2, s_ioBuffer, (uint16_t)(chunkPixels * 2U), HAL_MAX_DELAY);
        s_tx.remainingPixels = (uint16_t)(s_tx.remainingPixels - chunkPixels);

        while (s_tx.remainingPixels > 0U)
        {
            chunkPixels = (s_tx.remainingPixels > ST7735_COLOR_CHUNK_PIXELS)
                        ? ST7735_COLOR_CHUNK_PIXELS
                        : s_tx.remainingPixels;
            HAL_SPI_Transmit(&hspi2, s_ioBuffer, (uint16_t)(chunkPixels * 2U), HAL_MAX_DELAY);
            s_tx.remainingPixels = (uint16_t)(s_tx.remainingPixels - chunkPixels);
        }

        ST7735_Unselect();
        s_tx.type = ST7735_TX_NONE;
        return 1U;
    }

    s_tx.remainingPixels = (uint16_t)(s_tx.remainingPixels - chunkPixels);
    return 0U;
}

static uint8_t ST7735_BeginCharTransfer(uint8_t x, uint8_t y, char ch, uint16_t color, uint16_t bgColor)
{
    uint16_t p = 0U;
    const uint8_t* glyph;
    uint8_t row;
    uint8_t col;

    if (((uint8_t)ch < ST7735_FONT_7X10_FIRST_CHAR) || ((uint8_t)ch > ST7735_FONT_7X10_LAST_CHAR))
        ch = '?';

    glyph = &Font7x10[ST7735_FONT_GLYPH_INDEX(ch)];

    for (row = 0U; row < ST7735_FONT_7X10_HEIGHT; row++)
    {
        uint8_t rowBits = glyph[row];

        for (col = 0U; col < ST7735_FONT_7X10_WIDTH; col++)
        {
            uint16_t drawColor =
                (rowBits & (uint8_t)(0x80U >> col)) ? color : bgColor;

            s_ioBuffer[p++] = (uint8_t)(drawColor >> 8);
            s_ioBuffer[p++] = (uint8_t)(drawColor & 0xFFU);
        }

        s_ioBuffer[p++] = (uint8_t)(bgColor >> 8);
        s_ioBuffer[p++] = (uint8_t)(bgColor & 0xFFU);
    }

    ST7735_SetAddressWindow(
        x,
        y,
        (uint8_t)(x + ST7735_TEXT_CELL_WIDTH - 1U),
        (uint8_t)(y + ST7735_FONT_7X10_HEIGHT - 1U));

    ST7735_Select();
    ST7735_DC_Data();

    s_tx.type = ST7735_TX_CHAR;
    s_tx.remainingPixels = 0U;
    s_tx.charBytes = p;

    if (!ST7735_StartDma(s_ioBuffer, p))
    {
        HAL_SPI_Transmit(&hspi2, s_ioBuffer, p, HAL_MAX_DELAY);
        ST7735_Unselect();
        s_tx.type = ST7735_TX_NONE;
        return 1U;
    }

    return 0U;
}

static uint8_t ST7735_ContinueCharTransfer(void)
{
    if (s_tx.type != ST7735_TX_CHAR)
        return 1U;

    if (s_spiDmaBusy != 0U)
        return 0U;

    ST7735_Unselect();
    s_tx.type = ST7735_TX_NONE;
    s_tx.charBytes = 0U;
    return 1U;
}

/* ---------- Queue helpers ---------- */

static void ST7735_QueueReset(void)
{
    s_queueHead = 0U;
    s_queueTail = 0U;
    s_queueCount = 0U;
}

static uint8_t ST7735_QueuePush(const ST7735_Command* cmd)
{
    if (s_queueCount >= ST7735_QUEUE_SIZE)
        return 0U;

    s_queue[s_queueTail] = *cmd;
    s_queueTail = (uint8_t)((s_queueTail + 1U) % ST7735_QUEUE_SIZE);
    s_queueCount++;
    return 1U;
}

static uint8_t ST7735_QueuePop(ST7735_Command* cmd)
{
    if (s_queueCount == 0U)
        return 0U;

    *cmd = s_queue[s_queueHead];
    s_queueHead = (uint8_t)((s_queueHead + 1U) % ST7735_QUEUE_SIZE);
    s_queueCount--;
    return 1U;
}

static void ST7735_RuntimeInit(void)
{
    ST7735_QueueReset();
    memset(&s_active, 0, sizeof(s_active));
    memset(&s_tx, 0, sizeof(s_tx));
    s_spiDmaBusy = 0U;
    s_spiDmaError = 0U;
}

/* ---------- Circle helpers ---------- */

static void ST7735_CircleResetSegments(void)
{
    s_active.state.circle.segCount = 0U;
    s_active.state.circle.segIndex = 0U;
}

static void ST7735_CircleAddSegment(int16_t x, int16_t y, int16_t h)
{
    uint8_t idx;

    if (h <= 0)
        return;

    if ((x < 0) || (x >= (int16_t)ST7735_WIDTH))
        return;

    if (y < 0)
    {
        h += y;
        y = 0;
    }

    if (y >= (int16_t)ST7735_HEIGHT)
        return;

    if ((y + h) > (int16_t)ST7735_HEIGHT)
        h = (int16_t)ST7735_HEIGHT - y;

    if (h <= 0)
        return;

    idx = s_active.state.circle.segCount;
    if (idx >= 4U)
        return;

    s_active.state.circle.segX[idx] = (uint8_t)x;
    s_active.state.circle.segY[idx] = (uint8_t)y;
    s_active.state.circle.segH[idx] = (uint8_t)h;
    s_active.state.circle.segCount++;
}

static uint8_t ST7735_CircleStartCurrentSegment(uint16_t color)
{
    uint8_t idx = s_active.state.circle.segIndex;
    uint8_t x = s_active.state.circle.segX[idx];
    uint8_t y = s_active.state.circle.segY[idx];
    uint8_t h = s_active.state.circle.segH[idx];

    ST7735_SetAddressWindow(x, y, x, (uint8_t)(y + h - 1U));
    return ST7735_BeginColorRepeat(color, h);
}

/* ---------- Active command helpers ---------- */

static uint8_t ST7735_StartNextCommand(void)
{
    if (s_active.busy)
        return 1U;

    if (!ST7735_QueuePop(&s_active.cmd))
        return 0U;

    s_active.busy = 1U;

    switch ((ST7735_CommandType)s_active.cmd.type)
    {
        case ST7735_CMD_FILL_RECT:
            s_active.state.rect.currentRow = 0U;
            break;

        case ST7735_CMD_FILL_CIRCLE:
            s_active.state.circle.x = 0;
            s_active.state.circle.y = s_active.cmd.data.fillCircle.r;
            s_active.state.circle.d = 1 - s_active.cmd.data.fillCircle.r;
            s_active.state.circle.firstStepDone = 0U;
            ST7735_CircleResetSegments();
            break;

        case ST7735_CMD_DRAW_TEXT:
            s_active.state.text.index = 0U;
            s_active.state.text.cursorX = s_active.cmd.data.drawText.x;
            s_active.state.text.cursorY = s_active.cmd.data.drawText.y;
            break;

        default:
            s_active.busy = 0U;
            s_active.cmd.type = ST7735_CMD_NONE;
            return 0U;
    }

    return 1U;
}

static uint8_t ST7735_ProcessRectLike(uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint16_t color)
{
    if (s_tx.type == ST7735_TX_COLOR_REPEAT)
    {
        if (!ST7735_ContinueColorRepeat())
            return 0U;

        s_active.state.rect.currentRow++;
        return (s_active.state.rect.currentRow >= h) ? 1U : 0U;
    }

    if (s_active.state.rect.currentRow >= h)
        return 1U;

    ST7735_SetAddressWindow(
        x,
        (uint8_t)(y + s_active.state.rect.currentRow),
        (uint8_t)(x + w - 1U),
        (uint8_t)(y + s_active.state.rect.currentRow));

    if (ST7735_BeginColorRepeat(color, w))
    {
        s_active.state.rect.currentRow++;
        return (s_active.state.rect.currentRow >= h) ? 1U : 0U;
    }

    return 0U;
}

static uint8_t ST7735_ProcessDrawTextStep(void)
{
    ST7735_DrawTextCmd* t = &s_active.cmd.data.drawText;
    char ch;

    if (s_tx.type == ST7735_TX_CHAR)
    {
        if (!ST7735_ContinueCharTransfer())
            return 0U;

        s_active.state.text.cursorX =
            (uint8_t)(s_active.state.text.cursorX + ST7735_TEXT_CELL_WIDTH);
        s_active.state.text.index++;
        return 0U;
    }

    ch = t->text[s_active.state.text.index];

    if (ch == '\0')
        return 1U;

    if (ch == '\n')
    {
        s_active.state.text.cursorX = t->x;
        s_active.state.text.cursorY =
            (uint8_t)(s_active.state.text.cursorY + ST7735_FONT_7X10_HEIGHT);
        s_active.state.text.index++;
        return 0U;
    }

    if ((uint16_t)s_active.state.text.cursorX + ST7735_TEXT_CELL_WIDTH > ST7735_WIDTH)
    {
        s_active.state.text.cursorX = t->x;
        s_active.state.text.cursorY =
            (uint8_t)(s_active.state.text.cursorY + ST7735_FONT_7X10_HEIGHT);
    }

    if ((uint16_t)s_active.state.text.cursorY + ST7735_FONT_7X10_HEIGHT > ST7735_HEIGHT)
        return 1U;

    if (ST7735_BeginCharTransfer(
            s_active.state.text.cursorX,
            s_active.state.text.cursorY,
            ch,
            t->color,
            t->bgColor))
    {
        s_active.state.text.cursorX =
            (uint8_t)(s_active.state.text.cursorX + ST7735_TEXT_CELL_WIDTH);
        s_active.state.text.index++;
    }

    return 0U;
}

static uint8_t ST7735_ProcessFillCircleStep(void)
{
    ST7735_FillCircleCmd* c = &s_active.cmd.data.fillCircle;
    int16_t x;
    int16_t y;
    int16_t d;

    if (c->r == 0U)
        return 1U;

    if (s_tx.type == ST7735_TX_COLOR_REPEAT)
    {
        if (!ST7735_ContinueColorRepeat())
            return 0U;

        s_active.state.circle.segIndex++;
        if (s_active.state.circle.segIndex < s_active.state.circle.segCount)
            return 0U;

        ST7735_CircleResetSegments();
        return 0U;
    }

    if (s_active.state.circle.segCount > 0U)
    {
        if (ST7735_CircleStartCurrentSegment(c->color))
        {
            s_active.state.circle.segIndex++;
            if (s_active.state.circle.segIndex >= s_active.state.circle.segCount)
                ST7735_CircleResetSegments();
        }
        return 0U;
    }

    if (!s_active.state.circle.firstStepDone)
    {
        s_active.state.circle.firstStepDone = 1U;
        ST7735_CircleResetSegments();
        ST7735_CircleAddSegment(
            c->x0,
            (int16_t)c->y0 - c->r,
            (int16_t)(2 * c->r + 1));

        if (s_active.state.circle.segCount == 0U)
            return 0U;

        if (ST7735_CircleStartCurrentSegment(c->color))
        {
            s_active.state.circle.segIndex++;
            if (s_active.state.circle.segIndex >= s_active.state.circle.segCount)
                ST7735_CircleResetSegments();
        }

        return 0U;
    }

    x = s_active.state.circle.x;
    y = s_active.state.circle.y;
    d = s_active.state.circle.d;

    if (x >= y)
        return 1U;

    if (d < 0)
    {
        d += 2 * x + 3;
    }
    else
    {
        d += 2 * (x - y) + 5;
        y--;
    }
    x++;

    s_active.state.circle.x = x;
    s_active.state.circle.y = y;
    s_active.state.circle.d = d;

    ST7735_CircleResetSegments();

    ST7735_CircleAddSegment((int16_t)c->x0 + x, (int16_t)c->y0 - y, (int16_t)(2 * y + 1));
    ST7735_CircleAddSegment((int16_t)c->x0 - x, (int16_t)c->y0 - y, (int16_t)(2 * y + 1));
    ST7735_CircleAddSegment((int16_t)c->x0 + y, (int16_t)c->y0 - x, (int16_t)(2 * x + 1));
    ST7735_CircleAddSegment((int16_t)c->x0 - y, (int16_t)c->y0 - x, (int16_t)(2 * x + 1));

    if (s_active.state.circle.segCount == 0U)
        return 0U;

    if (ST7735_CircleStartCurrentSegment(c->color))
    {
        s_active.state.circle.segIndex++;
        if (s_active.state.circle.segIndex >= s_active.state.circle.segCount)
            ST7735_CircleResetSegments();
    }

    return 0U;
}

/* ---------- Public API ---------- */

void ST7735_Init(void)
{
    ST7735_Reset();
    ST7735_WriteCommand(0x11);
    HAL_Delay(120);

    { const uint8_t data[] = { 0x05, 0x3A, 0x3A }; ST7735_WriteCommandWithData(0xB1, data, sizeof(data)); }
    { const uint8_t data[] = { 0x05, 0x3A, 0x3A }; ST7735_WriteCommandWithData(0xB2, data, sizeof(data)); }
    { const uint8_t data[] = { 0x05, 0x3A, 0x3A, 0x05, 0x3A, 0x3A }; ST7735_WriteCommandWithData(0xB3, data, sizeof(data)); }
    { const uint8_t data[] = { 0x03 }; ST7735_WriteCommandWithData(0xB4, data, sizeof(data)); }
    { const uint8_t data[] = { 0x64, 0x04, 0x84 }; ST7735_WriteCommandWithData(0xC0, data, sizeof(data)); }
    { const uint8_t data[] = { 0xC5 }; ST7735_WriteCommandWithData(0xC1, data, sizeof(data)); }
    { const uint8_t data[] = { 0x0D, 0x00 }; ST7735_WriteCommandWithData(0xC2, data, sizeof(data)); }
    { const uint8_t data[] = { 0x8D, 0x2A }; ST7735_WriteCommandWithData(0xC3, data, sizeof(data)); }
    { const uint8_t data[] = { 0x8D, 0xEE }; ST7735_WriteCommandWithData(0xC4, data, sizeof(data)); }
    { const uint8_t data[] = { 0x0E }; ST7735_WriteCommandWithData(0xC5, data, sizeof(data)); }
    { const uint8_t data[] = { 0xA8 }; ST7735_WriteCommandWithData(0x36, data, sizeof(data)); }

    {
        const uint8_t data[] = {
            0x15, 0x0B, 0x02, 0x00, 0x08, 0x00, 0x00, 0x00,
            0x00, 0x05, 0x11, 0x35, 0x10, 0x12, 0x05, 0x3F
        };
        ST7735_WriteCommandWithData(0xE0, data, sizeof(data));
    }

    {
        const uint8_t data[] = {
            0x0E, 0x0E, 0x03, 0x00, 0x06, 0x00, 0x00, 0x00,
            0x00, 0x06, 0x12, 0x37, 0x10, 0x10, 0x06, 0x3F
        };
        ST7735_WriteCommandWithData(0xE1, data, sizeof(data));
    }

    { const uint8_t data[] = { 0x05 }; ST7735_WriteCommandWithData(0x3A, data, sizeof(data)); }

    ST7735_WriteCommand(0x29);
    HAL_Delay(20);

    ST7735_RuntimeInit();
}

uint8_t ST7735_Clear(uint16_t color)
{
    ST7735_Command cmd;

    memset(&cmd, 0, sizeof(cmd));
    cmd.type = ST7735_CMD_FILL_RECT;
    cmd.data.fillRect.x = 0U;
    cmd.data.fillRect.y = 0U;
    cmd.data.fillRect.w = ST7735_WIDTH;
    cmd.data.fillRect.h = ST7735_HEIGHT;
    cmd.data.fillRect.color = color;

    /* Clear прерывает текущую команду и очищает очередь */
    s_active.busy = 0U;
    s_active.cmd.type = ST7735_CMD_NONE;
    s_tx.type = ST7735_TX_NONE;
    s_spiDmaBusy = 0U;
    ST7735_QueueReset();

    return ST7735_QueuePush(&cmd);
}

uint8_t ST7735_FillRect(uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint16_t color)
{
    ST7735_Command cmd;

    if ((x >= ST7735_WIDTH) || (y >= ST7735_HEIGHT) || (w == 0U) || (h == 0U))
        return 0U;

    if ((uint16_t)x + w > ST7735_WIDTH)
        w = (uint8_t)(ST7735_WIDTH - x);

    if ((uint16_t)y + h > ST7735_HEIGHT)
        h = (uint8_t)(ST7735_HEIGHT - y);

    memset(&cmd, 0, sizeof(cmd));
    cmd.type = ST7735_CMD_FILL_RECT;
    cmd.data.fillRect.x = x;
    cmd.data.fillRect.y = y;
    cmd.data.fillRect.w = w;
    cmd.data.fillRect.h = h;
    cmd.data.fillRect.color = color;

    return ST7735_QueuePush(&cmd);
}

uint8_t ST7735_FillCircle(uint8_t x0, uint8_t y0, uint8_t r, uint16_t color)
{
    ST7735_Command cmd;

    if (r == 0U)
        return 0U;

    memset(&cmd, 0, sizeof(cmd));
    cmd.type = ST7735_CMD_FILL_CIRCLE;
    cmd.data.fillCircle.x0 = x0;
    cmd.data.fillCircle.y0 = y0;
    cmd.data.fillCircle.r = r;
    cmd.data.fillCircle.color = color;

    return ST7735_QueuePush(&cmd);
}

uint8_t ST7735_DrawText(uint8_t x, uint8_t y, const char* text, uint16_t color, uint16_t bgColor)
{
    ST7735_Command cmd;
    uint8_t i = 0U;

    if (text == NULL)
        return 0U;

    memset(&cmd, 0, sizeof(cmd));
    cmd.type = ST7735_CMD_DRAW_TEXT;
    cmd.data.drawText.x = x;
    cmd.data.drawText.y = y;
    cmd.data.drawText.color = color;
    cmd.data.drawText.bgColor = bgColor;

    while ((i < ST7735_MAX_TEXT_LEN) && (text[i] != '\0'))
    {
        cmd.data.drawText.text[i] = text[i];
        i++;
    }
    cmd.data.drawText.text[i] = '\0';

    return ST7735_QueuePush(&cmd);
}

uint8_t ST7735_GetQueueFill(void)
{
    return s_queueCount;
}

void ST7735_Process(void)
{
    uint8_t done = 0U;

    if (!ST7735_StartNextCommand())
        return;

    switch ((ST7735_CommandType)s_active.cmd.type)
    {
        case ST7735_CMD_FILL_RECT:
            done = ST7735_ProcessRectLike(
                s_active.cmd.data.fillRect.x,
                s_active.cmd.data.fillRect.y,
                s_active.cmd.data.fillRect.w,
                s_active.cmd.data.fillRect.h,
                s_active.cmd.data.fillRect.color);
            break;

        case ST7735_CMD_FILL_CIRCLE:
            done = ST7735_ProcessFillCircleStep();
            break;

        case ST7735_CMD_DRAW_TEXT:
            done = ST7735_ProcessDrawTextStep();
            break;

        default:
            done = 1U;
            break;
    }

    if (done)
    {
        s_active.busy = 0U;
        s_active.cmd.type = ST7735_CMD_NONE;
    }
}

/* ---------- HAL callbacks ---------- */

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance == SPI2)
    {
        s_spiDmaBusy = 0U;
    }
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance == SPI2)
    {
        s_spiDmaError = 1U;
        s_spiDmaBusy = 0U;
    }
}
