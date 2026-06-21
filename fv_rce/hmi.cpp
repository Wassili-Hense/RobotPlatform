#include "hmi.h"
#include <Arduino.h>
#include <Wire.h>
#include <string.h>

static constexpr uint8_t I2C_ADDR = 0x14;
static constexpr int I2C_SDA = 21;
static constexpr int I2C_SCL = 22;
static constexpr uint8_t I2C_READ_LEN = 6;

static constexpr uint8_t STATUS_BIT_USB_CONNECTED = 12;
static constexpr uint8_t STATUS_BIT_BACKLIGHT_ON  = 14;
static constexpr uint8_t STATUS_BIT_LCD_BUSY      = 15;

static constexpr uint8_t CMD_BACKLIGHT_TIMEOUT    = 0x03;
static constexpr uint8_t CMD_BACKLIGHT_BRIGHTNESS = 0x04;
static constexpr uint8_t CMD_TONE                 = 0x07;
static constexpr uint8_t CMD_LCD_CLEAR            = 0x10;
static constexpr uint8_t CMD_LCD_DRAW_MARKER      = 0x12;
static constexpr uint8_t CMD_LCD_DRAW_TEXT        = 0x13;
static constexpr uint8_t CMD_LCD_SET_BG           = 0x14;
static constexpr uint8_t CMD_LCD_INDICATOR        = 0x20;
static constexpr uint8_t CMD_LCD_PROGRESS         = 0x21;

static constexpr uint8_t PRIO_LOW  = 0;
static constexpr uint8_t PRIO_MED  = 1;
static constexpr uint8_t PRIO_HIGH = 2;

static constexpr size_t CMD_MAX_LEN = 32;
static constexpr size_t CMD_QUEUE_CAPACITY = 56;

struct CmdItem
{
    bool used = false;
    uint8_t data[CMD_MAX_LEN]{};
    uint8_t len = 0;
    uint8_t prio = PRIO_MED;
    bool isLcd = false;
    bool requiresLcdReady = false;
    uint32_t seq = 0;
};

static CmdItem s_cmdQueue[CMD_QUEUE_CAPACITY];
static uint32_t s_cmdSeq = 1U;
static bool s_lcdSendAllowed = false;
static bool s_initialized = false;
static uint32_t s_dataBits = 0U;
static uint32_t s_changed = 0U;
static uint16_t s_joyX = 0U;
static uint16_t s_joyY = 0U;

static bool QueueCommand(const uint8_t* data, uint8_t len, uint8_t prio, bool requiresLcdReady, bool isLcd)
{
    if (data == nullptr || len == 0U || len > CMD_MAX_LEN)
    {
        return false;
    }
    for (size_t i = 0; i < CMD_QUEUE_CAPACITY; ++i)
    {
        if (!s_cmdQueue[i].used)
        {
            s_cmdQueue[i].used = true;
            memcpy(s_cmdQueue[i].data, data, len);
            s_cmdQueue[i].len = len;
            s_cmdQueue[i].prio = prio;
            s_cmdQueue[i].requiresLcdReady = requiresLcdReady;
            s_cmdQueue[i].isLcd = isLcd;
            s_cmdQueue[i].seq = s_cmdSeq++;
            return true;
        }
    }
    return false;
}

static int FindBestQueuedCommand(void)
{
    int bestIndex = -1;
    uint8_t bestPrio = 0U;
    uint32_t bestSeq = 0xFFFFFFFFu;
    for (size_t i = 0; i < CMD_QUEUE_CAPACITY; ++i)
    {
        const CmdItem& item = s_cmdQueue[i];
        if (!item.used)
        {
            continue;
        }
        if (item.requiresLcdReady && !s_lcdSendAllowed)
        {
            continue;
        }
        if (bestIndex < 0 || item.prio > bestPrio || (item.prio == bestPrio && item.seq < bestSeq))
        {
            bestIndex = (int)i;
            bestPrio = item.prio;
            bestSeq = item.seq;
        }
    }
    return bestIndex;
}

static bool SendOneQueuedCommand(void)
{
    const int index = FindBestQueuedCommand();
    if (index < 0)
    {
        return true;
    }

    const CmdItem item = s_cmdQueue[index];
    Wire.beginTransmission(I2C_ADDR);
    const size_t written = Wire.write(item.data, item.len);
    const uint8_t rc = Wire.endTransmission(true);
    if (rc != 0U || written != item.len)
    {
        return false;
    }
    if (item.isLcd)
    {
        s_lcdSendAllowed = false;
    }
    s_cmdQueue[index].used = false;
    return true;
}

static uint32_t set_bit(uint32_t data, uint8_t idx, uint8_t value)
{
    return (data & ~(1UL << idx)) | (((uint32_t)(value & 1U)) << idx);
}

static uint16_t read_le16(const uint8_t* p)
{
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static bool ParsePacket(const uint8_t* rx)
{
    const uint16_t word0 = read_le16(&rx[0]);
    const uint16_t joyX  = (uint16_t)(read_le16(&rx[2]) & 0x0FFFU);
    const uint16_t joyY  = (uint16_t)(read_le16(&rx[4]) & 0x0FFFU);

    uint32_t newData = s_dataBits;

    newData = set_bit(newData, HMI_DATA_STAT_LCD_BUSY, (word0 & (1U << STATUS_BIT_LCD_BUSY)) ? 1U : 0U);
    newData = set_bit(newData, HMI_DATA_STAT_USB_CONN, (word0 & (1U << STATUS_BIT_USB_CONNECTED)) ? 1U : 0U);
    newData = set_bit(newData, HMI_DATA_STAT_BL_ON,    (word0 & (1U << STATUS_BIT_BACKLIGHT_ON)) ? 1U : 0U);

    s_lcdSendAllowed = ((word0 & (1U << STATUS_BIT_LCD_BUSY)) == 0U);

    if (s_joyX != joyX)
    {
        s_joyX = joyX;
        newData = set_bit(newData, HMI_DATA_JOY_X, 1U);
    }

    if (s_joyY != joyY)
    {
        s_joyY = joyY;
        newData = set_bit(newData, HMI_DATA_JOY_Y, 1U);
    }

    const uint32_t newButtons = (uint32_t)(word0 & 0x03FFU);
    newData &= ~(0x03FFUL << (uint8_t)HMI_DATA_BTN_ON);
    newData |=  (newButtons << (uint8_t)HMI_DATA_BTN_ON);

    s_changed |= (s_dataBits ^ newData);
    s_dataBits = newData & ~((1UL << HMI_DATA_JOY_X) | (1UL << HMI_DATA_JOY_Y));
    return true;
}

static hmi_tick_result_t PollI2C(void)
{
    uint8_t rx[I2C_READ_LEN];
    uint8_t len = 0U;
    hmi_tick_result_t result = HMI_TICK_OK;

    const int requested = Wire.requestFrom((int)I2C_ADDR, (int)I2C_READ_LEN, (int)true);
    while (Wire.available() && len < I2C_READ_LEN)
    {
        rx[len++] = (uint8_t)Wire.read();
    }

    if (requested != I2C_READ_LEN)
    {
        result = (hmi_tick_result_t)(result | HMI_TICK_ERR_I2C_REQUEST);
    }
    if (len != I2C_READ_LEN)
    {
        result = (hmi_tick_result_t)(result | HMI_TICK_ERR_I2C_READ);
    }
    if (result != HMI_TICK_OK)
    {
        return result;
    }
    if (!ParsePacket(rx))
    {
        result = (hmi_tick_result_t)(result | HMI_TICK_ERR_BAD_PACKET);
    }
    return result;
}

void hmi_init(void)
{
    memset(s_cmdQueue, 0, sizeof(s_cmdQueue));
    s_cmdSeq = 1U;
    s_lcdSendAllowed = false;
    s_dataBits = 0U;
    s_changed = 0U;
    s_joyX = 0U;
    s_joyY = 0U;

    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.setClock(100000);
    s_initialized = true;
}

hmi_tick_result_t hmi_tick(void)
{
    if (!s_initialized)
    {
        return HMI_TICK_ERR_NOT_INITIALIZED;
    }

    hmi_tick_result_t result = PollI2C();
    if (result == HMI_TICK_OK)
    {
        if (!SendOneQueuedCommand())
        {
            result = (hmi_tick_result_t)(result | HMI_TICK_ERR_TX);
        }
    }
    return result;
}

uint16_t hmi_get(hmi_data_idx_t idx)
{
    if ((uint8_t)idx >= (uint8_t)HMI_DATA_COUNT)
    {
        return 0U;
    }
    if (idx == HMI_DATA_JOY_X)
    {
        return s_joyX;
    }
    if (idx == HMI_DATA_JOY_Y)
    {
        return s_joyY;
    }
    return ((s_dataBits & (1UL << (uint8_t)idx)) != 0UL) ? 1U : 0U;
}

hmi_data_idx_t hmi_get_next_changed(void)
{
    for (uint8_t i = 0U; i < (uint8_t)HMI_DATA_COUNT; ++i)
    {
        if ((s_changed & (1UL << i)) != 0UL)
        {
            s_changed &= ~(1UL << i);
            return (hmi_data_idx_t)i;
        }
    }
    return HMI_DATA_COUNT;
}

bool hmi_cmd_set_backlight_timeout(uint32_t timeout_ms)
{
    uint8_t data[5];
    data[0] = CMD_BACKLIGHT_TIMEOUT;
    data[1] = (uint8_t)(timeout_ms & 0xFFU);
    data[2] = (uint8_t)((timeout_ms >> 8) & 0xFFU);
    data[3] = (uint8_t)((timeout_ms >> 16) & 0xFFU);
    data[4] = (uint8_t)((timeout_ms >> 24) & 0xFFU);
    return QueueCommand(data, sizeof(data), PRIO_MED, false, false);
}

bool hmi_cmd_set_brightness(uint8_t level)
{
    const uint8_t data[2] = { CMD_BACKLIGHT_BRIGHTNESS, level };
    return QueueCommand(data, sizeof(data), PRIO_MED, false, false);
}

bool hmi_cmd_play_tone(uint16_t divider, uint16_t delay_ms)
{
    uint8_t data[5];
    data[0] = CMD_TONE;
    data[1] = (uint8_t)(divider & 0xFFU);
    data[2] = (uint8_t)(divider >> 8);
    data[3] = (uint8_t)(delay_ms & 0xFFU);
    data[4] = (uint8_t)(delay_ms >> 8);
    return QueueCommand(data, sizeof(data), PRIO_HIGH, false, false);
}

bool hmi_cmd_lcd_clear(uint16_t rgb565_color)
{
    uint8_t data[3];
    data[0] = CMD_LCD_CLEAR;
    data[1] = (uint8_t)(rgb565_color & 0xFFU);
    data[2] = (uint8_t)(rgb565_color >> 8);
    return QueueCommand(data, sizeof(data), PRIO_MED, true, true);
}

bool hmi_cmd_lcd_set_bg(uint16_t rgb565_color)
{
    uint8_t data[3];
    data[0] = CMD_LCD_SET_BG;
    data[1] = (uint8_t)(rgb565_color & 0xFFU);
    data[2] = (uint8_t)(rgb565_color >> 8);
    return QueueCommand(data, sizeof(data), PRIO_MED, true, true);
}

bool hmi_cmd_lcd_draw_text(uint8_t x, uint8_t y, uint16_t rgb565_color, const char* text)
{
    if (text == nullptr)
    {
        return false;
    }
    const size_t textLen = strlen(text);
    if (textLen > 26U)
    {
        return false;
    }
    uint8_t data[32] = { 0 };
    data[0] = CMD_LCD_DRAW_TEXT;
    data[1] = x;
    data[2] = y;
    data[3] = (uint8_t)(rgb565_color & 0xFFU);
    data[4] = (uint8_t)(rgb565_color >> 8);
    memcpy(&data[5], text, textLen);
    data[5U + textLen] = 0U;
    return QueueCommand(data, (uint8_t)(6U + textLen), PRIO_MED, true, true);
}

bool hmi_cmd_lcd_draw_marker(uint8_t x, uint8_t y, uint8_t index, uint16_t rgb565_color)
{
    uint8_t data[6];
    data[0] = CMD_LCD_DRAW_MARKER;
    data[1] = x;
    data[2] = y;
    data[3] = index;
    data[4] = (uint8_t)(rgb565_color & 0xFFU);
    data[5] = (uint8_t)(rgb565_color >> 8);
    return QueueCommand(data, sizeof(data), PRIO_HIGH, true, true);
}

bool hmi_cmd_lcd_set_indicator(uint8_t index, bool state)
{
    const uint8_t data[3] = { CMD_LCD_INDICATOR, index, (uint8_t)(state ? 1U : 0U) };
    return QueueCommand(data, sizeof(data), PRIO_LOW, true, true);
}

bool hmi_cmd_lcd_set_progress(uint8_t index, uint8_t value)
{
    const uint8_t data[3] = { CMD_LCD_PROGRESS, index, value };
    return QueueCommand(data, sizeof(data), PRIO_LOW, true, true);
}
