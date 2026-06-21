#define DEBUG_HMI

#include <Arduino.h>
#include <ctype.h>
#include <stdlib.h>
#include <string.h>

#include "hmi.h"
#include "hmi_helpers.h"

static constexpr uint32_t HMI_TICK_PERIOD_MS = 5U;
static constexpr uint32_t TONE_BASE_HZ = 1000000UL;
static constexpr size_t SERIAL_LINE_CAP = 32U;

static hmiq_cmd_t s_hmiqUsbSt = HMIQ_INDICATOR(1);
static hmiq_cmd_t s_hmiqRfSt   = HMIQ_INDICATOR(0);
static hmiq_cmd_t s_hmiqBattery    = HMIQ_PROGRESS(0);
static hmiq_cmd_t s_hmiqRssiRx    = HMIQ_PROGRESS(1);
static hmiq_cmd_t s_hmiqRssiTx    = HMIQ_PROGRESS(2);
static hmiq_cmd_t s_cmdBeep         = HMIQ_BEEP();

static hmiq_cmd_t* s_hmiSys[] = {
    &s_hmiqUsbSt,
    &s_hmiqRfSt,
    &s_hmiqBattery,
    &s_hmiqRssiRx,
    &s_hmiqRssiTx,
    &s_cmdBeep
};

static uint32_t s_nextHmiTickMs = 0U;
static bool s_serialStarted = false;
static char s_serialLine[SERIAL_LINE_CAP];
static size_t s_serialLineLen = 0U;

static void EnsureSerialStarted(void)
{
    if (!s_serialStarted)
    {
        Serial.begin(115200);
        s_serialStarted = true;
    }
}

static void HmiLogToSerial(const char* text, bool emergency)
{
    if (emergency)
    {
        EnsureSerialStarted();
    }

    if (!s_serialStarted || text == nullptr)
    {
        return;
    }

    Serial.println(text);
}

static bool ParseFixed2Digits(const char* s, uint8_t* value)
{
    if ((s == nullptr) || (value == nullptr))
    {
        return false;
    }
    if (!isdigit((unsigned char)s[0]) || !isdigit((unsigned char)s[1]) || (s[2] != '\0'))
    {
        return false;
    }

    *value = (uint8_t)(((uint8_t)(s[0] - '0') * 10U) + (uint8_t)(s[1] - '0'));
    return true;
}

static bool ParseFixed1Digit01(const char* s, bool* value)
{
    if ((s == nullptr) || (value == nullptr))
    {
        return false;
    }
    if (((s[0] != '0') && (s[0] != '1')) || (s[1] != '\0'))
    {
        return false;
    }

    *value = (s[0] == '1');
    return true;
}

static bool ParseUnsigned32(const char* text, uint32_t* value)
{
    if ((text == nullptr) || (value == nullptr) || (*text == '\0'))
    {
        return false;
    }

    char* endPtr = nullptr;
    const unsigned long parsed = strtoul(text, &endPtr, 10);
    if ((endPtr == text) || (*endPtr != '\0'))
    {
        return false;
    }

    *value = (uint32_t)parsed;
    return true;
}

static void QueueUsbIndicator(bool value)
{
    s_hmiqUsbSt.state.indicator.value = value ? 1U : 0U;
    s_hmiqUsbSt.hasData = true;
}

static void HandleSerialCommand(const char* line)
{
    if ((line == nullptr) || (line[0] == '\0'))
    {
        return;
    }

    if (((line[0] == 'A') || (line[0] == 'B') || (line[0] == 'C')) && (line[1] != '\0'))
    {
        uint8_t value = 0U;
        if (!ParseFixed2Digits(&line[1], &value))
        {
            return;
        }

        const uint8_t index = (line[0] == 'A') ? 0U : ((line[0] == 'B') ? 1U : 2U);
    switch (index)
    {
        case 0U:
            s_hmiqBattery.state.progress.value = value;
            s_hmiqBattery.hasData = true;
            break;
        case 1U:
            s_hmiqRssiRx.state.progress.value = value;
            s_hmiqRssiRx.hasData = true;
            break;
        case 2U:
            s_hmiqRssiTx.state.progress.value = value;
            s_hmiqRssiTx.hasData = true;
            break;
        default:
            break;
    }
        return;
    }

    if ((line[0] == 'D') && (line[1] != '\0'))
    {
        bool value = false;
        if (!ParseFixed1Digit01(&line[1], &value))
        {
            return;
        }
        s_hmiqRfSt.state.indicator.value = value ? 1U : 0U;
        s_hmiqRfSt.hasData = true;
        return;
    }

    if (line[0] == 'T')
    {
        const char* comma = strchr(&line[1], ',');
        if (comma == nullptr)
        {
            return;
        }

        char hzText[12];
        char msText[12];
        const size_t hzLen = (size_t)(comma - (&line[1]));
        const size_t msLen = strlen(comma + 1);
        if ((hzLen == 0U) || (hzLen >= sizeof(hzText)) || (msLen == 0U) || (msLen >= sizeof(msText)))
        {
            return;
        }

        memcpy(hzText, &line[1], hzLen);
        hzText[hzLen] = '\0';
        memcpy(msText, comma + 1, msLen + 1U);

        uint32_t hz = 0U;
        uint32_t ms = 0U;
        if (!ParseUnsigned32(hzText, &hz) || !ParseUnsigned32(msText, &ms))
        {
            return;
        }
        if ((hz == 0U) || (hz > TONE_BASE_HZ) || (ms > 65535U))
        {
            return;
        }

        const uint32_t divider32 = TONE_BASE_HZ / hz;
        if ((divider32 == 0U) || (divider32 > 65535U))
        {
            return;
        }
        s_cmdBeep.state.beep.divider = (uint16_t)divider32;
        s_cmdBeep.state.beep.durationMs = (uint16_t)ms;
        s_cmdBeep.hasData = true;
    }
}

static void CommitSerialLine(void)
{
    s_serialLine[s_serialLineLen] = '\0';
    HandleSerialCommand(s_serialLine);
    s_serialLineLen = 0U;
}

static void PollSerialRx(void)
{
    while (s_serialStarted && (Serial.available() > 0))
    {
        const char ch = (char)Serial.read();
        if ((ch == '\r') || (ch == '\n'))
        {
            if (s_serialLineLen > 0U)
            {
                CommitSerialLine();
            }
            continue;
        }

        if ((s_serialLineLen + 1U) < SERIAL_LINE_CAP)
        {
            s_serialLine[s_serialLineLen++] = ch;
        }
        else
        {
            s_serialLineLen = 0U;
        }
    }
}

static void HandleUsbConnChanged(void)
{
    if (hmi_get(HMI_DATA_STAT_USB_CONN) != 0U)
    {
        EnsureSerialStarted();
        QueueUsbIndicator(true);
    }
    else
    {
        if (s_serialStarted)
        {
            Serial.end();
            s_serialStarted = false;
        }
        QueueUsbIndicator(false);
    }
}

static void TickHmiOnce(void)
{
    const hmi_tick_result_t rc = hmi_tick();
    if (rc != HMI_TICK_OK)
    {
        return;
    }

    hmi_data_idx_t idx;
    while ((idx = hmi_next_changed()) != HMI_DATA_COUNT)
    {
        if (idx == HMI_DATA_STAT_USB_CONN)
        {
            HandleUsbConnChanged();
        }
    }
    (void)HmiqServiceOne(s_hmiSys, sizeof(s_hmiSys) / sizeof(s_hmiSys[0]));
}

void setup()
{
    hmi_init(HmiLogToSerial);
    s_nextHmiTickMs = millis() + HMI_TICK_PERIOD_MS;
}

void loop()
{
    PollSerialRx();

    const uint32_t now = millis();
    if ((int32_t)(now - s_nextHmiTickMs) >= 0)
    {
            TickHmiOnce();
            s_nextHmiTickMs += HMI_TICK_PERIOD_MS;
    }
}
