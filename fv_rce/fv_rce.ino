#define DEBUG_HMI

#include <Arduino.h>
#include <ctype.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stddef.h>

#include "hmi.h"
#include "hmi_helpers.h"

static constexpr uint32_t HMI_TICK_PERIOD_MS = 5U;
static constexpr uint32_t TONE_BASE_HZ = 1000000UL;
static constexpr size_t SERIAL_LINE_CAP = 32U;

static hmiq_cmd_t s_cmdBeep = HMIQ_BEEP();
static hmiq_cmd_t s_hmiqRfSt = HMIQ_INDICATOR(0);
static hmiq_cmd_t s_hmiqUsbSt = HMIQ_INDICATOR(1);
static hmiq_cmd_t s_hmiqBattery = HMIQ_PROGRESS(0);
static hmiq_cmd_t s_hmiqRssiRx = HMIQ_PROGRESS(1);
static hmiq_cmd_t s_hmiqRssiTx = HMIQ_PROGRESS(2);

static hmiq_cmd_t* s_hmiSys[] = {
  &s_cmdBeep,
  &s_hmiqRfSt,
  &s_hmiqUsbSt,
  &s_hmiqBattery,
  &s_hmiqRssiRx,
  &s_hmiqRssiTx
};

static uint32_t s_nextHmiTickMs = 0U;
static bool s_serialStarted = false;
static char s_serialLine[SERIAL_LINE_CAP];
static size_t s_serialLineLen = 0U;

static void EnsureSerialStarted(void) {
  if (!s_serialStarted) {
    Serial.begin(115200);
    s_serialStarted = true;
  }
}

static void HmiLogToSerial(const char* text, bool emergency) {
  if (emergency) {
    EnsureSerialStarted();
  }

  if (!s_serialStarted || text == nullptr) {
    return;
  }

  Serial.println(text);
}
static void QueueUsbIndicator(bool value) {
  s_hmiqUsbSt.state.indicator.value = value ? 1U : 0U;
  s_hmiqUsbSt.hasData = true;
}
// Commands begin
typedef void (*cmd_func_t)(int32_t args[], uint8_t argsCount);
typedef struct {
  const char* name;
  uint8_t argsCount;
  cmd_func_t func;
} CommandEntry;

static const CommandEntry s_commands[] = {
  { "A", 1,
    [](int32_t args[], uint8_t argsCount) {
      if (argsCount < 1) return;
      s_hmiqBattery.state.progress.value = (uint8_t)args[0];
      s_hmiqBattery.hasData = true;
    } },
  { "B", 1,
    [](int32_t args[], uint8_t argsCount) {
      if (argsCount < 1) return;
      s_hmiqRssiRx.state.progress.value = (uint8_t)args[0];
      s_hmiqRssiRx.hasData = true;
    } },
  { "C", 1,
    [](int32_t args[], uint8_t argsCount) {
      if (argsCount < 1) return;
      s_hmiqRssiTx.state.progress.value = (uint8_t)args[0];
      s_hmiqRssiTx.hasData = true;
    } },
  { "D", 1,
    [](int32_t args[], uint8_t argsCount) {
      if (argsCount < 1) return;
      s_hmiqRfSt.state.indicator.value = args[0] ? 1U : 0U;
      s_hmiqRfSt.hasData = true;
    } },
  { "T", 2,
    [](int32_t args[], uint8_t argsCount) {
      if (argsCount < 2) return;

      uint32_t hz = (uint32_t)args[0];
      if (args[1] < 0 || (args[1] > 65535U)) return;
      const uint32_t divider32 = (hz > 20 && hz < 20000) ? (TONE_BASE_HZ / hz) : 0;
      s_cmdBeep.state.beep.divider = (uint16_t)divider32;
      s_cmdBeep.state.beep.durationMs = (uint16_t)args[1];
      s_cmdBeep.hasData = true;
    } }
};

static constexpr uint8_t COMMAND_COUNT =
  sizeof(s_commands) / sizeof(s_commands[0]);

static constexpr uint8_t MAX_ARGS = 8;


static bool ParseInt32(const char* text, int32_t* outValue) {
  if ((text == nullptr) || (*text == '\0') || (outValue == nullptr)) return false;
  char* endPtr = nullptr;
  long value = strtol(text, &endPtr, 10);
  if (*endPtr != '\0') return false;
  if ((value < INT32_MIN) || (value > INT32_MAX)) return false;
  *outValue = (int32_t)value;
  return true;
}


static void ParseAndDispatch(char* line) {
  if ((line == nullptr) || (*line == '\0')) return;
  char* savePtr = nullptr;

  char* token = strtok_r(line, " \t", &savePtr);
  if (token == nullptr) return;

  uint8_t cmdIdx;
  for (cmdIdx = 0; cmdIdx < COMMAND_COUNT; ++cmdIdx) {
    if (strcmp(token, s_commands[cmdIdx].name) == 0) break;
  }
  if (cmdIdx >= COMMAND_COUNT) return;
  const CommandEntry* cmd = &s_commands[cmdIdx];
  if (cmd == nullptr) return;
  if (cmd->argsCount > MAX_ARGS) return;

  int32_t args[MAX_ARGS];
  uint8_t argsCount = 0;

  while (true) {
    token = strtok_r(nullptr, " \t", &savePtr);
    if (token == nullptr) break;
    if (argsCount >= MAX_ARGS) return;
    if (!ParseInt32(token, &args[argsCount])) return;
    ++argsCount;
  }
  if (argsCount < cmd->argsCount) return;
  cmd->func(args, argsCount);
}

// Commands End

static void PollSerialRx(void) {
  while (s_serialStarted && (Serial.available() > 0)) {
    const char ch = (char)Serial.read();
    if ((ch == '\r') || (ch == '\n')) {
      if (s_serialLineLen > 0U) {
        s_serialLine[s_serialLineLen] = '\0';
        ParseAndDispatch(s_serialLine);
        s_serialLineLen = 0U;
      }
      continue;
    }

    if ((s_serialLineLen + 1U) < SERIAL_LINE_CAP) {
      s_serialLine[s_serialLineLen++] = ch;
    } else {
      s_serialLineLen = 0U;
    }
  }
}

static void HandleUsbConnChanged(void) {
  if (hmi_get(HMI_DATA_STAT_USB_CONN) != 0U) {
    EnsureSerialStarted();
    QueueUsbIndicator(true);
  } else {
    if (s_serialStarted) {
      Serial.end();
      s_serialStarted = false;
    }
    QueueUsbIndicator(false);
  }
}

static void TickHmiOnce(void) {
  const hmi_tick_result_t rc = hmi_tick();
  if (rc != HMI_TICK_OK) {
    return;
  }

  hmi_data_idx_t idx;
  while ((idx = hmi_next_changed()) != HMI_DATA_COUNT) {
    if (idx == HMI_DATA_STAT_USB_CONN) {
      HandleUsbConnChanged();
    }
  }
  (void)HmiqServiceOne(s_hmiSys, sizeof(s_hmiSys) / sizeof(s_hmiSys[0]));
}

void setup() {
  hmi_init(HmiLogToSerial);
  s_nextHmiTickMs = millis() + HMI_TICK_PERIOD_MS;
}

void loop() {
  PollSerialRx();

  const uint32_t now = millis();
  if ((int32_t)(now - s_nextHmiTickMs) >= 0) {
    TickHmiOnce();
    s_nextHmiTickMs += HMI_TICK_PERIOD_MS;
  }
}
