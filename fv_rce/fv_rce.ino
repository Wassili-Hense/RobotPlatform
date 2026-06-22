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
static constexpr uint8_t HMI_START_BRIGHTNESS = 30U;
static constexpr uint32_t HMI_START_BL_TIMEOUT_MS = 30000U;

static hmis_cmd_t s_cmdBeep = HMIS_BEEP();
static hmis_cmd_t s_hmisBrightness = HMIS_BRIGHTNESS();
static hmis_cmd_t s_hmisBlTimeout = HMIS_BL_TIMEOUT();
static hmis_cmd_t s_hmisRfSt = HMIS_INDICATOR(0);
static hmis_cmd_t s_hmisUsbSt = HMIS_INDICATOR(1);
static hmis_cmd_t s_hmisBattery = HMIS_PROGRESS(0);
static hmis_cmd_t s_hmisRssiRx = HMIS_PROGRESS(1);
static hmis_cmd_t s_hmisRssiTx = HMIS_PROGRESS(2);

static hmis_cmd_t* s_hmiSys[] = {
  &s_cmdBeep,
  &s_hmisBrightness,
  &s_hmisBlTimeout,
  &s_hmisRfSt,
  &s_hmisUsbSt,
  &s_hmisBattery,
  &s_hmisRssiRx,
  &s_hmisRssiTx
};

extern hmig_scene_t s_sceneHome;
extern hmig_scene_t s_sceneMainMenu;
extern hmig_scene_t s_sceneTest1;
extern hmig_scene_t s_sceneTest2;

static HmigClsComponent s_sceneHomeCls(0x0000U);
static HmigJViewComponent s_sceneHomeJView;
static HmigHotKeyComponent s_sceneHomeHotKeyOk(HMI_DATA_BTN_OK, &s_sceneMainMenu);
static HmigComponent* s_sceneHomeItems[] = {
  &s_sceneHomeCls,
  &s_sceneHomeJView,
  &s_sceneHomeHotKeyOk
};
hmig_scene_t s_sceneHome = {
  s_sceneHomeItems,
  sizeof(s_sceneHomeItems) / sizeof(s_sceneHomeItems[0])
};

static HmigClsComponent s_sceneMainMenuCls(0x0000U);
static HmigLabelComponent s_sceneMainMenuTitle(40U, 20U, 0xFFFFU, "Main menu");
static HmigMenuItemComponent s_sceneMainMenuItemCalCenter(20U, 45U, "Cal. center", &s_sceneTest1);
static HmigMenuItemComponent s_sceneMainMenuItemCalEdge(20U, 58U, "Cal. edge", &s_sceneTest2);
static HmigHotKeyComponent s_sceneMainMenuHotKeyBack(HMI_DATA_BTN_BACK, &s_sceneHome);
static HmigComponent* s_sceneMainMenuItems[] = {
  &s_sceneMainMenuCls,
  &s_sceneMainMenuTitle,
  &s_sceneMainMenuItemCalCenter,
  &s_sceneMainMenuItemCalEdge,
  &s_sceneMainMenuHotKeyBack
};
hmig_scene_t s_sceneMainMenu = {
  s_sceneMainMenuItems,
  sizeof(s_sceneMainMenuItems) / sizeof(s_sceneMainMenuItems[0])
};

static HmigClsComponent s_sceneTest1Cls(0x0000U);
static HmigLabelComponent s_sceneTest1Label(40U, 40U, 0xFFFFU, "Test scene 1");
static HmigHotKeyComponent s_sceneTest1HotKeyBack(HMI_DATA_BTN_BACK, &s_sceneMainMenu);
static HmigComponent* s_sceneTest1Items[] = {
  &s_sceneTest1Cls,
  &s_sceneTest1Label,
  &s_sceneTest1HotKeyBack
};
hmig_scene_t s_sceneTest1 = {
  s_sceneTest1Items,
  sizeof(s_sceneTest1Items) / sizeof(s_sceneTest1Items[0])
};

static HmigClsComponent s_sceneTest2Cls(0x0000U);
static HmigLabelComponent s_sceneTest2Label(40U, 40U, 0xFFFFU, "Test scene 2");
static HmigHotKeyComponent s_sceneTest2HotKeyBack(HMI_DATA_BTN_BACK, &s_sceneMainMenu);
static HmigComponent* s_sceneTest2Items[] = {
  &s_sceneTest2Cls,
  &s_sceneTest2Label,
  &s_sceneTest2HotKeyBack
};
hmig_scene_t s_sceneTest2 = {
  s_sceneTest2Items,
  sizeof(s_sceneTest2Items) / sizeof(s_sceneTest2Items[0])
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
  if (!s_serialStarted || (text == nullptr)) {
    return;
  }
  Serial.println(text);
}

static void QueueUsbIndicator(bool value) {
  s_hmisUsbSt.state.indicator.value = value ? 1U : 0U;
  s_hmisUsbSt.hasData = true;
}

typedef void (*cmd_func_t)(int32_t args[], uint8_t argsCount);
typedef struct {
  const char* name;
  uint8_t argsCount;
  cmd_func_t func;
} CommandEntry;

static const CommandEntry s_commands[] = {
  { "A", 1, [](int32_t args[], uint8_t argsCount) {
      if (argsCount < 1U) return;
      s_hmisBattery.state.progress.value = (uint8_t)args[0];
      s_hmisBattery.hasData = true;
    } },
  { "B", 1, [](int32_t args[], uint8_t argsCount) {
      if (argsCount < 1U) return;
      s_hmisRssiRx.state.progress.value = (uint8_t)args[0];
      s_hmisRssiRx.hasData = true;
    } },
  { "C", 1, [](int32_t args[], uint8_t argsCount) {
      if (argsCount < 1U) return;
      s_hmisRssiTx.state.progress.value = (uint8_t)args[0];
      s_hmisRssiTx.hasData = true;
    } },
  { "D", 1, [](int32_t args[], uint8_t argsCount) {
      if (argsCount < 1U) return;
      s_hmisRfSt.state.indicator.value = args[0] ? 1U : 0U;
      s_hmisRfSt.hasData = true;
    } },
  { "T", 2, [](int32_t args[], uint8_t argsCount) {
      if (argsCount < 2U) return;
      const uint32_t hz = (uint32_t)args[0];
      if ((args[1] < 0) || ((uint32_t)args[1] > 65535U)) return;
      const uint32_t divider32 = (hz > 20U && hz < 20000U) ? (TONE_BASE_HZ / hz) : 0U;
      s_cmdBeep.state.beep.divider = (uint16_t)divider32;
      s_cmdBeep.state.beep.durationMs = (uint16_t)args[1];
      s_cmdBeep.hasData = true;
    } }
};

static constexpr uint8_t COMMAND_COUNT = sizeof(s_commands) / sizeof(s_commands[0]);
static constexpr uint8_t MAX_ARGS = 8U;

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
  uint8_t cmdIdx = 0U;
  for (; cmdIdx < COMMAND_COUNT; ++cmdIdx) {
    if (strcmp(token, s_commands[cmdIdx].name) == 0) break;
  }
  if (cmdIdx >= COMMAND_COUNT) return;
  const CommandEntry* cmd = &s_commands[cmdIdx];
  if (cmd->argsCount > MAX_ARGS) return;
  int32_t args[MAX_ARGS];
  uint8_t argsCount = 0U;
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

static void InitStartupCommands(void) {
  s_hmisBrightness.state.brightness.level = HMI_START_BRIGHTNESS;
  s_hmisBrightness.hasData = true;
  s_hmisBlTimeout.state.blTimeout.timeoutMs = HMI_START_BL_TIMEOUT_MS;
  s_hmisBlTimeout.hasData = true;
}

static void TickHmiOnce(void) {
  const hmi_tick_result_t rc = hmi_tick();
  if (rc != HMI_TICK_OK) return;
  if (hmi_changed(HMI_DATA_STAT_USB_CONN)) {
    HandleUsbConnChanged();
  }
  const bool sceneSent = HmigServiceActiveScene();
  if (!sceneSent) {
    (void)HmisService(s_hmiSys, sizeof(s_hmiSys) / sizeof(s_hmiSys[0]));
  }
}

void setup() {
  hmi_init(HmiLogToSerial);
  InitStartupCommands();
  HmigSwitchScene(&s_sceneHome);
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
