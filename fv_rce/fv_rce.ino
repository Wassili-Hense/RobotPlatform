#define DEBUG_HMI
#include <Arduino.h>
#include <ctype.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stddef.h>
#include "hmi.h"
#include "gui.h"

static constexpr uint32_t HMI_TICK_PERIOD_MS = 5U;
static constexpr size_t SERIAL_LINE_CAP = 32U;

extern gui_scene_t s_sceneHome;
extern gui_scene_t s_sceneMainMenu;
extern gui_scene_t s_sceneTest1;
extern gui_scene_t s_sceneTest2;

static GUIClsComponent s_sceneHomeCls(0x0000U);
static GUIJViewComponent s_sceneHomeJView;
static GUIHotKeyComponent s_sceneHomeHotKeyOk(HMI_DATA_BTN_OK, &s_sceneMainMenu);
static GUIComponent* s_sceneHomeItems[] = {
  &s_sceneHomeCls,
  &s_sceneHomeJView,
  &s_sceneHomeHotKeyOk
};
gui_scene_t s_sceneHome = {
  s_sceneHomeItems,
  sizeof(s_sceneHomeItems) / sizeof(s_sceneHomeItems[0])
};

static GUIClsComponent s_sceneMainMenuCls(0x0000U);
static GUILabelComponent s_sceneMainMenuTitle(30U, 10U, 0x07FFU, "Main menu");
static GUIMenuItemComponent s_sceneMainMenuItemCalCenter(10U, 20U, "Cal. center", &s_sceneTest1);
static GUIMenuItemComponent s_sceneMainMenuItemCalEdge(10U, 30U, "Cal. edge", &s_sceneTest2);
static GUIHotKeyComponent s_sceneMainMenuHotKeyBack(HMI_DATA_BTN_BACK, &s_sceneHome);
static GUIComponent* s_sceneMainMenuItems[] = {
  &s_sceneMainMenuCls,
  &s_sceneMainMenuTitle,
  &s_sceneMainMenuItemCalCenter,
  &s_sceneMainMenuItemCalEdge,
  &s_sceneMainMenuHotKeyBack
};
gui_scene_t s_sceneMainMenu = {
  s_sceneMainMenuItems,
  sizeof(s_sceneMainMenuItems) / sizeof(s_sceneMainMenuItems[0])
};

static GUIClsComponent s_sceneTest1Cls(0x0000U);
static GUILabelComponent s_sceneTest1Label(40U, 40U, 0xFFFFU, "Test scene 1");
static GUIHotKeyComponent s_sceneTest1HotKeyBack(HMI_DATA_BTN_BACK, &s_sceneMainMenu);
static GUIComponent* s_sceneTest1Items[] = {
  &s_sceneTest1Cls,
  &s_sceneTest1Label,
  &s_sceneTest1HotKeyBack
};
gui_scene_t s_sceneTest1 = {
  s_sceneTest1Items,
  sizeof(s_sceneTest1Items) / sizeof(s_sceneTest1Items[0])
};

static GUIClsComponent s_sceneTest2Cls(0x0000U);
static GUILabelComponent s_sceneTest2Label(40U, 40U, 0xFFFFU, "Test scene 2");
static GUIHotKeyComponent s_sceneTest2HotKeyBack(HMI_DATA_BTN_BACK, &s_sceneMainMenu);
static GUIComponent* s_sceneTest2Items[] = {
  &s_sceneTest2Cls,
  &s_sceneTest2Label,
  &s_sceneTest2HotKeyBack
};
gui_scene_t s_sceneTest2 = {
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

typedef void (*cmd_func_t)(int32_t args[], uint8_t argsCount);

typedef struct {
  const char* name;
  uint8_t argsCount;
  cmd_func_t func;
} CommandEntry;

static const CommandEntry s_commands[] = {
  { "A", 1, [](int32_t args[], uint8_t argsCount) {
     hmi_cmd_lcd_set_progress(0U, (uint8_t)args[0]);
   } },
  { "B", 1, [](int32_t args[], uint8_t argsCount) {
     hmi_cmd_lcd_set_progress(1U, (uint8_t)args[0]);
   } },
  { "C", 1, [](int32_t args[], uint8_t argsCount) {
     hmi_cmd_lcd_set_progress(2U, (uint8_t)args[0]);
   } },
  { "D", 1, [](int32_t args[], uint8_t argsCount) {
     hmi_cmd_lcd_set_indicator(0U, args[0] != 0);
   } },
  { "T", 2, [](int32_t args[], uint8_t argsCount) {
     const uint32_t hz = (uint32_t)args[0];
     if ((args[1] < 0) || ((uint32_t)args[1] > 65535U)) return;
     const uint32_t divider32 = (hz > 20U && hz < 20000U) ? (1000000 / hz) : 0U;  // 0 for pause
     hmi_cmd_play_tone((uint16_t)divider32, (uint16_t)args[1]);
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
    hmi_cmd_lcd_set_indicator(1U, true);
  } else {
    if (s_serialStarted) {
      Serial.end();
      s_serialStarted = false;
    }
    hmi_cmd_lcd_set_indicator(1U, false);
  }
}


static void TickHmiOnce(void) {
  const hmi_tick_result_t rc = hmi_tick();
  if (rc != HMI_TICK_OK) return;

  if (hmi_changed(HMI_DATA_STAT_USB_CONN)) {
    HandleUsbConnChanged();
  }

  const bool sceneSent = GUIServiceActiveScene();
  if (!sceneSent) {
    hmi_sysSend();
  }
}

void setup() {
  hmi_init(HmiLogToSerial);
  (void)hmi_cmd_set_brightness(20);
  GUISwitchScene(&s_sceneHome);
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
