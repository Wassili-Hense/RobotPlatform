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
extern gui_scene_t s_sceneCCentr;
extern gui_scene_t s_sceneCEdge;

static gui_axis_cal_t s_axisCalX = { 226U, 1955U, 1955U, 4028U };
static gui_axis_cal_t s_axisCalY = { 0U, 1957U, 1958U, 4027U };

// [gui]
static GUIClsComponent s_sceneHomeCls(GUI_COLOR_BLACK, false);
static GUIJViewComponent s_sceneHomeJView(GUI_J_VIEW_MODE_TRACK, &s_axisCalX, &s_axisCalY, &s_sceneMainMenu);
static GUIBrightnessComponent s_sceneHomeBrightness(0U, 0U, 0U);
static GUIComponent* s_sceneHomeItems[] = {
  &s_sceneHomeCls,
  &s_sceneHomeJView,
  &s_sceneHomeBrightness
};
gui_scene_t s_sceneHome = GUI_SCENE(s_sceneHomeItems);

static GUIClsComponent s_sceneMainMenuCls(GUI_COLOR_BLACK, true);
static GUILabelComponent s_sceneMainMenuTitle(30U, 10U, GUI_COLOR_CYAN, "Main menu");
static GUIBrightnessComponent s_sceneMainMenuBrightness(1U, 118U, 10U);
static GUIMenuItemComponent s_sceneMainMenuItemCalCenter(10U, 25U, "Cal. center", &s_sceneCCentr);
static GUIMenuItemComponent s_sceneMainMenuItemCalEdge(10U, 40U, "Cal. edge", &s_sceneCEdge);
static GUIHotKeyComponent s_sceneMainMenuHotKeyBack(HMI_DATA_BTN_BACK, &s_sceneHome);
static GUIComponent* s_sceneMainMenuItems[] = {
  &s_sceneMainMenuCls,
  &s_sceneMainMenuTitle,
  &s_sceneMainMenuBrightness,
  &s_sceneMainMenuItemCalCenter,
  &s_sceneMainMenuItemCalEdge,
  &s_sceneMainMenuHotKeyBack
};
gui_scene_t s_sceneMainMenu = GUI_SCENE(s_sceneMainMenuItems);

static GUIClsComponent s_sceneCCentrCls(GUI_COLOR_BLACK, true);
static GUIJViewComponent s_sceneCCentrJView(GUI_J_VIEW_MODE_CAL_CENTER, &s_axisCalX, &s_axisCalY, &s_sceneMainMenu);
static GUILabelComponent s_sceneCalibrateBack(16U, 10U, GUI_COLOR_ORANGE, "D\n\nR\n\nO\n\nP");
static GUILabelComponent s_sceneCalibrateOk(126U, 10U, GUI_COLOR_GREEN, "S\n\nA\n\nV\n\nE");
static GUIComponent* s_sceneCCentrItems[] = {
  &s_sceneCCentrCls,
  &s_sceneCCentrJView,
  &s_sceneCalibrateBack,
  &s_sceneCalibrateOk
};
gui_scene_t s_sceneCCentr = GUI_SCENE(s_sceneCCentrItems);

static GUIClsComponent s_sceneCEdgeCls(GUI_COLOR_BLACK, true);
static GUIJViewComponent s_sceneCEdgeJView(GUI_J_VIEW_MODE_CAL_EDGE, &s_axisCalX, &s_axisCalY, &s_sceneMainMenu);
static GUIComponent* s_sceneCEdgeItems[] = {
  &s_sceneCEdgeCls,
  &s_sceneCEdgeJView,
  &s_sceneCalibrateBack,
  &s_sceneCalibrateOk
};
gui_scene_t s_sceneCEdge = GUI_SCENE(s_sceneCEdgeItems);

// [common]
static uint32_t s_nextHmiTickMs = 0U;
static bool s_serialStarted = false;
static char s_serialLine[SERIAL_LINE_CAP];
static size_t s_serialLineLen = 0U;

// [Serial]
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
     (void)argsCount;
     hmi_cmd_lcd_set_progress(0U, (uint8_t)args[0]);
   } },
  { "B", 1, [](int32_t args[], uint8_t argsCount) {
     (void)argsCount;
     hmi_cmd_lcd_set_progress(1U, (uint8_t)args[0]);
   } },
  { "C", 1, [](int32_t args[], uint8_t argsCount) {
     (void)argsCount;
     hmi_cmd_lcd_set_progress(2U, (uint8_t)args[0]);
   } },
  { "D", 1, [](int32_t args[], uint8_t argsCount) {
     (void)argsCount;
     hmi_cmd_lcd_set_indicator(0U, args[0] != 0);
   } },
  { "T", 2, [](int32_t args[], uint8_t argsCount) {
     (void)argsCount;
     const uint32_t hz = (uint32_t)args[0];
     if ((args[1] < 0) || ((uint32_t)args[1] > 65535U)) return;
     const uint32_t divider32 = (hz > 20U && hz < 20000U) ? (1000000U / hz) : 0U;
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

// [hmi]
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

void setup() {
  hmi_init(HmiLogToSerial);
  GUISwitchScene(&s_sceneHome);
  s_nextHmiTickMs = millis() + HMI_TICK_PERIOD_MS;
}

void loop() {
  PollSerialRx();
  const uint32_t now = millis();
  if ((int32_t)(now - s_nextHmiTickMs) >= 0) {
    s_nextHmiTickMs += HMI_TICK_PERIOD_MS;
    if (hmi_tick() == HMI_TICK_OK) {
      if (hmi_changed(HMI_DATA_STAT_USB_CONN)) {
        HandleUsbConnChanged();
      }
      if (!GUIServiceActiveScene()) hmi_sysSend();
    }
  }
}
