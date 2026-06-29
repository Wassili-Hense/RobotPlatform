
#include <Arduino.h>
#include <ctype.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stddef.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hmi.h"
#include "gui.h"
#include "serial_bg.h"

static gui_axis_cal_t s_axisCalX = { 226U, 1951U, 1959U, 4028U };
static gui_axis_cal_t s_axisCalY = { 0U, 1953U, 1962U, 4027U };

// [gui]
extern gui_scene_t s_sceneHome;
extern gui_scene_t s_sceneMainMenu;
extern gui_scene_t s_sceneCCentr;
extern gui_scene_t s_sceneCEdge;

static GUIClsComponent s_sceneHomeCls(GUI_COLOR_BLACK, false);
static GUIJViewComponent s_sceneHomeJView(GUI_J_VIEW_MODE_TRACK, &s_axisCalX, &s_axisCalY);
static GUIBrightnessComponent s_sceneHomeBrightness(0U, 0U, 0U);
static GUIHotKeyComponent s_sceneHomeHotKeyOk(HMI_DATA_BTN_OK, &s_sceneMainMenu);
static GUIComponent* s_sceneHomeItems[] = { &s_sceneHomeCls, &s_sceneHomeJView, &s_sceneHomeBrightness, &s_sceneHomeHotKeyOk };
gui_scene_t s_sceneHome = GUI_SCENE(s_sceneHomeItems);

static GUIClsComponent s_sceneMainMenuCls(GUI_COLOR_BLACK, true);
static GUILabelComponent s_sceneMainMenuTitle(30U, 10U, GUI_COLOR_GRAY, "Main menu");
static GUIBrightnessComponent s_sceneMainMenuBrightness(1U, 118U, 10U);
static GUIMenuItemComponent s_sceneMainMenuItemCalCenter(10U, 25U, "Cal. center", &s_sceneCCentr);
static GUIMenuItemComponent s_sceneMainMenuItemCalEdge(10U, 40U, "Cal. edge", &s_sceneCEdge);
static GUIHotKeyComponent s_sceneMainMenuHotKeyBack(HMI_DATA_BTN_BACK, &s_sceneHome);
static GUIComponent* s_sceneMainMenuItems[] = { &s_sceneMainMenuCls, &s_sceneMainMenuTitle, &s_sceneMainMenuBrightness, &s_sceneMainMenuItemCalCenter, &s_sceneMainMenuItemCalEdge, &s_sceneMainMenuHotKeyBack };
gui_scene_t s_sceneMainMenu = GUI_SCENE(s_sceneMainMenuItems);

static GUIClsComponent s_sceneCCentrCls(GUI_COLOR_BLACK, true);
static GUIJViewComponent s_sceneCCentrJView(GUI_J_VIEW_MODE_CAL_CENTER, &s_axisCalX, &s_axisCalY);
static GUIHotKeyComponent s_sceneCCentrHotKeyBack(HMI_DATA_BTN_BACK, &s_sceneMainMenu);
static GUIHotKeyComponent s_sceneCCentrHotKeyOk(HMI_DATA_BTN_OK, &s_sceneMainMenu);
static GUILabelComponent s_sceneCalibrateBack(16U, 10U, GUI_COLOR_ORANGE, "D\n\nR\n\nO\n\nP");
static GUILabelComponent s_sceneCalibrateOk(126U, 10U, GUI_COLOR_GREEN, "S\n\nA\n\nV\n\nE");
static GUIComponent* s_sceneCCentrItems[] = { &s_sceneCCentrCls, &s_sceneCCentrJView, &s_sceneCCentrHotKeyBack, &s_sceneCCentrHotKeyOk, &s_sceneCalibrateBack, &s_sceneCalibrateOk };
gui_scene_t s_sceneCCentr = GUI_SCENE(s_sceneCCentrItems);

static GUIClsComponent s_sceneCEdgeCls(GUI_COLOR_BLACK, true);
static GUIJViewComponent s_sceneCEdgeJView(GUI_J_VIEW_MODE_CAL_EDGE, &s_axisCalX, &s_axisCalY);
static GUIHotKeyComponent s_sceneCEdgeHotKeyBack(HMI_DATA_BTN_BACK, &s_sceneMainMenu);
static GUIHotKeyComponent s_sceneCEdgeHotKeyOk(HMI_DATA_BTN_OK, &s_sceneMainMenu);
static GUIComponent* s_sceneCEdgeItems[] = { &s_sceneCEdgeCls, &s_sceneCEdgeJView, &s_sceneCEdgeHotKeyBack, &s_sceneCEdgeHotKeyOk, &s_sceneCalibrateBack, &s_sceneCalibrateOk };
gui_scene_t s_sceneCEdge = GUI_SCENE(s_sceneCEdgeItems);

// [Command]
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
  { "M", 1, [](int32_t args[], uint8_t argsCount) {
     (void)argsCount;
     hmi_cmd_play_melody((hmi_melody_t)args[0]);
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
static constexpr uint8_t MAX_ARGS = 2U;
static constexpr uint32_t APP_TASK_PERIOD_MS = 5U;
static constexpr uint32_t APP_HOME_POWER_OFF_TIMEOUT_MS = 5UL * 60UL * 1000UL;
static constexpr BaseType_t APP_TASK_CORE_ID = 1;
static constexpr UBaseType_t APP_TASK_PRIORITY = 2;
static constexpr uint32_t APP_TASK_STACK_SIZE = 4096U;

static TaskHandle_t s_appTaskHandle = nullptr;
static uint32_t s_homeLastActivityMs = 0U;
static bool s_homePowerOffWarn15Done = false;
static bool s_homePowerOffWarn5Done = false;
static bool s_homePowerOffQueued = false;

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
  char* savePtr = nullptr;
  char* token;
  uint8_t cmdIdx = 0U;
  const CommandEntry* cmd;
  int32_t args[MAX_ARGS];
  uint8_t argsCount = 0U;

  if ((line == nullptr) || (*line == '\0')) return;

  token = strtok_r(line, " \t", &savePtr);
  if (token == nullptr) return;

  for (; cmdIdx < COMMAND_COUNT; ++cmdIdx) {
    if (strcmp(token, s_commands[cmdIdx].name) == 0) break;
  }
  if (cmdIdx >= COMMAND_COUNT) return;

  cmd = &s_commands[cmdIdx];
  if (cmd->argsCount > MAX_ARGS) return;

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
// [Log]
static void HmiLogToSerial(const char* text, bool emergency) {
  if (text == nullptr) {
    return;
  }

  if (emergency && !serial_bg_is_connected()) {
    serial_bg_set_connected(true);
  }

  if (!serial_bg_is_connected()) {
    return;
  }

  (void)serial_bg_send_line(text);
}

// [Auto power off]
static void AppProcessHomePowerOff(void) {
  const uint32_t now = millis();
  static uint32_t lastActivityMs = now;
  static int32_t prevRemSec = 301;

  if (GUIGetActiveScene() != &s_sceneHome || hmi_get(HMI_DATA_BTN_ANYKEY) 
    || (hmi_changed(HMI_DATA_JOY_X) && (hmi_get(HMI_DATA_JOY_X) < s_axisCalX.cMin || hmi_get(HMI_DATA_JOY_X) > s_axisCalX.cMax))
    || (hmi_changed(HMI_DATA_JOY_Y) && (hmi_get(HMI_DATA_JOY_Y) < s_axisCalY.cMin || hmi_get(HMI_DATA_JOY_Y) > s_axisCalY.cMax))) {
    lastActivityMs = now;
    prevRemSec = 301;
    return;
  }
  const uint32_t idleS = (uint32_t)(now - lastActivityMs) / 1000U;
  const int32_t remainingS = 300L - (int32_t)idleS;
  if((remainingS <= 6 && prevRemSec > 6) || (remainingS <= 4 && prevRemSec > 4) || (remainingS <= 2 && prevRemSec > 2)){
    hmi_cmd_play_tone(500U, 50U);
  } else if(remainingS <= 1 && prevRemSec > 1){
    hmi_cmd_play_melody(HMI_MELODY_DISCONNECTED);
  } else if(remainingS <= 0 && prevRemSec > 0){
    hmi_cmd_power_off();
  }
  prevRemSec = remainingS;
}

// [AppTask]
static void AppTask(void* arg) {
  (void)arg;

  TickType_t lastWakeTime = xTaskGetTickCount();
  const TickType_t periodTicks = pdMS_TO_TICKS(APP_TASK_PERIOD_MS);

  for (;;) {
    // HMI
    if (hmi_tick() == HMI_TICK_OK) {
      if (hmi_changed(HMI_DATA_STAT_USB_CONN)) { // Usb Connection Changed
        const bool connected = (hmi_get(HMI_DATA_STAT_USB_CONN) != 0U);
        serial_bg_set_connected(connected);
        hmi_cmd_lcd_set_indicator(1U, connected);
      }
      AppProcessHomePowerOff();
      //GUI
      if (!GUIServiceActiveScene()) {
        hmi_sysSend();
      }
      //Serial
      {
        char line[SERIAL_BG_LINE_CAP];

        if (serial_bg_receive_line(line, sizeof(line))) {
          ParseAndDispatch(line);
        }
      }
    }
    (void)xTaskDelayUntil(&lastWakeTime, periodTicks);
  }
}

void setup() {
  (void)serial_bg_begin(115200U, false, 1, 2, 4096U);
  hmi_init(HmiLogToSerial);
  //hmi_cmd_play_melody(HMI_MELODY_POWER_ON);
  hmi_cmd_play_tone(200, 50);
  GUISetHomeScene(&s_sceneHome);
  GUISwitchScene(&s_sceneHome);
  (void)xTaskCreatePinnedToCore(AppTask, "AppTask", 4096U, nullptr, 2, &s_appTaskHandle, 1);
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000U));
}
