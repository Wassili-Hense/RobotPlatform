
#include <Arduino.h>
#include <ctype.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <stdint.h>
#include <stddef.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hmi.h"
#include "gui.h"
#include "serial_bg.h"
#include "pec_link.h"
#include "pen_common.h"

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

// [App constants]
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


static bool PecLineToSerial(const char* text) {
  if ((text == nullptr) || !serial_bg_is_connected()) return false;
  return serial_bg_send_line(text);
}

static float AppNormalizeAxis(uint16_t raw, const gui_axis_cal_t& cal) {
  if (raw <= cal.eMin) return -1.0f;
  if (raw >= cal.eMax) return 1.0f;
  if ((raw >= cal.cMin) && (raw <= cal.cMax)) return 0.0f;
  if (raw < cal.cMin) {
    const uint16_t spanRaw = (cal.cMin > cal.eMin) ? (uint16_t)(cal.cMin - cal.eMin) : 1U;
    float value = -1.0f + ((float)(raw - cal.eMin) / (float)spanRaw);
    if (value > 0.0f) value = 0.0f;
    if (value < -1.0f) value = -1.0f;
    return value;
  }
  const uint16_t spanRaw = (cal.eMax > cal.cMax) ? (uint16_t)(cal.eMax - cal.cMax) : 1U;
  float value = (float)(raw - cal.cMax) / (float)spanRaw;
  if (value < 0.0f) value = 0.0f;
  if (value > 1.0f) value = 1.0f;
  return value;
}

static void AppProcessPecUi(void) {
  static bool prevConnected = false;
  static uint8_t prevP0 = 0xFFU;
  static uint8_t prevP1 = 0xFFU;
  static uint8_t prevP2 = 0xFFU;
  static uint8_t prevErr = 0U;

  pec_status_t st;
  if (!pec_take_status(&st)) return;

  const uint8_t p1 = st.connected ? pen_rssi_to_progress(st.peerRssi) : 0U;
  const uint8_t p2 = st.connected ? pen_rssi_to_progress(st.localRssi) : 0U;

  if (prevConnected != st.connected) {
    hmi_cmd_lcd_set_indicator(0U, st.connected);
    //hmi_cmd_play_melody(st.connected ? HMI_MELODY_CONNECTED : HMI_MELODY_DISCONNECTED);
    prevConnected = st.connected;
  }
  if (prevP0 != st.battery) { hmi_cmd_lcd_set_progress(2U, st.battery); prevP0 = st.battery; }
  if (prevP1 != p1) { hmi_cmd_lcd_set_progress(0U, p1); prevP1 = p1; }
  if (prevP2 != p2) { hmi_cmd_lcd_set_progress(1U, p2); prevP2 = p2; }

  if ((st.errorCode != 0U) && (prevErr != st.errorCode)) {
    char errText[8];
    snprintf(errText, sizeof(errText), "E%u", (unsigned)st.errorCode);
    hmi_cmd_lcd_draw_text(0U, 0U, GUI_COLOR_ORANGE, errText);
    prevErr = st.errorCode;
  }
}

static void AppProcessPecTx(void) {
  static bool lastUsbConnected = false;
  static int32_t lset = 0;
  static int32_t rset = 0;

  const uint32_t now = millis();
  if (!pec_is_connected()) {
    return;
  }

  if(hmi_changed(HMI_DATA_JOY_X)){
    const float joyX = AppNormalizeAxis(hmi_get(HMI_DATA_JOY_X), s_axisCalX);
    (void)pec_send_stream_f(PEC_VAR_JX, joyX, 500U);
  }
  if(hmi_changed(HMI_DATA_JOY_Y)){
    const float joyY = AppNormalizeAxis(hmi_get(HMI_DATA_JOY_Y), s_axisCalY);
    (void)pec_send_stream_f(PEC_VAR_JY, joyY, 500U);
  }

  // LUP/LDN/RUP/RDN используются как параметры только на Home.
  // В меню (GUIGetActiveScene() != &s_sceneHome) эти кнопки остаются за GUI.
  if (GUIGetActiveScene() == &s_sceneHome) {
    if (hmi_changed(HMI_DATA_BTN_LUP) && (hmi_get(HMI_DATA_BTN_LUP) != 0U)) {
      ++lset;
      (void)pec_send_state_i(PEC_VAR_LSET, lset, 0U);
    }
    if (hmi_changed(HMI_DATA_BTN_LDN) && (hmi_get(HMI_DATA_BTN_LDN) != 0U)) {
      --lset;
      (void)pec_send_state_i(PEC_VAR_LSET, lset, 0U);
    }
    if (hmi_changed(HMI_DATA_BTN_RUP) && (hmi_get(HMI_DATA_BTN_RUP) != 0U)) {
      ++rset;
      (void)pec_send_state_i(PEC_VAR_RSET, rset, 0U);
    }
    if (hmi_changed(HMI_DATA_BTN_RDN) && (hmi_get(HMI_DATA_BTN_RDN) != 0U)) {
      --rset;
      (void)pec_send_state_i(PEC_VAR_RSET, rset, 0U);
    }
  }

  const bool usbConnected = serial_bg_is_connected();
  if (usbConnected != lastUsbConnected) {  // TODO: || ec_just_connected
    (void)pec_send_state_i(PEC_VAR_USBC, usbConnected ? 1 : 0, 2000U);
    lastUsbConnected = usbConnected;
  }
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
        (void)pec_send_state_i(PEC_VAR_USBC, connected ? 1 : 0, 2000U);
      }
      AppProcessHomePowerOff();
      AppProcessPecUi();
      AppProcessPecTx();
      //GUI
      if (!GUIServiceActiveScene()) {
        hmi_sysSend();
      }
      //Serial
      {
        char line[SERIAL_BG_LINE_CAP];

        if (serial_bg_receive_line(line, sizeof(line))) {
          (void)pec_pc_rx_line(line);
        }
      }
    }
    (void)xTaskDelayUntil(&lastWakeTime, periodTicks);
  }
}

void setup() {
  (void)serial_bg_begin(115200U, false, 1, 2, 4096U);
  hmi_init(HmiLogToSerial);
  (void)pec_begin(PecLineToSerial, 1, 2, 4096U);
  //hmi_cmd_play_melody(HMI_MELODY_POWER_ON);
  hmi_cmd_play_tone(200, 50);
  GUISetHomeScene(&s_sceneHome);
  GUISwitchScene(&s_sceneHome);
  (void)xTaskCreatePinnedToCore(AppTask, "AppTask", 4096U, nullptr, 2, &s_appTaskHandle, 1);
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000U));
}
