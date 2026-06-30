
#include <Arduino.h>
#include <ctype.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <stdint.h>
#include <stddef.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
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
static constexpr BaseType_t APP_PEC_RX_TASK_CORE_ID = 1;
static constexpr UBaseType_t APP_PEC_RX_TASK_PRIORITY = 1;
static constexpr uint32_t APP_PEC_RX_TASK_STACK_SIZE = 4096U;
static constexpr size_t APP_PEC_RX_QUEUE_CAP = 16U;

static TaskHandle_t s_appTaskHandle = nullptr;
static TaskHandle_t s_pecRxTaskHandle = nullptr;
static QueueHandle_t s_pecRxQueue = nullptr;
static uint32_t s_homeLastActivityMs = 0U;
static bool s_homePowerOffWarn15Done = false;
static bool s_homePowerOffWarn5Done = false;
static bool s_homePowerOffQueued = false;
static int32_t s_usbConnPec;

// [Log]
static void HmiLogToSerial(const char* text, bool emergency) {
  if (text == nullptr) return;
  if (emergency && !serial_bg_is_connected()) {
    serial_bg_set_connected(true);
  }
  if (!serial_bg_is_connected()) return;
  (void)serial_bg_send_line(text);
}


static bool AppPcTxLine(const char* text) {
  if ((text != nullptr) && serial_bg_is_connected()) {
    (void)serial_bg_send_line(text);
  }
  return true;
}

static bool AppPecRxEnqueue(const pec_rx_event_t* ev) {
  if ((ev == nullptr) || (s_pecRxQueue == nullptr)) return false;
  return (xQueueSend(s_pecRxQueue, ev, 0U) == pdTRUE);
}

static bool VarIdFromText(const char* text, uint32_t* outVarId) {
  if ((text == nullptr) || (outVarId == nullptr)) return false;
  const size_t len = strlen(text);
  if ((len == 0U) || (len > 4U)) return false;
  uint8_t b[4] = { 0U, 0U, 0U, 0U };
  for (size_t i = 0; i < len; ++i) {
    if (!isGraph((unsigned char)text[i])) return false;
    b[i] = (uint8_t)text[i];
  }
  *outVarId = PEN_VAR_ID4(b[0], b[1], b[2], b[3]);
  return true;
}

static bool LooksFloat(const char* text) {
  return (text != nullptr) && ((strchr(text, '.') != nullptr) || (strchr(text, 'e') != nullptr) || (strchr(text, 'E') != nullptr));
}

static bool pec_pc_rx_line(const char* line) {
  if (line == nullptr) return false;
  char buf[32];
  strncpy(buf, line, sizeof(buf) - 1U);
  buf[sizeof(buf) - 1U] = '\0';

  char* savePtr = nullptr;
  char* varText = strtok_r(buf, " \t\r\n", &savePtr);
  char* valueText = strtok_r(nullptr, " \t\r\n", &savePtr);
  if ((varText == nullptr) || (valueText == nullptr)) return false;

  uint32_t varId = 0U;
  if (!VarIdFromText(varText, &varId)) return false;
  if (strcmp(valueText, "?") == 0) return pec_send_get_var(varId);

  if (LooksFloat(valueText)) {
    char* endPtr = nullptr;
    const float value = strtof(valueText, &endPtr);
    if ((endPtr == valueText) || (*endPtr != '\0')) return false;
    return pec_send_state_f(varId, value, 0U);
  }

  char* endPtr = nullptr;
  const long value = strtol(valueText, &endPtr, 10);
  if ((endPtr == valueText) || (*endPtr != '\0') || (value < INT32_MIN) || (value > INT32_MAX)) return false;
  return pec_send_state_i(varId, (int32_t)value, 0U);
}

static void AppFormatVarName(uint32_t varId, char out[5]) {
  pen_var_id_to_text(varId, out);
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

static void AppEmitVarI(uint32_t varId, int32_t value, uint8_t msgType, uint16_t seq) {
  char name[5];
  char line[32];
  AppFormatVarName(varId, name);
  snprintf(line, sizeof(line), "%s %ld I %u %u", name, (long)value, (unsigned)msgType, (unsigned)seq);
  (void)AppPcTxLine(line);
}

static void AppEmitVarF(uint32_t varId, float value, uint8_t msgType, uint16_t seq) {
  char name[5];
  char line[32];
  AppFormatVarName(varId, name);
  snprintf(line, sizeof(line), "%s %.4f F %u %u", name, (double)value, (unsigned)msgType, (unsigned)seq);
  (void)AppPcTxLine(line);
}

static void HandleVarI(const pec_rx_event_t& ev) {
  const uint32_t varId = ev.data.varI.varId;
  const int32_t value = ev.data.varI.value;
  if (varId == PEN_VAR_HB) return;

  if (varId == PEC_VAR_BATP) {
    int32_t v = value;
    if (v < 0) v = 0;
    if (v > 64) v = 64;
    hmi_cmd_lcd_set_progress(2U, (uint8_t)v);
  } else if (varId == PEC_VAR_RSSI || varId == PEC_VAR_RSSL) {
    int32_t v = 100 - value;
    if (v < 0) v = 0;
    if (v > 64) v = 0;
    hmi_cmd_lcd_set_progress(varId == PEC_VAR_RSSI ? 0U : 1U, pen_rssi_to_progress((int8_t)v));
    return;
  }

  AppEmitVarI(varId, value, ev.msgType, ev.data.varI.seq);
}

static void HandleVarF(const pec_rx_event_t& ev) {
  AppEmitVarF(ev.data.varF.varId, ev.data.varF.value, ev.msgType, ev.data.varF.seq);
}

static void HandleAck(const pec_rx_event_t& ev) {
  char name[5];
  char line[32];
  AppFormatVarName(ev.data.ack.varId, name);
  snprintf(line, sizeof(line), "@ACK %u %s", (unsigned)ev.data.ack.ackSeq, name);
  (void)AppPcTxLine(line);
}

static void HandleNack(const pec_rx_event_t& ev) {
  char name[5];
  char line[32];
  AppFormatVarName(ev.data.nack.varId, name);
  snprintf(line, sizeof(line), "@NACK %u %s %u", (unsigned)ev.data.nack.ackSeq, name, (unsigned)ev.data.nack.reason);
  (void)AppPcTxLine(line);
}

static void HandleLinkEvent(const pec_rx_event_t& ev) {
  switch (ev.data.link.code) {
    case PEC_LINK_READY: 
      (void)AppPcTxLine("@LINK READY"); 
      break;
    case PEC_LINK_DISC:
      {
        char line[32];
        snprintf(line, sizeof(line), "@DISC %d", (int)ev.data.link.rssi);
        (void)AppPcTxLine(line);
        break;
      }
    case PEC_LINK_CONNECTED: 
      (void)AppPcTxLine("@LINK CONNECTED"); 
      break;
    case PEC_LINK_AUTH_OK: 
      (void)AppPcTxLine("@LINK AUTH_OK"); 
      break;
    case PEC_LINK_SECURE:
      //hmi_cmd_play_melody(HMI_MELODY_CONNECTED);
      hmi_cmd_lcd_set_indicator(0U, true);
      (void)AppPcTxLine("@LINK SECURE");
      s_usbConnPec = -1;  // resend
      break;
    case PEC_LINK_LOST:
      //hmi_cmd_play_melody(HMI_MELODY_DISCONNECTED);
      hmi_cmd_lcd_set_indicator(0U, false);
      hmi_cmd_lcd_set_progress(0U, 0U);
      hmi_cmd_lcd_set_progress(1U, 0U);
      (void)AppPcTxLine("@LINK LOST");
      break;
    case PEC_LINK_CONN_TO: 
      (void)AppPcTxLine("@LINK CONN_TO"); 
      break;
    case PEC_LINK_AUTH_TO: 
      (void)AppPcTxLine("@LINK AUTH_TO"); 
      break;
    case PEC_LINK_MAC_BAD: 
      (void)AppPcTxLine("@LINK MAC_BAD"); 
      break;
    case PEC_LINK_AUTH_BAD: 
      (void)AppPcTxLine("@LINK AUTH_BAD"); 
      break;
    case PEC_LINK_SEC_BAD: 
      (void)AppPcTxLine("@LINK SEC_BAD"); 
      break;
    default: 
      break;
  }
}

static void HandleErrorEvent(const pec_rx_event_t& ev) {
  char line[32];
  snprintf(line, sizeof(line), "@ERR %u %ld", (unsigned)ev.data.error.code, (long)ev.data.error.detail);
  (void)AppPcTxLine(line);
  if (ev.data.error.code != PEC_HW_ERR_NONE) {
    char errText[8];
    snprintf(errText, sizeof(errText), "E%u", (unsigned)ev.data.error.code);
    hmi_cmd_lcd_draw_text(0U, 0U, GUI_COLOR_ORANGE, errText);
  }
}

static void AppProcessPecRxEvent(const pec_rx_event_t& ev) {
  switch (ev.type) {
    case PEC_RX_LINK: HandleLinkEvent(ev); break;
    case PEC_RX_ERROR: HandleErrorEvent(ev); break;
    case PEC_RX_VAR_I: HandleVarI(ev); break;
    case PEC_RX_VAR_F: HandleVarF(ev); break;
    case PEC_RX_ACK: HandleAck(ev); break;
    case PEC_RX_NACK: HandleNack(ev); break;
    default: break;
  }
}

static void AppPecRxTask(void* arg) {
  (void)arg;
  pec_rx_event_t ev = {};
  for (;;) {
    if ((s_pecRxQueue != nullptr) && (xQueueReceive(s_pecRxQueue, &ev, portMAX_DELAY) == pdTRUE)) {
      AppProcessPecRxEvent(ev);
    }
  }
}

static void AppProcessPecTx(void) {
  static int32_t lset = 0;
  static int32_t rset = 0;

  const uint32_t now = millis();
  if (!pec_is_connected()) {
    return;
  }

  if (hmi_changed(HMI_DATA_JOY_X)) {
    const float joyX = AppNormalizeAxis(hmi_get(HMI_DATA_JOY_X), s_axisCalX);
    (void)pec_send_stream_f(PEC_VAR_JX, joyX, 500U);
  }
  if (hmi_changed(HMI_DATA_JOY_Y)) {
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

  const int32_t usbConnected = serial_bg_is_connected()?1:0;
  if (usbConnected != s_usbConnPec) {
    (void)pec_send_state_i(PEC_VAR_USBC, usbConnected, 0U);
    s_usbConnPec = usbConnected;
  }
}

// [Auto power off]
static void AppProcessHomePowerOff(void) {
  const uint32_t now = millis();
  static uint32_t lastActivityMs = now;
  static int32_t prevRemSec = 301;

  if (GUIGetActiveScene() != &s_sceneHome || hmi_get(HMI_DATA_BTN_ANYKEY) || pec_is_connected() || serial_bg_is_connected()) {
    lastActivityMs = now;
    prevRemSec = 301;
    return;
  }
  const uint32_t idleS = (uint32_t)(now - lastActivityMs) / 1000U;
  const int32_t remainingS = 300L - (int32_t)idleS;
  if ((remainingS <= 6 && prevRemSec > 6) || (remainingS <= 4 && prevRemSec > 4) || (remainingS <= 2 && prevRemSec > 2)) {
    hmi_cmd_play_tone(500U, 50U);
  } else if (remainingS <= 1 && prevRemSec > 1) {
    hmi_cmd_play_melody(HMI_MELODY_DISCONNECTED);
  } else if (remainingS <= 0 && prevRemSec > 0) {
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
      if (hmi_changed(HMI_DATA_STAT_USB_CONN)) {  // Usb Connection Changed
        const bool connected = (hmi_get(HMI_DATA_STAT_USB_CONN) != 0U);
        serial_bg_set_connected(connected);
        hmi_cmd_lcd_set_indicator(1U, connected);
        (void)pec_send_state_i(PEC_VAR_USBC, connected ? 1 : 0, 2000U);
      }
      AppProcessHomePowerOff();
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
  s_pecRxQueue = xQueueCreate(APP_PEC_RX_QUEUE_CAP, sizeof(pec_rx_event_t));
  (void)pec_begin(AppPecRxEnqueue, 1, 2, 4096U);
  (void)xTaskCreatePinnedToCore(AppPecRxTask, "PecRx", APP_PEC_RX_TASK_STACK_SIZE, nullptr, APP_PEC_RX_TASK_PRIORITY, &s_pecRxTaskHandle, APP_PEC_RX_TASK_CORE_ID);
  //hmi_cmd_play_melody(HMI_MELODY_POWER_ON);
  hmi_cmd_play_tone(200, 50);
  GUISetHomeScene(&s_sceneHome);
  GUISwitchScene(&s_sceneHome);
  (void)xTaskCreatePinnedToCore(AppTask, "AppTask", 4096U, nullptr, 2, &s_appTaskHandle, 1);
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000U));
}
