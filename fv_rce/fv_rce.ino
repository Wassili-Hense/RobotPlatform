
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
#include "pen_link.h"

static gui_axis_cal_t s_axisCalX = { 226U, 1951U, 1959U, 4028U };
static gui_axis_cal_t s_axisCalY = { 0U, 1953U, 1962U, 4027U };

// [gui]
static int32_t s_lset = 0;
static int32_t s_rset = 0;

extern gui_scene_t s_sceneHome;
extern gui_scene_t s_sceneMainMenu;
extern gui_scene_t s_sceneCCentr;
extern gui_scene_t s_sceneCEdge;

static GUIClsComponent s_sceneHomeCls(GUI_COLOR_BLACK, false);
static GUIJViewComponent s_sceneHomeJView(GUI_J_VIEW_MODE_TRACK, &s_axisCalX, &s_axisCalY);
static GUIVarComponent s_sceneHomeLSet(5U, 40U, GUI_COLOR_WHITE, &s_lset);
static GUIVarComponent s_sceneHomeRSet(120U, 40U, GUI_COLOR_WHITE, &s_rset);
static GUIBrightnessComponent s_sceneHomeBrightness(0U, 0U, 0U);
static GUIHotKeyComponent s_sceneHomeHotKeyOk(HMI_DATA_BTN_OK, &s_sceneMainMenu);
static GUIComponent* s_sceneHomeItems[] = { &s_sceneHomeCls, &s_sceneHomeJView, &s_sceneHomeLSet, &s_sceneHomeRSet, &s_sceneHomeBrightness, &s_sceneHomeHotKeyOk };
gui_scene_t s_sceneHome = GUI_SCENE(s_sceneHomeItems);

static GUIClsComponent s_sceneMainMenuCls(GUI_COLOR_BLACK, true);
static GUILabelComponent s_sceneMainMenuTitle(30U, 10U, GUI_COLOR_GRAY, "Main menu");
static GUIBrightnessComponent s_sceneMainMenuBrightness(1U, 118U, 10U);
static GUIMenuItemComponent s_sceneMainMenuItemCalCenter(5U, 25U, "Cal. center", &s_sceneCCentr);
static GUIMenuItemComponent s_sceneMainMenuItemCalEdge(5U, 40U, "Cal. edge", &s_sceneCEdge);
static GUIHotKeyComponent s_sceneMainMenuHotKeyBack(HMI_DATA_BTN_BACK, &s_sceneHome);
static GUIComponent* s_sceneMainMenuItems[] = { &s_sceneMainMenuCls, &s_sceneMainMenuTitle, &s_sceneMainMenuBrightness, &s_sceneMainMenuItemCalCenter, &s_sceneMainMenuItemCalEdge, &s_sceneMainMenuHotKeyBack };
gui_scene_t s_sceneMainMenu = GUI_SCENE(s_sceneMainMenuItems);

static GUIClsComponent s_sceneCCentrCls(GUI_COLOR_BLACK, true);
static GUIJViewComponent s_sceneCCentrJView(GUI_J_VIEW_MODE_CAL_CENTER, &s_axisCalX, &s_axisCalY);
static GUIHotKeyComponent s_sceneCCentrHotKeyBack(HMI_DATA_BTN_BACK, &s_sceneMainMenu);
static GUIHotKeyComponent s_sceneCCentrHotKeyOk(HMI_DATA_BTN_OK, &s_sceneMainMenu);
static GUILabelComponent s_sceneCalibrateBack(30U, 18U, GUI_COLOR_ORANGE, "D\nR\nO\nP");
static GUILabelComponent s_sceneCalibrateOk(120U, 18U, GUI_COLOR_GREEN, "S\nA\nV\nE");
static GUIComponent* s_sceneCCentrItems[] = { &s_sceneCCentrCls, &s_sceneCCentrJView, &s_sceneCCentrHotKeyBack, &s_sceneCCentrHotKeyOk, &s_sceneCalibrateBack, &s_sceneCalibrateOk };
gui_scene_t s_sceneCCentr = GUI_SCENE(s_sceneCCentrItems);

static GUIClsComponent s_sceneCEdgeCls(GUI_COLOR_BLACK, true);
static GUIJViewComponent s_sceneCEdgeJView(GUI_J_VIEW_MODE_CAL_EDGE, &s_axisCalX, &s_axisCalY);
static GUIHotKeyComponent s_sceneCEdgeHotKeyBack(HMI_DATA_BTN_BACK, &s_sceneMainMenu);
static GUIHotKeyComponent s_sceneCEdgeHotKeyOk(HMI_DATA_BTN_OK, &s_sceneMainMenu);
static GUIComponent* s_sceneCEdgeItems[] = { &s_sceneCEdgeCls, &s_sceneCEdgeJView, &s_sceneCEdgeHotKeyBack, &s_sceneCEdgeHotKeyOk, &s_sceneCalibrateBack, &s_sceneCalibrateOk };
gui_scene_t s_sceneCEdge = GUI_SCENE(s_sceneCEdgeItems);

// [PEN app variable IDs]
static constexpr uint32_t PEN_VAR_JX_APP   = PEN_VAR_ID2('J', 'X');
static constexpr uint32_t PEN_VAR_JY_APP   = PEN_VAR_ID2('J', 'Y');
static constexpr uint32_t PEN_VAR_BATP_APP = PEN_VAR_ID4('B', 'A', 'T', 'P');
static constexpr uint32_t PEN_VAR_LSET_APP = PEN_VAR_ID4('L', 'S', 'E', 'T');
static constexpr uint32_t PEN_VAR_RSET_APP = PEN_VAR_ID4('R', 'S', 'E', 'T');
static constexpr uint32_t PEN_VAR_USBC_APP = PEN_VAR_ID4('U', 'S', 'B', 'C');
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
static int32_t s_usbConnPen;

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

static bool pen_pc_rx_line(const char* line) {
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
  if (strcmp(valueText, "?") == 0) return pen_send_get_var(varId);

  if (LooksFloat(valueText)) {
    char* endPtr = nullptr;
    const float value = strtof(valueText, &endPtr);
    if ((endPtr == valueText) || (*endPtr != '\0')) return false;
    return pen_send_state(varId, value);
  }

  char* endPtr = nullptr;
  const long value = strtol(valueText, &endPtr, 10);
  if ((endPtr == valueText) || (*endPtr != '\0') || (value < INT32_MIN) || (value > INT32_MAX)) return false;
  return pen_send_state(varId, (int32_t)value);
}

static void AppFormatVarName(uint32_t varId, char out[5]) {
  out[0] = (char)((varId >> 0) & 0xFFU);
  out[1] = (char)((varId >> 8) & 0xFFU);
  out[2] = (char)((varId >> 16) & 0xFFU);
  out[3] = (char)((varId >> 24) & 0xFFU);
  out[4] = '\0';
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

static void HandleVarI(const pen_rx_event_t& ev) {
  const uint32_t varId = ev.data.varI.varId;
  const int32_t value = ev.data.varI.value;

  if (varId == PEN_VAR_BATP_APP) {
    int32_t v = value;
    if (v < 0) v = 0;
    if (v > 64) v = 64;
    hmi_cmd_lcd_set_progress(2U, (uint8_t)v);
  } else if (varId == PEN_VAR_RSSI || varId == PEN_VAR_RSSL) {
    int32_t v = 100 + value;
    if (v < 0) v = 0;
    if (v > 64) v = 64;
    hmi_cmd_lcd_set_progress(varId == PEN_VAR_RSSI ? 0U : 1U, (int8_t)v);
    return;
  }

  AppEmitVarI(varId, value, ev.msgType, ev.data.varI.seq);
}

static void HandleVarF(const pen_rx_event_t& ev) {
  AppEmitVarF(ev.data.varF.varId, ev.data.varF.value, ev.msgType, ev.data.varF.seq);
}

static void HandleAck(const pen_rx_event_t& ev) {
  char name[5];
  char line[32];
  AppFormatVarName(ev.data.ack.varId, name);
  snprintf(line, sizeof(line), "@ACK %u %s", (unsigned)ev.data.ack.ackSeq, name);
  (void)AppPcTxLine(line);
}

static void HandleNack(const pen_rx_event_t& ev) {
  char name[5];
  char line[32];
  AppFormatVarName(ev.data.nack.varId, name);
  snprintf(line, sizeof(line), "@NACK %u %s %u", (unsigned)ev.data.nack.ackSeq, name, (unsigned)ev.data.nack.reason);
  (void)AppPcTxLine(line);
}

static void HandleLinkEvent(const pen_rx_event_t& ev) {
  switch (ev.data.link.code) {
    case PEN_LINK_READY: 
      (void)AppPcTxLine("@LINK READY"); 
      break;
    case PEN_LINK_DISC:
      {
        char line[32];
        snprintf(line, sizeof(line), "@DISC %d", (int)ev.data.link.rssi);
        (void)AppPcTxLine(line);
        break;
      }
    case PEN_LINK_CONNECTED: 
      (void)AppPcTxLine("@LINK CONNECTED"); 
      break;
    case PEN_LINK_AUTH_OK: 
      (void)AppPcTxLine("@LINK AUTH_OK"); 
      break;
    case PEN_LINK_SECURE:
      hmi_cmd_play_melody(HMI_MELODY_CONNECTED);
      hmi_cmd_lcd_set_indicator(0U, true);
      (void)AppPcTxLine("@LINK SECURE");
      s_usbConnPen = -1;  // resend
      break;
    case PEN_LINK_LOST:
      hmi_cmd_play_melody(HMI_MELODY_DISCONNECTED);
      hmi_cmd_lcd_set_indicator(0U, false);
      hmi_cmd_lcd_set_progress(0U, 0U);
      hmi_cmd_lcd_set_progress(1U, 0U);
      (void)AppPcTxLine("@LINK LOST");
      break;
    case PEN_LINK_CONN_TO: 
      (void)AppPcTxLine("@LINK CONN_TO"); 
      break;
    case PEN_LINK_AUTH_TO: 
      (void)AppPcTxLine("@LINK AUTH_TO"); 
      break;
    case PEN_LINK_MAC_BAD: 
      (void)AppPcTxLine("@LINK MAC_BAD"); 
      break;
    case PEN_LINK_AUTH_BAD: 
      (void)AppPcTxLine("@LINK AUTH_BAD"); 
      break;
    case PEN_LINK_SEC_BAD: 
      (void)AppPcTxLine("@LINK SEC_BAD"); 
      break;
    default: 
      break;
  }
}

static void HandleErrorEvent(const pen_rx_event_t& ev) {
  char line[32];
  snprintf(line, sizeof(line), "@ERR %u %ld", (unsigned)ev.data.error.code, (long)ev.data.error.detail);
  (void)AppPcTxLine(line);
  if (ev.data.error.code != PEN_HW_ERR_NONE) {
    char errText[8];
    snprintf(errText, sizeof(errText), "E%u", (unsigned)ev.data.error.code);
    hmi_cmd_lcd_draw_text(0U, 0U, GUI_COLOR_ORANGE, errText);
  }
}

static void AppProcessPenRxEvent(const pen_rx_event_t& ev) {
  switch (ev.type) {
    case PEN_RX_LINK: HandleLinkEvent(ev); break;
    case PEN_RX_ERROR: HandleErrorEvent(ev); break;
    case PEN_RX_VAR_I: HandleVarI(ev); break;
    case PEN_RX_VAR_F: HandleVarF(ev); break;
    case PEN_RX_ACK: HandleAck(ev); break;
    case PEN_RX_NACK: HandleNack(ev); break;
    default: break;
  }
}

static bool AppPenRxEvent(const pen_rx_event_t* ev) {
  if (ev == nullptr) return false;
  AppProcessPenRxEvent(*ev);
  return true;
}

static void AppProcessPenTx(void) {
  if (!pen_is_connected()) {
    return;
  }

  if (hmi_changed(HMI_DATA_JOY_X)) {
    const float joyX = AppNormalizeAxis(hmi_get(HMI_DATA_JOY_X), s_axisCalX);
    (void)pen_send_stream(PEN_VAR_JX_APP, joyX, 500U);
  }
  if (hmi_changed(HMI_DATA_JOY_Y)) {
    const float joyY = AppNormalizeAxis(hmi_get(HMI_DATA_JOY_Y), s_axisCalY);
    (void)pen_send_stream(PEN_VAR_JY_APP, joyY, 500U);
  }

  // LUP/LDN/RUP/RDN используются как параметры только на Home.
  // В меню (GUIGetActiveScene() != &s_sceneHome) эти кнопки остаются за GUI.
  if (GUIGetActiveScene() == &s_sceneHome) {
    if (hmi_changed(HMI_DATA_BTN_LUP) && (hmi_get(HMI_DATA_BTN_LUP) != 0U)) {
      ++s_lset;
      (void)pen_send_state(PEN_VAR_LSET_APP, s_lset);
    }
    if (hmi_changed(HMI_DATA_BTN_LDN) && (hmi_get(HMI_DATA_BTN_LDN) != 0U)) {
      --s_lset;
      (void)pen_send_state(PEN_VAR_LSET_APP, s_lset);
    }
    if (hmi_changed(HMI_DATA_BTN_RUP) && (hmi_get(HMI_DATA_BTN_RUP) != 0U)) {
      ++s_rset;
      (void)pen_send_state(PEN_VAR_RSET_APP, s_rset);
    }
    if (hmi_changed(HMI_DATA_BTN_RDN) && (hmi_get(HMI_DATA_BTN_RDN) != 0U)) {
      --s_rset;
      (void)pen_send_state(PEN_VAR_RSET_APP, s_rset);
    }
  }

  const int32_t usbConnected = serial_bg_is_connected()?1:0;
  if (usbConnected != s_usbConnPen) {
    (void)pen_send_state(PEN_VAR_USBC_APP, usbConnected);
    s_usbConnPen = usbConnected;
  }
}

// [Auto power off]
static void AppProcessHomePowerOff(void) {
  const uint32_t now = millis();
  static uint32_t lastActivityMs = now;
  static int32_t prevRemSec = 301;

  if (GUIGetActiveScene() != &s_sceneHome || hmi_get(HMI_DATA_BTN_ANYKEY) || pen_is_connected() || serial_bg_is_connected()) {
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
        (void)pen_send_state(PEN_VAR_USBC_APP, connected ? 1L : 0L);
      }
      AppProcessHomePowerOff();
      AppProcessPenTx();
      //GUI
      if (!GUIServiceActiveScene()) {
        hmi_sysSend();
      }
      //Serial
      {
        char line[SERIAL_BG_LINE_CAP];

        if (serial_bg_receive_line(line, sizeof(line))) {
          (void)pen_pc_rx_line(line);
        }
      }
    }
    (void)xTaskDelayUntil(&lastWakeTime, periodTicks);
  }
}

void setup() {
  (void)serial_bg_begin(115200U, false, 1, 2, 4096U);
  hmi_init(HmiLogToSerial);
  (void)pen_begin(AppPenRxEvent);
  hmi_cmd_play_melody(HMI_MELODY_POWER_ON);
  //hmi_cmd_play_tone(200, 50);
  GUISetHomeScene(&s_sceneHome);
  GUISwitchScene(&s_sceneHome);
  (void)xTaskCreatePinnedToCore(AppTask, "AppTask", 4096U, nullptr, 2, &s_appTaskHandle, 1);
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000U));
}
