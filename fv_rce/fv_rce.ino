
#include <Arduino.h>
#include <ctype.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <stdint.h>
#include <stddef.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "rio.h"
#include "gui.h"
#include "st7735.h"
#include "serial_bg.h"
#include "pen_rc.h"

#define MELODY 1

static gui_axis_cal_t s_axisCalX = { 226U, 1951U, 1959U, 4028U };
static gui_axis_cal_t s_axisCalY = { 0U, 1953U, 1962U, 4027U };

// [gui]
static int32_t s_lset = 0;
static int32_t s_rset = 0;

extern gui_scene_t s_sceneHome;
extern gui_scene_t s_sceneMainMenu;
extern gui_scene_t s_sceneCCentr;
extern gui_scene_t s_sceneCEdge;

static GUIClsComponent s_sceneHomeCls(GUI_COLOR_BLACK);
static GUIJViewComponent s_sceneHomeJView(GUI_J_VIEW_MODE_TRACK, &s_axisCalX, &s_axisCalY);
static GUIVarComponent s_sceneHomeLSet(5U, 40U, GUI_COLOR_WHITE, &s_lset);
static GUIVarComponent s_sceneHomeRSet(120U, 40U, GUI_COLOR_WHITE, &s_rset);
static GUIBrightnessComponent s_sceneHomeBrightness(0U, 0U, 0U);
static GUIHotKeyComponent s_sceneHomeHotKeyOk(RIO_DATA_BTN_OK, &s_sceneMainMenu);
static GUIComponent* s_sceneHomeItems[] = { &s_sceneHomeCls, &s_sceneHomeJView, &s_sceneHomeLSet, &s_sceneHomeRSet, &s_sceneHomeBrightness, &s_sceneHomeHotKeyOk };
gui_scene_t s_sceneHome = GUI_SCENE(s_sceneHomeItems);

static GUIClsComponent s_sceneMainMenuCls(GUI_COLOR_BLACK);
static GUILabelComponent s_sceneMainMenuTitle(30U, 10U, GUI_COLOR_GRAY, "Main menu");
static GUIBrightnessComponent s_sceneMainMenuBrightness(1U, 118U, 10U);
static GUIMenuItemComponent s_sceneMainMenuItemCalCenter(5U, 25U, "Cal. center", &s_sceneCCentr);
static GUIMenuItemComponent s_sceneMainMenuItemCalEdge(5U, 40U, "Cal. edge", &s_sceneCEdge);
static GUIHotKeyComponent s_sceneMainMenuHotKeyBack(RIO_DATA_BTN_BACK, &s_sceneHome);
static GUIComponent* s_sceneMainMenuItems[] = { &s_sceneMainMenuCls, &s_sceneMainMenuTitle, &s_sceneMainMenuBrightness, &s_sceneMainMenuItemCalCenter, &s_sceneMainMenuItemCalEdge, &s_sceneMainMenuHotKeyBack };
gui_scene_t s_sceneMainMenu = GUI_SCENE(s_sceneMainMenuItems);

static GUIClsComponent s_sceneCCentrCls(GUI_COLOR_BLACK);
static GUIJViewComponent s_sceneCCentrJView(GUI_J_VIEW_MODE_CAL_CENTER, &s_axisCalX, &s_axisCalY);
static GUIHotKeyComponent s_sceneCCentrHotKeyBack(RIO_DATA_BTN_BACK, &s_sceneMainMenu);
static GUIHotKeyComponent s_sceneCCentrHotKeyOk(RIO_DATA_BTN_OK, &s_sceneMainMenu);
static GUILabelComponent s_sceneCalibrateBack(30U, 18U, GUI_COLOR_ORANGE, "D\nR\nO\nP");
static GUILabelComponent s_sceneCalibrateOk(120U, 18U, GUI_COLOR_GREEN, "S\nA\nV\nE");
static GUIComponent* s_sceneCCentrItems[] = { &s_sceneCCentrCls, &s_sceneCCentrJView, &s_sceneCCentrHotKeyBack, &s_sceneCCentrHotKeyOk, &s_sceneCalibrateBack, &s_sceneCalibrateOk };
gui_scene_t s_sceneCCentr = GUI_SCENE(s_sceneCCentrItems);

static GUIClsComponent s_sceneCEdgeCls(GUI_COLOR_BLACK);
static GUIJViewComponent s_sceneCEdgeJView(GUI_J_VIEW_MODE_CAL_EDGE, &s_axisCalX, &s_axisCalY);
static GUIHotKeyComponent s_sceneCEdgeHotKeyBack(RIO_DATA_BTN_BACK, &s_sceneMainMenu);
static GUIHotKeyComponent s_sceneCEdgeHotKeyOk(RIO_DATA_BTN_OK, &s_sceneMainMenu);
static GUIComponent* s_sceneCEdgeItems[] = { &s_sceneCEdgeCls, &s_sceneCEdgeJView, &s_sceneCEdgeHotKeyBack, &s_sceneCEdgeHotKeyOk, &s_sceneCalibrateBack, &s_sceneCalibrateOk };
gui_scene_t s_sceneCEdge = GUI_SCENE(s_sceneCEdgeItems);

// [PEN app variable IDs]
static constexpr uint32_t PEN_VAR_JX_APP   = PEN_VAR_ID2('J', 'X');
static constexpr uint32_t PEN_VAR_JY_APP   = PEN_VAR_ID2('J', 'Y');
static constexpr uint32_t PEN_VAR_BATP_APP = PEN_VAR_ID4('B', 'A', 'T', 'P');
static constexpr uint32_t PEN_VAR_LSET_APP = PEN_VAR_ID4('L', 'S', 'E', 'T');
static constexpr uint32_t PEN_VAR_RSET_APP = PEN_VAR_ID4('R', 'S', 'E', 'T');
static constexpr uint32_t PEN_VAR_USBC_APP = PEN_VAR_ID4('U', 'S', 'B', 'C');

static TaskHandle_t s_appTaskHandle = nullptr;
static TaskHandle_t s_rxTaskHandle = nullptr;
static int32_t s_usbConnPen;

// [Log]
static void RioLogToSerial(const char* text, bool emergency) {
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


/* -------------------------------------------------------------------------- */
/* Progress bar / battery SOC                                                 */
/* -------------------------------------------------------------------------- */
/* Ubat = ADC_V * 0.00399446 - 0.19284 */
/* Full battery: 4.10 V -> ADC = 1075 */
static const uint16_t ocv_adc[] =
  { 895, 919, 964, 985, 1010, 1020, 1027, 1035, 1047, 1055, 1060, 1067, 1075 };

/* Progress bar range: 0..64 */
static const uint8_t ocv_soc[] =
  { 0, 4, 13, 19, 29, 32, 36, 39, 45, 49, 52, 58, 64 };

#define OCV_POINTS (sizeof(ocv_adc) / sizeof(ocv_adc[0]))

static inline uint8_t interp_fast(uint16_t x, uint16_t x1, uint16_t x2, uint8_t y1, uint8_t y2) {
  uint16_t dx = (uint16_t)(x2 - x1);
  uint16_t num = (uint16_t)(x - x1);
  uint16_t t = (uint16_t)((num << 8) / dx);
  return (uint8_t)(y1 + ((((uint16_t)(y2 - y1)) * t) >> 8));
}

static uint8_t adc_to_soc(uint16_t adc) {
  int low;
  int high;

  if (adc <= ocv_adc[0]) return ocv_soc[0];
  if (adc >= ocv_adc[OCV_POINTS - 1U]) return ocv_soc[OCV_POINTS - 1U];

  low = 0;
  high = (int)OCV_POINTS - 1;
  while ((high - low) > 1) {
    int mid = (low + high) >> 1;
    if (adc < ocv_adc[mid]) {
      high = mid;
    } else {
      low = mid;
    }
  }
  return interp_fast(adc, ocv_adc[low], ocv_adc[high], ocv_soc[low], ocv_soc[high]);
}

static void AppProcessRioBattery(void) {
  if (rio_changed(RIO_DATA_ADC_V)) {
    GUISetProgress(3U, adc_to_soc(rio_get(RIO_DATA_ADC_V)));
  }
}

static void AppEmitVarI(uint32_t varId, int32_t value) {
  char name[5];
  char line[32];
  AppFormatVarName(varId, name);
  snprintf(line, sizeof(line), "%s %ld", name, (long)value);
  (void)AppPcTxLine(line);
}

static void AppEmitVarF(uint32_t varId, float value) {
  char name[5];
  char line[32];
  AppFormatVarName(varId, name);
  snprintf(line, sizeof(line), "%s %.4f", name, (double)value);
  (void)AppPcTxLine(line);
}

static void HandleVarI(const pen_rx_event_t& ev) {
  const uint32_t varId = ev.data.varI.varId;
  const int32_t value = ev.data.varI.value;
  const bool retry = ev.data.varI.retry;

  if (varId == PEN_VAR_BATP_APP && !retry) {
    int32_t v = value*64/100;
    if (v < 0) v = 0;
    if (v > 64) v = 64;
    GUISetProgress(2U, (uint8_t)v);
  } else if (varId == PEN_VAR_RSSI || varId == PEN_VAR_RSSL) {
    int32_t v = 100 + value;
    if (v < 0) v = 0;
    if (v > 64) v = 64;
    GUISetProgress(varId == PEN_VAR_RSSI ? 0U : 1U, (uint8_t)v);
    return;
  }
  if(!retry){
    AppEmitVarI(varId, value);
  }
}

static void HandleVarF(const pen_rx_event_t& ev) {
  if (ev.data.varF.retry) return;
  AppEmitVarF(ev.data.varF.varId, ev.data.varF.value);
}

static void HandleAck(const pen_rx_event_t& ev) {
  char name[5];
  char line[32];
  AppFormatVarName(ev.data.ack.varId, name);
  snprintf(line, sizeof(line), "@ACK %s", name);
  (void)AppPcTxLine(line);
}

static void HandleNack(const pen_rx_event_t& ev) {
  char name[5];
  char line[32];
  AppFormatVarName(ev.data.nack.varId, name);
  snprintf(line, sizeof(line), "@NACK %s %u", name, (unsigned)ev.data.nack.reason);
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
#ifdef MELODY 
      rio_cmd_play_melody(RIO_MELODY_CONNECTED);
#endif
      GUISetIndicator(0U, true);
      (void)AppPcTxLine("@LINK SECURE");
      s_usbConnPen = -1;  // resend
      break;
    case PEN_LINK_LOST:
#ifdef MELODY
      rio_cmd_play_melody(RIO_MELODY_DISCONNECTED);
#endif
      GUISetIndicator(0U, false);
      GUISetProgress(0U, 0U);
      GUISetProgress(1U, 0U);
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
    LCD_DrawText(120U, 15U, GUI_COLOR_ORANGE, errText);
  }
}

static bool AppPenRxEvent(const pen_rx_event_t* ev) {
  if (ev == nullptr) return false;
  switch (ev->type) {
    case PEN_RX_LINK: HandleLinkEvent(*ev); break;
    case PEN_RX_ERROR: HandleErrorEvent(*ev); break;
    case PEN_RX_VAR_I: HandleVarI(*ev); break;
    case PEN_RX_VAR_F: HandleVarF(*ev); break;
    case PEN_RX_ACK: HandleAck(*ev); break;
    case PEN_RX_NACK: HandleNack(*ev); break;
    default: break;
  }
  return true;
}

static void AppProcessPenTx(void) {
  if (!pen_is_connected()) return;

  if (rio_changed(RIO_DATA_JOY_X)) {
    const float joyX = AppNormalizeAxis(rio_get(RIO_DATA_JOY_X), s_axisCalX);
    (void)pen_send_stream(PEN_VAR_JX_APP, joyX, 500U);
  }
  if (rio_changed(RIO_DATA_JOY_Y)) {
    const float joyY = AppNormalizeAxis(rio_get(RIO_DATA_JOY_Y), s_axisCalY);
    (void)pen_send_stream(PEN_VAR_JY_APP, joyY, 500U);
  }

  if (GUIGetActiveScene() == &s_sceneHome) {
    if (rio_changed(RIO_DATA_BTN_LUP) && (rio_get(RIO_DATA_BTN_LUP) != 0U)) {
      ++s_lset;
      (void)pen_send_state(PEN_VAR_LSET_APP, s_lset);
    }
    if (rio_changed(RIO_DATA_BTN_LDN) && (rio_get(RIO_DATA_BTN_LDN) != 0U)) {
      --s_lset;
      (void)pen_send_state(PEN_VAR_LSET_APP, s_lset);
    }
    if (rio_changed(RIO_DATA_BTN_RUP) && (rio_get(RIO_DATA_BTN_RUP) != 0U)) {
      ++s_rset;
      (void)pen_send_state(PEN_VAR_RSET_APP, s_rset);
    }
    if (rio_changed(RIO_DATA_BTN_RDN) && (rio_get(RIO_DATA_BTN_RDN) != 0U)) {
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

  if (GUIGetActiveScene() != &s_sceneHome || rio_get(RIO_DATA_BTN_ANYKEY) || pen_is_connected() || serial_bg_is_connected()) {
    lastActivityMs = now;
    prevRemSec = 301;
    return;
  }
  const uint32_t idleS = (uint32_t)(now - lastActivityMs) / 1000U;
  const int32_t remainingS = 300L - (int32_t)idleS;
  if(prevRemSec != remainingS){
    if (remainingS==6 || remainingS==4 || remainingS==2) {
      rio_cmd_play_tone(500U, 50U);
    } else if (remainingS == 1) {
      rio_cmd_play_melody(RIO_MELODY_DISCONNECTED);
    } else if (remainingS <= 0) {
      rio_cmd_power_off();
    }
    prevRemSec = remainingS;
  }
}

void setup() {
  (void)serial_bg_begin(115200U, false, 1, 2, 4096U);
  rio_init(RioLogToSerial);

  LCD_Init();
  LCD_FillRect(0U, 0U, LCD_WIDTH, 8U, LCD_BLACK);
  LCD_DrawMarker(LCD_WIDTH / 2U, 4, 9, LCD_WHITE);  // battery
  GUISetIndicator(0U, 0U);
  GUISetIndicator(1U, 0U);

  (void)pen_begin();
#ifdef MELODY
  rio_cmd_play_melody(RIO_MELODY_POWER_ON);
#else
  rio_cmd_play_tone(200, 50);
#endif
  GUISetHomeScene(&s_sceneHome);
  GUISwitchScene(&s_sceneHome);
  vTaskPrioritySet(xTaskGetHandle("loopTask"), 2);
}

void loop() {
  static TickType_t lastHmiTick = 0;
  static char line[SERIAL_BG_LINE_CAP];

  pen_rx_event_t ev = {};
  TickType_t now = xTaskGetTickCount();

  if(now - lastHmiTick >= pdMS_TO_TICKS(5)){
    lastHmiTick = now;
    if (rio_tick() == RIO_TICK_OK) {
      if (rio_changed(RIO_DATA_STAT_USB_CONN)) {  // Usb Connection Changed
        const bool connected = (rio_get(RIO_DATA_STAT_USB_CONN) != 0U);
        serial_bg_set_connected(connected);
        GUISetIndicator(1U, connected);
      }
      AppProcessRioBattery();
      AppProcessHomePowerOff();
      AppProcessPenTx();
      //GUI
      (void)GUIServiceActiveScene();
      rio_sysSend();
    }
  } 
  if(pen_receive(&ev)) {
    (void)AppPenRxEvent(&ev);
  }
  if (serial_bg_receive_line(line, sizeof(line))) {
    (void)pen_pc_rx_line(line);
  } 
  (void)LCD_Process();
  vTaskDelay(1);
}
