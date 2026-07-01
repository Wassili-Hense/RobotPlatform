#include <Arduino.h>
#include <ctype.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stddef.h>

#define PEN_RC 0
#include "pen_link.h"

static constexpr uint32_t APP_SERIAL_BAUD = 115200U;
static constexpr uint16_t APP_STREAM_TTL_MS = 500U;
static constexpr size_t APP_LINE_CAP = 64U;

static char s_line[APP_LINE_CAP];
static size_t s_lineLen = 0U;

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

static void VarIdToText(uint32_t varId, char out[5]) {
  out[0] = (char)((varId >> 0) & 0xFFU);
  out[1] = (char)((varId >> 8) & 0xFFU);
  out[2] = (char)((varId >> 16) & 0xFFU);
  out[3] = (char)((varId >> 24) & 0xFFU);
  out[4] = '\0';
}

static bool LooksFloat(const char* text) {
  return (text != nullptr) && ((strchr(text, '.') != nullptr) || (strchr(text, 'e') != nullptr) || (strchr(text, 'E') != nullptr));
}

static void SerialPrintVarI(uint32_t varId, int32_t value, uint8_t msgType, uint16_t seq) {
  char name[5];
  VarIdToText(varId, name);
  Serial.printf("%s %ld I %u %u\r\n", name, (long)value, (unsigned)msgType, (unsigned)seq);
}

static void SerialPrintVarF(uint32_t varId, float value, uint8_t msgType, uint16_t seq) {
  char name[5];
  VarIdToText(varId, name);
  Serial.printf("%s %.4f F %u %u\r\n", name, (double)value, (unsigned)msgType, (unsigned)seq);
}

static void HandleLinkEvent(const pen_rx_event_t& ev) {
  switch (ev.data.link.code) {
    case PEN_LINK_READY:
      Serial.println("@LINK READY");
      break;
    case PEN_LINK_DISC:
      Serial.printf("@DISC %d\r\n", (int)ev.data.link.rssi);
      break;
    case PEN_LINK_CONNECTED:
      Serial.println("@LINK CONNECTED");
      break;
    case PEN_LINK_AUTH_OK:
      Serial.println("@LINK AUTH_OK");
      break;
    case PEN_LINK_SECURE:
      Serial.println("@LINK SECURE");
      break;
    case PEN_LINK_LOST:
      Serial.println("@LINK LOST");
      break;
    case PEN_LINK_CONN_TO:
      Serial.println("@LINK CONN_TO");
      break;
    case PEN_LINK_AUTH_TO:
      Serial.println("@LINK AUTH_TO");
      break;
    case PEN_LINK_MAC_BAD:
      Serial.println("@LINK MAC_BAD");
      break;
    case PEN_LINK_AUTH_BAD:
      Serial.println("@LINK AUTH_BAD");
      break;
    case PEN_LINK_SEC_BAD:
      Serial.println("@LINK SEC_BAD");
      break;
    default:
      Serial.printf("@LINK %u\r\n", (unsigned)ev.data.link.code);
      break;
  }
}

static void HandleErrorEvent(const pen_rx_event_t& ev) {
  Serial.printf("@ERR %u %ld\r\n", (unsigned)ev.data.error.code, (long)ev.data.error.detail);
}

static void HandleAck(const pen_rx_event_t& ev) {
  char name[5];
  VarIdToText(ev.data.ack.varId, name);
  Serial.printf("@ACK %u %s\r\n", (unsigned)ev.data.ack.ackSeq, name);
}

static void HandleNack(const pen_rx_event_t& ev) {
  char name[5];
  VarIdToText(ev.data.nack.varId, name);
  Serial.printf("@NACK %u %s %u\r\n", (unsigned)ev.data.nack.ackSeq, name, (unsigned)ev.data.nack.reason);
}

static void HandleGetVar(const pen_rx_event_t& ev) {
  char name[5];
  VarIdToText(ev.data.getVar.varId, name);
  Serial.printf("@GET %u %s\r\n", (unsigned)ev.data.getVar.seq, name);
}

static bool PenRxEvent(const pen_rx_event_t* ev) {
  if (ev == nullptr) return false;
  switch (ev->type) {
    case PEN_RX_LINK:
      HandleLinkEvent(*ev);
      break;
    case PEN_RX_ERROR:
      HandleErrorEvent(*ev);
      break;
    case PEN_RX_VAR_I:
      SerialPrintVarI(ev->data.varI.varId, ev->data.varI.value, ev->msgType, ev->data.varI.seq);
      break;
    case PEN_RX_VAR_F:
      SerialPrintVarF(ev->data.varF.varId, ev->data.varF.value, ev->msgType, ev->data.varF.seq);
      break;
    case PEN_RX_ACK:
      HandleAck(*ev);
      break;
    case PEN_RX_NACK:
      HandleNack(*ev);
      break;
    case PEN_RX_GET_VAR:
      HandleGetVar(*ev);
      break;
    default:
      Serial.printf("@RX %u %u\r\n", (unsigned)ev->type, (unsigned)ev->msgType);
      break;
  }
  return true;
}

static bool ProcessSerialLine(const char* line) {
  if (line == nullptr) return false;
  char buf[APP_LINE_CAP];
  strncpy(buf, line, sizeof(buf) - 1U);
  buf[sizeof(buf) - 1U] = '\0';

  char* savePtr = nullptr;
  char* varText = strtok_r(buf, " \t\r\n", &savePtr);
  char* valueText = strtok_r(nullptr, " \t\r\n", &savePtr);
  if ((varText == nullptr) || (valueText == nullptr)) return false;

  uint32_t varId = 0U;
  if (!VarIdFromText(varText, &varId)) return false;

  if (LooksFloat(valueText)) {
    char* endPtr = nullptr;
    const float value = strtof(valueText, &endPtr);
    if ((endPtr == valueText) || (*endPtr != '\0')) return false;
    return pen_send_stream(varId, value, APP_STREAM_TTL_MS);
  }

  char* endPtr = nullptr;
  const long value = strtol(valueText, &endPtr, 10);
  if ((endPtr == valueText) || (*endPtr != '\0') || (value < INT32_MIN) || (value > INT32_MAX)) return false;
  return pen_send_stream(varId, (int32_t)value, APP_STREAM_TTL_MS);
}

static void ProcessSerialRx(void) {
  while (Serial.available() > 0) {
    const char c = (char)Serial.read();
    if ((c == '\r') || (c == '\n')) {
      if (s_lineLen > 0U) {
        s_line[s_lineLen] = '\0';
        if (!ProcessSerialLine(s_line)) Serial.println("@ERR SERIAL");
        s_lineLen = 0U;
      }
    } else if (s_lineLen < (sizeof(s_line) - 1U)) {
      s_line[s_lineLen++] = c;
    } else {
      s_lineLen = 0U;
      Serial.println("@ERR LINE_OVF");
    }
  }
}

void setup() {
  Serial.begin(APP_SERIAL_BAUD);
  while (!Serial) {
    delay(10);
  }
  Serial.println("@BOOT RP");
  (void)pen_begin(PenRxEvent);
}

void loop() {
  ProcessSerialRx();
  delay(5);
}
