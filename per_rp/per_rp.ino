
#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_system.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "pen_proto.h"
#include "pen_common.h"

static constexpr uint8_t PER_LED_PIN = 5U;
static constexpr uint32_t PER_LED_MS = 100U;
static constexpr float PER_JOY_EPS = 0.0005f;
static constexpr uint32_t PER_VAR_JX = PEN_VAR_ID2('J', 'X');
static constexpr uint32_t PER_VAR_JY = PEN_VAR_ID2('J', 'Y');
static constexpr uint32_t PER_VAR_BATP = PEN_VAR_ID4('B', 'A', 'T', 'P');

enum per_state_t : uint8_t { PER_WAIT, PER_CONNECTED, PER_AUTH_RSP_SENT, PER_SECURE };

static per_state_t s_state = PER_WAIT;
static uint8_t s_rcMac[6] = {};
static uint8_t s_ownMac[6] = {};
static uint32_t s_deviceId = 0U;
static uint32_t s_rcId = 0U;
static uint32_t s_sessionId = 0U;
static uint16_t s_seq = 1U;
static uint16_t s_authRspSeq = 0U;
static uint8_t s_rcNonce[16] = {};
static uint8_t s_devNonce[16] = {};
static uint8_t s_sessionKey[32] = {};
static uint8_t s_pmk[16] = {};
static uint8_t s_lmk[16] = {};
static uint8_t s_rcProof[16] = {};
static uint8_t s_devProof[16] = {};
static uint32_t s_ledOffMs = 0U;
static float s_lastJx = 0.0f;
static float s_lastJy = 0.0f;
static bool s_hasJx = false;
static bool s_hasJy = false;

static void PerLog(const char* text) { if (text != nullptr) Serial.println(text); }
static void PerLedPulse(void) { digitalWrite(PER_LED_PIN, HIGH); s_ledOffMs = millis() + PER_LED_MS; }

static void PerSendDiscoveryRsp(const uint8_t* mac) {
  pen_discovery_rsp_payload_t p = {};
  p.deviceId = s_deviceId;
  p.caps = PEN_CAPS;
  p.workChannel = PEN_CHANNEL;
  strncpy(p.name, "RP", sizeof(p.name) - 1U);
  (void)pen_add_peer(mac, false, nullptr);
  if (pen_send_frame(mac, MSG_DISCOVERY_RSP, 0U, s_seq++, p)) PerLog("PER DISC RSP");
}

static void PerSendConnectRsp(const uint8_t* mac) {
  pen_connect_rsp_payload_t p = {};
  p.deviceId = s_deviceId;
  p.caps = PEN_CAPS;
  p.sessionId = s_sessionId;
  memcpy(p.devMac, s_ownMac, sizeof(p.devMac));
  memcpy(p.devNonce, s_devNonce, sizeof(p.devNonce));
  if (pen_send_frame(mac, MSG_CONNECT_RSP, 0U, s_seq++, p)) PerLog("PER CONN RSP");
}

static bool PerSendAuthRsp(const uint8_t* mac) {
  pen_auth_rsp_payload_t p = {};
  p.sessionId = s_sessionId;
  memcpy(p.devProof, s_devProof, sizeof(p.devProof));
  const uint16_t seq = s_seq++;
  const bool ok = pen_send_frame(mac, MSG_AUTH_RSP, s_sessionId, seq, p);
  if (ok) { s_authRspSeq = seq; PerLog("PER AUTH RSP"); }
  return ok;
}

static void PerInstallEncryptedPeer(void) {
  (void)esp_now_set_pmk(s_pmk);
  (void)pen_add_peer(s_rcMac, true, s_lmk);
  s_state = PER_SECURE;
  s_hasJx = false;
  s_hasJy = false;
  PerLog("PER SECURE");
}

static void PerSendPong(const uint8_t* mac, uint16_t pingSeq, int8_t rssiAtPeer) {
  pen_pong_payload_t p = {};
  p.pingSeq = pingSeq;
  p.rssiAtPeer = rssiAtPeer;
  (void)pen_send_frame(mac, MSG_PONG, s_sessionId, s_seq++, p);
}

static void PerSendStateI(uint32_t varId, int32_t value) {
  if (s_state != PER_SECURE) {
    PerLog("PER NO LINK");
    return;
  }
  pen_var_i_payload_t p = {};
  p.varId = varId;
  p.ttlMs = 0U;
  p.value = value;
  (void)pen_send_frame(s_rcMac, MSG_STATE_I_VAR, s_sessionId, s_seq++, p);
}

static bool PerStartNewSession(const esp_now_recv_info_t* info, const pen_connect_req_payload_t* p) {
  if (memcmp(p->rcMac, info->src_addr, 6U) != 0) {
    PerLog("PER MAC BAD");
    return false;
  }
  memcpy(s_rcMac, p->rcMac, sizeof(s_rcMac));
  s_rcId = p->rcId;
  memcpy(s_rcNonce, p->rcNonce, sizeof(s_rcNonce));
  esp_fill_random(s_devNonce, sizeof(s_devNonce));
  esp_fill_random(&s_sessionId, sizeof(s_sessionId));
  if (s_sessionId == 0U) s_sessionId = 1U;
  pen_derive_session(s_rcId, s_deviceId, s_sessionId, s_rcMac, s_ownMac, s_rcNonce, s_devNonce, s_sessionKey, s_pmk, s_lmk, s_rcProof, s_devProof);
  s_state = PER_CONNECTED;
  return true;
}

static void PerPrintJoy(const char* name, float value) {
  char line[32];
  snprintf(line, sizeof(line), "PER %s %.4f", name, value);
  Serial.println(line);
}

static void PerHandleStreamFloat(const pen_var_f_payload_t* p) {
  if (p == nullptr) return;
  if (p->varId == PER_VAR_JX) {
    if (!s_hasJx || fabsf(p->value - s_lastJx) >= PER_JOY_EPS) {
      s_lastJx = p->value;
      s_hasJx = true;
      PerPrintJoy("JX", p->value);
    }
  } else if (p->varId == PER_VAR_JY) {
    if (!s_hasJy || fabsf(p->value - s_lastJy) >= PER_JOY_EPS) {
      s_lastJy = p->value;
      s_hasJy = true;
      PerPrintJoy("JY", p->value);
    }
  }
}

static void PerHandleSerial(void) {
  static char line[32];
  static size_t len = 0U;
  while (Serial.available() > 0) {
    const int ch = Serial.read();
    if (ch < 0) break;
    if ((ch == '\r') || (ch == '\n')) {
      if (len == 0U) continue;
      line[len] = '\0';
      len = 0U;
      char* savePtr = nullptr;
      char* cmd = strtok_r(line, " \t", &savePtr);
      if (cmd == nullptr) continue;
      if (strcmp(cmd, "BATP") == 0) {
        char* arg = strtok_r(nullptr, " \t", &savePtr);
        if (arg == nullptr) continue;
        char* endPtr = nullptr;
        long value = strtol(arg, &endPtr, 10);
        if ((*endPtr == '\0') && (value >= 0) && (value <= 100)) {
          PerSendStateI(PER_VAR_BATP, (int32_t)value);
        }
      }
      continue;
    }
    if (len + 1U < sizeof(line)) line[len++] = (char)ch;
  }
}

static bool PerIsStateOrEvent(uint8_t type) {
  return (type == MSG_STATE_I_VAR) || (type == MSG_STATE_F_VAR) || (type == MSG_EVENT_I_VAR) || (type == MSG_EVENT_F_VAR);
}

static void PerSendAck(uint16_t ackSeq, uint32_t varId) {
  pen_ack_payload_t p = {};
  p.ackSeq = ackSeq;
  p.varId = varId;
  (void)pen_send_frame(s_rcMac, MSG_ACK, s_sessionId, s_seq++, p);
}

static void OnRecv(const esp_now_recv_info_t* info, const uint8_t* data, int len) {
  if ((info == nullptr) || (info->src_addr == nullptr) || (data == nullptr) || !pen_frame_valid(data, (size_t)len)) return;
  const pen_hdr_t* hdr = reinterpret_cast<const pen_hdr_t*>(data);
  const uint8_t type = pen_msg_base(hdr->msgType);
  const uint8_t* payload = data + sizeof(pen_hdr_t);
  const int8_t rssi = (info->rx_ctrl != nullptr) ? info->rx_ctrl->rssi : -100;

  if (type == MSG_DISCOVERY_REQ) {
    PerSendDiscoveryRsp(info->src_addr);
    PerLedPulse();
  } else if (type == MSG_CONNECT_REQ) {
    const auto* p = reinterpret_cast<const pen_connect_req_payload_t*>(payload);
    if (PerStartNewSession(info, p)) PerSendConnectRsp(info->src_addr);
  } else if ((type == MSG_AUTH_REQ) && ((s_state == PER_CONNECTED) || (s_state == PER_AUTH_RSP_SENT)) && (memcmp(info->src_addr, s_rcMac, 6U) == 0)) {
    const auto* p = reinterpret_cast<const pen_auth_req_payload_t*>(payload);
    if ((p->sessionId == s_sessionId) && (memcmp(p->rcProof, s_rcProof, 16U) == 0)) {
      if (PerSendAuthRsp(info->src_addr)) s_state = PER_AUTH_RSP_SENT;
    } else {
      PerLog("PER AUTH BAD");
    }
  } else if ((type == MSG_ACK) && (memcmp(info->src_addr, s_rcMac, 6U) == 0)) {
    const auto* p = reinterpret_cast<const pen_ack_payload_t*>(payload);
    if ((s_state == PER_AUTH_RSP_SENT) && (p->ackSeq == s_authRspSeq)) PerInstallEncryptedPeer();
  } else if ((type == MSG_NACK) && (s_state == PER_SECURE) && (memcmp(info->src_addr, s_rcMac, 6U) == 0)) {
    PerLog("PER NACK");
  } else if ((type == MSG_PING) && (s_state == PER_SECURE) && (memcmp(info->src_addr, s_rcMac, 6U) == 0)) {
    const auto* p = reinterpret_cast<const pen_ping_payload_t*>(payload);
    PerSendPong(info->src_addr, p->pingSeq, rssi);
  } else if ((type == MSG_STREAM_F_VAR) && (s_state == PER_SECURE) && (memcmp(info->src_addr, s_rcMac, 6U) == 0)) {
    PerHandleStreamFloat(reinterpret_cast<const pen_var_f_payload_t*>(payload));
  } else if ((PerIsStateOrEvent(type)) && (s_state == PER_SECURE) && (memcmp(info->src_addr, s_rcMac, 6U) == 0)) {
    if ((type == MSG_STATE_I_VAR) || (type == MSG_EVENT_I_VAR)) {
      const auto* p = reinterpret_cast<const pen_var_i_payload_t*>(payload);
      PerSendAck(hdr->seq, p->varId);
    } else {
      const auto* p = reinterpret_cast<const pen_var_f_payload_t*>(payload);
      PerSendAck(hdr->seq, p->varId);
    }
  }
}

void setup() {
  pinMode(PER_LED_PIN, OUTPUT);
  digitalWrite(PER_LED_PIN, LOW);
  Serial.begin(115200U);
  delay(50U);
  WiFi.mode(WIFI_STA);
  WiFi.disconnect(true, true);
  if (!pen_get_sta_mac(s_ownMac)) {
    PerLog("PER MAC ERR");
    return;
  }
  s_deviceId = pen_make_id_from_efuse();
  (void)esp_wifi_set_promiscuous(true);
  (void)esp_wifi_set_channel(PEN_CHANNEL, WIFI_SECOND_CHAN_NONE);
  (void)esp_wifi_set_promiscuous(false);
  (void)esp_now_init();
  (void)esp_now_register_recv_cb(OnRecv);
  PerLog("PER READY");
}

void loop() {
  PerHandleSerial();
  if ((s_ledOffMs != 0U) && ((int32_t)(millis() - s_ledOffMs) >= 0)) {
    digitalWrite(PER_LED_PIN, LOW);
    s_ledOffMs = 0U;
  }
  delay(2U);
}
