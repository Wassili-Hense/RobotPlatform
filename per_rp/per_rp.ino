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
static constexpr uint32_t PER_HEARTBEAT_MS = 150U;
static constexpr uint16_t PER_HEARTBEAT_TTL_MS = 300U;
static constexpr uint32_t PER_LINK_TIMEOUT_MS = 500U;
static constexpr size_t PER_STREAM_CACHE_CAP = 8U;

static constexpr uint32_t PER_VAR_JX = PEN_VAR_ID2('J', 'X');
static constexpr uint32_t PER_VAR_JY = PEN_VAR_ID2('J', 'Y');
static constexpr uint32_t PER_VAR_BATP = PEN_VAR_ID4('B', 'A', 'T', 'P');
static constexpr uint32_t PER_VAR_LSET = PEN_VAR_ID4('L', 'S', 'E', 'T');
static constexpr uint32_t PER_VAR_RSET = PEN_VAR_ID4('R', 'S', 'E', 'T');
static constexpr uint32_t PER_VAR_USBC = PEN_VAR_ID4('U', 'S', 'B', 'C');
static constexpr uint32_t PER_VAR_RSSI = PEN_VAR_ID4('R','S','S','I');

enum per_state_t : uint8_t { PER_WAIT, PER_CONNECTED, PER_AUTH_RSP_SENT, PER_SECURE };

struct per_stream_cache_t {
  bool used;
  bool isFloat;
  uint32_t varId;
  uint16_t ttlMs;
  uint32_t lastTxMs;
  int32_t iValue;
  float fValue;
};

static per_state_t s_state = PER_WAIT;
static uint8_t s_rcMac[6] = {};
static uint8_t s_ownMac[6] = {};
static uint32_t s_deviceId = 0U;
static uint32_t s_rcId = 0U;
static uint32_t s_sessionId = 0U;
static uint16_t s_seq = 1U;
static uint16_t s_authRspSeq = 0U;
static uint32_t s_hbCounter = 0U;
static uint8_t s_rcNonce[16] = {};
static uint8_t s_devNonce[16] = {};
static uint8_t s_sessionKey[32] = {};
static uint8_t s_pmk[16] = {};
static uint8_t s_lmk[16] = {};
static uint8_t s_rcProof[16] = {};
static uint8_t s_devProof[16] = {};
static uint32_t s_ledOffMs = 0U;
static uint32_t s_lastRxMs = 0U;
static uint32_t s_lastTxMs = 0U;
static per_stream_cache_t s_streams[PER_STREAM_CACHE_CAP];
static float s_lastJx = 0.0f;
static float s_lastJy = 0.0f;
static bool s_hasJx = false;
static bool s_hasJy = false;
static int32_t s_stateLset = 0;
static int32_t s_stateRset = 0;
static int32_t s_stateUsbc = 0;
static int8_t s_lastRxRssi = -100;

static void PerLog(const char* text) { if (text != nullptr) Serial.println(text); }
static void PerLedPulse(void) { digitalWrite(PER_LED_PIN, HIGH); s_ledOffMs = millis() + PER_LED_MS; }
static void PerClearStreamCache(void) { memset(s_streams, 0, sizeof(s_streams)); }

static bool PerSecurePeerOk(const esp_now_recv_info_t* info, const pen_hdr_t* hdr) {
  return (s_state == PER_SECURE) && (info != nullptr) && (hdr != nullptr) &&
         (hdr->sessionId == s_sessionId) && (memcmp(info->src_addr, s_rcMac, 6U) == 0);
}

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
  memcpy(p.devProof, s_devProof, sizeof(p.devProof));
  const uint16_t seq = s_seq++;
  const bool ok = pen_send_frame(mac, MSG_AUTH_RSP, s_sessionId, seq, p);
  if (ok) { s_authRspSeq = seq; PerLog("PER AUTH RSP"); }
  return ok;
}

static void PerResetToWait(const char* reason) {
  if (esp_now_is_peer_exist(s_rcMac)) (void)esp_now_del_peer(s_rcMac);
  memset(s_rcMac, 0, sizeof(s_rcMac));
  s_state = PER_WAIT;
  s_sessionId = 0U;
  s_lastRxMs = 0U;
  s_lastTxMs = 0U;
  s_hbCounter = 0U;
  s_hasJx = false;
  s_hasJy = false;
  PerClearStreamCache();
  if (reason != nullptr) PerLog(reason);
}

static void PerInstallEncryptedPeer(void) {
  (void)esp_now_set_pmk(s_pmk);
  (void)pen_add_peer(s_rcMac, true, s_lmk);
  const uint32_t now = millis();
  s_state = PER_SECURE;
  s_hasJx = false;
  s_hasJy = false;
  s_lastRxMs = now;
  s_lastTxMs = now;
  s_hbCounter = 0U;
  PerClearStreamCache();
  PerCacheStreamI(PER_VAR_RSSI, (int32_t)s_lastRxRssi, 300);
  PerLog("PER SECURE");
}

static void PerCacheStreamI(uint32_t varId, int32_t value, uint16_t ttlMs) {
  if (varId == PEN_VAR_HB) return;
  size_t slot = PER_STREAM_CACHE_CAP;
  for (size_t i = 0; i < PER_STREAM_CACHE_CAP; ++i) {
    if (s_streams[i].used && !s_streams[i].isFloat && (s_streams[i].varId == varId)) { slot = i; break; }
    if (!s_streams[i].used && (slot == PER_STREAM_CACHE_CAP)) slot = i;
  }
  if (slot < PER_STREAM_CACHE_CAP) { s_streams[slot].used = true; s_streams[slot].isFloat = false; s_streams[slot].varId = varId; s_streams[slot].ttlMs = ttlMs; s_streams[slot].lastTxMs = millis(); s_streams[slot].iValue = value; }
}

static bool PerSendStreamIRaw(uint32_t varId, int32_t value, uint16_t ttlMs) {
  if (s_state != PER_SECURE) return false;
  pen_var_i_payload_t p = {};
  p.varId = varId;
  p.ttlMs = ttlMs;
  p.value = value;
  const bool ok = pen_send_frame(s_rcMac, MSG_STREAM_I_VAR, s_sessionId, s_seq++, p);
  if (ok) s_lastTxMs = millis();
  return ok;
}

static bool PerSendVarI(uint8_t msgType, uint32_t varId, int32_t value, uint16_t ttlMs) {
  if (s_state != PER_SECURE) return false;
  pen_var_i_payload_t p = {};
  p.varId = varId;
  p.ttlMs = ttlMs;
  p.value = value;
  const bool ok = pen_send_frame(s_rcMac, msgType, s_sessionId, s_seq++, p);
  if (ok) {
    s_lastTxMs = millis();
    if (msgType == MSG_STREAM_I_VAR) PerCacheStreamI(varId, value, ttlMs);
  }
  return ok;
}

static void PerSendStateI(uint32_t varId, int32_t value) {
  if (!PerSendVarI(MSG_STATE_I_VAR, varId, value, 0U)) PerLog("PER NO LINK");
}

static bool PerSendStreamHeartbeat(uint32_t now) {
  int expired = -1;
  int oldest = -1;
  uint32_t expiredAge = 0U;
  uint32_t oldestAge = 0U;
  for (size_t i = 0; i < PER_STREAM_CACHE_CAP; ++i) {
    if (!s_streams[i].used) continue;
    const uint32_t age = now - s_streams[i].lastTxMs;
    if ((oldest < 0) || (age > oldestAge)) { oldest = (int)i; oldestAge = age; }
    if ((s_streams[i].ttlMs != 0U) && (age >= s_streams[i].ttlMs) && ((expired < 0) || (age > expiredAge))) { expired = (int)i; expiredAge = age; }
  }
  const int pick = (expired >= 0) ? expired : (((now - s_lastTxMs) >= PER_HEARTBEAT_MS) ? oldest : -1);
  if (pick >= 0) {
    per_stream_cache_t item = s_streams[pick];
    s_streams[pick].lastTxMs = now;
    return PerSendStreamIRaw(item.varId, item.iValue, item.ttlMs);
  }
  if ((oldest < 0) && ((now - s_lastTxMs) >= PER_HEARTBEAT_MS)) return PerSendStreamIRaw(PEN_VAR_HB, (int32_t)(++s_hbCounter), PER_HEARTBEAT_TTL_MS);
  return true;
}

static bool PerStartNewSession(const esp_now_recv_info_t* info, const pen_connect_req_payload_t* p) {
  if (memcmp(p->rcMac, info->src_addr, 6U) != 0) { PerLog("PER MAC BAD"); return false; }
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
    if (!s_hasJx || fabsf(p->value - s_lastJx) >= PER_JOY_EPS) { s_lastJx = p->value; s_hasJx = true; PerPrintJoy("JX", p->value); }
  } else if (p->varId == PER_VAR_JY) {
    if (!s_hasJy || fabsf(p->value - s_lastJy) >= PER_JOY_EPS) { s_lastJy = p->value; s_hasJy = true; PerPrintJoy("JY", p->value); }
  }
}

static bool PerHandleStateI(const pen_var_i_payload_t* p) {
  if (p == nullptr) return false;
  if (p->varId == PER_VAR_LSET) { s_stateLset = p->value; return true; }
  if (p->varId == PER_VAR_RSET) { s_stateRset = p->value; return true; }
  if (p->varId == PER_VAR_USBC) { s_stateUsbc = p->value; return true; }
  return true;
}

static bool PerHandleStateF(const pen_var_f_payload_t* p) { (void)p; return true; }

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
        if ((*endPtr == '\0') && (value >= 0) && (value <= 100)) PerSendStateI(PER_VAR_BATP, (int32_t)value);
      }
      continue;
    }
    if (len + 1U < sizeof(line)) line[len++] = (char)ch;
  }
}

static bool PerIsStateOrEvent(uint8_t type) { return (type == MSG_STATE_I_VAR) || (type == MSG_STATE_F_VAR) || (type == MSG_EVENT_I_VAR) || (type == MSG_EVENT_F_VAR); }

static void PerSendAck(uint16_t ackSeq, uint32_t varId) {
  pen_ack_payload_t p = {};
  p.ackSeq = ackSeq;
  p.varId = varId;
  (void)pen_send_frame(s_rcMac, MSG_ACK, s_sessionId, s_seq++, p);
  s_lastTxMs = millis();
}

static void PerSendNack(uint16_t ackSeq, uint32_t varId, uint8_t reason) {
  pen_nack_payload_t p = {};
  p.ackSeq = ackSeq;
  p.varId = varId;
  p.reason = reason;
  (void)pen_send_frame(s_rcMac, MSG_NACK, s_sessionId, s_seq++, p);
  s_lastTxMs = millis();
}

static void OnRecv(const esp_now_recv_info_t* info, const uint8_t* data, int len) {
  if ((info == nullptr) || (info->src_addr == nullptr) || (data == nullptr) || !pen_frame_valid(data, (size_t)len)) return;
  const pen_hdr_t* hdr = reinterpret_cast<const pen_hdr_t*>(data);
  const uint8_t type = pen_msg_base(hdr->msgType);
  const uint8_t* payload = data + sizeof(pen_hdr_t);

  const int8_t rssi = (info->rx_ctrl != nullptr) ? info->rx_ctrl->rssi : -100;

  if (PerSecurePeerOk(info, hdr)) {
    s_lastRxMs = millis();
    s_lastRxRssi = rssi;
    PerCacheStreamI(PER_VAR_RSSI, (int32_t)s_lastRxRssi, 300);
  }

  if (type == MSG_DISCOVERY_REQ) {
    PerSendDiscoveryRsp(info->src_addr);
    PerLedPulse();
  } else if (type == MSG_CONNECT_REQ) {
    const auto* p = reinterpret_cast<const pen_connect_req_payload_t*>(payload);
    if (PerStartNewSession(info, p)) PerSendConnectRsp(info->src_addr);
  } else if ((type == MSG_AUTH_REQ) && ((s_state == PER_CONNECTED) || (s_state == PER_AUTH_RSP_SENT)) && (memcmp(info->src_addr, s_rcMac, 6U) == 0)) {
    const auto* p = reinterpret_cast<const pen_auth_req_payload_t*>(payload);
    if ((hdr->sessionId == s_sessionId) && (memcmp(p->rcProof, s_rcProof, 16U) == 0)) { if (PerSendAuthRsp(info->src_addr)) s_state = PER_AUTH_RSP_SENT; }
    else PerLog("PER AUTH BAD");
  } else if ((type == MSG_ACK) && (memcmp(info->src_addr, s_rcMac, 6U) == 0)) {
    const auto* p = reinterpret_cast<const pen_ack_payload_t*>(payload);
    if ((s_state == PER_AUTH_RSP_SENT) && (p->ackSeq == s_authRspSeq)) PerInstallEncryptedPeer();
  } else if ((type == MSG_NACK) && PerSecurePeerOk(info, hdr)) {
    PerLog("PER NACK");
  } else if ((type == MSG_STREAM_F_VAR) && PerSecurePeerOk(info, hdr)) {
    PerHandleStreamFloat(reinterpret_cast<const pen_var_f_payload_t*>(payload));
  } else if ((type == MSG_STREAM_I_VAR) && PerSecurePeerOk(info, hdr)) {
    const auto* p = reinterpret_cast<const pen_var_i_payload_t*>(payload);
    (void)p;
  } else if (PerIsStateOrEvent(type) && PerSecurePeerOk(info, hdr)) {
    bool handled = false;
    uint32_t varId = 0U;
    if ((type == MSG_STATE_I_VAR) || (type == MSG_EVENT_I_VAR)) {
      const auto* p = reinterpret_cast<const pen_var_i_payload_t*>(payload);
      varId = p->varId;
      handled = (type == MSG_EVENT_I_VAR) ? true : PerHandleStateI(p);
    } else {
      const auto* p = reinterpret_cast<const pen_var_f_payload_t*>(payload);
      varId = p->varId;
      handled = (type == MSG_EVENT_F_VAR) ? true : PerHandleStateF(p);
    }
    if (handled) PerSendAck(hdr->seq, varId);
    else PerSendNack(hdr->seq, varId, PEN_NACK_UNSUPPORTED_VAR);
  }
}

void setup() {
  pinMode(PER_LED_PIN, OUTPUT);
  digitalWrite(PER_LED_PIN, LOW);
  Serial.begin(115200U);
  delay(50U);
  WiFi.mode(WIFI_STA);
  WiFi.disconnect(true, true);
  if (!pen_get_sta_mac(s_ownMac)) { PerLog("PER MAC ERR"); return; }
  s_deviceId = pen_make_id_from_efuse();
  (void)esp_wifi_set_promiscuous(true);
  (void)esp_wifi_set_channel(PEN_CHANNEL, WIFI_SECOND_CHAN_NONE);
  esp_wifi_config_espnow_rate(WIFI_IF_STA, WIFI_PHY_RATE_1M_L);
  (void)esp_wifi_set_promiscuous(false);
  (void)esp_now_init();
  (void)esp_now_register_recv_cb(OnRecv);
  PerLog("PER READY");
}

void loop() {
  PerHandleSerial();
  const uint32_t now = millis();
  if ((s_state == PER_SECURE) && ((now - s_lastRxMs) >= PER_LINK_TIMEOUT_MS)) PerResetToWait("PER LINK LOST");
  if (s_state == PER_SECURE) (void)PerSendStreamHeartbeat(now);
  if ((s_ledOffMs != 0U) && ((int32_t)(now - s_ledOffMs) >= 0)) { digitalWrite(PER_LED_PIN, LOW); s_ledOffMs = 0U; }
  delay(2U);
}
