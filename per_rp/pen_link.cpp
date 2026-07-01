#include "pen_link.h"
#include "pen_proto.h"

#include <WiFi.h>
#include <esp_crc.h>
#include <esp_now.h>
#include <esp_system.h>
#include <esp_wifi.h>
#include <mbedtls/sha256.h>
#include <mbedtls/version.h>
#include <string.h>

#include "freertos/task.h"

namespace {

enum pen_state_t : uint8_t {
#if PEN_RC
  PEN_DISCOVERY,
  PEN_CONNECTING,
#else
  PEN_WAIT_RC,
#endif
  PEN_AUTHING,
  PEN_SECURE_WAIT,
  PEN_SECURE
};

enum pen_evt_t : uint8_t {
  PEN_EVT_NONE = 0U,
  PEN_EVT_DISC_REQ,
  PEN_EVT_DISC_RSP,
  PEN_EVT_CONN_REQ,
  PEN_EVT_CONN_RSP,
  PEN_EVT_AUTH_REQ,
  PEN_EVT_AUTH_RSP,
  PEN_EVT_VAR_I,
  PEN_EVT_VAR_F,
  PEN_EVT_ACK,
  PEN_EVT_NACK,
  PEN_EVT_GET_VAR
};

struct pen_frame_event_t {
  volatile bool used;
  uint8_t type;
  uint8_t msgType;
  uint16_t seq;
  uint32_t sessionId;
  uint8_t mac[6];
  int8_t rssi;
  uint16_t len;
  uint8_t data[64];
};

struct pen_stream_cache_t {
  bool used;
  bool isFloat;
  uint32_t varId;
  uint16_t ttlMs;
  uint32_t lastTxMs;
  union {
    int32_t iValue;
    float fValue;
  } value;
};

struct pen_pending_tx_t {
  bool used;
  bool isFloat;
  uint8_t msgType;
  uint16_t seq;
  uint16_t ttlMs;
  uint32_t varId;
  uint32_t lastTxMs;
  uint8_t retryCount;
  union {
    int32_t iValue;
    float fValue;
  } value;
};

struct pen_rx_recent_t {
  bool used;
  uint8_t msgType;
  uint16_t seq;
  uint32_t varId;
};

static constexpr uint32_t PEN_DISCOVERY_MS = 1000U;
static constexpr uint32_t PEN_RETRY_MS = 500U;
static constexpr uint32_t PEN_AUTH_TIMEOUT_MS = 3000U;
static constexpr uint32_t PEN_SECURE_DELAY_MS = 200U;
static constexpr uint32_t PEN_HEARTBEAT_MS = 150U;
static constexpr uint16_t PEN_HEARTBEAT_TTL_MS = 300U;
static constexpr uint32_t PEN_LINK_TIMEOUT_MS = 500U;
static constexpr uint32_t PEN_ACK_TIMEOUT_MS = 150U;
static constexpr uint8_t PEN_MAX_RETRY = 3U;
static constexpr size_t PEN_FRAME_Q_CAP = 12U;
static constexpr size_t PEN_APP_Q_CAP = 16U;
static constexpr size_t PEN_STREAM_CACHE_CAP = 8U;
static constexpr size_t PEN_PENDING_CAP = 8U;
static constexpr size_t PEN_RX_RECENT_CAP = 8U;

static TaskHandle_t s_mainTask = nullptr;
static TaskHandle_t s_rxTask = nullptr;
static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;
static pen_frame_event_t s_frameQ[PEN_FRAME_Q_CAP];
static pen_rx_event_t s_appQ[PEN_APP_Q_CAP];
static volatile uint8_t s_appHead = 0U;
static volatile uint8_t s_appTail = 0U;
static pen_stream_cache_t s_streams[PEN_STREAM_CACHE_CAP];
static pen_pending_tx_t s_pending[PEN_PENDING_CAP];
static pen_rx_recent_t s_recent[PEN_RX_RECENT_CAP];
static pen_rx_event_fn_t s_rxEvent = nullptr;

#if PEN_RC
static pen_state_t s_state = PEN_DISCOVERY;
#else
static pen_state_t s_state = PEN_WAIT_RC;
#endif

static uint8_t s_peerMac[6] = {};
static bool s_peerValid = false;
static uint8_t s_ownMac[6] = {};
static uint8_t s_rcMac[6] = {};
static uint8_t s_devMac[6] = {};
static uint32_t s_rcId = 0U;
static uint32_t s_deviceId = 0U;
static uint32_t s_sessionId = 0U;
static uint16_t s_seq = 1U;
static uint32_t s_hbCounter = 0U;
static uint8_t s_rcNonce[16] = {};
static uint8_t s_devNonce[16] = {};
static uint8_t s_sessionKey[32] = {};
static uint8_t s_pmk[16] = {};
static uint8_t s_lmk[16] = {};
static uint8_t s_rcProof[16] = {};
static uint8_t s_devProof[16] = {};
static uint32_t s_lastTxMs = 0U;
static uint32_t s_lastRxMs = 0U;
static uint32_t s_stateStartMs = 0U;
static uint32_t s_secureInstallAtMs = 0U;
static volatile uint32_t s_frameDropCount = 0U;
static volatile uint32_t s_badFrameCount = 0U;
static volatile uint32_t s_appDropCount = 0U;
static uint32_t s_reportedFrameDropCount = 0U;
static uint32_t s_reportedBadFrameCount = 0U;
static uint32_t s_reportedAppDropCount = 0U;

static uint8_t MsgBase(uint8_t rawMsgType) { return rawMsgType & PEN_MSG_TYPE_MASK; }
static bool MsgIsRetry(uint8_t rawMsgType) { return (rawMsgType & PEN_MSG_FLAG_RETRY) != 0U; }
static uint8_t MsgMake(uint8_t msgType, bool retry) { return (uint8_t)((msgType & PEN_MSG_TYPE_MASK) | (retry ? PEN_MSG_FLAG_RETRY : 0U)); }

static uint32_t MakeIdFromEfuse(void) {
  return (uint32_t)(ESP.getEfuseMac() & 0xFFFFFFFFULL);
}

static bool GetStaMac(uint8_t outMac[6]) {
  if (outMac == nullptr) return false;
  return esp_wifi_get_mac(WIFI_IF_STA, outMac) == ESP_OK;
}

static uint16_t Crc16(const void* data, size_t len) {
  if (data == nullptr) return 0U;
  return esp_crc16_le(PEN_CRC_INIT, reinterpret_cast<const uint8_t*>(data), (uint32_t)len);
}

static uint16_t ReadLe16(const uint8_t* ptr) {
  return (uint16_t)ptr[0] | ((uint16_t)ptr[1] << 8);
}

static void WriteLe16(uint8_t* ptr, uint16_t value) {
  ptr[0] = (uint8_t)(value & 0xFFU);
  ptr[1] = (uint8_t)((value >> 8) & 0xFFU);
}

static size_t PayloadSize(uint8_t rawMsgType) {
  switch (MsgBase(rawMsgType)) {
    case MSG_DISCOVERY_REQ: return 0U;
    case MSG_DISCOVERY_RSP: return sizeof(pen_discovery_rsp_payload_t);
    case MSG_CONNECT_REQ: return sizeof(pen_connect_req_payload_t);
    case MSG_CONNECT_RSP: return sizeof(pen_connect_rsp_payload_t);
    case MSG_AUTH_REQ: return sizeof(pen_auth_req_payload_t);
    case MSG_AUTH_RSP: return sizeof(pen_auth_rsp_payload_t);
    case MSG_ACK: return sizeof(pen_ack_payload_t);
    case MSG_NACK: return sizeof(pen_nack_payload_t);
    case MSG_GET_VAR: return sizeof(pen_get_var_payload_t);
    case MSG_STREAM_I_VAR:
    case MSG_STATE_I_VAR:
    case MSG_EVENT_I_VAR: return sizeof(pen_var_i_payload_t);
    case MSG_STREAM_F_VAR:
    case MSG_STATE_F_VAR:
    case MSG_EVENT_F_VAR: return sizeof(pen_var_f_payload_t);
    default: return SIZE_MAX;
  }
}

static size_t FrameSize(uint8_t rawMsgType) {
  const size_t ps = PayloadSize(rawMsgType);
  return (ps == SIZE_MAX) ? SIZE_MAX : sizeof(pen_hdr_t) + ps + sizeof(pen_crc_t);
}

static bool LenValid(uint8_t rawMsgType, size_t len) {
  const size_t fs = FrameSize(rawMsgType);
  return (fs != SIZE_MAX) && (len == fs);
}

static bool CrcValid(const void* frame, size_t len) {
  if ((frame == nullptr) || (len < (sizeof(pen_hdr_t) + sizeof(pen_crc_t)))) return false;
  const uint8_t* b = reinterpret_cast<const uint8_t*>(frame);
  const size_t dl = len - sizeof(pen_crc_t);
  return ReadLe16(&b[dl]) == Crc16(frame, dl);
}

static bool FrameValid(const void* frame, size_t len) {
  if ((frame == nullptr) || (len < (sizeof(pen_hdr_t) + sizeof(pen_crc_t)))) return false;
  const pen_hdr_t* h = reinterpret_cast<const pen_hdr_t*>(frame);
  return (h->magic == PEN_MAGIC) && LenValid(h->msgType, len) && CrcValid(frame, len);
}

static bool FinalizeFrame(void* frame, size_t len) {
  if ((frame == nullptr) || (len < (sizeof(pen_hdr_t) + sizeof(pen_crc_t)))) return false;
  pen_hdr_t* h = reinterpret_cast<pen_hdr_t*>(frame);
  if (!LenValid(h->msgType, len)) return false;
  const size_t dl = len - sizeof(pen_crc_t);
  uint8_t* b = reinterpret_cast<uint8_t*>(frame);
  WriteLe16(&b[dl], Crc16(frame, dl));
  return true;
}

static bool AddPeer(const uint8_t mac[6], bool encrypt, const uint8_t lmk[16]) {
  if (mac == nullptr) return false;
  if (esp_now_is_peer_exist(mac)) (void)esp_now_del_peer(mac);
  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, mac, 6U);
  peer.channel = PEN_CHANNEL;
  peer.ifidx = WIFI_IF_STA;
  peer.encrypt = encrypt;
  if (encrypt && (lmk != nullptr)) memcpy(peer.lmk, lmk, 16U);
  return esp_now_add_peer(&peer) == ESP_OK;
}

template <typename PayloadT>
static bool SendFrame(const uint8_t mac[6], uint8_t msgType, uint32_t sessionId, uint16_t seq, const PayloadT& payload) {
  struct __attribute__((packed)) frame_t {
    pen_hdr_t hdr;
    PayloadT payload;
    pen_crc_t crc;
  } frame = {};
  frame.hdr.magic = PEN_MAGIC;
  frame.hdr.msgType = msgType;
  frame.hdr.sessionId = sessionId;
  frame.hdr.seq = seq;
  frame.payload = payload;
  if (!FinalizeFrame(&frame, sizeof(frame))) return false;
  return esp_now_send(mac, reinterpret_cast<const uint8_t*>(&frame), sizeof(frame)) == ESP_OK;
}

static bool SendEmptyFrame(const uint8_t mac[6], uint8_t msgType, uint32_t sessionId, uint16_t seq) {
  struct __attribute__((packed)) frame_t {
    pen_hdr_t hdr;
    pen_crc_t crc;
  } frame = {};
  frame.hdr.magic = PEN_MAGIC;
  frame.hdr.msgType = msgType;
  frame.hdr.sessionId = sessionId;
  frame.hdr.seq = seq;
  if (!FinalizeFrame(&frame, sizeof(frame))) return false;
  return esp_now_send(mac, reinterpret_cast<const uint8_t*>(&frame), sizeof(frame)) == ESP_OK;
}

static void Sha256Start(mbedtls_sha256_context* ctx) {
  mbedtls_sha256_init(ctx);
#if defined(MBEDTLS_VERSION_NUMBER) && (MBEDTLS_VERSION_NUMBER >= 0x03000000)
  (void)mbedtls_sha256_starts(ctx, 0);
#else
  (void)mbedtls_sha256_starts_ret(ctx, 0);
#endif
}

static void Sha256Update(mbedtls_sha256_context* ctx, const void* data, size_t len) {
#if defined(MBEDTLS_VERSION_NUMBER) && (MBEDTLS_VERSION_NUMBER >= 0x03000000)
  (void)mbedtls_sha256_update(ctx, reinterpret_cast<const unsigned char*>(data), len);
#else
  (void)mbedtls_sha256_update_ret(ctx, reinterpret_cast<const unsigned char*>(data), len);
#endif
}

static void Sha256Finish(mbedtls_sha256_context* ctx, uint8_t out[32]) {
#if defined(MBEDTLS_VERSION_NUMBER) && (MBEDTLS_VERSION_NUMBER >= 0x03000000)
  (void)mbedtls_sha256_finish(ctx, out);
#else
  (void)mbedtls_sha256_finish_ret(ctx, out);
#endif
  mbedtls_sha256_free(ctx);
}

static void Sha256Label(const char* label, const uint8_t sessionKey[32], uint8_t out[32]) {
  mbedtls_sha256_context ctx;
  Sha256Start(&ctx);
  Sha256Update(&ctx, sessionKey, 32U);
  Sha256Update(&ctx, label, strlen(label));
  Sha256Finish(&ctx, out);
}

static void DeriveSession(uint32_t rcId, uint32_t deviceId, uint32_t sessionId, const uint8_t rcMac[6], const uint8_t devMac[6], const uint8_t rcNonce[16], const uint8_t devNonce[16], uint8_t sessionKey[32], uint8_t pmk[16], uint8_t lmk[16], uint8_t rcProof[16], uint8_t devProof[16]) {
  mbedtls_sha256_context ctx;
  uint8_t tmp[32];
  Sha256Start(&ctx);
  Sha256Update(&ctx, PEN_BIND_SECRET, strlen(PEN_BIND_SECRET));
  Sha256Update(&ctx, &rcId, sizeof(rcId));
  Sha256Update(&ctx, &deviceId, sizeof(deviceId));
  Sha256Update(&ctx, &sessionId, sizeof(sessionId));
  Sha256Update(&ctx, rcMac, 6U);
  Sha256Update(&ctx, devMac, 6U);
  Sha256Update(&ctx, rcNonce, 16U);
  Sha256Update(&ctx, devNonce, 16U);
  Sha256Finish(&ctx, sessionKey);
  Sha256Label("PMK", sessionKey, tmp); memcpy(pmk, tmp, 16U);
  Sha256Label("LMK", sessionKey, tmp); memcpy(lmk, tmp, 16U);
  Sha256Label("RC", sessionKey, tmp); memcpy(rcProof, tmp, 16U);
  Sha256Label("DEV", sessionKey, tmp); memcpy(devProof, tmp, 16U);
}

static bool IsStateMsg(uint8_t rawMsgType) {
  const uint8_t t = MsgBase(rawMsgType);
  return (t == MSG_STATE_I_VAR) || (t == MSG_STATE_F_VAR);
}

static bool IsEventMsg(uint8_t rawMsgType) {
  const uint8_t t = MsgBase(rawMsgType);
  return (t == MSG_EVENT_I_VAR) || (t == MSG_EVENT_F_VAR);
}

static bool RequiresAck(uint8_t rawMsgType) {
  return IsStateMsg(rawMsgType) || IsEventMsg(rawMsgType);
}

static bool IsSecureMsg(uint8_t rawMsgType) {
  const uint8_t t = MsgBase(rawMsgType);
  return (t == MSG_STREAM_I_VAR) || (t == MSG_STREAM_F_VAR) ||
         (t == MSG_STATE_I_VAR) || (t == MSG_STATE_F_VAR) ||
         (t == MSG_EVENT_I_VAR) || (t == MSG_EVENT_F_VAR) ||
         (t == MSG_ACK) || (t == MSG_NACK) || (t == MSG_GET_VAR);
}

static bool MacIsSet(const uint8_t mac[6]) {
  static const uint8_t zero[6] = {};
  return (mac != nullptr) && (memcmp(mac, zero, 6U) != 0);
}

static const uint8_t* Payload(const pen_frame_event_t& ev) {
  return ev.data + sizeof(pen_hdr_t);
}

static bool QueueApp(const pen_rx_event_t& ev) {
  bool ok = false;
  portENTER_CRITICAL(&s_mux);
  const uint8_t next = (uint8_t)((s_appHead + 1U) % PEN_APP_Q_CAP);
  if (next != s_appTail) {
    s_appQ[s_appHead] = ev;
    s_appHead = next;
    ok = true;
  } else {
    ++s_appDropCount;
  }
  portEXIT_CRITICAL(&s_mux);
  return ok;
}

static bool EmitLink(uint8_t code, int8_t rssi = -100) {
  pen_rx_event_t ev = {};
  ev.type = PEN_RX_LINK;
  ev.data.link.code = code;
  ev.data.link.rssi = rssi;
  return QueueApp(ev);
}

static void EmitError(uint8_t code, int32_t detail) {
  pen_rx_event_t ev = {};
  ev.type = PEN_RX_ERROR;
  ev.data.error.code = code;
  ev.data.error.detail = detail;
  (void)QueueApp(ev);
}

static void ClearReliable(void) {
  memset(s_pending, 0, sizeof(s_pending));
  memset(s_recent, 0, sizeof(s_recent));
}

static void ClearSession(void) {
  s_peerValid = false;
  memset(s_peerMac, 0, sizeof(s_peerMac));
  memset(s_rcMac, 0, sizeof(s_rcMac));
  memset(s_devMac, 0, sizeof(s_devMac));
  s_sessionId = 0U;
  memset(s_rcNonce, 0, sizeof(s_rcNonce));
  memset(s_devNonce, 0, sizeof(s_devNonce));
  memset(s_sessionKey, 0, sizeof(s_sessionKey));
  memset(s_pmk, 0, sizeof(s_pmk));
  memset(s_lmk, 0, sizeof(s_lmk));
  memset(s_rcProof, 0, sizeof(s_rcProof));
  memset(s_devProof, 0, sizeof(s_devProof));
  memset(s_streams, 0, sizeof(s_streams));
  ClearReliable();
}

static void ResetLink(uint8_t reasonCode) {
  if (s_peerValid && MacIsSet(s_peerMac) && esp_now_is_peer_exist(s_peerMac)) (void)esp_now_del_peer(s_peerMac);
  ClearSession();
#if PEN_RC
  s_state = PEN_DISCOVERY;
#else
  s_state = PEN_WAIT_RC;
#endif
  s_lastTxMs = 0U;
  s_lastRxMs = 0U;
  s_stateStartMs = millis();
  s_secureInstallAtMs = 0U;
  if (reasonCode != 0U) (void)EmitLink(reasonCode);
}

static bool SecureEventOk(const pen_frame_event_t& ev) {
  return (s_state == PEN_SECURE) && s_peerValid && (ev.sessionId == s_sessionId) && (memcmp(ev.mac, s_peerMac, 6U) == 0);
}

static void QueueFrame(const uint8_t* mac, int8_t rssi, const uint8_t* data, size_t len) {
  if ((mac == nullptr) || (data == nullptr) || (len > sizeof(s_frameQ[0].data))) {
    portENTER_CRITICAL(&s_mux);
    ++s_frameDropCount;
    portEXIT_CRITICAL(&s_mux);
    return;
  }
  const pen_hdr_t* h = reinterpret_cast<const pen_hdr_t*>(data);
  portENTER_CRITICAL(&s_mux);
  for (size_t i = 0; i < PEN_FRAME_Q_CAP; ++i) {
    if (!s_frameQ[i].used) {
      s_frameQ[i].used = true;
      s_frameQ[i].type = PEN_EVT_NONE;
      s_frameQ[i].msgType = h->msgType;
      s_frameQ[i].seq = h->seq;
      s_frameQ[i].sessionId = h->sessionId;
      s_frameQ[i].len = (uint16_t)len;
      memcpy((void*)s_frameQ[i].mac, mac, 6U);
      s_frameQ[i].rssi = rssi;
      memcpy((void*)s_frameQ[i].data, data, len);
      portEXIT_CRITICAL(&s_mux);
      return;
    }
  }
  ++s_frameDropCount;
  portEXIT_CRITICAL(&s_mux);
}

static void OnRecv(const esp_now_recv_info_t* info, const uint8_t* data, int len) {
  if ((info == nullptr) || (info->src_addr == nullptr) || (data == nullptr) || (len <= 0) || !FrameValid(data, (size_t)len)) {
    portENTER_CRITICAL(&s_mux);
    ++s_badFrameCount;
    portEXIT_CRITICAL(&s_mux);
    return;
  }
  const pen_hdr_t* h = reinterpret_cast<const pen_hdr_t*>(data);
  if ((s_state == PEN_SECURE) && s_peerValid && IsSecureMsg(h->msgType) && (h->sessionId == s_sessionId) && (memcmp(info->src_addr, s_peerMac, 6U) == 0)) {
    s_lastRxMs = millis();
  }
  const int8_t rssi = (info->rx_ctrl != nullptr) ? info->rx_ctrl->rssi : -100;
  QueueFrame(info->src_addr, rssi, data, (size_t)len);
}

static bool DecodeFrame(pen_frame_event_t& ev) {
  if (!FrameValid(ev.data, ev.len)) return false;
  const pen_hdr_t* h = reinterpret_cast<const pen_hdr_t*>(ev.data);
  const uint8_t t = MsgBase(h->msgType);
  ev.msgType = h->msgType;
  ev.seq = h->seq;
  ev.sessionId = h->sessionId;
#if PEN_RC
  if (t == MSG_DISCOVERY_RSP) ev.type = PEN_EVT_DISC_RSP;
  else if (t == MSG_CONNECT_RSP) ev.type = PEN_EVT_CONN_RSP;
  else if (t == MSG_AUTH_RSP) ev.type = PEN_EVT_AUTH_RSP;
#else
  if (t == MSG_DISCOVERY_REQ) ev.type = PEN_EVT_DISC_REQ;
  else if (t == MSG_CONNECT_REQ) ev.type = PEN_EVT_CONN_REQ;
  else if (t == MSG_AUTH_REQ) ev.type = PEN_EVT_AUTH_REQ;
#endif
  else if ((t == MSG_STREAM_I_VAR) || (t == MSG_STATE_I_VAR) || (t == MSG_EVENT_I_VAR)) ev.type = PEN_EVT_VAR_I;
  else if ((t == MSG_STREAM_F_VAR) || (t == MSG_STATE_F_VAR) || (t == MSG_EVENT_F_VAR)) ev.type = PEN_EVT_VAR_F;
  else if (t == MSG_ACK) ev.type = PEN_EVT_ACK;
  else if (t == MSG_NACK) ev.type = PEN_EVT_NACK;
  else if (t == MSG_GET_VAR) ev.type = PEN_EVT_GET_VAR;
  else return false;
  return true;
}

static bool SendAckTo(const uint8_t mac[6], uint16_t ackSeq, uint32_t varId) {
  pen_ack_payload_t p = {};
  p.ackSeq = ackSeq;
  p.varId = varId;
  const bool ok = SendFrame(mac, MSG_ACK, s_sessionId, s_seq++, p);
  if (ok) s_lastTxMs = millis();
  else EmitError(PEN_HW_ERR_SEND, MSG_ACK);
  return ok;
}

static bool SendNackTo(const uint8_t mac[6], uint16_t ackSeq, uint32_t varId, uint8_t reason) {
  pen_nack_payload_t p = {};
  p.ackSeq = ackSeq;
  p.varId = varId;
  p.reason = reason;
  const bool ok = SendFrame(mac, MSG_NACK, s_sessionId, s_seq++, p);
  if (ok) s_lastTxMs = millis();
  else EmitError(PEN_HW_ERR_SEND, MSG_NACK);
  return ok;
}

#if PEN_RC
static bool SendDiscovery(void) {
  (void)AddPeer(PEN_BROADCAST_MAC, false, nullptr);
  const bool ok = SendEmptyFrame(PEN_BROADCAST_MAC, MSG_DISCOVERY_REQ, 0U, s_seq++);
  if (!ok) EmitError(PEN_HW_ERR_SEND, MSG_DISCOVERY_REQ);
  return ok;
}

static bool SendConnect(void) {
  pen_connect_req_payload_t p = {};
  p.rcId = s_rcId;
  p.caps = PEN_CAPS;
  memcpy(p.rcMac, s_ownMac, sizeof(p.rcMac));
  memcpy(p.rcNonce, s_rcNonce, sizeof(p.rcNonce));
  if (!AddPeer(s_peerMac, false, nullptr)) {
    EmitError(PEN_HW_ERR_ESPNOW, MSG_CONNECT_REQ);
    return false;
  }
  const bool ok = SendFrame(s_peerMac, MSG_CONNECT_REQ, 0U, s_seq++, p);
  if (!ok) EmitError(PEN_HW_ERR_SEND, MSG_CONNECT_REQ);
  return ok;
}

static bool SendAuth(void) {
  pen_auth_req_payload_t p = {};
  memcpy(p.rcProof, s_rcProof, sizeof(p.rcProof));
  const bool ok = SendFrame(s_peerMac, MSG_AUTH_REQ, s_sessionId, s_seq++, p);
  if (!ok) EmitError(PEN_HW_ERR_SEND, MSG_AUTH_REQ);
  return ok;
}
#else
static bool SendDiscoveryResponse(const uint8_t mac[6]) {
  pen_discovery_rsp_payload_t p = {};
  p.deviceId = s_deviceId;
  p.caps = PEN_CAPS;
  p.workChannel = PEN_CHANNEL;
  memcpy(p.name, "PEN-RP", 6U);
  (void)AddPeer(mac, false, nullptr);
  const bool ok = SendFrame(mac, MSG_DISCOVERY_RSP, 0U, s_seq++, p);
  if (!ok) EmitError(PEN_HW_ERR_SEND, MSG_DISCOVERY_RSP);
  return ok;
}

static bool SendConnectResponse(void) {
  pen_connect_rsp_payload_t p = {};
  p.deviceId = s_deviceId;
  p.caps = PEN_CAPS;
  p.sessionId = s_sessionId;
  memcpy(p.devMac, s_ownMac, sizeof(p.devMac));
  memcpy(p.devNonce, s_devNonce, sizeof(p.devNonce));
  const bool ok = SendFrame(s_peerMac, MSG_CONNECT_RSP, 0U, s_seq++, p);
  if (!ok) EmitError(PEN_HW_ERR_SEND, MSG_CONNECT_RSP);
  return ok;
}

static bool SendAuthResponse(void) {
  pen_auth_rsp_payload_t p = {};
  memcpy(p.devProof, s_devProof, sizeof(p.devProof));
  const bool ok = SendFrame(s_peerMac, MSG_AUTH_RSP, s_sessionId, s_seq++, p);
  if (!ok) EmitError(PEN_HW_ERR_SEND, MSG_AUTH_RSP);
  return ok;
}
#endif

static bool InstallEncryptedPeer(void) {
  if (esp_now_set_pmk(s_pmk) != ESP_OK) {
    EmitError(PEN_HW_ERR_ESPNOW, 100);
    return false;
  }
  if (!AddPeer(s_peerMac, true, s_lmk)) {
    EmitError(PEN_HW_ERR_ESPNOW, 101);
    return false;
  }
  const uint32_t now = millis();
  s_state = PEN_SECURE;
  s_lastRxMs = now;
  s_lastTxMs = now;
  s_secureInstallAtMs = 0U;
  s_hbCounter = 0U;
  ClearReliable();
  (void)EmitLink(PEN_LINK_SECURE);
  return true;
}

static void CacheStreamI(uint32_t varId, int32_t value, uint16_t ttlMs) {
  if (varId == PEN_VAR_HB) return;
  portENTER_CRITICAL(&s_mux);
  size_t slot = PEN_STREAM_CACHE_CAP;
  for (size_t i = 0; i < PEN_STREAM_CACHE_CAP; ++i) {
    if (s_streams[i].used && !s_streams[i].isFloat && (s_streams[i].varId == varId)) { slot = i; break; }
    if (!s_streams[i].used && (slot == PEN_STREAM_CACHE_CAP)) slot = i;
  }
  if (slot < PEN_STREAM_CACHE_CAP) {
    s_streams[slot].used = true;
    s_streams[slot].isFloat = false;
    s_streams[slot].varId = varId;
    s_streams[slot].ttlMs = ttlMs;
    s_streams[slot].lastTxMs = millis();
    s_streams[slot].value.iValue = value;
  }
  portEXIT_CRITICAL(&s_mux);
}

static void CacheStreamF(uint32_t varId, float value, uint16_t ttlMs) {
  if (varId == PEN_VAR_HB) return;
  portENTER_CRITICAL(&s_mux);
  size_t slot = PEN_STREAM_CACHE_CAP;
  for (size_t i = 0; i < PEN_STREAM_CACHE_CAP; ++i) {
    if (s_streams[i].used && s_streams[i].isFloat && (s_streams[i].varId == varId)) { slot = i; break; }
    if (!s_streams[i].used && (slot == PEN_STREAM_CACHE_CAP)) slot = i;
  }
  if (slot < PEN_STREAM_CACHE_CAP) {
    s_streams[slot].used = true;
    s_streams[slot].isFloat = true;
    s_streams[slot].varId = varId;
    s_streams[slot].ttlMs = ttlMs;
    s_streams[slot].lastTxMs = millis();
    s_streams[slot].value.fValue = value;
  }
  portEXIT_CRITICAL(&s_mux);
}

static bool SendStreamIRaw(uint32_t varId, int32_t value, uint16_t ttlMs) {
  pen_var_i_payload_t p = {};
  p.varId = varId;
  p.ttlMs = ttlMs;
  p.value = value;
  const bool ok = SendFrame(s_peerMac, MSG_STREAM_I_VAR, s_sessionId, s_seq++, p);
  if (ok) s_lastTxMs = millis();
  else EmitError(PEN_HW_ERR_SEND, MSG_STREAM_I_VAR);
  return ok;
}

static bool SendStreamFRaw(uint32_t varId, float value, uint16_t ttlMs) {
  pen_var_f_payload_t p = {};
  p.varId = varId;
  p.ttlMs = ttlMs;
  p.value = value;
  const bool ok = SendFrame(s_peerMac, MSG_STREAM_F_VAR, s_sessionId, s_seq++, p);
  if (ok) s_lastTxMs = millis();
  else EmitError(PEN_HW_ERR_SEND, MSG_STREAM_F_VAR);
  return ok;
}

static bool SendHeartbeat(uint32_t now) {
  int expired = -1;
  int oldest = -1;
  uint32_t expiredAge = 0U;
  uint32_t oldestAge = 0U;
  portENTER_CRITICAL(&s_mux);
  for (size_t i = 0; i < PEN_STREAM_CACHE_CAP; ++i) {
    if (!s_streams[i].used) continue;
    const uint32_t age = now - s_streams[i].lastTxMs;
    if ((oldest < 0) || (age > oldestAge)) { oldest = (int)i; oldestAge = age; }
    if ((s_streams[i].ttlMs != 0U) && (age >= s_streams[i].ttlMs) && ((expired < 0) || (age > expiredAge))) { expired = (int)i; expiredAge = age; }
  }
  const int pick = (expired >= 0) ? expired : (((now - s_lastTxMs) >= PEN_HEARTBEAT_MS) ? oldest : -1);
  pen_stream_cache_t item = {};
  if (pick >= 0) {
    item = s_streams[pick];
    s_streams[pick].lastTxMs = now;
  }
  portEXIT_CRITICAL(&s_mux);
  if (pick >= 0) return item.isFloat ? SendStreamFRaw(item.varId, item.value.fValue, item.ttlMs) : SendStreamIRaw(item.varId, item.value.iValue, item.ttlMs);
  if ((oldest < 0) && ((now - s_lastTxMs) >= PEN_HEARTBEAT_MS)) return SendStreamIRaw(PEN_VAR_HB, (int32_t)(++s_hbCounter), PEN_HEARTBEAT_TTL_MS);
  return true;
}

static bool PendingAddI(uint8_t msgType, uint16_t seq, uint32_t varId, int32_t value, uint16_t ttlMs, uint32_t now) {
  if (!RequiresAck(msgType)) return true;
  portENTER_CRITICAL(&s_mux);
  for (size_t i = 0; i < PEN_PENDING_CAP; ++i) {
    if (!s_pending[i].used) {
      s_pending[i].used = true;
      s_pending[i].isFloat = false;
      s_pending[i].msgType = MsgBase(msgType);
      s_pending[i].seq = seq;
      s_pending[i].ttlMs = ttlMs;
      s_pending[i].varId = varId;
      s_pending[i].lastTxMs = now;
      s_pending[i].retryCount = 0U;
      s_pending[i].value.iValue = value;
      portEXIT_CRITICAL(&s_mux);
      return true;
    }
  }
  portEXIT_CRITICAL(&s_mux);
  return false;
}

static bool PendingAddF(uint8_t msgType, uint16_t seq, uint32_t varId, float value, uint16_t ttlMs, uint32_t now) {
  if (!RequiresAck(msgType)) return true;
  portENTER_CRITICAL(&s_mux);
  for (size_t i = 0; i < PEN_PENDING_CAP; ++i) {
    if (!s_pending[i].used) {
      s_pending[i].used = true;
      s_pending[i].isFloat = true;
      s_pending[i].msgType = MsgBase(msgType);
      s_pending[i].seq = seq;
      s_pending[i].ttlMs = ttlMs;
      s_pending[i].varId = varId;
      s_pending[i].lastTxMs = now;
      s_pending[i].retryCount = 0U;
      s_pending[i].value.fValue = value;
      portEXIT_CRITICAL(&s_mux);
      return true;
    }
  }
  portEXIT_CRITICAL(&s_mux);
  return false;
}

static bool PendingRemove(uint16_t seq) {
  portENTER_CRITICAL(&s_mux);
  for (size_t i = 0; i < PEN_PENDING_CAP; ++i) {
    if (s_pending[i].used && (s_pending[i].seq == seq)) {
      memset(&s_pending[i], 0, sizeof(s_pending[i]));
      portEXIT_CRITICAL(&s_mux);
      return true;
    }
  }
  portEXIT_CRITICAL(&s_mux);
  return false;
}

static bool ResendPending(const pen_pending_tx_t& item, uint32_t now) {
  const uint8_t retryMsgType = MsgMake(item.msgType, true);
  bool ok;
  if (item.isFloat) {
    pen_var_f_payload_t p = {};
    p.varId = item.varId;
    p.ttlMs = item.ttlMs;
    p.value = item.value.fValue;
    ok = SendFrame(s_peerMac, retryMsgType, s_sessionId, item.seq, p);
  } else {
    pen_var_i_payload_t p = {};
    p.varId = item.varId;
    p.ttlMs = item.ttlMs;
    p.value = item.value.iValue;
    ok = SendFrame(s_peerMac, retryMsgType, s_sessionId, item.seq, p);
  }
  if (ok) s_lastTxMs = now;
  else EmitError(PEN_HW_ERR_SEND, item.msgType);
  return ok;
}

static void ProcessPending(uint32_t now) {
  for (size_t i = 0; i < PEN_PENDING_CAP; ++i) {
    pen_pending_tx_t item = {};
    bool resend = false;
    bool timeout = false;
    portENTER_CRITICAL(&s_mux);
    if (s_pending[i].used && ((now - s_pending[i].lastTxMs) >= PEN_ACK_TIMEOUT_MS)) {
      if (s_pending[i].retryCount >= PEN_MAX_RETRY) {
        item = s_pending[i];
        memset(&s_pending[i], 0, sizeof(s_pending[i]));
        timeout = true;
      } else {
        ++s_pending[i].retryCount;
        s_pending[i].lastTxMs = now;
        item = s_pending[i];
        resend = true;
      }
    }
    portEXIT_CRITICAL(&s_mux);
    if (timeout) EmitError(PEN_HW_ERR_ACK_TIMEOUT, (int32_t)item.seq);
    else if (resend) (void)ResendPending(item, now);
  }
}

static bool RecentSeen(uint8_t msgType, uint16_t seq, uint32_t varId) {
  const uint8_t base = MsgBase(msgType);
  for (size_t i = 0; i < PEN_RX_RECENT_CAP; ++i) {
    if (s_recent[i].used && (s_recent[i].msgType == base) && (s_recent[i].seq == seq) && (s_recent[i].varId == varId)) return true;
  }
  return false;
}

static void RecentRemember(uint8_t msgType, uint16_t seq, uint32_t varId) {
  for (size_t i = PEN_RX_RECENT_CAP - 1U; i > 0U; --i) s_recent[i] = s_recent[i - 1U];
  s_recent[0].used = true;
  s_recent[0].msgType = MsgBase(msgType);
  s_recent[0].seq = seq;
  s_recent[0].varId = varId;
}

static bool SendVarI(uint8_t msgType, uint32_t varId, int32_t value, uint16_t ttlMs) {
  if (!pen_is_connected()) return false;
  const uint16_t seq = s_seq++;
  const uint32_t now = millis();
  if (!PendingAddI(msgType, seq, varId, value, ttlMs, now)) {
    EmitError(PEN_HW_ERR_RETRY_FULL, msgType);
    return false;
  }
  pen_var_i_payload_t p = {};
  p.varId = varId;
  p.ttlMs = ttlMs;
  p.value = value;
  const bool ok = SendFrame(s_peerMac, MsgBase(msgType), s_sessionId, seq, p);
  if (ok) {
    s_lastTxMs = now;
    if (MsgBase(msgType) == MSG_STREAM_I_VAR) CacheStreamI(varId, value, ttlMs);
  } else {
    if (RequiresAck(msgType)) (void)PendingRemove(seq);
    EmitError(PEN_HW_ERR_SEND, msgType);
  }
  return ok;
}

static bool SendVarF(uint8_t msgType, uint32_t varId, float value, uint16_t ttlMs) {
  if (!pen_is_connected()) return false;
  const uint16_t seq = s_seq++;
  const uint32_t now = millis();
  if (!PendingAddF(msgType, seq, varId, value, ttlMs, now)) {
    EmitError(PEN_HW_ERR_RETRY_FULL, msgType);
    return false;
  }
  pen_var_f_payload_t p = {};
  p.varId = varId;
  p.ttlMs = ttlMs;
  p.value = value;
  const bool ok = SendFrame(s_peerMac, MsgBase(msgType), s_sessionId, seq, p);
  if (ok) {
    s_lastTxMs = now;
    if (MsgBase(msgType) == MSG_STREAM_F_VAR) CacheStreamF(varId, value, ttlMs);
  } else {
    if (RequiresAck(msgType)) (void)PendingRemove(seq);
    EmitError(PEN_HW_ERR_SEND, msgType);
  }
  return ok;
}

static bool EmitVarI(uint8_t msgType, uint16_t seq, uint32_t varId, int32_t value, uint16_t ttlMs) {
  pen_rx_event_t out = {};
  out.type = PEN_RX_VAR_I;
  out.msgType = MsgBase(msgType);
  out.data.varI.seq = seq;
  out.data.varI.ttlMs = ttlMs;
  out.data.varI.varId = varId;
  out.data.varI.value = value;
  return QueueApp(out);
}

static bool EmitVarF(uint8_t msgType, uint16_t seq, uint32_t varId, float value, uint16_t ttlMs) {
  pen_rx_event_t out = {};
  out.type = PEN_RX_VAR_F;
  out.msgType = MsgBase(msgType);
  out.data.varF.seq = seq;
  out.data.varF.ttlMs = ttlMs;
  out.data.varF.varId = varId;
  out.data.varF.value = value;
  return QueueApp(out);
}

static bool QueueRxVarI(const pen_var_i_payload_t* p, const pen_frame_event_t& ev) {
  if (p == nullptr) return false;
  if (p->varId == PEN_VAR_HB) return true;
  if (!EmitVarI(ev.msgType, ev.seq, p->varId, p->value, p->ttlMs)) return false;
#if PEN_RC
  if (p->varId == PEN_VAR_RSSI) return EmitVarI(MSG_STREAM_I_VAR, ev.seq, PEN_VAR_RSSL, (int32_t)ev.rssi, p->ttlMs);
#endif
  return true;
}

static bool QueueRxVarF(const pen_var_f_payload_t* p, const pen_frame_event_t& ev) {
  if (p == nullptr) return false;
  return EmitVarF(ev.msgType, ev.seq, p->varId, p->value, p->ttlMs);
}

static void HandleVarI(const pen_frame_event_t& ev) {
  if (!SecureEventOk(ev)) return;
  const auto* p = reinterpret_cast<const pen_var_i_payload_t*>(Payload(ev));
  const bool reliable = RequiresAck(ev.msgType);
  if (reliable && MsgIsRetry(ev.msgType) && RecentSeen(ev.msgType, ev.seq, p->varId)) {
    (void)SendAckTo(ev.mac, ev.seq, p->varId);
    return;
  }
  const bool handled = QueueRxVarI(p, ev);
  if (reliable) {
    if (handled) {
      RecentRemember(ev.msgType, ev.seq, p->varId);
      (void)SendAckTo(ev.mac, ev.seq, p->varId);
    } else {
      (void)SendNackTo(ev.mac, ev.seq, p->varId, PEN_NACK_UNSUPPORTED_VAR);
    }
  }
}

static void HandleVarF(const pen_frame_event_t& ev) {
  if (!SecureEventOk(ev)) return;
  const auto* p = reinterpret_cast<const pen_var_f_payload_t*>(Payload(ev));
  const bool reliable = RequiresAck(ev.msgType);
  if (reliable && MsgIsRetry(ev.msgType) && RecentSeen(ev.msgType, ev.seq, p->varId)) {
    (void)SendAckTo(ev.mac, ev.seq, p->varId);
    return;
  }
  const bool handled = QueueRxVarF(p, ev);
  if (reliable) {
    if (handled) {
      RecentRemember(ev.msgType, ev.seq, p->varId);
      (void)SendAckTo(ev.mac, ev.seq, p->varId);
    } else {
      (void)SendNackTo(ev.mac, ev.seq, p->varId, PEN_NACK_UNSUPPORTED_VAR);
    }
  }
}

static void HandleAck(const pen_frame_event_t& ev) {
  if (!SecureEventOk(ev)) return;
  const auto* p = reinterpret_cast<const pen_ack_payload_t*>(Payload(ev));
  (void)PendingRemove(p->ackSeq);
  pen_rx_event_t out = {};
  out.type = PEN_RX_ACK;
  out.msgType = MsgBase(ev.msgType);
  out.data.ack.seq = ev.seq;
  out.data.ack.ackSeq = p->ackSeq;
  out.data.ack.varId = p->varId;
  (void)QueueApp(out);
}

static void HandleNack(const pen_frame_event_t& ev) {
  if (!SecureEventOk(ev)) return;
  const auto* p = reinterpret_cast<const pen_nack_payload_t*>(Payload(ev));
  (void)PendingRemove(p->ackSeq);
  pen_rx_event_t out = {};
  out.type = PEN_RX_NACK;
  out.msgType = MsgBase(ev.msgType);
  out.data.nack.seq = ev.seq;
  out.data.nack.ackSeq = p->ackSeq;
  out.data.nack.varId = p->varId;
  out.data.nack.reason = p->reason;
  (void)QueueApp(out);
}

static void HandleGetVar(const pen_frame_event_t& ev) {
  if (!SecureEventOk(ev)) return;
  const auto* p = reinterpret_cast<const pen_get_var_payload_t*>(Payload(ev));
  pen_rx_event_t out = {};
  out.type = PEN_RX_GET_VAR;
  out.msgType = MSG_GET_VAR;
  out.data.getVar.seq = ev.seq;
  out.data.getVar.varId = p->varId;
  (void)QueueApp(out);
}

static void ProcessEvent(const pen_frame_event_t& ev) {
#if PEN_RC
  if ((ev.type == PEN_EVT_DISC_RSP) && (s_state == PEN_DISCOVERY)) {
    const auto* p = reinterpret_cast<const pen_discovery_rsp_payload_t*>(Payload(ev));
    memcpy(s_peerMac, ev.mac, 6U);
    s_peerValid = true;
    s_deviceId = p->deviceId;
    memcpy(s_devMac, ev.mac, 6U);
    esp_fill_random(s_rcNonce, sizeof(s_rcNonce));
    (void)EmitLink(PEN_LINK_DISC, ev.rssi);
    s_state = PEN_CONNECTING;
    s_stateStartMs = millis();
    s_lastTxMs = 0U;
  } else if ((ev.type == PEN_EVT_CONN_RSP) && (s_state == PEN_CONNECTING) && (memcmp(ev.mac, s_peerMac, 6U) == 0)) {
    const auto* p = reinterpret_cast<const pen_connect_rsp_payload_t*>(Payload(ev));
    if ((ev.sessionId != 0U) || (memcmp(p->devMac, ev.mac, 6U) != 0)) {
      ResetLink(PEN_LINK_MAC_BAD);
      return;
    }
    memcpy(s_devMac, p->devMac, sizeof(s_devMac));
    s_deviceId = p->deviceId;
    s_sessionId = p->sessionId;
    memcpy(s_devNonce, p->devNonce, sizeof(s_devNonce));
    DeriveSession(s_rcId, s_deviceId, s_sessionId, s_ownMac, s_devMac, s_rcNonce, s_devNonce, s_sessionKey, s_pmk, s_lmk, s_rcProof, s_devProof);
    s_state = PEN_AUTHING;
    s_stateStartMs = millis();
    s_lastTxMs = 0U;
    (void)EmitLink(PEN_LINK_CONNECTED);
  } else if ((ev.type == PEN_EVT_AUTH_RSP) && (s_state == PEN_AUTHING) && (memcmp(ev.mac, s_peerMac, 6U) == 0)) {
    const auto* p = reinterpret_cast<const pen_auth_rsp_payload_t*>(Payload(ev));
    if ((ev.sessionId == s_sessionId) && (memcmp(p->devProof, s_devProof, 16U) == 0)) {
      (void)EmitLink(PEN_LINK_AUTH_OK);
      (void)SendAckTo(ev.mac, ev.seq, 0U);
      s_state = PEN_SECURE_WAIT;
      s_secureInstallAtMs = millis() + PEN_SECURE_DELAY_MS;
    } else {
      ResetLink(PEN_LINK_AUTH_BAD);
    }
  } else
#else
  if (ev.type == PEN_EVT_DISC_REQ) {
    (void)SendDiscoveryResponse(ev.mac);
    (void)EmitLink(PEN_LINK_DISC, ev.rssi);
  } else if ((ev.type == PEN_EVT_CONN_REQ) && ((s_state == PEN_WAIT_RC) || (s_state == PEN_AUTHING))) {
    const auto* p = reinterpret_cast<const pen_connect_req_payload_t*>(Payload(ev));
    memcpy(s_peerMac, ev.mac, 6U);
    memcpy(s_rcMac, p->rcMac, sizeof(s_rcMac));
    memcpy(s_devMac, s_ownMac, sizeof(s_devMac));
    memcpy(s_rcNonce, p->rcNonce, sizeof(s_rcNonce));
    s_peerValid = true;
    s_rcId = p->rcId;
    s_sessionId = esp_random();
    if (s_sessionId == 0U) s_sessionId = 1U;
    esp_fill_random(s_devNonce, sizeof(s_devNonce));
    if (!AddPeer(s_peerMac, false, nullptr)) {
      EmitError(PEN_HW_ERR_ESPNOW, MSG_CONNECT_RSP);
      return;
    }
    DeriveSession(s_rcId, s_deviceId, s_sessionId, s_rcMac, s_devMac, s_rcNonce, s_devNonce, s_sessionKey, s_pmk, s_lmk, s_rcProof, s_devProof);
    s_state = PEN_AUTHING;
    s_stateStartMs = millis();
    s_lastTxMs = millis();
    (void)SendConnectResponse();
    (void)EmitLink(PEN_LINK_CONNECTED);
  } else if ((ev.type == PEN_EVT_AUTH_REQ) && (s_state == PEN_AUTHING) && (memcmp(ev.mac, s_peerMac, 6U) == 0)) {
    const auto* p = reinterpret_cast<const pen_auth_req_payload_t*>(Payload(ev));
    if ((ev.sessionId == s_sessionId) && (memcmp(p->rcProof, s_rcProof, 16U) == 0)) {
      (void)EmitLink(PEN_LINK_AUTH_OK);
      (void)SendAuthResponse();
      s_state = PEN_SECURE_WAIT;
      s_secureInstallAtMs = millis() + PEN_SECURE_DELAY_MS;
    } else {
      ResetLink(PEN_LINK_AUTH_BAD);
    }
  } else
#endif
  if (ev.type == PEN_EVT_VAR_I) HandleVarI(ev);
  else if (ev.type == PEN_EVT_VAR_F) HandleVarF(ev);
  else if (ev.type == PEN_EVT_ACK) HandleAck(ev);
  else if (ev.type == PEN_EVT_NACK) HandleNack(ev);
  else if (ev.type == PEN_EVT_GET_VAR) HandleGetVar(ev);
}

static void DrainFrames(void) {
  for (size_t i = 0; i < PEN_FRAME_Q_CAP; ++i) {
    pen_frame_event_t ev = {};
    bool used = false;
    portENTER_CRITICAL(&s_mux);
    if (s_frameQ[i].used) {
      used = true;
      ev = s_frameQ[i];
      s_frameQ[i].used = false;
    }
    portEXIT_CRITICAL(&s_mux);
    if (used && DecodeFrame(ev)) ProcessEvent(ev);
  }
}

static void ReportCounters(void) {
  uint32_t frameDrops;
  uint32_t badFrames;
  uint32_t appDrops;
  portENTER_CRITICAL(&s_mux);
  frameDrops = s_frameDropCount;
  badFrames = s_badFrameCount;
  appDrops = s_appDropCount;
  portEXIT_CRITICAL(&s_mux);
  if (frameDrops != s_reportedFrameDropCount) {
    EmitError(PEN_HW_ERR_EVENT_DROP, (int32_t)(frameDrops - s_reportedFrameDropCount));
    s_reportedFrameDropCount = frameDrops;
  }
  if (badFrames != s_reportedBadFrameCount) {
    EmitError(PEN_HW_ERR_BAD_FRAME, (int32_t)(badFrames - s_reportedBadFrameCount));
    s_reportedBadFrameCount = badFrames;
  }
  if (appDrops != s_reportedAppDropCount) {
    EmitError(PEN_HW_ERR_RX_DROP, (int32_t)(appDrops - s_reportedAppDropCount));
    s_reportedAppDropCount = appDrops;
  }
}

static void RxTask(void*) {
  for (;;) {
    pen_rx_event_t ev = {};
    bool has = false;
    portENTER_CRITICAL(&s_mux);
    if (s_appTail != s_appHead) {
      ev = s_appQ[s_appTail];
      s_appTail = (uint8_t)((s_appTail + 1U) % PEN_APP_Q_CAP);
      has = true;
    }
    portEXIT_CRITICAL(&s_mux);
    if (has && (s_rxEvent != nullptr)) (void)s_rxEvent(&ev);
    else vTaskDelay(pdMS_TO_TICKS(10U));
  }
}

static void MainTask(void*) {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect(false, true);
  if (!GetStaMac(s_ownMac)) {
    EmitError(PEN_HW_ERR_WIFI, 1);
    s_mainTask = nullptr;
    vTaskDelete(nullptr);
    return;
  }
  if (esp_wifi_set_promiscuous(true) != ESP_OK) EmitError(PEN_HW_ERR_WIFI, 2);
  if (esp_wifi_set_channel(PEN_CHANNEL, WIFI_SECOND_CHAN_NONE) != ESP_OK) EmitError(PEN_HW_ERR_WIFI, 3);
  if (esp_wifi_config_espnow_rate(WIFI_IF_STA, WIFI_PHY_RATE_1M_L) != ESP_OK) EmitError(PEN_HW_ERR_WIFI, 4);
  if (esp_wifi_set_promiscuous(false) != ESP_OK) EmitError(PEN_HW_ERR_WIFI, 5);
  if (esp_now_init() != ESP_OK) {
    EmitError(PEN_HW_ERR_ESPNOW, 1);
    s_mainTask = nullptr;
    vTaskDelete(nullptr);
    return;
  }
  if (esp_now_register_recv_cb(OnRecv) != ESP_OK) {
    EmitError(PEN_HW_ERR_ESPNOW, 2);
    (void)esp_now_deinit();
    s_mainTask = nullptr;
    vTaskDelete(nullptr);
    return;
  }
#if PEN_RC
  s_rcId = MakeIdFromEfuse();
#else
  s_deviceId = MakeIdFromEfuse();
#endif
  (void)EmitLink(PEN_LINK_READY);
  TickType_t last = xTaskGetTickCount();
  for (;;) {
    DrainFrames();
    ReportCounters();
    const uint32_t now = millis();
#if PEN_RC
    if ((s_state == PEN_DISCOVERY) && ((now - s_lastTxMs) >= PEN_DISCOVERY_MS)) {
      s_lastTxMs = now;
      (void)SendDiscovery();
    } else if (s_state == PEN_CONNECTING) {
      if ((now - s_stateStartMs) > PEN_AUTH_TIMEOUT_MS) ResetLink(PEN_LINK_CONN_TO);
      else if ((now - s_lastTxMs) >= PEN_RETRY_MS) {
        s_lastTxMs = now;
        (void)SendConnect();
      }
    } else
#endif
    if (s_state == PEN_AUTHING) {
#if PEN_RC
      if ((now - s_stateStartMs) > PEN_AUTH_TIMEOUT_MS) ResetLink(PEN_LINK_AUTH_TO);
      else if ((now - s_lastTxMs) >= PEN_RETRY_MS) {
        s_lastTxMs = now;
        (void)SendAuth();
      }
#else
      if ((now - s_stateStartMs) > PEN_AUTH_TIMEOUT_MS) ResetLink(PEN_LINK_AUTH_TO);
#endif
    } else if (s_state == PEN_SECURE_WAIT) {
      if ((s_secureInstallAtMs != 0U) && ((int32_t)(now - s_secureInstallAtMs) >= 0)) {
        if (!InstallEncryptedPeer()) ResetLink(PEN_LINK_SEC_BAD);
      }
    } else if (s_state == PEN_SECURE) {
      if ((now - s_lastRxMs) >= PEN_LINK_TIMEOUT_MS) ResetLink(PEN_LINK_LOST);
      else {
        ProcessPending(now);
        (void)SendHeartbeat(now);
      }
    }
    vTaskDelayUntil(&last, pdMS_TO_TICKS(50U));
  }
}

}  // namespace

bool pen_begin(pen_rx_event_fn_t rxEventFn) {
  if (s_mainTask != nullptr) return true;
  s_rxEvent = rxEventFn;
  memset(s_frameQ, 0, sizeof(s_frameQ));
  memset(s_appQ, 0, sizeof(s_appQ));
  s_appHead = 0U;
  s_appTail = 0U;
  ClearSession();
#if PEN_RC
  s_state = PEN_DISCOVERY;
#else
  s_state = PEN_WAIT_RC;
#endif
  s_seq = 1U;
  s_hbCounter = 0U;
  s_lastTxMs = 0U;
  s_lastRxMs = 0U;
  s_stateStartMs = 0U;
  s_secureInstallAtMs = 0U;
  s_frameDropCount = 0U;
  s_badFrameCount = 0U;
  s_appDropCount = 0U;
  s_reportedFrameDropCount = 0U;
  s_reportedBadFrameCount = 0U;
  s_reportedAppDropCount = 0U;
  if (xTaskCreatePinnedToCore(RxTask, "PEN_RX", PEN_LINK_RX_TASK_STACK, nullptr, PEN_LINK_RX_TASK_PRIORITY, &s_rxTask, PEN_LINK_CORE_ID) != pdPASS) return false;
  if (xTaskCreatePinnedToCore(MainTask, "PEN", PEN_LINK_TASK_STACK, nullptr, PEN_LINK_TASK_PRIORITY, &s_mainTask, PEN_LINK_CORE_ID) != pdPASS) {
    vTaskDelete(s_rxTask);
    s_rxTask = nullptr;
    return false;
  }
  return true;
}

bool pen_is_connected(void) {
  return s_state == PEN_SECURE;
}

#if PEN_RC
bool pen_send_get_var(uint32_t varId) {
  if (!pen_is_connected()) return false;
  pen_get_var_payload_t p = {};
  p.varId = varId;
  const bool ok = SendFrame(s_peerMac, MSG_GET_VAR, s_sessionId, s_seq++, p);
  if (ok) s_lastTxMs = millis();
  else EmitError(PEN_HW_ERR_SEND, MSG_GET_VAR);
  return ok;
}
#endif

bool pen_send_stream(uint32_t varId, int32_t value, uint16_t ttlMs) {
  return SendVarI(MSG_STREAM_I_VAR, varId, value, ttlMs);
}

bool pen_send_stream(uint32_t varId, float value, uint16_t ttlMs) {
  return SendVarF(MSG_STREAM_F_VAR, varId, value, ttlMs);
}

bool pen_send_state(uint32_t varId, int32_t value) {
  return SendVarI(MSG_STATE_I_VAR, varId, value, 0U);
}

bool pen_send_state(uint32_t varId, float value) {
  return SendVarF(MSG_STATE_F_VAR, varId, value, 0U);
}

bool pen_send_event(uint32_t varId, int32_t value, uint16_t ttlMs) {
  return SendVarI(MSG_EVENT_I_VAR, varId, value, ttlMs);
}

bool pen_send_event(uint32_t varId, float value, uint16_t ttlMs) {
  return SendVarF(MSG_EVENT_F_VAR, varId, value, ttlMs);
}
