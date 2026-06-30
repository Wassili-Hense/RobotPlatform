#include "pec_link.h"
#include "pen_common.h"

#include <WiFi.h>
#include <ctype.h>
#include <esp_wifi.h>
#include <esp_system.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "freertos/task.h"

namespace {

enum pec_state_t : uint8_t { PEC_DISCOVERY, PEC_CONNECTING, PEC_AUTHING, PEC_SECURE_WAIT, PEC_SECURE };
enum pec_event_type_t : uint8_t { PEC_EVT_DISC, PEC_EVT_CONNECT_RSP, PEC_EVT_AUTH_RSP, PEC_EVT_VAR_I, PEC_EVT_VAR_F, PEC_EVT_ACK, PEC_EVT_NACK };

struct pec_event_t {
    volatile bool used;
    uint8_t type;
    uint8_t msgType;
    uint16_t seq;
    uint32_t sessionId;
    uint8_t mac[6];
    int8_t rssi;
    uint8_t payload[40];
};

struct pec_stream_cache_t {
    bool used;
    bool isFloat;
    uint32_t varId;
    uint16_t ttlMs;
    uint32_t lastTxMs;
    int32_t iValue;
    float fValue;
};

static constexpr uint32_t PEC_DISCOVERY_MS = 1000U;
static constexpr uint32_t PEC_RETRY_MS = 500U;
static constexpr uint32_t PEC_AUTH_TIMEOUT_MS = 3000U;
static constexpr uint32_t PEC_SECURE_DELAY_MS = 200U;
static constexpr uint32_t PEC_HEARTBEAT_MS = 150U;
static constexpr uint16_t PEC_HEARTBEAT_TTL_MS = 300U;
static constexpr uint32_t PEC_LINK_TIMEOUT_MS = 500U;
static constexpr size_t PEC_EVENT_CAP = 10U;
static constexpr size_t PEC_STREAM_CACHE_CAP = 8U;

static TaskHandle_t s_task = nullptr;
static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;
static pec_event_t s_events[PEC_EVENT_CAP];
static pec_stream_cache_t s_streams[PEC_STREAM_CACHE_CAP];
static pec_line_fn_t s_line = nullptr;
static pec_state_t s_state = PEC_DISCOVERY;
static uint8_t s_peerMac[6] = {};
static bool s_peerValid = false;
static uint8_t s_ownMac[6] = {};
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
static volatile bool s_statusDirty = true;
static volatile bool s_connected = false;
static volatile int8_t s_peerRssi = -100;
static volatile int8_t s_localRssi = -100;
static volatile uint8_t s_battery = 0U;
static volatile uint8_t s_errorCode = PEC_HW_ERR_NONE;
static volatile int32_t s_errorDetail = 0;
static volatile uint32_t s_eventDropCount = 0U;
static volatile uint32_t s_badFrameCount = 0U;
static uint32_t s_reportedEventDropCount = 0U;
static uint32_t s_reportedBadFrameCount = 0U;

static bool SendVarI(uint8_t msgType, uint32_t varId, int32_t value, uint16_t ttlMs);
static bool SendVarF(uint8_t msgType, uint32_t varId, float value, uint16_t ttlMs);

static bool Line(const char* text) { return (s_line != nullptr) && (text != nullptr) && s_line(text); }

static void EmitError(uint8_t code, int32_t detail) {
    char line[32];
    portENTER_CRITICAL(&s_mux);
    s_errorCode = code;
    s_errorDetail = detail;
    s_statusDirty = true;
    portEXIT_CRITICAL(&s_mux);
    snprintf(line, sizeof(line), "@ERR %u %ld", (unsigned)code, (long)detail);
    (void)Line(line);
}

static bool MacIsSet(const uint8_t mac[6]) {
    static const uint8_t zero[6] = {};
    return (mac != nullptr) && (memcmp(mac, zero, 6U) != 0);
}

static void SetLinkStatus(bool connected, int8_t peerRssi, int8_t localRssi) {
    portENTER_CRITICAL(&s_mux);
    s_connected = connected;
    s_peerRssi = peerRssi;
    s_localRssi = localRssi;
    s_statusDirty = true;
    portEXIT_CRITICAL(&s_mux);
}

static void SetBattery(uint8_t battery) {
    portENTER_CRITICAL(&s_mux);
    s_battery = battery;
    s_statusDirty = true;
    portEXIT_CRITICAL(&s_mux);
}

static void ClearStreamCache(void) { memset(s_streams, 0, sizeof(s_streams)); }

static void ClearSessionMaterial(void) {
    s_peerValid = false;
    memset(s_peerMac, 0, sizeof(s_peerMac));
    memset(s_devMac, 0, sizeof(s_devMac));
    s_deviceId = 0U;
    s_sessionId = 0U;
    memset(s_rcNonce, 0, sizeof(s_rcNonce));
    memset(s_devNonce, 0, sizeof(s_devNonce));
    memset(s_sessionKey, 0, sizeof(s_sessionKey));
    memset(s_pmk, 0, sizeof(s_pmk));
    memset(s_lmk, 0, sizeof(s_lmk));
    memset(s_rcProof, 0, sizeof(s_rcProof));
    memset(s_devProof, 0, sizeof(s_devProof));
    ClearStreamCache();
}

static void ResetToDiscovery(const char* reason) {
    if (s_peerValid && MacIsSet(s_peerMac) && esp_now_is_peer_exist(s_peerMac)) (void)esp_now_del_peer(s_peerMac);
    ClearSessionMaterial();
    s_state = PEC_DISCOVERY;
    s_lastTxMs = 0U;
    s_lastRxMs = 0U;
    s_stateStartMs = millis();
    s_secureInstallAtMs = 0U;
    SetLinkStatus(false, -100, -100);
    if (reason != nullptr) (void)Line(reason);
}

static bool IsStateMsg(uint8_t type) { return (type == MSG_STATE_I_VAR) || (type == MSG_STATE_F_VAR); }
static bool IsEventMsg(uint8_t type) { return (type == MSG_EVENT_I_VAR) || (type == MSG_EVENT_F_VAR); }
static bool IsSecureMsg(uint8_t type) {
    return (type == MSG_STREAM_I_VAR) || (type == MSG_STREAM_F_VAR) ||
           (type == MSG_STATE_I_VAR) || (type == MSG_STATE_F_VAR) ||
           (type == MSG_EVENT_I_VAR) || (type == MSG_EVENT_F_VAR) ||
           (type == MSG_ACK) || (type == MSG_NACK) || (type == MSG_GET_VAR);
}
static bool SecureEventOk(const pec_event_t& ev) { return (s_state == PEC_SECURE) && s_peerValid && (ev.sessionId == s_sessionId) && (memcmp(ev.mac, s_peerMac, 6U) == 0); }

static void QueueEvent(uint8_t eventType, uint8_t msgType, uint32_t sessionId, const uint8_t* mac, int8_t rssi, uint16_t seq, const void* payload, size_t payloadLen) {
    portENTER_CRITICAL(&s_mux);
    for (size_t i = 0; i < PEC_EVENT_CAP; ++i) {
        if (!s_events[i].used) {
            s_events[i].used = true;
            s_events[i].type = eventType;
            s_events[i].msgType = msgType;
            s_events[i].seq = seq;
            s_events[i].sessionId = sessionId;
            memcpy((void*)s_events[i].mac, mac, 6U);
            s_events[i].rssi = rssi;
            memset((void*)s_events[i].payload, 0, sizeof(s_events[i].payload));
            if ((payload != nullptr) && (payloadLen <= sizeof(s_events[i].payload))) memcpy((void*)s_events[i].payload, payload, payloadLen);
            portEXIT_CRITICAL(&s_mux);
            return;
        }
    }
    ++s_eventDropCount;
    portEXIT_CRITICAL(&s_mux);
}

static void OnRecv(const esp_now_recv_info_t* info, const uint8_t* data, int len) {
    if ((info == nullptr) || (info->src_addr == nullptr) || (data == nullptr) || (len <= 0) || !pen_frame_valid(data, (size_t)len)) {
        portENTER_CRITICAL(&s_mux); ++s_badFrameCount; portEXIT_CRITICAL(&s_mux); return;
    }
    const pen_hdr_t* hdr = reinterpret_cast<const pen_hdr_t*>(data);
    const uint8_t type = pen_msg_base(hdr->msgType);
    const uint8_t* payload = data + sizeof(pen_hdr_t);
    const int8_t rssi = (info->rx_ctrl != nullptr) ? info->rx_ctrl->rssi : -100;
    if ((s_state == PEC_SECURE) && s_peerValid && IsSecureMsg(type) && (hdr->sessionId == s_sessionId) && (memcmp(info->src_addr, s_peerMac, 6U) == 0)) {
        s_lastRxMs = millis();
        SetLinkStatus(true, s_peerRssi, rssi);
    }
    if (type == MSG_DISCOVERY_RSP) QueueEvent(PEC_EVT_DISC, type, hdr->sessionId, info->src_addr, rssi, hdr->seq, payload, sizeof(pen_discovery_rsp_payload_t));
    else if (type == MSG_CONNECT_RSP) QueueEvent(PEC_EVT_CONNECT_RSP, type, hdr->sessionId, info->src_addr, rssi, hdr->seq, payload, sizeof(pen_connect_rsp_payload_t));
    else if (type == MSG_AUTH_RSP) QueueEvent(PEC_EVT_AUTH_RSP, type, hdr->sessionId, info->src_addr, rssi, hdr->seq, payload, sizeof(pen_auth_rsp_payload_t));
    else if ((type == MSG_STREAM_I_VAR) || (type == MSG_STATE_I_VAR) || (type == MSG_EVENT_I_VAR)) QueueEvent(PEC_EVT_VAR_I, type, hdr->sessionId, info->src_addr, rssi, hdr->seq, payload, sizeof(pen_var_i_payload_t));
    else if ((type == MSG_STREAM_F_VAR) || (type == MSG_STATE_F_VAR) || (type == MSG_EVENT_F_VAR)) QueueEvent(PEC_EVT_VAR_F, type, hdr->sessionId, info->src_addr, rssi, hdr->seq, payload, sizeof(pen_var_f_payload_t));
    else if (type == MSG_ACK) QueueEvent(PEC_EVT_ACK, type, hdr->sessionId, info->src_addr, rssi, hdr->seq, payload, sizeof(pen_ack_payload_t));
    else if (type == MSG_NACK) QueueEvent(PEC_EVT_NACK, type, hdr->sessionId, info->src_addr, rssi, hdr->seq, payload, sizeof(pen_nack_payload_t));
}

static bool SendAckTo(const uint8_t mac[6], uint16_t ackSeq, uint32_t varId) {
    pen_ack_payload_t p = {}; p.ackSeq = ackSeq; p.varId = varId;
    const bool ok = pen_send_frame(mac, MSG_ACK, s_sessionId, s_seq++, p);
    if (ok) s_lastTxMs = millis(); else EmitError(PEC_HW_ERR_SEND, MSG_ACK);
    return ok;
}

static bool SendNackTo(const uint8_t mac[6], uint16_t ackSeq, uint32_t varId, uint8_t reason) {
    pen_nack_payload_t p = {}; p.ackSeq = ackSeq; p.varId = varId; p.reason = reason;
    const bool ok = pen_send_frame(mac, MSG_NACK, s_sessionId, s_seq++, p);
    if (ok) s_lastTxMs = millis(); else EmitError(PEC_HW_ERR_SEND, MSG_NACK);
    return ok;
}

static bool SendDiscovery(void) {
    (void)pen_add_peer(PEN_BROADCAST_MAC, false, nullptr);
    const bool ok = pen_send_empty_frame(PEN_BROADCAST_MAC, MSG_DISCOVERY_REQ, 0U, s_seq++);
    if (!ok) EmitError(PEC_HW_ERR_SEND, MSG_DISCOVERY_REQ);
    return ok;
}

static bool SendConnect(void) {
    pen_connect_req_payload_t p = {};
    p.rcId = s_rcId; p.caps = PEN_CAPS; memcpy(p.rcMac, s_ownMac, sizeof(p.rcMac)); memcpy(p.rcNonce, s_rcNonce, sizeof(p.rcNonce));
    if (!pen_add_peer(s_peerMac, false, nullptr)) { EmitError(PEC_HW_ERR_ESPNOW, MSG_CONNECT_REQ); return false; }
    const bool ok = pen_send_frame(s_peerMac, MSG_CONNECT_REQ, 0U, s_seq++, p);
    if (!ok) EmitError(PEC_HW_ERR_SEND, MSG_CONNECT_REQ);
    return ok;
}

static bool SendAuth(void) {
    pen_auth_req_payload_t p = {}; memcpy(p.rcProof, s_rcProof, sizeof(p.rcProof));
    const bool ok = pen_send_frame(s_peerMac, MSG_AUTH_REQ, s_sessionId, s_seq++, p);
    if (!ok) EmitError(PEC_HW_ERR_SEND, MSG_AUTH_REQ);
    return ok;
}

static bool InstallEncryptedPeer(void) {
    if (esp_now_set_pmk(s_pmk) != ESP_OK) { EmitError(PEC_HW_ERR_ESPNOW, 100); return false; }
    if (!pen_add_peer(s_peerMac, true, s_lmk)) { EmitError(PEC_HW_ERR_ESPNOW, 101); return false; }
    const uint32_t now = millis();
    s_state = PEC_SECURE;
    s_lastRxMs = now;
    s_lastTxMs = now;
    s_secureInstallAtMs = 0U;
    s_hbCounter = 0U;
    SetLinkStatus(true, -100, -100);
    (void)Line("@LINK SECURE");
    return true;
}

static void CacheStreamI(uint32_t varId, int32_t value, uint16_t ttlMs) {
    if (varId == PEN_VAR_HB) return;
    portENTER_CRITICAL(&s_mux);
    size_t slot = PEC_STREAM_CACHE_CAP;
    for (size_t i = 0; i < PEC_STREAM_CACHE_CAP; ++i) {
        if (s_streams[i].used && !s_streams[i].isFloat && (s_streams[i].varId == varId)) { slot = i; break; }
        if (!s_streams[i].used && (slot == PEC_STREAM_CACHE_CAP)) slot = i;
    }
    if (slot < PEC_STREAM_CACHE_CAP) { s_streams[slot].used = true; s_streams[slot].isFloat = false; s_streams[slot].varId = varId; s_streams[slot].ttlMs = ttlMs; s_streams[slot].lastTxMs = millis(); s_streams[slot].iValue = value; }
    portEXIT_CRITICAL(&s_mux);
}

static void CacheStreamF(uint32_t varId, float value, uint16_t ttlMs) {
    portENTER_CRITICAL(&s_mux);
    size_t slot = PEC_STREAM_CACHE_CAP;
    for (size_t i = 0; i < PEC_STREAM_CACHE_CAP; ++i) {
        if (s_streams[i].used && s_streams[i].isFloat && (s_streams[i].varId == varId)) { slot = i; break; }
        if (!s_streams[i].used && (slot == PEC_STREAM_CACHE_CAP) && (varId != PEN_VAR_HB)) slot = i;
    }
    if (slot < PEC_STREAM_CACHE_CAP) { s_streams[slot].used = true; s_streams[slot].isFloat = true; s_streams[slot].varId = varId; s_streams[slot].ttlMs = ttlMs; s_streams[slot].lastTxMs = millis(); s_streams[slot].fValue = value; }
    portEXIT_CRITICAL(&s_mux);
}

static bool SendVarIRaw(uint32_t varId, int32_t value, uint16_t ttlMs) {
    pen_var_i_payload_t p = {}; p.varId = varId; p.ttlMs = ttlMs; p.value = value;
    const bool ok = pen_send_frame(s_peerMac, MSG_STREAM_I_VAR, s_sessionId, s_seq++, p);
    if (ok) s_lastTxMs = millis(); else EmitError(PEC_HW_ERR_SEND, MSG_STREAM_I_VAR);
    return ok;
}

static bool SendVarFRaw(uint32_t varId, float value, uint16_t ttlMs) {
    pen_var_f_payload_t p = {}; p.varId = varId; p.ttlMs = ttlMs; p.value = value;
    const bool ok = pen_send_frame(s_peerMac, MSG_STREAM_F_VAR, s_sessionId, s_seq++, p);
    if (ok) s_lastTxMs = millis(); else EmitError(PEC_HW_ERR_SEND, MSG_STREAM_F_VAR);
    return ok;
}

static bool SendStreamHeartbeat(uint32_t now) {
    int expired = -1;
    int oldest = -1;
    uint32_t expiredAge = 0U;
    uint32_t oldestAge = 0U;
    portENTER_CRITICAL(&s_mux);
    for (size_t i = 0; i < PEC_STREAM_CACHE_CAP; ++i) {
        if (!s_streams[i].used) continue;
        const uint32_t age = now - s_streams[i].lastTxMs;
        if ((oldest < 0) || (age > oldestAge)) { oldest = (int)i; oldestAge = age; }
        if ((s_streams[i].ttlMs != 0U) && (age >= s_streams[i].ttlMs) && ((expired < 0) || (age > expiredAge))) { expired = (int)i; expiredAge = age; }
    }
    const int pick = (expired >= 0) ? expired : (((now - s_lastTxMs) >= PEC_HEARTBEAT_MS) ? oldest : -1);
    pec_stream_cache_t item = {};
    if (pick >= 0) { item = s_streams[pick]; s_streams[pick].lastTxMs = now; }
    portEXIT_CRITICAL(&s_mux);

    if (pick >= 0) return item.isFloat ? SendVarFRaw(item.varId, item.fValue, item.ttlMs) : SendVarIRaw(item.varId, item.iValue, item.ttlMs);
    if ((oldest < 0) && ((now - s_lastTxMs) >= PEC_HEARTBEAT_MS)) return SendVarIRaw(PEN_VAR_HB, (int32_t)(++s_hbCounter), PEC_HEARTBEAT_TTL_MS);
    return true;
}

static bool SendVarI(uint8_t msgType, uint32_t varId, int32_t value, uint16_t ttlMs) {
    if (!pec_is_connected()) return false;
    pen_var_i_payload_t p = {}; p.varId = varId; p.ttlMs = ttlMs; p.value = value;
    const bool ok = pen_send_frame(s_peerMac, msgType, s_sessionId, s_seq++, p);
    if (ok) { s_lastTxMs = millis(); if (msgType == MSG_STREAM_I_VAR) CacheStreamI(varId, value, ttlMs); }
    else EmitError(PEC_HW_ERR_SEND, msgType);
    return ok;
}

static bool SendVarF(uint8_t msgType, uint32_t varId, float value, uint16_t ttlMs) {
    if (!pec_is_connected()) return false;
    pen_var_f_payload_t p = {}; p.varId = varId; p.ttlMs = ttlMs; p.value = value;
    const bool ok = pen_send_frame(s_peerMac, msgType, s_sessionId, s_seq++, p);
    if (ok) { s_lastTxMs = millis(); if (msgType == MSG_STREAM_F_VAR) CacheStreamF(varId, value, ttlMs); }
    else EmitError(PEC_HW_ERR_SEND, msgType);
    return ok;
}

static bool ForwardVarIToPc(const pen_var_i_payload_t* p, uint8_t msgType, uint16_t seq) {
    if (p->varId == PEN_VAR_HB) return true;
    char name[5]; char line[32]; pen_var_id_to_text(p->varId, name);
    snprintf(line, sizeof(line), "%s %ld I %u %u", name, (long)p->value, (unsigned)msgType, (unsigned)seq);
    return Line(line);
}

static bool ForwardVarFToPc(const pen_var_f_payload_t* p, uint8_t msgType, uint16_t seq) {
    char name[5]; char line[32]; pen_var_id_to_text(p->varId, name);
    snprintf(line, sizeof(line), "%s %.4f F %u %u", name, (double)p->value, (unsigned)msgType, (unsigned)seq);
    return Line(line);
}
static void SetPeerRssi(int8_t peerRssi) {
    portENTER_CRITICAL(&s_mux);
    s_peerRssi = peerRssi;
    s_statusDirty = true;
    portEXIT_CRITICAL(&s_mux);
}
static void HandleVarI(const pec_event_t& ev) {
    if (!SecureEventOk(ev)) return;
    const auto* p = reinterpret_cast<const pen_var_i_payload_t*>(ev.payload);
    bool handled = false;
    if (p->varId == PEC_VAR_BATP) {
      int32_t v = p->value;
      if (v < 0) v = 0;
      if (v > 100) v = 100;
      SetBattery((uint8_t)v);
      handled = true;

    } else if (p->varId == PEC_VAR_RSSI) {

      int32_t v = p->value;

      // RSSI [-127..0]
      if (v < -127) v = -127;
      if (v > 0) v = 0;

      SetPeerRssi((int8_t)v);
      handled = true;

    } else {
      handled = ForwardVarIToPc(p, ev.msgType, ev.seq);
    }
    if (IsStateMsg(ev.msgType) || IsEventMsg(ev.msgType)) { if (handled) (void)SendAckTo(ev.mac, ev.seq, p->varId); else (void)SendNackTo(ev.mac, ev.seq, p->varId, PEN_NACK_UNSUPPORTED_VAR); }
}

static void HandleVarF(const pec_event_t& ev) {
    if (!SecureEventOk(ev)) return;
    const auto* p = reinterpret_cast<const pen_var_f_payload_t*>(ev.payload);
    const bool handled = ForwardVarFToPc(p, ev.msgType, ev.seq);
    if (IsStateMsg(ev.msgType) || IsEventMsg(ev.msgType)) { if (handled) (void)SendAckTo(ev.mac, ev.seq, p->varId); else (void)SendNackTo(ev.mac, ev.seq, p->varId, PEN_NACK_UNSUPPORTED_VAR); }
}

static void HandleAck(const pec_event_t& ev) {
    if (!SecureEventOk(ev)) return;
    const auto* p = reinterpret_cast<const pen_ack_payload_t*>(ev.payload);
    char name[5]; char line[32]; pen_var_id_to_text(p->varId, name);
    snprintf(line, sizeof(line), "@ACK %u %s", (unsigned)p->ackSeq, name);
    (void)Line(line);
}

static void HandleNack(const pec_event_t& ev) {
    if (!SecureEventOk(ev)) return;
    const auto* p = reinterpret_cast<const pen_nack_payload_t*>(ev.payload);
    char name[5]; char line[32]; pen_var_id_to_text(p->varId, name);
    snprintf(line, sizeof(line), "@NACK %u %s %u", (unsigned)p->ackSeq, name, (unsigned)p->reason);
    (void)Line(line);
}

static void ProcessEvent(const pec_event_t& ev) {
    char line[32];
    if ((ev.type == PEC_EVT_DISC) && (s_state == PEC_DISCOVERY)) {
        const auto* p = reinterpret_cast<const pen_discovery_rsp_payload_t*>(ev.payload);
        memcpy(s_peerMac, ev.mac, 6U); s_peerValid = true; s_deviceId = p->deviceId; esp_fill_random(s_rcNonce, sizeof(s_rcNonce));
        snprintf(line, sizeof(line), "@DISC %d", (int)ev.rssi); (void)Line(line);
        s_state = PEC_CONNECTING; s_stateStartMs = millis(); s_lastTxMs = 0U; SetLinkStatus(false, -100, -100);
    } else if ((ev.type == PEC_EVT_CONNECT_RSP) && (s_state == PEC_CONNECTING) && (memcmp(ev.mac, s_peerMac, 6U) == 0)) {
        const auto* p = reinterpret_cast<const pen_connect_rsp_payload_t*>(ev.payload);
        if ((ev.sessionId != 0U) || (memcmp(p->devMac, ev.mac, 6U) != 0)) { ResetToDiscovery("@LINK MAC_BAD"); return; }
        memcpy(s_devMac, p->devMac, sizeof(s_devMac)); s_deviceId = p->deviceId; s_sessionId = p->sessionId; memcpy(s_devNonce, p->devNonce, sizeof(s_devNonce));
        pen_derive_session(s_rcId, s_deviceId, s_sessionId, s_ownMac, s_devMac, s_rcNonce, s_devNonce, s_sessionKey, s_pmk, s_lmk, s_rcProof, s_devProof);
        s_state = PEC_AUTHING; s_stateStartMs = millis(); s_lastTxMs = 0U; (void)Line("@LINK CONNECTED");
    } else if ((ev.type == PEC_EVT_AUTH_RSP) && (s_state == PEC_AUTHING) && (memcmp(ev.mac, s_peerMac, 6U) == 0)) {
        const auto* p = reinterpret_cast<const pen_auth_rsp_payload_t*>(ev.payload);
        if ((ev.sessionId == s_sessionId) && (memcmp(p->devProof, s_devProof, 16U) == 0)) { (void)Line("@LINK AUTH_OK"); (void)SendAckTo(ev.mac, ev.seq, 0U); s_state = PEC_SECURE_WAIT; s_secureInstallAtMs = millis() + PEC_SECURE_DELAY_MS; }
        else ResetToDiscovery("@LINK AUTH_BAD");
    } else if (ev.type == PEC_EVT_VAR_I) HandleVarI(ev);
    else if (ev.type == PEC_EVT_VAR_F) HandleVarF(ev);
    else if (ev.type == PEC_EVT_ACK) HandleAck(ev);
    else if (ev.type == PEC_EVT_NACK) HandleNack(ev);
}

static void DrainEvents(void) {
    for (size_t i = 0; i < PEC_EVENT_CAP; ++i) {
        pec_event_t ev = {}; bool used = false;
        portENTER_CRITICAL(&s_mux);
        if (s_events[i].used) { used = true; ev.type = s_events[i].type; ev.msgType = s_events[i].msgType; ev.seq = s_events[i].seq; ev.sessionId = s_events[i].sessionId; memcpy(ev.mac, (const void*)s_events[i].mac, 6U); ev.rssi = s_events[i].rssi; memcpy(ev.payload, (const void*)s_events[i].payload, sizeof(ev.payload)); s_events[i].used = false; }
        portEXIT_CRITICAL(&s_mux);
        if (used) ProcessEvent(ev);
    }
}

static void ReportCounters(void) {
    uint32_t drops; uint32_t badFrames;
    portENTER_CRITICAL(&s_mux); drops = s_eventDropCount; badFrames = s_badFrameCount; portEXIT_CRITICAL(&s_mux);
    if (drops != s_reportedEventDropCount) { EmitError(PEC_HW_ERR_EVENT_DROP, (int32_t)(drops - s_reportedEventDropCount)); s_reportedEventDropCount = drops; }
    if (badFrames != s_reportedBadFrameCount) { EmitError(PEC_HW_ERR_BAD_FRAME, (int32_t)(badFrames - s_reportedBadFrameCount)); s_reportedBadFrameCount = badFrames; }
}

static void Task(void*) {
    WiFi.mode(WIFI_STA); WiFi.disconnect(false, true);
    if (!pen_get_sta_mac(s_ownMac)) { EmitError(PEC_HW_ERR_WIFI, 1); s_task = nullptr; vTaskDelete(nullptr); return; }
    if (esp_wifi_set_promiscuous(true) != ESP_OK) EmitError(PEC_HW_ERR_WIFI, 2);
    if (esp_wifi_set_channel(PEN_CHANNEL, WIFI_SECOND_CHAN_NONE) != ESP_OK) EmitError(PEC_HW_ERR_WIFI, 3);
    if (esp_wifi_set_promiscuous(false) != ESP_OK) EmitError(PEC_HW_ERR_WIFI, 4);
    if (esp_now_init() != ESP_OK) { EmitError(PEC_HW_ERR_ESPNOW, 1); s_task = nullptr; vTaskDelete(nullptr); return; }
    if (esp_now_register_recv_cb(OnRecv) != ESP_OK) { EmitError(PEC_HW_ERR_ESPNOW, 2); (void)esp_now_deinit(); s_task = nullptr; vTaskDelete(nullptr); return; }
    s_rcId = pen_make_id_from_efuse(); (void)Line("@LINK READY");
    TickType_t last = xTaskGetTickCount();
    for (;;) {
        DrainEvents(); ReportCounters(); const uint32_t now = millis();
        if ((s_state == PEC_DISCOVERY) && ((now - s_lastTxMs) >= PEC_DISCOVERY_MS)) { s_lastTxMs = now; (void)SendDiscovery(); }
        else if (s_state == PEC_CONNECTING) { if ((now - s_stateStartMs) > PEC_AUTH_TIMEOUT_MS) ResetToDiscovery("@LINK CONN_TO"); else if ((now - s_lastTxMs) >= PEC_RETRY_MS) { s_lastTxMs = now; (void)SendConnect(); } }
        else if (s_state == PEC_AUTHING) { if ((now - s_stateStartMs) > PEC_AUTH_TIMEOUT_MS) ResetToDiscovery("@LINK AUTH_TO"); else if ((now - s_lastTxMs) >= PEC_RETRY_MS) { s_lastTxMs = now; (void)SendAuth(); } }
        else if (s_state == PEC_SECURE_WAIT) { if ((s_secureInstallAtMs != 0U) && ((int32_t)(now - s_secureInstallAtMs) >= 0)) { if (!InstallEncryptedPeer()) ResetToDiscovery("@LINK SEC_BAD"); } }
        else if (s_state == PEC_SECURE) { if ((now - s_lastRxMs) >= PEC_LINK_TIMEOUT_MS) ResetToDiscovery("@LINK LOST"); else (void)SendStreamHeartbeat(now); }
        vTaskDelayUntil(&last, pdMS_TO_TICKS(50U));
    }
}

static bool VarIdFromText(const char* text, uint32_t* outVarId) {
    if ((text == nullptr) || (outVarId == nullptr)) return false;
    const size_t len = strlen(text); if ((len == 0U) || (len > 4U)) return false;
    uint8_t b[4] = {0U, 0U, 0U, 0U};
    for (size_t i = 0; i < len; ++i) { if (!isGraph((unsigned char)text[i])) return false; b[i] = (uint8_t)text[i]; }
    *outVarId = PEN_VAR_ID4(b[0], b[1], b[2], b[3]); return true;
}

static bool LooksFloat(const char* text) { return (text != nullptr) && ((strchr(text, '.') != nullptr) || (strchr(text, 'e') != nullptr) || (strchr(text, 'E') != nullptr)); }

} // namespace

bool pec_begin(pec_line_fn_t lineFn, BaseType_t coreId, UBaseType_t priority, uint32_t stackSize) {
    if (s_task != nullptr) return true;
    s_line = lineFn; memset(s_events, 0, sizeof(s_events)); ClearSessionMaterial();
    s_state = PEC_DISCOVERY; s_seq = 1U; s_hbCounter = 0U; s_lastTxMs = 0U; s_lastRxMs = 0U; s_stateStartMs = 0U; s_secureInstallAtMs = 0U;
    s_statusDirty = true; s_connected = false; s_peerRssi = -100; s_localRssi = -100; s_battery = 0U; s_errorCode = PEC_HW_ERR_NONE; s_errorDetail = 0;
    s_eventDropCount = 0U; s_badFrameCount = 0U; s_reportedEventDropCount = 0U; s_reportedBadFrameCount = 0U;
    return xTaskCreatePinnedToCore(Task, "PEC", stackSize, nullptr, priority, &s_task, coreId) == pdPASS;
}

bool pec_is_connected(void) { return s_state == PEC_SECURE; }

void pec_get_status(pec_status_t* outStatus) {
    if (outStatus == nullptr) return;
    portENTER_CRITICAL(&s_mux);
    outStatus->connected = s_connected; outStatus->battery = s_battery; outStatus->peerRssi = s_peerRssi; outStatus->localRssi = s_localRssi; outStatus->errorCode = s_errorCode; outStatus->errorDetail = s_errorDetail;
    portEXIT_CRITICAL(&s_mux);
}

bool pec_take_status(pec_status_t* outStatus) {
    if (outStatus == nullptr) return false;
    bool dirty;
    portENTER_CRITICAL(&s_mux);
    dirty = s_statusDirty; outStatus->connected = s_connected; outStatus->battery = s_battery; outStatus->peerRssi = s_peerRssi; outStatus->localRssi = s_localRssi; outStatus->errorCode = s_errorCode; outStatus->errorDetail = s_errorDetail; s_statusDirty = false;
    portEXIT_CRITICAL(&s_mux);
    return dirty;
}

bool pec_send_get_var(uint32_t varId) {
    if (!pec_is_connected()) return false;
    pen_get_var_payload_t p = {}; p.varId = varId;
    const bool ok = pen_send_frame(s_peerMac, MSG_GET_VAR, s_sessionId, s_seq++, p);
    if (ok) s_lastTxMs = millis(); else EmitError(PEC_HW_ERR_SEND, MSG_GET_VAR);
    return ok;
}

bool pec_send_stream_i(uint32_t varId, int32_t value, uint16_t ttlMs) { return SendVarI(MSG_STREAM_I_VAR, varId, value, ttlMs); }
bool pec_send_stream_f(uint32_t varId, float value, uint16_t ttlMs) { return SendVarF(MSG_STREAM_F_VAR, varId, value, ttlMs); }
bool pec_send_state_i(uint32_t varId, int32_t value, uint16_t ttlMs) { return SendVarI(MSG_STATE_I_VAR, varId, value, ttlMs); }
bool pec_send_state_f(uint32_t varId, float value, uint16_t ttlMs) { return SendVarF(MSG_STATE_F_VAR, varId, value, ttlMs); }
bool pec_send_event_i(uint32_t varId, int32_t value, uint16_t ttlMs) { return SendVarI(MSG_EVENT_I_VAR, varId, value, ttlMs); }
bool pec_send_event_f(uint32_t varId, float value, uint16_t ttlMs) { return SendVarF(MSG_EVENT_F_VAR, varId, value, ttlMs); }

bool pec_pc_rx_line(const char* line) {
    if (line == nullptr) return false;
    char buf[32]; strncpy(buf, line, sizeof(buf) - 1U); buf[sizeof(buf) - 1U] = '\0';
    char* savePtr = nullptr; char* varText = strtok_r(buf, " \t\r\n", &savePtr); char* valueText = strtok_r(nullptr, " \t\r\n", &savePtr);
    if ((varText == nullptr) || (valueText == nullptr)) return false;
    uint32_t varId; if (!VarIdFromText(varText, &varId)) return false;
    if (strcmp(valueText, "?") == 0) return pec_send_get_var(varId);
    if (LooksFloat(valueText)) { char* endPtr = nullptr; const float value = strtof(valueText, &endPtr); if ((endPtr == valueText) || (*endPtr != '\0')) return false; return pec_send_state_f(varId, value, 0U); }
    char* endPtr = nullptr; const long value = strtol(valueText, &endPtr, 10);
    if ((endPtr == valueText) || (*endPtr != '\0') || (value < INT32_MIN) || (value > INT32_MAX)) return false;
    return pec_send_state_i(varId, (int32_t)value, 0U);
}
