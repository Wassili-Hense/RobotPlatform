
#include "pen_rc.h"

#define PEN_COMM_IMPLEMENTATION
#include "pen_comm.cpp"
#undef PEN_COMM_IMPLEMENTATION

namespace {
static link_state_t RoleInitialState(void) {
  return ST_DISCOVERY;
}
static bool RoleDecodeFrameType(uint8_t t, frame_evt_t& ev) {
  if (t == MSG_DISCOVERY_RSP) ev.type = EVT_DISC_RSP;
  else if (t == MSG_CONNECT_RSP) ev.type = EVT_CONN_RSP;
  else if (t == MSG_AUTH_RSP) ev.type = EVT_AUTH_RSP;
  else return false;
  return true;
}
static void RoleClearSession(void) {}
static void RoleInitIdentity(void) {
  s_rcId = MakeId();
}
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
  if (!ok){
    EmitError(PEN_HW_ERR_SEND, MSG_AUTH_REQ);
  }
  return ok;
}
static void RoleTick(uint32_t now) {
  if ((s_state == ST_DISCOVERY) && ((now - s_lastTxMs) >= DISCOVERY_MS)) {
    s_lastTxMs = now;
    (void)SendDiscovery();
  } else if (s_state == ST_CONNECTING) {
    if ((now - s_stateStartMs) > AUTH_TIMEOUT_MS){ 
      ResetLink(PEN_LINK_CONN_TO);
    } else if ((now - s_lastTxMs) >= RETRY_MS) {
      s_lastTxMs = now;
      (void)SendConnect();
    }
  } else if (s_state == ST_AUTHING) {
    if ((now - s_stateStartMs) > AUTH_TIMEOUT_MS){
       ResetLink(PEN_LINK_AUTH_TO);
    } else if ((now - s_lastTxMs) >= RETRY_MS) {
      s_lastTxMs = now;
      (void)SendAuth();
    }
  }
}
static bool RoleProcessEvent(const frame_evt_t& ev) {
  if ((ev.type == EVT_DISC_RSP) && (s_state == ST_DISCOVERY)) {
    const auto* p = reinterpret_cast<const pen_discovery_rsp_payload_t*>(Payload(ev));
    memcpy(s_peerMac, ev.mac, 6U);
    s_peerValid = true;
    s_deviceId = p->deviceId;
    memcpy(s_devMac, ev.mac, 6U);
    esp_fill_random(s_rcNonce, sizeof(s_rcNonce));
    (void)EmitLink(PEN_LINK_DISC, ev.rssi);
    s_state = ST_CONNECTING;
    s_stateStartMs = millis();
    s_lastTxMs = 0U;
    return true;
  }
  if ((ev.type == EVT_CONN_RSP) && (s_state == ST_CONNECTING) && (memcmp(ev.mac, s_peerMac, 6U) == 0)) {
    const auto* p = reinterpret_cast<const pen_connect_rsp_payload_t*>(Payload(ev));
    if ((ev.sessionId != 0U) || (memcmp(p->devMac, ev.mac, 6U) != 0)) {
      ResetLink(PEN_LINK_MAC_BAD);
      return true;
    }
    memcpy(s_devMac, p->devMac, sizeof(s_devMac));
    s_deviceId = p->deviceId;
    s_sessionId = p->sessionId;
    memcpy(s_devNonce, p->devNonce, sizeof(s_devNonce));
    DeriveSession(s_rcId, s_deviceId, s_sessionId, s_ownMac, s_devMac, s_rcNonce, s_devNonce, s_sessionKey, s_pmk, s_lmk, s_rcProof, s_devProof);
    s_state = ST_AUTHING;
    s_stateStartMs = millis();
    s_lastTxMs = 0U;
    (void)EmitLink(PEN_LINK_CONNECTED);
    return true;
  }
  if ((ev.type == EVT_AUTH_RSP) && (s_state == ST_AUTHING) && (memcmp(ev.mac, s_peerMac, 6U) == 0)) {
    const auto* p = reinterpret_cast<const pen_auth_rsp_payload_t*>(Payload(ev));
    if ((ev.sessionId == s_sessionId) && (memcmp(p->devProof, s_devProof, 16U) == 0)) {
      (void)EmitLink(PEN_LINK_AUTH_OK);
      (void)SendAckTo(ev.mac, ev.seq, 0U);
      s_state = ST_SECURE_WAIT;
      s_secureInstallAtMs = millis() + SECURE_DELAY_MS;
    } else{
      ResetLink(PEN_LINK_AUTH_BAD);
    }
    return true;
  }
  return false;
}
static void RolePrepareHeartbeatItem(stream_cache_t& item) {
  (void)item;
}
static bool RoleAfterQueueRxVarI(const pen_var_i_payload_t* p, const frame_evt_t& ev, bool retry) {
  if (p->varId == PEN_VAR_RSSI){
    return EmitVarI(MSG_STREAM_I_VAR, PEN_VAR_RSSL, (int32_t)ev.rssi, retry);
  }
  return true;
}
}  // namespace

bool pen_send_get_var(uint32_t varId) {
  if (!pen_is_connected()) return false;
  const uint16_t seq = s_seq++;
  const uint32_t now = millis();
  if (!PendingAddGet(seq, varId, now)) {
    EmitError(PEN_HW_ERR_RETRY_FULL, MSG_GET_VAR);
    return false;
  }
  pen_get_var_payload_t p = {};
  p.varId = varId;
  const bool ok = SendFrame(s_peerMac, MSG_GET_VAR, s_sessionId, seq, p);
  if (ok){ 
    s_lastTxMs = now;
  } else {
    (void)PendingRemove(seq);
    EmitError(PEN_HW_ERR_SEND, MSG_GET_VAR);
  }
  return ok;
}
