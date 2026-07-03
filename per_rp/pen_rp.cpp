#include "pen_rp.h"

#define PEN_COMM_IMPLEMENTATION
#include "pen_comm.cpp"
#undef PEN_COMM_IMPLEMENTATION

namespace {
static link_state_t RoleInitialState(void) {
  return ST_WAIT_RC;
}
static bool RoleDecodeFrameType(uint8_t t, frame_evt_t& ev) {
  if (t == MSG_DISCOVERY_REQ) ev.type = EVT_DISC_REQ;
  else if (t == MSG_CONNECT_REQ) ev.type = EVT_CONN_REQ;
  else if (t == MSG_AUTH_REQ) ev.type = EVT_AUTH_REQ;
  else return false;
  return true;
}
static void RoleClearSession(void) {
  SetStreamIValue(PEN_VAR_RSSI, (int32_t)s_peerRssi, RSSI_TTL_MS, false);
}
static void RoleInitIdentity(void) {
  s_deviceId = MakeId();
}
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
static void RoleTick(uint32_t now) {
  if ((s_state == ST_AUTHING) && ((now - s_stateStartMs) > AUTH_TIMEOUT_MS)) ResetLink(PEN_LINK_AUTH_TO);
}
static bool RoleProcessEvent(const frame_evt_t& ev) {
  if (ev.type == EVT_DISC_REQ) {
    (void)SendDiscoveryResponse(ev.mac);
    (void)EmitLink(PEN_LINK_DISC, ev.rssi);
    return true;
  }
  if ((ev.type == EVT_CONN_REQ) && ((s_state == ST_WAIT_RC) || (s_state == ST_AUTHING))) {
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
      return true;
    }
    DeriveSession(s_rcId, s_deviceId, s_sessionId, s_rcMac, s_devMac, s_rcNonce, s_devNonce, s_sessionKey, s_pmk, s_lmk, s_rcProof, s_devProof);
    s_state = ST_AUTHING;
    s_stateStartMs = millis();
    s_lastTxMs = millis();
    (void)SendConnectResponse();
    (void)EmitLink(PEN_LINK_CONNECTED);
    return true;
  }
  if ((ev.type == EVT_AUTH_REQ) && (s_state == ST_AUTHING) && (memcmp(ev.mac, s_peerMac, 6U) == 0)) {
    const auto* p = reinterpret_cast<const pen_auth_req_payload_t*>(Payload(ev));
    if ((ev.sessionId == s_sessionId) && (memcmp(p->rcProof, s_rcProof, 16U) == 0)) {
      (void)EmitLink(PEN_LINK_AUTH_OK);
      (void)SendAuthResponse();
      s_state = ST_SECURE_WAIT;
      s_secureInstallAtMs = millis() + SECURE_DELAY_MS;
    } else ResetLink(PEN_LINK_AUTH_BAD);
    return true;
  }
  return false;
}
static void RolePrepareHeartbeatItem(stream_cache_t& item) {
  if (item.varId == PEN_VAR_RSSI) item.value.iValue = (int32_t)s_peerRssi;
}
static bool RoleAfterQueueRxVarI(const pen_var_i_payload_t* p, const frame_evt_t& ev, bool retry) {
  (void)p;
  (void)ev;
  (void)retry;
  return true;
}
}  // namespace
