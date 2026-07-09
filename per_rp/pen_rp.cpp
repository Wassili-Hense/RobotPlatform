// ESP32-D0WDQ6 (revision v1.1)
#include "pen_rp.h"

#define PEN_COMM_IMPLEMENTATION
#include "pen_comm.cpp"
#undef PEN_COMM_IMPLEMENTATION

namespace {

static constexpr size_t RP_VAR_CAP = 128U;

enum rp_var_kind_t : uint8_t {
  RP_VAR_STREAM = 1U,
  RP_VAR_STATE = 2U,
  RP_VAR_EVENT = 3U
};

struct rp_var_binding_t {
  bool used;
  uint32_t varId;
  uint8_t kind;
  bool isFloat;
  bool dirty;
  union {
    int32_t* iPtr;
    float* fPtr;
    pen_rp_event_i_cb_t iCb;
    pen_rp_event_f_cb_t fCb;
  } target;
};

static rp_var_binding_t s_rpVars[RP_VAR_CAP];
static size_t s_rpVarCount = 0U;
static constexpr uint32_t RP_STATE_SAVE_DELAY_MS = 1000U;
static bool s_rpStateDirtyPending = false;
static uint32_t s_rpStateDirtyAtMs = 0U;
static size_t s_rpStateSaveIndex = 0U;
static size_t s_rpStateDirtyCount = 0U;

static uint8_t RpKindFromMsg(uint8_t msgType) {
  switch (MsgBase(msgType)) {
    case MSG_STREAM_I_VAR:
    case MSG_STREAM_F_VAR: return RP_VAR_STREAM;
    case MSG_STATE_I_VAR:
    case MSG_STATE_F_VAR: return RP_VAR_STATE;
    case MSG_EVENT_I_VAR:
    case MSG_EVENT_F_VAR: return RP_VAR_EVENT;
    default: return 0U;
  }
}

static size_t RpLowerBoundNoLock(uint32_t varId) {
  size_t lo = 0U;
  size_t hi = s_rpVarCount;
  while (lo < hi) {
    const size_t mid = lo + ((hi - lo) >> 1U);
    if (s_rpVars[mid].varId < varId) lo = mid + 1U;
    else hi = mid;
  }
  return lo;
}

static bool RpFindBinding(uint32_t varId, rp_var_binding_t& out) {
  bool found = false;
  portENTER_CRITICAL(&s_mux);
  const size_t pos = RpLowerBoundNoLock(varId);
  if ((pos < s_rpVarCount) && s_rpVars[pos].used && (s_rpVars[pos].varId == varId)) {
    out = s_rpVars[pos];
    found = true;
  }
  portEXIT_CRITICAL(&s_mux);
  return found;
}

static bool RpInsertOrReplace(rp_var_binding_t binding) {
  if ((binding.varId == 0U) || !binding.used) return false;

  bool ok = false;
  portENTER_CRITICAL(&s_mux);
  const size_t pos = RpLowerBoundNoLock(binding.varId);

  if ((pos < s_rpVarCount) && s_rpVars[pos].used && (s_rpVars[pos].varId == binding.varId)) {
    s_rpVars[pos] = binding;
    ok = true;
  } else if (s_rpVarCount < RP_VAR_CAP) {
    for (size_t i = s_rpVarCount; i > pos; --i) {
      s_rpVars[i] = s_rpVars[i - 1U];
    }
    s_rpVars[pos] = binding;
    ++s_rpVarCount;
    ok = true;
  }
  portEXIT_CRITICAL(&s_mux);
  return ok;
}

static bool RpRegisterPtr(uint32_t varId, uint8_t kind, bool isFloat, void* ptr) {
  if ((varId == 0U) || (ptr == nullptr) || ((kind != RP_VAR_STREAM) && (kind != RP_VAR_STATE))) return false;

  rp_var_binding_t b = {};
  b.used = true;
  b.varId = varId;
  b.kind = kind;
  b.isFloat = isFloat;
  if (isFloat) b.target.fPtr = static_cast<float*>(ptr);
  else b.target.iPtr = static_cast<int32_t*>(ptr);
  return RpInsertOrReplace(b);
}

static bool RpRegisterCbI(uint32_t varId, pen_rp_event_i_cb_t cb) {
  if ((varId == 0U) || (cb == nullptr)) return false;

  rp_var_binding_t b = {};
  b.used = true;
  b.varId = varId;
  b.kind = RP_VAR_EVENT;
  b.isFloat = false;
  b.target.iCb = cb;
  return RpInsertOrReplace(b);
}

static bool RpRegisterCbF(uint32_t varId, pen_rp_event_f_cb_t cb) {
  if ((varId == 0U) || (cb == nullptr)) return false;

  rp_var_binding_t b = {};
  b.used = true;
  b.varId = varId;
  b.kind = RP_VAR_EVENT;
  b.isFloat = true;
  b.target.fCb = cb;
  return RpInsertOrReplace(b);
}

static void RpMarkStateDirty(uint32_t varId) {
  portENTER_CRITICAL(&s_mux);
  const size_t pos = RpLowerBoundNoLock(varId);
  if ((pos < s_rpVarCount) &&
      s_rpVars[pos].used &&
      (s_rpVars[pos].varId == varId) &&
      (s_rpVars[pos].kind == RP_VAR_STATE)) {
    if (!s_rpVars[pos].dirty) {
      if (s_rpStateDirtyCount == 0U) {
        s_rpStateSaveIndex = pos;
      }
      ++s_rpStateDirtyCount;
      s_rpVars[pos].dirty = true;
    }
    s_rpStateDirtyPending = true;
    s_rpStateDirtyAtMs = millis();
  }
  portEXIT_CRITICAL(&s_mux);
}

static bool RpQueueOneStateSave(void) {
  pen_rx_event_t ev = {};
  bool has = false;
  uint32_t varId = 0U;
  size_t picked = 0U;

  portENTER_CRITICAL(&s_mux);

  if ((s_rpVarCount == 0U) || (s_rpStateDirtyCount == 0U)) {
    s_rpStateDirtyPending = false;
    s_rpStateDirtyCount = 0U;
    s_rpStateSaveIndex = 0U;
    portEXIT_CRITICAL(&s_mux);
    return false;
  }

  if (s_rpStateSaveIndex >= s_rpVarCount) {
    s_rpStateSaveIndex = 0U;
  }

  for (size_t n = 0U; n < s_rpVarCount; ++n) {
    const size_t i = (s_rpStateSaveIndex + n) % s_rpVarCount;
    rp_var_binding_t& b = s_rpVars[i];

    if (!b.used || !b.dirty || (b.kind != RP_VAR_STATE)) continue;

    if (b.isFloat) {
      if (b.target.fPtr == nullptr) continue;

      ev.type = PEN_RX_STATE_SAVE_F;
      ev.msgType = MSG_STATE_F_VAR;
      ev.data.varF.varId = b.varId;
      ev.data.varF.value = *b.target.fPtr;
    } else {
      if (b.target.iPtr == nullptr) continue;

      ev.type = PEN_RX_STATE_SAVE_I;
      ev.msgType = MSG_STATE_I_VAR;
      ev.data.varI.varId = b.varId;
      ev.data.varI.value = *b.target.iPtr;
    }
    ev.data.varF.retry = false;

    varId = b.varId;
    picked = i;
    has = true;
    break;
  }

  if (!has) {
    s_rpStateDirtyPending = false;
    s_rpStateDirtyCount = 0U;
    s_rpStateSaveIndex = 0U;
  }

  portEXIT_CRITICAL(&s_mux);

  if (!has) return false;

  if (!QueueApp(ev)) {
    return false;
  }

  portENTER_CRITICAL(&s_mux);

  const size_t pos = RpLowerBoundNoLock(varId);
  if ((pos < s_rpVarCount) &&
      s_rpVars[pos].used &&
      (s_rpVars[pos].varId == varId) &&
      (s_rpVars[pos].kind == RP_VAR_STATE) &&
      s_rpVars[pos].dirty) {
    s_rpVars[pos].dirty = false;
    if (s_rpStateDirtyCount > 0U) --s_rpStateDirtyCount;
  }

  if (s_rpVarCount > 0U) {
    s_rpStateSaveIndex = (picked + 1U) % s_rpVarCount;
  } else {
    s_rpStateSaveIndex = s_rpVarCount;
  }

  s_rpStateDirtyPending = s_rpStateDirtyCount != 0U;
  if (s_rpStateDirtyPending) {
    s_rpStateDirtyAtMs = millis();
  }

  portEXIT_CRITICAL(&s_mux);

  return true;
}

static void RpProcessStateSave(uint32_t now) {
  if (!s_rpStateDirtyPending) return;
  if ((now - s_rpStateDirtyAtMs) < RP_STATE_SAVE_DELAY_MS) return;
  (void)RpQueueOneStateSave();
}

static bool RpApplyValue(uint8_t msgType, uint32_t varId, bool incomingFloat, int32_t iValue, float fValue, bool retry) {
  const uint8_t kind = RpKindFromMsg(msgType);
  if (kind == 0U) return false;
  rp_var_binding_t b = {};
  if (!RpFindBinding(varId, b)) {
    return kind == RP_VAR_STREAM;
  }
  if (b.kind != kind) {
    return kind == RP_VAR_STREAM;
  }
  if (kind == RP_VAR_EVENT) {
    if (retry) return true;
    if (b.isFloat) {
      if (b.target.fCb == nullptr) return false;
      b.target.fCb(varId, incomingFloat ? fValue : (float)iValue, false);
    } else {
      if (b.target.iCb == nullptr) return false;
      b.target.iCb(varId, incomingFloat ? (int32_t)fValue : iValue, false);
    }
    return true;
  }
  if (b.isFloat) {
    if (b.target.fPtr == nullptr) return false;
    const float newValue = incomingFloat ? fValue : (float)iValue;
    const bool changed = (*b.target.fPtr != newValue);
    *b.target.fPtr = newValue;
    if ((kind == RP_VAR_STATE) && changed) RpMarkStateDirty(varId);
  } else {
    if (b.target.iPtr == nullptr) return false;
    const int32_t newValue = incomingFloat ? (int32_t)fValue : iValue;
    const bool changed = (*b.target.iPtr != newValue);
    *b.target.iPtr = newValue;
    if ((kind == RP_VAR_STATE) && changed) RpMarkStateDirty(varId);
  }
  return true;
}
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
  RpProcessStateSave(now);
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
  if (p == nullptr) return false;
  return RpApplyValue(ev.msgType, p->varId, false, p->value, 0.0F, retry);
}
static bool RoleAfterQueueRxVarF(const pen_var_f_payload_t* p, const frame_evt_t& ev, bool retry) {
  if (p == nullptr) return false;
  return RpApplyValue(ev.msgType, p->varId, true, 0, p->value, retry);
}
static bool RoleGetVarSupported(const pen_get_var_payload_t* p, const frame_evt_t& ev) {
  (void)ev;
  if (p == nullptr) return false;
  rp_var_binding_t b = {};
  return RpFindBinding(p->varId, b);
}
static bool RoleSendGetVarValue(const pen_get_var_payload_t* p, const frame_evt_t& ev, bool retry) {
  (void)ev;
  (void)retry;
  if (p == nullptr) return false;

  rp_var_binding_t b = {};
  if (!RpFindBinding(p->varId, b)) return false;

  if (b.isFloat) {
    if (b.target.fPtr == nullptr) return false;
    return pen_send_state(p->varId, *b.target.fPtr);
  }

  if (b.target.iPtr == nullptr) return false;
  return pen_send_state(p->varId, *b.target.iPtr);
}
}  // namespace

bool pen_rp_register_stream(uint32_t varId, int32_t* value) {
  return RpRegisterPtr(varId, RP_VAR_STREAM, false, value);
}
bool pen_rp_register_stream(uint32_t varId, float* value) {
  return RpRegisterPtr(varId, RP_VAR_STREAM, true, value);
}
bool pen_rp_register_state(uint32_t varId, int32_t* value) {
  return RpRegisterPtr(varId, RP_VAR_STATE, false, value);
}
bool pen_rp_register_state(uint32_t varId, float* value) {
  return RpRegisterPtr(varId, RP_VAR_STATE, true, value);
}
bool pen_rp_register_event(uint32_t varId, pen_rp_event_i_cb_t cb) {
  return RpRegisterCbI(varId, cb);
}
bool pen_rp_register_event(uint32_t varId, pen_rp_event_f_cb_t cb) {
  return RpRegisterCbF(varId, cb);
}
bool pen_rp_unregister(uint32_t varId) {
  if (varId == 0U) return false;

  bool removed = false;
  portENTER_CRITICAL(&s_mux);
  const size_t pos = RpLowerBoundNoLock(varId);
  if ((pos < s_rpVarCount) && s_rpVars[pos].used && (s_rpVars[pos].varId == varId)) {
    for (size_t i = pos; (i + 1U) < s_rpVarCount; ++i) {
      s_rpVars[i] = s_rpVars[i + 1U];
    }
    --s_rpVarCount;
    s_rpVars[s_rpVarCount] = {};
    removed = true;
  }
  portEXIT_CRITICAL(&s_mux);
  return removed;
}
