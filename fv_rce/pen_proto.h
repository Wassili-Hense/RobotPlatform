#ifndef PEN_PROTO_H
#define PEN_PROTO_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <esp_crc.h>

// -----------------------------------------------------------------------------
// PEN protocol rules
// -----------------------------------------------------------------------------
// - ESP-NOW is only a packet transport.
// - Frame: pen_hdr_t + fixed payload selected by msgType + pen_crc_t.
// - payloadLen is not present; payload size is derived from msgType.
// - version is not present.
// - msgType bit7 is retry flag; bits0..6 are proto_msg_type_t.
// - CRC16 is calculated over header + payload, excluding pen_crc_t.
// - CRC16 is stored little-endian on wire.
// - CRC implementation uses esp_crc16_le().
//
// Session:
// - pen_hdr_t.sessionId is the common session identifier.
// - DISCOVERY_REQ, DISCOVERY_RSP and CONNECT_REQ use hdr.sessionId = 0.
// - CONNECT_RSP uses hdr.sessionId = 0 and carries the newly allocated
//   sessionId in pen_connect_rsp_payload_t.sessionId.
// - After CONNECT_RSP, all session-bound packets use hdr.sessionId.
// - AUTH_REQ and AUTH_RSP do not duplicate sessionId in payload.
//
// Link activity / heartbeat:
// - MSG_PING and MSG_PONG are removed.
// - Link activity is maintained by normal traffic.
// - Any valid packet from the current peer with current sessionId refreshes RX
//   activity timestamp.
// - Heartbeat is implemented as STREAM scheduling:
//     1) send expired STREAM variable first;
//     2) if none expired, send the oldest STREAM variable;
//     3) if STREAM cache is empty, send fallback PEN_VAR_HB as STREAM_I.
// - Heartbeat STREAM does not require ACK.
//
// STREAM / STATE / EVENT:
// - STREAM is used for frequent/current values and heartbeat. STREAM has no ACK.
// - STATE is used for idempotent configuration/state exchange. STATE requires ACK.
// - EVENT is reserved for non-idempotent events that must be delivered once.
// - LSET, RSET and USBC are STATE variables, not EVENT variables.
//
// ACK/NACK:
// - ACK confirms successful processing.
// - NACK reports failed processing.
// - ACK/NACK do not carry telemetry.
// - Telemetry must be sent as normal STREAM/STATE variables.
// -----------------------------------------------------------------------------

static constexpr uint8_t  PEN_MAGIC = 0xA7U;
static constexpr uint8_t  PEN_MSG_FLAG_RETRY = 0x80U;
static constexpr uint8_t  PEN_MSG_TYPE_MASK  = 0x7FU;
static constexpr uint16_t PEN_CRC_INIT = 0xFFFFU;

#define PEN_VAR_ID4(a, b, c, d) \
    ((uint32_t)(uint8_t)(a)        | \
    ((uint32_t)(uint8_t)(b) << 8)  | \
    ((uint32_t)(uint8_t)(c) << 16) | \
    ((uint32_t)(uint8_t)(d) << 24))
#define PEN_VAR_ID3(a, b, c) PEN_VAR_ID4((a), (b), (c), '\0')
#define PEN_VAR_ID2(a, b)    PEN_VAR_ID4((a), (b), '\0', '\0')
#define PEN_VAR_ID1(a)       PEN_VAR_ID4((a), '\0', '\0', '\0')

static constexpr uint32_t PEN_VAR_HB = PEN_VAR_ID2('H', 'B');

enum proto_msg_type_t : uint8_t {
    MSG_DISCOVERY_REQ = 0x02U,
    MSG_DISCOVERY_RSP = 0x03U,
    MSG_CONNECT_REQ   = 0x04U,
    MSG_CONNECT_RSP   = 0x05U,
    MSG_AUTH_REQ      = 0x06U,
    MSG_AUTH_RSP      = 0x07U,
    // 0x08 and 0x09 are reserved; previously PING/PONG.
    MSG_ACK           = 0x0AU,
    MSG_NACK          = 0x0BU,
    MSG_GET_VAR       = 0x0CU,
    MSG_STREAM_I_VAR  = 0x20U,
    MSG_STREAM_F_VAR  = 0x21U,
    MSG_STATE_I_VAR   = 0x22U,
    MSG_STATE_F_VAR   = 0x23U,
    MSG_EVENT_I_VAR   = 0x24U,
    MSG_EVENT_F_VAR   = 0x25U
};

enum pen_nack_reason_t : uint8_t {
    PEN_NACK_UNSUPPORTED_VAR = 1U,
    PEN_NACK_INVALID_TYPE    = 2U,
    PEN_NACK_INVALID_VALUE   = 3U,
    PEN_NACK_NOT_ALLOWED     = 4U,
    PEN_NACK_BAD_SESSION     = 5U,
    PEN_NACK_BAD_CRC         = 6U
};

static inline uint8_t pen_msg_base(uint8_t rawMsgType) { return rawMsgType & PEN_MSG_TYPE_MASK; }
static inline bool pen_msg_is_retry(uint8_t rawMsgType) { return (rawMsgType & PEN_MSG_FLAG_RETRY) != 0U; }
static inline uint8_t pen_msg_make(uint8_t msgType, bool retry) { return (uint8_t)((msgType & PEN_MSG_TYPE_MASK) | (retry ? PEN_MSG_FLAG_RETRY : 0U)); }
static inline void pen_var_id_to_text(uint32_t varId, char out[5]) {
    out[0] = (char)((varId >> 0)  & 0xFFU);
    out[1] = (char)((varId >> 8)  & 0xFFU);
    out[2] = (char)((varId >> 16) & 0xFFU);
    out[3] = (char)((varId >> 24) & 0xFFU);
    out[4] = '\0';
}

#pragma pack(push, 1)
struct pen_hdr_t { uint8_t magic; uint8_t msgType; uint32_t sessionId; uint16_t seq; };
struct pen_discovery_rsp_payload_t { uint32_t deviceId; uint32_t caps; uint8_t workChannel; char name[12]; };
struct pen_connect_req_payload_t { uint32_t rcId; uint32_t caps; uint8_t rcMac[6]; uint8_t rcNonce[16]; };
struct pen_connect_rsp_payload_t { uint32_t deviceId; uint32_t caps; uint32_t sessionId; uint8_t devMac[6]; uint8_t devNonce[16]; };
struct pen_auth_req_payload_t { uint8_t rcProof[16]; };
struct pen_auth_rsp_payload_t { uint8_t devProof[16]; };
struct pen_var_i_payload_t { uint32_t varId; uint16_t ttlMs; int32_t value; };
struct pen_var_f_payload_t { uint32_t varId; uint16_t ttlMs; float value; };
struct pen_get_var_payload_t { uint32_t varId; };
struct pen_ack_payload_t { uint16_t ackSeq; uint32_t varId; };
struct pen_nack_payload_t { uint16_t ackSeq; uint32_t varId; uint8_t reason; };
struct pen_crc_t { uint16_t crc16; };
#pragma pack(pop)

static inline size_t pen_payload_size(uint8_t rawMsgType) {
    switch (pen_msg_base(rawMsgType)) {
        case MSG_DISCOVERY_REQ: return 0U;
        case MSG_DISCOVERY_RSP: return sizeof(pen_discovery_rsp_payload_t);
        case MSG_CONNECT_REQ:   return sizeof(pen_connect_req_payload_t);
        case MSG_CONNECT_RSP:   return sizeof(pen_connect_rsp_payload_t);
        case MSG_AUTH_REQ:      return sizeof(pen_auth_req_payload_t);
        case MSG_AUTH_RSP:      return sizeof(pen_auth_rsp_payload_t);
        case MSG_ACK:           return sizeof(pen_ack_payload_t);
        case MSG_NACK:          return sizeof(pen_nack_payload_t);
        case MSG_GET_VAR:       return sizeof(pen_get_var_payload_t);
        case MSG_STREAM_I_VAR:  return sizeof(pen_var_i_payload_t);
        case MSG_STREAM_F_VAR:  return sizeof(pen_var_f_payload_t);
        case MSG_STATE_I_VAR:   return sizeof(pen_var_i_payload_t);
        case MSG_STATE_F_VAR:   return sizeof(pen_var_f_payload_t);
        case MSG_EVENT_I_VAR:   return sizeof(pen_var_i_payload_t);
        case MSG_EVENT_F_VAR:   return sizeof(pen_var_f_payload_t);
        default:                return SIZE_MAX;
    }
}
static inline size_t pen_frame_size(uint8_t rawMsgType) { const size_t ps = pen_payload_size(rawMsgType); return (ps == SIZE_MAX) ? SIZE_MAX : sizeof(pen_hdr_t) + ps + sizeof(pen_crc_t); }
static inline bool pen_len_valid(uint8_t rawMsgType, size_t len) { const size_t fs = pen_frame_size(rawMsgType); return (fs != SIZE_MAX) && (len == fs); }
static inline uint16_t pen_crc16(const void* data, size_t len) { if (data == nullptr) return 0U; return esp_crc16_le(PEN_CRC_INIT, reinterpret_cast<const uint8_t*>(data), (uint32_t)len); }
static inline uint16_t pen_read_le16(const uint8_t* ptr) { return (uint16_t)ptr[0] | ((uint16_t)ptr[1] << 8); }
static inline void pen_write_le16(uint8_t* ptr, uint16_t value) { ptr[0] = (uint8_t)(value & 0xFFU); ptr[1] = (uint8_t)((value >> 8) & 0xFFU); }
static inline bool pen_crc_valid(const void* frame, size_t len) { if ((frame == nullptr) || (len < (sizeof(pen_hdr_t) + sizeof(pen_crc_t)))) return false; const uint8_t* b = reinterpret_cast<const uint8_t*>(frame); const size_t dl = len - sizeof(pen_crc_t); return pen_read_le16(&b[dl]) == pen_crc16(frame, dl); }
static inline bool pen_frame_valid(const void* frame, size_t len) { if ((frame == nullptr) || (len < (sizeof(pen_hdr_t) + sizeof(pen_crc_t)))) return false; const pen_hdr_t* h = reinterpret_cast<const pen_hdr_t*>(frame); return (h->magic == PEN_MAGIC) && pen_len_valid(h->msgType, len) && pen_crc_valid(frame, len); }
static inline bool pen_finalize_frame(void* frame, size_t len) { if ((frame == nullptr) || (len < (sizeof(pen_hdr_t) + sizeof(pen_crc_t)))) return false; pen_hdr_t* h = reinterpret_cast<pen_hdr_t*>(frame); if (!pen_len_valid(h->msgType, len)) return false; const size_t dl = len - sizeof(pen_crc_t); uint8_t* b = reinterpret_cast<uint8_t*>(frame); pen_write_le16(&b[dl], pen_crc16(frame, dl)); return true; }

static_assert(sizeof(pen_hdr_t) == 8U, "pen_hdr_t size changed");
static_assert(sizeof(pen_discovery_rsp_payload_t) == 21U, "pen_discovery_rsp_payload_t size changed");
static_assert(sizeof(pen_connect_req_payload_t) == 30U, "pen_connect_req_payload_t size changed");
static_assert(sizeof(pen_connect_rsp_payload_t) == 34U, "pen_connect_rsp_payload_t size changed");
static_assert(sizeof(pen_auth_req_payload_t) == 16U, "pen_auth_req_payload_t size changed");
static_assert(sizeof(pen_auth_rsp_payload_t) == 16U, "pen_auth_rsp_payload_t size changed");
static_assert(sizeof(pen_var_i_payload_t) == 10U, "pen_var_i_payload_t size changed");
static_assert(sizeof(pen_var_f_payload_t) == 10U, "pen_var_f_payload_t size changed");
static_assert(sizeof(pen_get_var_payload_t) == 4U, "pen_get_var_payload_t size changed");
static_assert(sizeof(pen_ack_payload_t) == 6U, "pen_ack_payload_t size changed");
static_assert(sizeof(pen_nack_payload_t) == 7U, "pen_nack_payload_t size changed");
static_assert((PEN_MSG_FLAG_RETRY & PEN_MSG_TYPE_MASK) == 0U, "retry flag overlaps type mask");

#endif // PEN_PROTO_H
