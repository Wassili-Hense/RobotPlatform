
#ifndef PEN_PROTO_H
#define PEN_PROTO_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <esp_crc.h>

// -----------------------------------------------------------------------------
// Section: PEN protocol rules
// -----------------------------------------------------------------------------
// Transport:
// - ESP-NOW is only a packet transport.
// - Discovery may be plaintext broadcast.
// - CONNECT and AUTH are plaintext unicast with bind-secret proof.
// - PING/PONG after successful AUTH are sent through encrypted ESP-NOW peer.
// - bind_secret, PMK and LMK are never transmitted over radio.
// - PMK/LMK are derived locally from bind_secret + nonces + ids + MACs.
//
// Frame format:
// - Every packet is: pen_hdr_t + fixed payload selected by msgType + pen_crc_t.
// - payloadLen is not present. Payload size is derived from msgType.
// - version is not present. Protocol compatibility is negotiated during connect.
// - msgType bit7 is retry flag. bits0..6 are proto_msg_type_t.
// - CRC16 is calculated over header + payload, excluding the CRC field itself.
// - CRC16 is stored little-endian on wire.
// - CRC implementation uses esp_crc16_le().
//
// Sequence:
// - seq is a per-direction frame sequence number.
// - STREAM packets do not require ACK.
// - STATE packets require ACK. Pending STATE with the same varId may be replaced.
// - EVENT packets require ACK. EVENT retry uses the same seq with retry bit set.
// - Receiver must execute EVENT only once; duplicate retry must only be ACKed.
// - varSeq is intentionally not used.
//
// Variables:
// - Each VAR packet carries exactly one variable.
// - varId is uint32 FourCC, padded with '\0' for IDs shorter than 4 chars.
// - Only int32_t and float values are supported in this protocol version.
// - Joystick variables should be sent when changed || expired.
//
// GET_VAR:
// - GET_VAR requests one variable by varId.
// - Response is STATE_I_VAR / STATE_F_VAR with the same varId, or NACK.
//
// PING/PONG:
// - PING/PONG are used for idle keepalive and bidirectional RSSI diagnostics.
// - PONG returns RSSI measured at peer while receiving PING.
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

// -----------------------------------------------------------------------------
// Section: Message types
// -----------------------------------------------------------------------------

enum proto_msg_type_t : uint8_t {
    MSG_DISCOVERY_REQ = 0x02U,
    MSG_DISCOVERY_RSP = 0x03U,

    MSG_CONNECT_REQ   = 0x04U,
    MSG_CONNECT_RSP   = 0x05U,
    MSG_AUTH_REQ      = 0x06U,
    MSG_AUTH_RSP      = 0x07U,

    MSG_PING          = 0x08U,
    MSG_PONG          = 0x09U,

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

// -----------------------------------------------------------------------------
// Section: Common helpers
// -----------------------------------------------------------------------------

static inline uint8_t pen_msg_base(uint8_t rawMsgType) {
    return rawMsgType & PEN_MSG_TYPE_MASK;
}

static inline bool pen_msg_is_retry(uint8_t rawMsgType) {
    return (rawMsgType & PEN_MSG_FLAG_RETRY) != 0U;
}

static inline uint8_t pen_msg_make(uint8_t msgType, bool retry) {
    return (uint8_t)((msgType & PEN_MSG_TYPE_MASK) | (retry ? PEN_MSG_FLAG_RETRY : 0U));
}

static inline void pen_var_id_to_text(uint32_t varId, char out[5]) {
    out[0] = (char)((varId >> 0)  & 0xFFU);
    out[1] = (char)((varId >> 8)  & 0xFFU);
    out[2] = (char)((varId >> 16) & 0xFFU);
    out[3] = (char)((varId >> 24) & 0xFFU);
    out[4] = '\0';
}

// -----------------------------------------------------------------------------
// Section: Wire structures
// -----------------------------------------------------------------------------

#pragma pack(push, 1)

struct pen_hdr_t {
    uint8_t  magic;
    uint8_t  msgType;
    uint32_t sessionId;
    uint16_t seq;
};

struct pen_discovery_rsp_payload_t {
    uint32_t deviceId;
    uint32_t caps;
    uint8_t  workChannel;
    char     name[12];
};

struct pen_connect_req_payload_t {
    uint32_t rcId;
    uint32_t caps;
    uint8_t  rcMac[6];
    uint8_t  rcNonce[16];
};

struct pen_connect_rsp_payload_t {
    uint32_t deviceId;
    uint32_t caps;
    uint32_t sessionId;
    uint8_t  devMac[6];
    uint8_t  devNonce[16];
};

struct pen_auth_req_payload_t {
    uint32_t sessionId;
    uint8_t  rcProof[16];
};

struct pen_auth_rsp_payload_t {
    uint32_t sessionId;
    uint8_t  devProof[16];
};

struct pen_var_i_payload_t {
    uint32_t varId;
    uint16_t ttlMs;
    int32_t  value;
};

struct pen_var_f_payload_t {
    uint32_t varId;
    uint16_t ttlMs;
    float    value;
};

struct pen_get_var_payload_t {
    uint32_t varId;
};

struct pen_ack_payload_t {
    uint16_t ackSeq;
    uint32_t varId;
};

struct pen_nack_payload_t {
    uint16_t ackSeq;
    uint32_t varId;
    uint8_t  reason;
};

struct pen_ping_payload_t {
    uint16_t pingSeq;
};

struct pen_pong_payload_t {
    uint16_t pingSeq;
    int8_t   rssiAtPeer;
};

struct pen_crc_t {
    uint16_t crc16;
};

#pragma pack(pop)

// -----------------------------------------------------------------------------
// Section: Size helpers
// -----------------------------------------------------------------------------

static inline size_t pen_payload_size(uint8_t rawMsgType) {
    switch (pen_msg_base(rawMsgType)) {
        case MSG_DISCOVERY_REQ: return 0U;
        case MSG_DISCOVERY_RSP: return sizeof(pen_discovery_rsp_payload_t);
        case MSG_CONNECT_REQ:   return sizeof(pen_connect_req_payload_t);
        case MSG_CONNECT_RSP:   return sizeof(pen_connect_rsp_payload_t);
        case MSG_AUTH_REQ:      return sizeof(pen_auth_req_payload_t);
        case MSG_AUTH_RSP:      return sizeof(pen_auth_rsp_payload_t);
        case MSG_PING:          return sizeof(pen_ping_payload_t);
        case MSG_PONG:          return sizeof(pen_pong_payload_t);
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

static inline size_t pen_frame_size(uint8_t rawMsgType) {
    const size_t payloadSize = pen_payload_size(rawMsgType);
    if (payloadSize == SIZE_MAX) return SIZE_MAX;
    return sizeof(pen_hdr_t) + payloadSize + sizeof(pen_crc_t);
}

static inline bool pen_len_valid(uint8_t rawMsgType, size_t len) {
    const size_t frameSize = pen_frame_size(rawMsgType);
    return (frameSize != SIZE_MAX) && (len == frameSize);
}

// -----------------------------------------------------------------------------
// Section: CRC
// -----------------------------------------------------------------------------

static inline uint16_t pen_crc16(const void* data, size_t len) {
    if (data == nullptr) return 0U;
    return esp_crc16_le(PEN_CRC_INIT, reinterpret_cast<const uint8_t*>(data), (uint32_t)len);
}

static inline uint16_t pen_read_le16(const uint8_t* ptr) {
    return (uint16_t)ptr[0] | ((uint16_t)ptr[1] << 8);
}

static inline void pen_write_le16(uint8_t* ptr, uint16_t value) {
    ptr[0] = (uint8_t)(value & 0xFFU);
    ptr[1] = (uint8_t)((value >> 8) & 0xFFU);
}

static inline bool pen_crc_valid(const void* frame, size_t len) {
    if ((frame == nullptr) || (len < (sizeof(pen_hdr_t) + sizeof(pen_crc_t)))) return false;
    const uint8_t* bytes = reinterpret_cast<const uint8_t*>(frame);
    const size_t dataLen = len - sizeof(pen_crc_t);
    return pen_read_le16(&bytes[dataLen]) == pen_crc16(frame, dataLen);
}

static inline bool pen_frame_valid(const void* frame, size_t len) {
    if ((frame == nullptr) || (len < (sizeof(pen_hdr_t) + sizeof(pen_crc_t)))) return false;
    const pen_hdr_t* hdr = reinterpret_cast<const pen_hdr_t*>(frame);
    return (hdr->magic == PEN_MAGIC) && pen_len_valid(hdr->msgType, len) && pen_crc_valid(frame, len);
}

static inline bool pen_finalize_frame(void* frame, size_t len) {
    if ((frame == nullptr) || (len < (sizeof(pen_hdr_t) + sizeof(pen_crc_t)))) return false;
    pen_hdr_t* hdr = reinterpret_cast<pen_hdr_t*>(frame);
    if (!pen_len_valid(hdr->msgType, len)) return false;
    const size_t dataLen = len - sizeof(pen_crc_t);
    uint8_t* bytes = reinterpret_cast<uint8_t*>(frame);
    pen_write_le16(&bytes[dataLen], pen_crc16(frame, dataLen));
    return true;
}

// -----------------------------------------------------------------------------
// Section: Static size checks
// -----------------------------------------------------------------------------

static_assert(sizeof(pen_hdr_t) == 8U, "pen_hdr_t size changed");
static_assert(sizeof(pen_discovery_rsp_payload_t) == 21U, "pen_discovery_rsp_payload_t size changed");
static_assert(sizeof(pen_connect_req_payload_t) == 30U, "pen_connect_req_payload_t size changed");
static_assert(sizeof(pen_connect_rsp_payload_t) == 34U, "pen_connect_rsp_payload_t size changed");
static_assert(sizeof(pen_auth_req_payload_t) == 20U, "pen_auth_req_payload_t size changed");
static_assert(sizeof(pen_auth_rsp_payload_t) == 20U, "pen_auth_rsp_payload_t size changed");
static_assert(sizeof(pen_var_i_payload_t) == 10U, "pen_var_i_payload_t size changed");
static_assert(sizeof(pen_var_f_payload_t) == 10U, "pen_var_f_payload_t size changed");
static_assert(sizeof(pen_get_var_payload_t) == 4U, "pen_get_var_payload_t size changed");
static_assert(sizeof(pen_ack_payload_t) == 6U, "pen_ack_payload_t size changed");
static_assert(sizeof(pen_nack_payload_t) == 7U, "pen_nack_payload_t size changed");
static_assert(sizeof(pen_ping_payload_t) == 2U, "pen_ping_payload_t size changed");
static_assert(sizeof(pen_pong_payload_t) == 3U, "pen_pong_payload_t size changed");
static_assert((PEN_MSG_FLAG_RETRY & PEN_MSG_TYPE_MASK) == 0U, "retry flag overlaps type mask");

#endif // PEN_PROTO_H
