#ifndef PEN_PROTO_H
#define PEN_PROTO_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

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
// -----------------------------------------------------------------------------

static constexpr uint8_t  PEN_MAGIC = 0xA7U;
static constexpr uint8_t  PEN_MSG_FLAG_RETRY = 0x80U;
static constexpr uint8_t  PEN_MSG_TYPE_MASK  = 0x7FU;
static constexpr uint16_t PEN_CRC_INIT = 0xFFFFU;
static constexpr uint8_t  PEN_CHANNEL = 1U;
static constexpr uint32_t PEN_CAPS = 0x00000001U;
static constexpr char PEN_BIND_SECRET[] = "PEN-DEMO-BIND-SECRET";
static constexpr uint8_t PEN_BROADCAST_MAC[6] = { 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU };


enum proto_msg_type_t : uint8_t {
    MSG_DISCOVERY_REQ = 0x02U,
    MSG_DISCOVERY_RSP = 0x03U,
    MSG_CONNECT_REQ   = 0x04U,
    MSG_CONNECT_RSP   = 0x05U,
    MSG_AUTH_REQ      = 0x06U,
    MSG_AUTH_RSP      = 0x07U,
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

#endif
