#pragma once
#include <Arduino.h>
#include <stdbool.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "pen_proto.h"

enum pec_rx_type_t : uint8_t {
    PEC_RX_LINK  = 1U,
    PEC_RX_ERROR = 2U,
    PEC_RX_VAR_I = 3U,
    PEC_RX_VAR_F = 4U,
    PEC_RX_ACK   = 5U,
    PEC_RX_NACK  = 6U
};

enum pec_link_code_t : uint8_t {
    PEC_LINK_READY     = 1U,
    PEC_LINK_DISC      = 2U,
    PEC_LINK_CONNECTED = 3U,
    PEC_LINK_AUTH_OK   = 4U,
    PEC_LINK_SECURE    = 5U,
    PEC_LINK_LOST      = 6U,
    PEC_LINK_CONN_TO   = 7U,
    PEC_LINK_AUTH_TO   = 8U,
    PEC_LINK_MAC_BAD   = 9U,
    PEC_LINK_AUTH_BAD  = 10U,
    PEC_LINK_SEC_BAD   = 11U
};

enum pec_hw_error_t : uint8_t {
    PEC_HW_ERR_NONE       = 0U,
    PEC_HW_ERR_EVENT_DROP = 1U,
    PEC_HW_ERR_BAD_FRAME  = 2U,
    PEC_HW_ERR_SEND       = 3U,
    PEC_HW_ERR_WIFI       = 4U,
    PEC_HW_ERR_ESPNOW     = 5U
};

struct pec_rx_event_t {
    uint8_t type;
    uint8_t msgType;
    union {
        struct {
            uint8_t code;
            int8_t rssi;
        } link;
        struct {
            uint8_t code;
            int32_t detail;
        } error;
        struct {
            uint16_t seq;
            uint16_t ttlMs;
            uint32_t varId;
            int32_t value;
        } varI;
        struct {
            uint16_t seq;
            uint16_t ttlMs;
            uint32_t varId;
            float value;
        } varF;
        struct {
            uint16_t seq;
            uint16_t ackSeq;
            uint32_t varId;
        } ack;
        struct {
            uint16_t seq;
            uint16_t ackSeq;
            uint32_t varId;
            uint8_t reason;
        } nack;
    } data;
};

typedef bool (*pec_rx_event_fn_t)(const pec_rx_event_t* ev);

static constexpr uint32_t PEC_VAR_JX   = PEN_VAR_ID2('J', 'X');
static constexpr uint32_t PEC_VAR_JY   = PEN_VAR_ID2('J', 'Y');
static constexpr uint32_t PEC_VAR_BATP = PEN_VAR_ID4('B', 'A', 'T', 'P');
static constexpr uint32_t PEC_VAR_LSET = PEN_VAR_ID4('L', 'S', 'E', 'T');
static constexpr uint32_t PEC_VAR_RSET = PEN_VAR_ID4('R', 'S', 'E', 'T');
static constexpr uint32_t PEC_VAR_USBC = PEN_VAR_ID4('U', 'S', 'B', 'C');
static constexpr uint32_t PEC_VAR_RSSI = PEN_VAR_ID4('R', 'S', 'S', 'I');
static constexpr uint32_t PEC_VAR_RSSL = PEN_VAR_ID4('R', 'S', 'S', 'L');

bool pec_begin(pec_rx_event_fn_t rxEventFn,
               BaseType_t coreId = 1,
               UBaseType_t priority = 2,
               uint32_t stackSize = 4096U);
bool pec_is_connected(void);

bool pec_send_get_var(uint32_t varId);
bool pec_send_stream_i(uint32_t varId, int32_t value, uint16_t ttlMs);
bool pec_send_stream_f(uint32_t varId, float value, uint16_t ttlMs);
bool pec_send_state_i(uint32_t varId, int32_t value, uint16_t ttlMs);
bool pec_send_state_f(uint32_t varId, float value, uint16_t ttlMs);
bool pec_send_event_i(uint32_t varId, int32_t value, uint16_t ttlMs);
bool pec_send_event_f(uint32_t varId, float value, uint16_t ttlMs);
