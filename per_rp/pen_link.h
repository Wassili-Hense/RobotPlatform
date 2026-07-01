#pragma once
#include <Arduino.h>
#include <stdbool.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"

#ifndef PEN_RC
#define PEN_RC 0
#endif
#ifndef PEN_LINK_CORE_ID
#define PEN_LINK_CORE_ID 1
#endif
#ifndef PEN_LINK_TASK_PRIORITY
#define PEN_LINK_TASK_PRIORITY 2
#endif
#ifndef PEN_LINK_RX_TASK_PRIORITY
#define PEN_LINK_RX_TASK_PRIORITY 1
#endif
#ifndef PEN_LINK_TASK_STACK
#define PEN_LINK_TASK_STACK 4096U
#endif
#ifndef PEN_LINK_RX_TASK_STACK
#define PEN_LINK_RX_TASK_STACK 4096U
#endif

#define PEN_VAR_ID4(a, b, c, d) \
    ((uint32_t)(uint8_t)(a)        | \
    ((uint32_t)(uint8_t)(b) << 8)  | \
    ((uint32_t)(uint8_t)(c) << 16) | \
    ((uint32_t)(uint8_t)(d) << 24))
#define PEN_VAR_ID3(a, b, c) PEN_VAR_ID4((a), (b), (c), '\0')
#define PEN_VAR_ID2(a, b)    PEN_VAR_ID4((a), (b), '\0', '\0')
#define PEN_VAR_ID1(a)       PEN_VAR_ID4((a), '\0', '\0', '\0')

enum pen_rx_type_t : uint8_t { PEN_RX_LINK = 1U, PEN_RX_ERROR = 2U, PEN_RX_VAR_I = 3U, PEN_RX_VAR_F = 4U, PEN_RX_ACK = 5U, PEN_RX_NACK = 6U, PEN_RX_GET_VAR = 7U };
enum pen_link_code_t : uint8_t { PEN_LINK_READY = 1U, PEN_LINK_DISC = 2U, PEN_LINK_CONNECTED = 3U, PEN_LINK_AUTH_OK = 4U, PEN_LINK_SECURE = 5U, PEN_LINK_LOST = 6U, PEN_LINK_CONN_TO = 7U, PEN_LINK_AUTH_TO = 8U, PEN_LINK_MAC_BAD = 9U, PEN_LINK_AUTH_BAD = 10U, PEN_LINK_SEC_BAD = 11U };
enum pen_hw_error_t : uint8_t { PEN_HW_ERR_NONE = 0U, PEN_HW_ERR_EVENT_DROP = 1U, PEN_HW_ERR_BAD_FRAME = 2U, PEN_HW_ERR_SEND = 3U, PEN_HW_ERR_WIFI = 4U, PEN_HW_ERR_ESPNOW = 5U, PEN_HW_ERR_RETRY_FULL = 6U, PEN_HW_ERR_ACK_TIMEOUT = 7U, PEN_HW_ERR_RX_DROP = 8U };

struct pen_rx_event_t {
    uint8_t type;
    uint8_t msgType;
    union {
        struct { uint8_t code; int8_t rssi; } link;
        struct { uint8_t code; int32_t detail; } error;
        struct { uint16_t seq; uint16_t ttlMs; uint32_t varId; int32_t value; } varI;
        struct { uint16_t seq; uint16_t ttlMs; uint32_t varId; float value; } varF;
        struct { uint16_t seq; uint16_t ackSeq; uint32_t varId; } ack;
        struct { uint16_t seq; uint16_t ackSeq; uint32_t varId; uint8_t reason; } nack;
        struct { uint16_t seq; uint32_t varId; } getVar;
    } data;
};

typedef bool (*pen_rx_event_fn_t)(const pen_rx_event_t* ev);

static constexpr uint32_t PEN_VAR_RSSI = PEN_VAR_ID4('R', 'S', 'S', 'I');
#if PEN_RC
static constexpr uint32_t PEN_VAR_RSSL = PEN_VAR_ID4('R', 'S', 'S', 'L');
#endif

bool pen_begin(pen_rx_event_fn_t rxEventFn);
bool pen_is_connected(void);
#if PEN_RC
bool pen_send_get_var(uint32_t varId);
#endif
bool pen_send_stream(uint32_t varId, int32_t value, uint16_t ttlMs);
bool pen_send_stream(uint32_t varId, float value, uint16_t ttlMs);
bool pen_send_state(uint32_t varId, int32_t value);
bool pen_send_state(uint32_t varId, float value);
bool pen_send_event(uint32_t varId, int32_t value, uint16_t ttlMs);
bool pen_send_event(uint32_t varId, float value, uint16_t ttlMs);
