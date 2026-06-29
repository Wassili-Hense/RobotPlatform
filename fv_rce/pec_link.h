#pragma once

#include <Arduino.h>
#include <stdbool.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "pen_proto.h"

typedef bool (*pec_line_fn_t)(const char* text);

static constexpr uint32_t PEC_VAR_JX   = PEN_VAR_ID2('J', 'X');
static constexpr uint32_t PEC_VAR_JY   = PEN_VAR_ID2('J', 'Y');
static constexpr uint32_t PEC_VAR_BATP = PEN_VAR_ID4('B', 'A', 'T', 'P');
static constexpr uint32_t PEC_VAR_LSET = PEN_VAR_ID4('L', 'S', 'E', 'T');
static constexpr uint32_t PEC_VAR_RSET = PEN_VAR_ID4('R', 'S', 'E', 'T');
static constexpr uint32_t PEC_VAR_USBC = PEN_VAR_ID4('U', 'S', 'B', 'C');

enum pec_hw_error_t : uint8_t {
    PEC_HW_ERR_NONE       = 0U,
    PEC_HW_ERR_EVENT_DROP = 1U,
    PEC_HW_ERR_BAD_FRAME  = 2U,
    PEC_HW_ERR_SEND       = 3U,
    PEC_HW_ERR_WIFI       = 4U,
    PEC_HW_ERR_ESPNOW     = 5U
};

struct pec_status_t {
    bool connected;
    uint8_t battery;
    int8_t peerRssi;
    int8_t localRssi;
    uint8_t errorCode;
    int32_t errorDetail;
};

bool pec_begin(pec_line_fn_t lineFn,
               BaseType_t coreId = 1,
               UBaseType_t priority = 2,
               uint32_t stackSize = 4096U);

bool pec_is_connected(void);
void pec_get_status(pec_status_t* outStatus);
bool pec_take_status(pec_status_t* outStatus);

bool pec_pc_rx_line(const char* line);

bool pec_send_get_var(uint32_t varId);
bool pec_send_stream_i(uint32_t varId, int32_t value, uint16_t ttlMs);
bool pec_send_stream_f(uint32_t varId, float value, uint16_t ttlMs);
bool pec_send_state_i(uint32_t varId, int32_t value, uint16_t ttlMs);
bool pec_send_state_f(uint32_t varId, float value, uint16_t ttlMs);
bool pec_send_event_i(uint32_t varId, int32_t value, uint16_t ttlMs);
bool pec_send_event_f(uint32_t varId, float value, uint16_t ttlMs);
