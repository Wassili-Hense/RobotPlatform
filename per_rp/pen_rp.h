#pragma once
#include "pen_comm.h"

typedef void (*pen_rp_event_i_cb_t)(uint32_t varId, int32_t value, bool retry);
typedef void (*pen_rp_event_f_cb_t)(uint32_t varId, float value, bool retry);

bool pen_rp_register_stream(uint32_t varId, int32_t* value);
bool pen_rp_register_stream(uint32_t varId, float* value);

bool pen_rp_register_state(uint32_t varId, int32_t* value);
bool pen_rp_register_state(uint32_t varId, float* value);

bool pen_rp_register_event(uint32_t varId, pen_rp_event_i_cb_t cb);
bool pen_rp_register_event(uint32_t varId, pen_rp_event_f_cb_t cb);

bool pen_rp_unregister(uint32_t varId);
