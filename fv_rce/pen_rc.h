#pragma once
#include "pen_comm.h"

static constexpr uint32_t PEN_VAR_RSSL = PEN_VAR_ID4('R', 'S', 'S', 'L');

bool pen_send_get_var(uint32_t varId);
