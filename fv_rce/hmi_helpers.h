#ifndef HMI_HELPERS_H
#define HMI_HELPERS_H

#include <stddef.h>
#include <stdint.h>

#include "hmi.h"

enum hmiq_cmd_type_t {
  HMIQ_CMD_PROGRESS = 0,
  HMIQ_CMD_INDICATOR,
  HMIQ_CMD_BEEP
};

typedef struct {
  uint8_t index;
} hmiq_cmd_config_t;

typedef union {
  struct {
    uint8_t value;
  } progress;
  struct {
    uint8_t value;
  } indicator;
  struct {
    uint16_t divider;
    uint16_t durationMs;
  } beep;
} hmiq_cmd_state_t;

struct hmiq_cmd_t;
typedef hmi_cmd_result_t (*hmiq_send_fn_t)(hmiq_cmd_t* cmd);

typedef struct hmiq_cmd_t {
  hmiq_cmd_type_t type;
  bool isLcd;
  bool hasData;
  hmiq_send_fn_t send;
  hmiq_cmd_config_t config;
  hmiq_cmd_state_t state;
} hmiq_cmd_t;

static inline hmi_cmd_result_t HmiqSendProgress(hmiq_cmd_t* cmd) {
  return hmi_cmd_lcd_set_progress(cmd->config.index, cmd->state.progress.value);
}

static inline hmi_cmd_result_t HmiqSendIndicator(hmiq_cmd_t* cmd) {
  return hmi_cmd_lcd_set_indicator(cmd->config.index, (cmd->state.indicator.value != 0U));
}

static inline hmi_cmd_result_t HmiqSendBeep(hmiq_cmd_t* cmd) {
  return hmi_cmd_play_tone(cmd->state.beep.divider, cmd->state.beep.durationMs);
}

#define HMIQ_PROGRESS(_index) { HMIQ_CMD_PROGRESS, true, false, HmiqSendProgress, { (uint8_t)(_index) }, { { 0U } } }
#define HMIQ_INDICATOR(_index) { HMIQ_CMD_INDICATOR, true, false, HmiqSendIndicator, { (uint8_t)(_index) }, { { 0U } } }
#define HMIQ_BEEP() { HMIQ_CMD_BEEP, false, false, HmiqSendBeep, { 0U }, { { 0U } } }

static inline bool HmiqIsLcdReady(void) {
  return (hmi_get(HMI_DATA_STAT_LCD_BUSY) == 0U);
}

static inline bool HmiqServiceOne(hmiq_cmd_t* const* cmds, size_t count) {
  if (cmds == NULL) return false;

  for (size_t i = 0U; i < count; ++i) {
    hmiq_cmd_t* cmd = cmds[i];
    if (cmd == NULL || !cmd->hasData) continue;
    if (cmd->isLcd && !HmiqIsLcdReady()) return false;

    const hmi_cmd_result_t rc = cmd->send(cmd);
    if ((rc == HMI_CMD_OK) || (rc == HMI_CMD_ERR_INVALID_ARG)) {
      cmd->hasData = false;
    }
    return true;
  }
  return false;
}

#endif  // HMI_HELPERS_H
