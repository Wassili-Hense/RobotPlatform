#ifndef HMI_HELPERS_H
#define HMI_HELPERS_H

#include <stddef.h>
#include <stdint.h>

#include "hmi.h"

enum hmis_cmd_type_t {
  HMIS_CMD_PROGRESS = 0,
  HMIS_CMD_INDICATOR,
  HMIS_CMD_BEEP
};

typedef struct {
  uint8_t index;
} hmis_cmd_config_t;

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
} hmis_cmd_state_t;

struct hmis_cmd_t;
typedef hmi_cmd_result_t (*hmis_send_fn_t)(struct hmis_cmd_t* cmd);

typedef struct hmis_cmd_t {
  hmis_cmd_type_t type;
  bool isLcd;
  bool hasData;
  hmis_send_fn_t send;
  hmis_cmd_config_t config;
  hmis_cmd_state_t state;
} hmis_cmd_t;

static inline hmi_cmd_result_t HmisSendProgress(hmis_cmd_t* cmd) {
  return hmi_cmd_lcd_set_progress(cmd->config.index, cmd->state.progress.value);
}

static inline hmi_cmd_result_t HmisSendIndicator(hmis_cmd_t* cmd) {
  return hmi_cmd_lcd_set_indicator(cmd->config.index, (cmd->state.indicator.value != 0U));
}

static inline hmi_cmd_result_t HmisSendBeep(hmis_cmd_t* cmd) {
  return hmi_cmd_play_tone(cmd->state.beep.divider, cmd->state.beep.durationMs);
}

#define HMIS_PROGRESS(_index)  { HMIS_CMD_PROGRESS,  true,  false, HmisSendProgress,  { (uint8_t)(_index) }, { { 0U } } }
#define HMIS_INDICATOR(_index) { HMIS_CMD_INDICATOR, true,  false, HmisSendIndicator, { (uint8_t)(_index) }, { { 0U } } }
#define HMIS_BEEP()            { HMIS_CMD_BEEP,      false, false, HmisSendBeep,      { 0U },               { { 0U } } }

static inline bool HmisIsLcdReady(void) {
  return (hmi_get(HMI_DATA_STAT_LCD_BUSY) == 0U);
}

static inline bool HmisService(hmis_cmd_t* const* cmds, size_t count) {
  if (cmds == NULL) return false;

  for (size_t i = 0U; i < count; ++i) {
    hmis_cmd_t* cmd = cmds[i];
    if (cmd == NULL || !cmd->hasData) continue;
    if (cmd->isLcd && !HmisIsLcdReady()) return false;

    const hmi_cmd_result_t rc = cmd->send(cmd);
    if ((rc == HMI_CMD_OK) || (rc == HMI_CMD_ERR_INVALID_ARG)) {
      cmd->hasData = false;
    }
    return true;
  }
  return false;
}

typedef enum {
  HMIG_TYPE_CLS = 0,
  HMIG_TYPE_J_VIEW
} hmig_type_t;

typedef enum {
  HMIG_CALL_ENTER = 0,
  HMIG_CALL_PROCESS,
  HMIG_CALL_PROCESS_AND_SEND,
  HMIG_CALL_EXIT
} hmig_call_t;

typedef enum {
  HMIG_J_VIEW_PHASE_IDLE = 0,
  HMIG_J_VIEW_PHASE_ERASE,
  HMIG_J_VIEW_PHASE_DRAW
} hmig_j_view_phase_t;

typedef struct {
  uint16_t color;
  bool pending;
} hmig_cls_t;

typedef struct {
  uint8_t currentX;
  uint8_t currentY;
  uint8_t nextX;
  uint8_t nextY;
  bool visible;
  hmig_j_view_phase_t phase;
} hmig_j_view_t;

typedef union {
  hmig_cls_t cls;
  hmig_j_view_t jView;
} hmig_state_t;

struct hmig_component_t;
typedef bool (*hmig_handle_fn_t)(struct hmig_component_t* component, hmig_call_t call);

typedef struct hmig_component_t {
  hmig_type_t type;
  hmig_handle_fn_t handle;
  hmig_state_t state;
} hmig_component_t;

typedef struct {
  hmig_component_t* components;
  size_t componentCount;
} hmig_scene_t;

static inline bool HmigHandleCls(hmig_component_t* component, hmig_call_t call);
static inline bool HmigHandleJView(hmig_component_t* component, hmig_call_t call);
static inline hmig_component_t HmigMakeCls(uint16_t color);
static inline hmig_component_t HmigMakeJView(void);

#define HMIG_CLS(_color) HmigMakeCls((uint16_t)(_color))
#define HMIG_J_VIEW()    HmigMakeJView()
#define HMIG_SCENE(_componentsArray)   { (_componentsArray), sizeof((_componentsArray)) / sizeof((_componentsArray)[0]) }

static inline hmig_component_t HmigMakeCls(uint16_t color) {
  hmig_component_t component;
  component.type = HMIG_TYPE_CLS;
  component.handle = HmigHandleCls;
  component.state.cls.color = color;
  component.state.cls.pending = false;
  return component;
}

static inline hmig_component_t HmigMakeJView(void) {
  hmig_component_t component;
  component.type = HMIG_TYPE_J_VIEW;
  component.handle = HmigHandleJView;
  component.state.jView.currentX = 0U;
  component.state.jView.currentY = 0U;
  component.state.jView.nextX = 0U;
  component.state.jView.nextY = 0U;
  component.state.jView.visible = false;
  component.state.jView.phase = HMIG_J_VIEW_PHASE_IDLE;
  return component;
}

static inline uint8_t HmigMapAxis(uint16_t value, uint8_t outMin, uint8_t outMax) {
  if (outMax <= outMin) return outMin;
  if (value > 4095U) value = 4095U;
  return (uint8_t)(outMin + (((uint32_t)value * (uint32_t)(outMax - outMin)) / 4095U));
}

static inline void HmigEnter(hmig_scene_t* scene) {
  if ((scene == NULL) || (scene->components == NULL)) return;

  for (size_t i = 0U; i < scene->componentCount; ++i) {
    hmig_component_t* component = &scene->components[i];
    if (component->handle != NULL) {
      (void)component->handle(component, HMIG_CALL_ENTER);
    }
  }
}

static inline void HmigLeave(hmig_scene_t* scene) {
  if ((scene == NULL) || (scene->components == NULL)) return;

  for (size_t i = 0U; i < scene->componentCount; ++i) {
    hmig_component_t* component = &scene->components[i];
    if (component->handle != NULL) {
      (void)component->handle(component, HMIG_CALL_EXIT);
    }
  }
}

static inline bool HmigUpdateJView(hmig_j_view_t* state) {
  if (state == NULL) return false;

  const bool backlightOn = (hmi_get(HMI_DATA_STAT_BL_ON) != 0U);
  if (!backlightOn) {
    if (state->visible) {
      state->phase = HMIG_J_VIEW_PHASE_ERASE;
      return true;
    }
    state->phase = HMIG_J_VIEW_PHASE_IDLE;
    return false;
  }

  state->nextX = HmigMapAxis(hmi_get(HMI_DATA_JOY_X), 45U, 114U);
  state->nextY = HmigMapAxis(hmi_get(HMI_DATA_JOY_Y), 10U, 79U);

  if (state->phase == HMIG_J_VIEW_PHASE_DRAW) {
    return true;
  }

  if (state->phase == HMIG_J_VIEW_PHASE_ERASE) {
    return true;
  }

  if (!state->visible) {
    state->phase = HMIG_J_VIEW_PHASE_DRAW;
    return true;
  }

  if ((state->currentX == state->nextX) && (state->currentY == state->nextY)) {
    state->phase = HMIG_J_VIEW_PHASE_IDLE;
    return false;
  }

  state->phase = HMIG_J_VIEW_PHASE_ERASE;
  return true;
}

static inline bool HmigHandleCls(hmig_component_t* component, hmig_call_t call) {
  if (component == NULL) return false;

  hmig_cls_t* const state = &component->state.cls;
  switch (call) {
    case HMIG_CALL_ENTER:
      state->pending = true;
      return false;

    case HMIG_CALL_PROCESS:
      return false;

    case HMIG_CALL_PROCESS_AND_SEND:
      if (!state->pending) return false;
      if (!HmisIsLcdReady()) return false;
      {
        const hmi_cmd_result_t rc = hmi_cmd_lcd_clear(state->color);
        if ((rc == HMI_CMD_OK) || (rc == HMI_CMD_ERR_INVALID_ARG)) {
          state->pending = false;
        }
      }
      return true;

    case HMIG_CALL_EXIT:
      state->pending = false;
      return false;

    default:
      return false;
  }
}

static inline bool HmigHandleJView(hmig_component_t* component, hmig_call_t call) {
  if (component == NULL) return false;

  hmig_j_view_t* const state = &component->state.jView;
  switch (call) {
    case HMIG_CALL_ENTER:
      state->currentX = 0U;
      state->currentY = 0U;
      state->nextX = 0U;
      state->nextY = 0U;
      state->visible = false;
      state->phase = HMIG_J_VIEW_PHASE_IDLE;
      return false;

    case HMIG_CALL_PROCESS:
      (void)HmigUpdateJView(state);
      return false;

    case HMIG_CALL_PROCESS_AND_SEND:
      (void)HmigUpdateJView(state);
      if (state->phase == HMIG_J_VIEW_PHASE_IDLE) return false;
      if (!HmisIsLcdReady()) return false;

      if (state->phase == HMIG_J_VIEW_PHASE_ERASE) {
        const hmi_cmd_result_t rc = hmi_cmd_lcd_draw_marker(state->currentX, state->currentY, 3U, 0x0000U);
        if ((rc == HMI_CMD_OK) || (rc == HMI_CMD_ERR_INVALID_ARG)) {
          state->visible = false;
          if ((hmi_get(HMI_DATA_STAT_BL_ON) != 0U) &&
              ((state->currentX != state->nextX) || (state->currentY != state->nextY))) {
            state->phase = HMIG_J_VIEW_PHASE_DRAW;
          } else {
            state->phase = HMIG_J_VIEW_PHASE_IDLE;
          }
        }
        return true;
      }

      if (state->phase == HMIG_J_VIEW_PHASE_DRAW) {
        const hmi_cmd_result_t rc = hmi_cmd_lcd_draw_marker(state->nextX, state->nextY, 3U, 0xFFFFU);
        if ((rc == HMI_CMD_OK) || (rc == HMI_CMD_ERR_INVALID_ARG)) {
          state->currentX = state->nextX;
          state->currentY = state->nextY;
          state->visible = true;
          state->phase = HMIG_J_VIEW_PHASE_IDLE;
        }
        return true;
      }
      return false;

    case HMIG_CALL_EXIT:
      if (!state->visible) {
        state->phase = HMIG_J_VIEW_PHASE_IDLE;
        return false;
      }
      if (!HmisIsLcdReady()) return false;
      {
        const hmi_cmd_result_t rc = hmi_cmd_lcd_draw_marker(state->currentX, state->currentY, 3U, 0x0000U);
        if ((rc == HMI_CMD_OK) || (rc == HMI_CMD_ERR_INVALID_ARG)) {
          state->visible = false;
          state->phase = HMIG_J_VIEW_PHASE_IDLE;
        }
      }
      return true;

    default:
      return false;
  }
}

static inline bool HmigService(hmig_scene_t* scene) {
  if ((scene == NULL) || (scene->components == NULL)) return false;

  bool sent = false;
  for (size_t i = 0U; i < scene->componentCount; ++i) {
    hmig_component_t* component = &scene->components[i];
    if ((component == NULL) || (component->handle == NULL)) continue;

    const hmig_call_t call = sent ? HMIG_CALL_PROCESS : HMIG_CALL_PROCESS_AND_SEND;
    if (component->handle(component, call)) {
      sent = true;
    }
  }
  return sent;
}

#endif  // HMI_HELPERS_H
