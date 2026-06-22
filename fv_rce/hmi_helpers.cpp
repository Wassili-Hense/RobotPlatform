#include "hmi_helpers.h"

hmi_cmd_result_t HmisSendProgress(hmis_cmd_t* cmd) {
  return hmi_cmd_lcd_set_progress(cmd->config.index, cmd->state.progress.value);
}

hmi_cmd_result_t HmisSendIndicator(hmis_cmd_t* cmd) {
  return hmi_cmd_lcd_set_indicator(cmd->config.index, (cmd->state.indicator.value != 0U));
}

hmi_cmd_result_t HmisSendBeep(hmis_cmd_t* cmd) {
  return hmi_cmd_play_tone(cmd->state.beep.divider, cmd->state.beep.durationMs);
}

bool HmisIsLcdReady(void) {
  return (hmi_get(HMI_DATA_STAT_LCD_BUSY) == 0U);
}

bool HmisService(hmis_cmd_t* const* cmds, size_t count) {
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

HmigClsComponent::HmigClsComponent(uint16_t color)
  : m_color(color),
    m_pending(false) {
}

bool HmigClsComponent::Handle(hmig_call_t call) {
  switch (call) {
    case HMIG_CALL_ENTER:
      m_pending = true;
      return false;

    case HMIG_CALL_PROCESS:
      return false;

    case HMIG_CALL_PROCESS_AND_SEND:
      if (!m_pending) return false;
      if (!HmisIsLcdReady()) return false;
      {
        const hmi_cmd_result_t rc = hmi_cmd_lcd_clear(m_color);
        if ((rc == HMI_CMD_OK) || (rc == HMI_CMD_ERR_INVALID_ARG)) {
          m_pending = false;
        }
      }
      return true;

    case HMIG_CALL_EXIT:
      m_pending = false;
      return false;

    default:
      return false;
  }
}

HmigJViewComponent::HmigJViewComponent()
  : m_currentX(0U),
    m_currentY(0U),
    m_nextX(0U),
    m_nextY(0U),
    m_visible(false),
    m_phase(HMIG_J_VIEW_PHASE_IDLE) {
}

bool HmigJViewComponent::Update() {
  const bool backlightOn = (hmi_get(HMI_DATA_STAT_BL_ON) != 0U);
  if (!backlightOn) {
    if (m_visible) {
      m_phase = HMIG_J_VIEW_PHASE_ERASE;
      return true;
    }
    m_phase = HMIG_J_VIEW_PHASE_IDLE;
    return false;
  }

  m_nextX = HmigMapAxis(hmi_get(HMI_DATA_JOY_X), 45U, 114U);
  m_nextY = HmigMapAxis(hmi_get(HMI_DATA_JOY_Y), 10U, 79U);

  if (m_phase == HMIG_J_VIEW_PHASE_DRAW) {
    return true;
  }
  if (m_phase == HMIG_J_VIEW_PHASE_ERASE) {
    return true;
  }
  if (!m_visible) {
    m_phase = HMIG_J_VIEW_PHASE_DRAW;
    return true;
  }
  if ((m_currentX == m_nextX) && (m_currentY == m_nextY)) {
    m_phase = HMIG_J_VIEW_PHASE_IDLE;
    return false;
  }

  m_phase = HMIG_J_VIEW_PHASE_ERASE;
  return true;
}

bool HmigJViewComponent::Handle(hmig_call_t call) {
  switch (call) {
    case HMIG_CALL_ENTER:
      m_currentX = 0U;
      m_currentY = 0U;
      m_nextX = 0U;
      m_nextY = 0U;
      m_visible = false;
      m_phase = HMIG_J_VIEW_PHASE_IDLE;
      return false;

    case HMIG_CALL_PROCESS:
      (void)Update();
      return false;

    case HMIG_CALL_PROCESS_AND_SEND:
      (void)Update();
      if (m_phase == HMIG_J_VIEW_PHASE_IDLE) return false;
      if (!HmisIsLcdReady()) return false;

      if (m_phase == HMIG_J_VIEW_PHASE_ERASE) {
        const hmi_cmd_result_t rc = hmi_cmd_lcd_draw_marker(m_currentX, m_currentY, 3U, 0x0000U);
        if ((rc == HMI_CMD_OK) || (rc == HMI_CMD_ERR_INVALID_ARG)) {
          m_visible = false;
          if ((hmi_get(HMI_DATA_STAT_BL_ON) != 0U) &&
              ((m_currentX != m_nextX) || (m_currentY != m_nextY))) {
            m_phase = HMIG_J_VIEW_PHASE_DRAW;
          } else {
            m_phase = HMIG_J_VIEW_PHASE_IDLE;
          }
        }
        return true;
      }

      if (m_phase == HMIG_J_VIEW_PHASE_DRAW) {
        const hmi_cmd_result_t rc = hmi_cmd_lcd_draw_marker(m_nextX, m_nextY, 3U, 0xFFFFU);
        if ((rc == HMI_CMD_OK) || (rc == HMI_CMD_ERR_INVALID_ARG)) {
          m_currentX = m_nextX;
          m_currentY = m_nextY;
          m_visible = true;
          m_phase = HMIG_J_VIEW_PHASE_IDLE;
        }
        return true;
      }

      return false;

    case HMIG_CALL_EXIT:
      if (!m_visible) {
        m_phase = HMIG_J_VIEW_PHASE_IDLE;
        return false;
      }
      if (!HmisIsLcdReady()) return false;
      {
        const hmi_cmd_result_t rc = hmi_cmd_lcd_draw_marker(m_currentX, m_currentY, 3U, 0x0000U);
        if ((rc == HMI_CMD_OK) || (rc == HMI_CMD_ERR_INVALID_ARG)) {
          m_visible = false;
          m_phase = HMIG_J_VIEW_PHASE_IDLE;
        }
      }
      return true;

    default:
      return false;
  }
}

uint8_t HmigMapAxis(uint16_t value, uint8_t outMin, uint8_t outMax) {
  if (outMax <= outMin) return outMin;
  if (value > 4095U) value = 4095U;
  return (uint8_t)(outMin + (((uint32_t)value * (uint32_t)(outMax - outMin)) / 4095U));
}

void HmigEnter(hmig_scene_t* scene) {
  if ((scene == NULL) || (scene->components == NULL)) return;
  for (size_t i = 0U; i < scene->componentCount; ++i) {
    HmigComponent* component = scene->components[i];
    if (component != NULL) {
      (void)component->Handle(HMIG_CALL_ENTER);
    }
  }
}

void HmigLeave(hmig_scene_t* scene) {
  if ((scene == NULL) || (scene->components == NULL)) return;
  for (size_t i = 0U; i < scene->componentCount; ++i) {
    HmigComponent* component = scene->components[i];
    if (component != NULL) {
      (void)component->Handle(HMIG_CALL_EXIT);
    }
  }
}

bool HmigService(hmig_scene_t* scene) {
  if ((scene == NULL) || (scene->components == NULL)) return false;
  bool sent = false;
  for (size_t i = 0U; i < scene->componentCount; ++i) {
    HmigComponent* component = scene->components[i];
    if (component == NULL) continue;
    const hmig_call_t call = sent ? HMIG_CALL_PROCESS : HMIG_CALL_PROCESS_AND_SEND;
    if (component->Handle(call)) {
      sent = true;
    }
  }
  return sent;
}
