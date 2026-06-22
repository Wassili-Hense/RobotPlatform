#include "hmi_helpers.h"

#include <stdio.h>

static hmig_scene_t* s_hmigActiveScene = nullptr;

static void HmigSceneEnter(hmig_scene_t* scene) {
  if ((scene == NULL) || (scene->components == NULL)) return;
  for (size_t i = 0U; i < scene->componentCount; ++i) {
    HmigComponent* component = scene->components[i];
    if (component != NULL) {
      (void)component->Handle(HMIG_CALL_ENTER);
    }
  }
}

static void HmigSceneLeave(hmig_scene_t* scene) {
  if ((scene == NULL) || (scene->components == NULL)) return;
  for (size_t i = 0U; i < scene->componentCount; ++i) {
    HmigComponent* component = scene->components[i];
    if (component != NULL) {
      (void)component->Handle(HMIG_CALL_EXIT);
    }
  }
}

static int HmigFindActiveMenuIndex(const hmig_scene_t* scene) {
  if ((scene == NULL) || (scene->components == NULL)) return -1;
  for (size_t i = 0U; i < scene->componentCount; ++i) {
    const HmigComponent* component = scene->components[i];
    if ((component != NULL) && component->IsMenuItem() && component->IsMenuActive()) {
      return (int)i;
    }
  }
  return -1;
}

static int HmigFindFirstMenuIndex(const hmig_scene_t* scene) {
  if ((scene == NULL) || (scene->components == NULL)) return -1;
  for (size_t i = 0U; i < scene->componentCount; ++i) {
    const HmigComponent* component = scene->components[i];
    if ((component != NULL) && component->IsMenuItem()) {
      return (int)i;
    }
  }
  return -1;
}

static int HmigFindAdjacentMenuIndex(const hmig_scene_t* scene, int startIndex, int step) {
  if ((scene == NULL) || (scene->components == NULL) || (step == 0)) return -1;
  const int count = (int)scene->componentCount;
  int index = startIndex;
  for (int n = 0; n < count; ++n) {
    index += step;
    if (index < 0) {
      index = count - 1;
    } else if (index >= count) {
      index = 0;
    }
    const HmigComponent* component = scene->components[index];
    if ((component != NULL) && component->IsMenuItem()) {
      return index;
    }
  }
  return -1;
}

static void HmigActivateMenuIndex(hmig_scene_t* scene, int menuIndex) {
  if ((scene == NULL) || (scene->components == NULL)) return;
  for (size_t i = 0U; i < scene->componentCount; ++i) {
    HmigComponent* component = scene->components[i];
    if ((component != NULL) && component->IsMenuItem()) {
      component->SetMenuActive(((int)i) == menuIndex);
    }
  }
}

static void HmigEnsureMenuSelection(hmig_scene_t* scene) {
  if ((scene == NULL) || (scene->components == NULL)) return;
  if (HmigFindActiveMenuIndex(scene) >= 0) return;
  const int firstIndex = HmigFindFirstMenuIndex(scene);
  if (firstIndex >= 0) {
    HmigActivateMenuIndex(scene, firstIndex);
  }
}

static bool HmigMoveMenuSelection(hmig_scene_t* scene, int step) {
  if ((scene == NULL) || (scene->components == NULL)) return false;
  HmigEnsureMenuSelection(scene);
  const int activeIndex = HmigFindActiveMenuIndex(scene);
  if (activeIndex < 0) return false;
  const int nextIndex = HmigFindAdjacentMenuIndex(scene, activeIndex, step);
  if ((nextIndex < 0) || (nextIndex == activeIndex)) return false;
  HmigActivateMenuIndex(scene, nextIndex);
  return true;
}

static bool HmigHandleMenuNavigation(hmig_scene_t* scene) {
  if ((scene == NULL) || (scene->components == NULL)) return false;
  HmigEnsureMenuSelection(scene);

  if (hmi_changed(HMI_DATA_BTN_UP) && (hmi_get(HMI_DATA_BTN_UP) != 0U)) {
    (void)HmigMoveMenuSelection(scene, -1);
  }
  if (hmi_changed(HMI_DATA_BTN_DOWN) && (hmi_get(HMI_DATA_BTN_DOWN) != 0U)) {
    (void)HmigMoveMenuSelection(scene, 1);
  }
  if (hmi_changed(HMI_DATA_BTN_OK) && (hmi_get(HMI_DATA_BTN_OK) != 0U)) {
    const int activeIndex = HmigFindActiveMenuIndex(scene);
    if (activeIndex >= 0) {
      HmigComponent* component = scene->components[activeIndex];
      hmig_scene_t* const targetScene = (component != NULL) ? component->GetMenuTargetScene() : NULL;
      if (targetScene != NULL) {
        HmigSwitchScene(targetScene);
        return true;
      }
    }
  }
  return false;
}

static bool HmigSceneService(hmig_scene_t* scene) {
  if ((scene == NULL) || (scene->components == NULL)) return false;
  if (HmigHandleMenuNavigation(scene)) return false;
  if (s_hmigActiveScene != scene) return false;

  bool sent = false;
  for (size_t i = 0U; i < scene->componentCount; ++i) {
    HmigComponent* component = scene->components[i];
    if (component == NULL) continue;
    const hmig_call_t call = sent ? HMIG_CALL_PROCESS : HMIG_CALL_PROCESS_AND_SEND;
    if (component->Handle(call)) {
      if (s_hmigActiveScene != scene) return false;
      sent = true;
    } else if (s_hmigActiveScene != scene) {
      return false;
    }
  }
  return sent;
}

hmi_cmd_result_t HmisSendProgress(hmis_cmd_t* cmd) {
  return hmi_cmd_lcd_set_progress(cmd->config.index, cmd->state.progress.value);
}

hmi_cmd_result_t HmisSendIndicator(hmis_cmd_t* cmd) {
  return hmi_cmd_lcd_set_indicator(cmd->config.index, (cmd->state.indicator.value != 0U));
}

hmi_cmd_result_t HmisSendBeep(hmis_cmd_t* cmd) {
  return hmi_cmd_play_tone(cmd->state.beep.divider, cmd->state.beep.durationMs);
}

hmi_cmd_result_t HmisSendBrightness(hmis_cmd_t* cmd) {
  return hmi_cmd_set_brightness(cmd->state.brightness.level);
}

hmi_cmd_result_t HmisSendBlTimeout(hmis_cmd_t* cmd) {
  return hmi_cmd_set_backlight_timeout(cmd->state.blTimeout.timeoutMs);
}

bool HmisIsLcdReady(void) {
  return (hmi_get(HMI_DATA_STAT_LCD_BUSY) == 0U);
}

bool HmisService(hmis_cmd_t* const* cmds, size_t count) {
  if (cmds == NULL) return false;
  for (size_t i = 0U; i < count; ++i) {
    hmis_cmd_t* cmd = cmds[i];
    if ((cmd == NULL) || !cmd->hasData) continue;
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
      if (!m_pending || !HmisIsLcdReady()) return false;
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
  if ((m_phase == HMIG_J_VIEW_PHASE_DRAW) || (m_phase == HMIG_J_VIEW_PHASE_ERASE)) return true;
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
      if ((m_phase == HMIG_J_VIEW_PHASE_IDLE) || !HmisIsLcdReady()) return false;
      if (m_phase == HMIG_J_VIEW_PHASE_ERASE) {
        const hmi_cmd_result_t rc = hmi_cmd_lcd_draw_marker(m_currentX, m_currentY, 3U, 0x0000U);
        if ((rc == HMI_CMD_OK) || (rc == HMI_CMD_ERR_INVALID_ARG)) {
          m_visible = false;
          if ((hmi_get(HMI_DATA_STAT_BL_ON) != 0U) && ((m_currentX != m_nextX) || (m_currentY != m_nextY))) {
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
      if (!m_visible || !HmisIsLcdReady()) {
        if (!m_visible) m_phase = HMIG_J_VIEW_PHASE_IDLE;
        return false;
      }
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

HmigHotKeyComponent::HmigHotKeyComponent(hmi_data_idx_t idx, hmig_scene_t* targetScene)
  : m_idx(idx),
    m_targetScene(targetScene) {
}

bool HmigHotKeyComponent::Handle(hmig_call_t call) {
  switch (call) {
    case HMIG_CALL_ENTER:
      return false;
    case HMIG_CALL_PROCESS:
    case HMIG_CALL_PROCESS_AND_SEND:
      if (hmi_changed(m_idx) && (hmi_get(m_idx) != 0U) && (m_targetScene != NULL)) {
        HmigSwitchScene(m_targetScene);
      }
      return false;
    case HMIG_CALL_EXIT:
      return false;
    default:
      return false;
  }
}

HmigLabelComponent::HmigLabelComponent(uint8_t x, uint8_t y, uint16_t color, const char* text)
  : m_x(x),
    m_y(y),
    m_color(color),
    m_text(text),
    m_pending(false) {
}

bool HmigLabelComponent::Handle(hmig_call_t call) {
  switch (call) {
    case HMIG_CALL_ENTER:
      m_pending = true;
      return false;
    case HMIG_CALL_PROCESS:
      return false;
    case HMIG_CALL_PROCESS_AND_SEND:
      if (!m_pending || !HmisIsLcdReady()) return false;
      {
        const hmi_cmd_result_t rc = hmi_cmd_lcd_draw_text(m_x, m_y, m_color, m_text);
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

HmigMenuItemComponent::HmigMenuItemComponent(uint8_t x, uint8_t y, const char* text, hmig_scene_t* targetScene)
  : m_x(x),
    m_y(y),
    m_text(text),
    m_targetScene(targetScene),
    m_active(false),
    m_pending(false) {
}

bool HmigMenuItemComponent::IsMenuItem(void) const {
  return true;
}

bool HmigMenuItemComponent::IsMenuActive(void) const {
  return m_active;
}

void HmigMenuItemComponent::SetMenuActive(bool active) {
  if (m_active != active) {
    m_active = active;
    m_pending = true;
  }
}

hmig_scene_t* HmigMenuItemComponent::GetMenuTargetScene(void) const {
  return m_targetScene;
}

bool HmigMenuItemComponent::Draw(bool active) {
  char text[32];
  text[0] = active ? '>' : ' ';
  text[1] = ' ';
  if (m_text == NULL) {
    text[2] = '\0';
  } else {
    (void)snprintf(&text[2], sizeof(text) - 2U, "%s", m_text);
    text[sizeof(text) - 1U] = '\0';
  }
  const uint16_t color = active ? 0xFFFFU : 0x7BEFU;
  const hmi_cmd_result_t rc = hmi_cmd_lcd_draw_text(m_x, m_y, color, text);
  if ((rc == HMI_CMD_OK) || (rc == HMI_CMD_ERR_INVALID_ARG)) {
    m_pending = false;
  }
  return true;
}

bool HmigMenuItemComponent::Handle(hmig_call_t call) {
  switch (call) {
    case HMIG_CALL_ENTER:
      m_pending = true;
      return false;
    case HMIG_CALL_PROCESS:
      return false;
    case HMIG_CALL_PROCESS_AND_SEND:
      if (!m_pending || !HmisIsLcdReady()) return false;
      return Draw(m_active);
    case HMIG_CALL_EXIT:
      m_pending = false;
      m_active = false;
      return false;
    default:
      return false;
  }
}

uint8_t HmigMapAxis(uint16_t value, uint8_t outMin, uint8_t outMax) {
  if (outMax <= outMin) return outMin;
  if (value > 4095U) value = 4095U;
  return (uint8_t)(outMin + (((uint32_t)value * (uint32_t)(outMax - outMin)) / 4095U));
}

void HmigSwitchScene(hmig_scene_t* scene) {
  if (s_hmigActiveScene == scene) return;
  if (s_hmigActiveScene != NULL) {
    HmigSceneLeave(s_hmigActiveScene);
  }
  s_hmigActiveScene = scene;
  if (s_hmigActiveScene != NULL) {
    HmigSceneEnter(s_hmigActiveScene);
    HmigEnsureMenuSelection(s_hmigActiveScene);
  }
}

hmig_scene_t* HmigGetActiveScene(void) {
  return s_hmigActiveScene;
}

bool HmigServiceActiveScene(void) {
  hmig_scene_t* const scene = s_hmigActiveScene;
  if (scene == NULL) return false;
  HmigEnsureMenuSelection(scene);
  return HmigSceneService(scene);
}
