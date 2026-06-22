#include "gui.h"
#include <stdio.h>

static gui_scene_t* s_guiActiveScene = nullptr;

static bool GUIIsLcdReady(void) {
  return (hmi_get(HMI_DATA_STAT_LCD_BUSY) == 0U);
}

static void GUISceneEnter(gui_scene_t* scene) {
  if ((scene == NULL) || (scene->components == NULL)) return;
  for (size_t i = 0U; i < scene->componentCount; ++i) {
    GUIComponent* component = scene->components[i];
    if (component != NULL) {
      (void)component->Handle(GUI_CALL_ENTER);
    }
  }
}

static void GUISceneLeave(gui_scene_t* scene) {
  if ((scene == NULL) || (scene->components == NULL)) return;
  for (size_t i = 0U; i < scene->componentCount; ++i) {
    GUIComponent* component = scene->components[i];
    if (component != NULL) {
      (void)component->Handle(GUI_CALL_EXIT);
    }
  }
}

static int GUIFindActiveMenuIndex(const gui_scene_t* scene) {
  if ((scene == NULL) || (scene->components == NULL)) return -1;
  for (size_t i = 0U; i < scene->componentCount; ++i) {
    const GUIComponent* component = scene->components[i];
    if ((component != NULL) && component->IsMenuItem() && component->IsMenuActive()) {
      return (int)i;
    }
  }
  return -1;
}

static int GUIFindFirstMenuIndex(const gui_scene_t* scene) {
  if ((scene == NULL) || (scene->components == NULL)) return -1;
  for (size_t i = 0U; i < scene->componentCount; ++i) {
    const GUIComponent* component = scene->components[i];
    if ((component != NULL) && component->IsMenuItem()) {
      return (int)i;
    }
  }
  return -1;
}

static int GUIFindAdjacentMenuIndex(const gui_scene_t* scene, int startIndex, int step) {
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
    const GUIComponent* component = scene->components[index];
    if ((component != NULL) && component->IsMenuItem()) {
      return index;
    }
  }
  return -1;
}

static void GUIActivateMenuIndex(gui_scene_t* scene, int menuIndex) {
  if ((scene == NULL) || (scene->components == NULL)) return;
  for (size_t i = 0U; i < scene->componentCount; ++i) {
    GUIComponent* component = scene->components[i];
    if ((component != NULL) && component->IsMenuItem()) {
      component->SetMenuActive(((int)i) == menuIndex);
    }
  }
}

static void GUIEnsureMenuSelection(gui_scene_t* scene) {
  if ((scene == NULL) || (scene->components == NULL)) return;
  if (GUIFindActiveMenuIndex(scene) >= 0) return;
  const int firstIndex = GUIFindFirstMenuIndex(scene);
  if (firstIndex >= 0) {
    GUIActivateMenuIndex(scene, firstIndex);
  }
}

static bool GUIMoveMenuSelection(gui_scene_t* scene, int step) {
  if ((scene == NULL) || (scene->components == NULL)) return false;
  GUIEnsureMenuSelection(scene);
  const int activeIndex = GUIFindActiveMenuIndex(scene);
  if (activeIndex < 0) return false;
  const int nextIndex = GUIFindAdjacentMenuIndex(scene, activeIndex, step);
  if ((nextIndex < 0) || (nextIndex == activeIndex)) return false;
  GUIActivateMenuIndex(scene, nextIndex);
  return true;
}

static bool GUIHandleMenuNavigation(gui_scene_t* scene) {
  if ((scene == NULL) || (scene->components == NULL)) return false;
  GUIEnsureMenuSelection(scene);

  if (hmi_changed(HMI_DATA_BTN_UP) && (hmi_get(HMI_DATA_BTN_UP) != 0U)) {
    (void)GUIMoveMenuSelection(scene, -1);
  }
  if (hmi_changed(HMI_DATA_BTN_DOWN) && (hmi_get(HMI_DATA_BTN_DOWN) != 0U)) {
    (void)GUIMoveMenuSelection(scene, 1);
  }
  if (hmi_changed(HMI_DATA_BTN_OK) && (hmi_get(HMI_DATA_BTN_OK) != 0U)) {
    const int activeIndex = GUIFindActiveMenuIndex(scene);
    if (activeIndex >= 0) {
      GUIComponent* component = scene->components[activeIndex];
      gui_scene_t* const targetScene = (component != NULL) ? component->GetMenuTargetScene() : NULL;
      if (targetScene != NULL) {
        GUISwitchScene(targetScene);
        return true;
      }
    }
  }
  return false;
}

static bool GUISceneService(gui_scene_t* scene) {
  if ((scene == NULL) || (scene->components == NULL)) return false;
  if (GUIHandleMenuNavigation(scene)) return false;
  if (s_guiActiveScene != scene) return false;

  bool sent = false;
  for (size_t i = 0U; i < scene->componentCount; ++i) {
    GUIComponent* component = scene->components[i];
    if (component == NULL) continue;
    const gui_call_t call = sent ? GUI_CALL_PROCESS : GUI_CALL_PROCESS_AND_SEND;
    if (component->Handle(call)) {
      if (s_guiActiveScene != scene) return false;
      sent = true;
    } else if (s_guiActiveScene != scene) {
      return false;
    }
  }
  return sent;
}

GUIClsComponent::GUIClsComponent(uint16_t color)
  : m_color(color),
    m_pending(false) {
}

bool GUIClsComponent::Handle(gui_call_t call) {
  switch (call) {
    case GUI_CALL_ENTER:
      m_pending = true;
      return false;
    case GUI_CALL_PROCESS:
      return false;
    case GUI_CALL_PROCESS_AND_SEND:
      if (!m_pending || !GUIIsLcdReady()) return false;
      {
        const hmi_cmd_result_t rc = hmi_cmd_lcd_clear(m_color);
        if ((rc == HMI_CMD_OK) || (rc == HMI_CMD_ERR_INVALID_ARG)) {
          m_pending = false;
        }
      }
      return true;
    case GUI_CALL_EXIT:
      m_pending = false;
      return false;
    default:
      return false;
  }
}

GUIJViewComponent::GUIJViewComponent()
  : m_currentX(0U),
    m_currentY(0U),
    m_nextX(0U),
    m_nextY(0U),
    m_visible(false),
    m_phase(GUI_J_VIEW_PHASE_IDLE) {
}

bool GUIJViewComponent::Update() {
  const bool backlightOn = (hmi_get(HMI_DATA_STAT_BL_ON) != 0U);
  if (!backlightOn) {
    if (m_visible) {
      m_phase = GUI_J_VIEW_PHASE_ERASE;
      return true;
    }
    m_phase = GUI_J_VIEW_PHASE_IDLE;
    return false;
  }

  m_nextX = GUIMapAxis(hmi_get(HMI_DATA_JOY_X), 45U, 114U);
  m_nextY = GUIMapAxis(hmi_get(HMI_DATA_JOY_Y), 10U, 79U);

  if ((m_phase == GUI_J_VIEW_PHASE_DRAW) || (m_phase == GUI_J_VIEW_PHASE_ERASE)) return true;
  if (!m_visible) {
    m_phase = GUI_J_VIEW_PHASE_DRAW;
    return true;
  }
  if ((m_currentX == m_nextX) && (m_currentY == m_nextY)) {
    m_phase = GUI_J_VIEW_PHASE_IDLE;
    return false;
  }
  m_phase = GUI_J_VIEW_PHASE_ERASE;
  return true;
}

bool GUIJViewComponent::Handle(gui_call_t call) {
  switch (call) {
    case GUI_CALL_ENTER:
      m_currentX = 0U;
      m_currentY = 0U;
      m_nextX = 0U;
      m_nextY = 0U;
      m_visible = false;
      m_phase = GUI_J_VIEW_PHASE_IDLE;
      return false;
    case GUI_CALL_PROCESS:
      (void)Update();
      return false;
    case GUI_CALL_PROCESS_AND_SEND:
      (void)Update();
      if ((m_phase == GUI_J_VIEW_PHASE_IDLE) || !GUIIsLcdReady()) return false;
      if (m_phase == GUI_J_VIEW_PHASE_ERASE) {
        const hmi_cmd_result_t rc = hmi_cmd_lcd_draw_marker(m_currentX, m_currentY, 3U, 0x0000U);
        if ((rc == HMI_CMD_OK) || (rc == HMI_CMD_ERR_INVALID_ARG)) {
          m_visible = false;
          if ((hmi_get(HMI_DATA_STAT_BL_ON) != 0U) && ((m_currentX != m_nextX) || (m_currentY != m_nextY))) {
            m_phase = GUI_J_VIEW_PHASE_DRAW;
          } else {
            m_phase = GUI_J_VIEW_PHASE_IDLE;
          }
        }
        return true;
      }
      if (m_phase == GUI_J_VIEW_PHASE_DRAW) {
        const hmi_cmd_result_t rc = hmi_cmd_lcd_draw_marker(m_nextX, m_nextY, 3U, 0xFFFFU);
        if ((rc == HMI_CMD_OK) || (rc == HMI_CMD_ERR_INVALID_ARG)) {
          m_currentX = m_nextX;
          m_currentY = m_nextY;
          m_visible = true;
          m_phase = GUI_J_VIEW_PHASE_IDLE;
        }
        return true;
      }
      return false;
    case GUI_CALL_EXIT:
      if (!m_visible || !GUIIsLcdReady()) {
        if (!m_visible) m_phase = GUI_J_VIEW_PHASE_IDLE;
        return false;
      }
      {
        const hmi_cmd_result_t rc = hmi_cmd_lcd_draw_marker(m_currentX, m_currentY, 3U, 0x0000U);
        if ((rc == HMI_CMD_OK) || (rc == HMI_CMD_ERR_INVALID_ARG)) {
          m_visible = false;
          m_phase = GUI_J_VIEW_PHASE_IDLE;
        }
      }
      return true;
    default:
      return false;
  }
}

GUIHotKeyComponent::GUIHotKeyComponent(hmi_data_idx_t idx, gui_scene_t* targetScene)
  : m_idx(idx),
    m_targetScene(targetScene) {
}

bool GUIHotKeyComponent::Handle(gui_call_t call) {
  switch (call) {
    case GUI_CALL_ENTER:
      return false;
    case GUI_CALL_PROCESS:
    case GUI_CALL_PROCESS_AND_SEND:
      if (hmi_changed(m_idx) && (hmi_get(m_idx) != 0U) && (m_targetScene != NULL)) {
        GUISwitchScene(m_targetScene);
      }
      return false;
    case GUI_CALL_EXIT:
      return false;
    default:
      return false;
  }
}

GUILabelComponent::GUILabelComponent(uint8_t x, uint8_t y, uint16_t color, const char* text)
  : m_x(x),
    m_y(y),
    m_color(color),
    m_text(text),
    m_pending(false) {
}

bool GUILabelComponent::Handle(gui_call_t call) {
  switch (call) {
    case GUI_CALL_ENTER:
      m_pending = true;
      return false;
    case GUI_CALL_PROCESS:
      return false;
    case GUI_CALL_PROCESS_AND_SEND:
      if (!m_pending || !GUIIsLcdReady()) return false;
      {
        const hmi_cmd_result_t rc = hmi_cmd_lcd_draw_text(m_x, m_y, m_color, m_text);
        if ((rc == HMI_CMD_OK) || (rc == HMI_CMD_ERR_INVALID_ARG)) {
          m_pending = false;
        }
      }
      return true;
    case GUI_CALL_EXIT:
      m_pending = false;
      return false;
    default:
      return false;
  }
}

GUIMenuItemComponent::GUIMenuItemComponent(uint8_t x, uint8_t y, const char* text, gui_scene_t* targetScene)
  : m_x(x),
    m_y(y),
    m_text(text),
    m_targetScene(targetScene),
    m_active(false),
    m_pending(false) {
}

bool GUIMenuItemComponent::IsMenuItem(void) const {
  return true;
}

bool GUIMenuItemComponent::IsMenuActive(void) const {
  return m_active;
}

void GUIMenuItemComponent::SetMenuActive(bool active) {
  if (m_active != active) {
    m_active = active;
    m_pending = true;
  }
}

gui_scene_t* GUIMenuItemComponent::GetMenuTargetScene(void) const {
  return m_targetScene;
}

bool GUIMenuItemComponent::Draw(bool active) {
  char text[32];
  text[0] = active ? '>' : ' ';
  text[1] = ' ';
  if (m_text == NULL) {
    text[2] = ' ';
  } else {
    (void)snprintf(&text[2], sizeof(text) - 2U, "%s", m_text);
    text[sizeof(text) - 1U] = ' ';
  }

  const uint16_t color = active ? 0xFFFFU : 0x7BEFU;
  const hmi_cmd_result_t rc = hmi_cmd_lcd_draw_text(m_x, m_y, color, text);
  if ((rc == HMI_CMD_OK) || (rc == HMI_CMD_ERR_INVALID_ARG)) {
    m_pending = false;
  }
  return true;
}

bool GUIMenuItemComponent::Handle(gui_call_t call) {
  switch (call) {
    case GUI_CALL_ENTER:
      m_pending = true;
      return false;
    case GUI_CALL_PROCESS:
      return false;
    case GUI_CALL_PROCESS_AND_SEND:
      if (!m_pending || !GUIIsLcdReady()) return false;
      return Draw(m_active);
    case GUI_CALL_EXIT:
      m_pending = false;
      m_active = false;
      return false;
    default:
      return false;
  }
}

uint8_t GUIMapAxis(uint16_t value, uint8_t outMin, uint8_t outMax) {
  if (outMax <= outMin) return outMin;
  if (value > 4095U) value = 4095U;
  return (uint8_t)(outMin + (((uint32_t)value * (uint32_t)(outMax - outMin)) / 4095U));
}

void GUISwitchScene(gui_scene_t* scene) {
  if (s_guiActiveScene == scene) return;
  if (s_guiActiveScene != NULL) {
    GUISceneLeave(s_guiActiveScene);
  }
  s_guiActiveScene = scene;
  if (s_guiActiveScene != NULL) {
    GUISceneEnter(s_guiActiveScene);
    GUIEnsureMenuSelection(s_guiActiveScene);
  }
}

gui_scene_t* GUIGetActiveScene(void) {
  return s_guiActiveScene;
}

bool GUIServiceActiveScene(void) {
  gui_scene_t* const scene = s_guiActiveScene;
  if (scene == NULL) return false;
  GUIEnsureMenuSelection(scene);
  return GUISceneService(scene);
}
