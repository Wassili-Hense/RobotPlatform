#include "gui.h"
#include <stdio.h>

static gui_scene_t* s_guiActiveScene = nullptr;

static bool GUIIsLcdReady(void) {
  return (hmi_get(HMI_DATA_STAT_LCD_BUSY) == 0U);
}

static void GUISceneEnter(gui_scene_t* scene) {
  if ((scene == nullptr) || (scene->components == nullptr)) return;
  for (size_t i = 0U; i < scene->componentCount; ++i) {
    GUIComponent* component = scene->components[i];
    if (component != nullptr) {
      (void)component->Enter();
    }
  }
}

static void GUISceneLeave(gui_scene_t* scene) {
  if ((scene == nullptr) || (scene->components == nullptr)) return;
  for (size_t i = 0U; i < scene->componentCount; ++i) {
    GUIComponent* component = scene->components[i];
    if (component != nullptr) {
      (void)component->Exit();
    }
  }
}

static bool GUISceneService(gui_scene_t* scene) {
  if ((scene == nullptr) || (scene->components == nullptr)) return false;

  GUIMenuItemComponent::EnsureSceneSelection(scene);

  bool sent = false;
  for (size_t i = 0U; i < scene->componentCount; ++i) {
    GUIComponent* component = scene->components[i];
    if (component == nullptr) continue;

    const bool handled = sent ? component->Process() : component->ProcessAndSend();
    if (handled) {
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

bool GUIClsComponent::Enter(void) {
  m_pending = true;
  return false;
}

bool GUIClsComponent::Process(void) {
  return false;
}

bool GUIClsComponent::ProcessAndSend(void) {
  if (!m_pending || !GUIIsLcdReady()) return false;
  const hmi_cmd_result_t rc = hmi_cmd_lcd_clear(m_color);
  if ((rc == HMI_CMD_OK) || (rc == HMI_CMD_ERR_INVALID_ARG)) {
    m_pending = false;
  }
  return true;
}

bool GUIClsComponent::Exit(void) {
  m_pending = false;
  return false;
}

GUIJViewComponent::GUIJViewComponent()
  : m_currentX(0U),
    m_currentY(0U),
    m_nextX(0U),
    m_nextY(0U),
    m_visible(false),
    m_phase(GUI_J_VIEW_PHASE_IDLE) {
}

bool GUIJViewComponent::Update(void) {
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

bool GUIJViewComponent::Enter(void) {
  m_currentX = 0U;
  m_currentY = 0U;
  m_nextX = 0U;
  m_nextY = 0U;
  m_visible = false;
  m_phase = GUI_J_VIEW_PHASE_IDLE;
  return false;
}

bool GUIJViewComponent::Process(void) {
  (void)Update();
  return false;
}

bool GUIJViewComponent::ProcessAndSend(void) {
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
}

bool GUIJViewComponent::Exit(void) {
  if (!m_visible || !GUIIsLcdReady()) {
    if (!m_visible) m_phase = GUI_J_VIEW_PHASE_IDLE;
    return false;
  }

  const hmi_cmd_result_t rc = hmi_cmd_lcd_draw_marker(m_currentX, m_currentY, 3U, 0x0000U);
  if ((rc == HMI_CMD_OK) || (rc == HMI_CMD_ERR_INVALID_ARG)) {
    m_visible = false;
    m_phase = GUI_J_VIEW_PHASE_IDLE;
  }
  return true;
}

GUIHotKeyComponent::GUIHotKeyComponent(hmi_data_idx_t idx, gui_scene_t* targetScene)
  : m_idx(idx),
    m_targetScene(targetScene) {
}

bool GUIHotKeyComponent::Enter(void) {
  return false;
}

bool GUIHotKeyComponent::ProcessImpl(void) {
  if (hmi_changed(m_idx) && (hmi_get(m_idx) != 0U) && (m_targetScene != nullptr)) {
    GUISwitchScene(m_targetScene);
  }
  return false;
}

bool GUIHotKeyComponent::Process(void) {
  return ProcessImpl();
}

bool GUIHotKeyComponent::ProcessAndSend(void) {
  return ProcessImpl();
}

bool GUIHotKeyComponent::Exit(void) {
  return false;
}

GUILabelComponent::GUILabelComponent(uint8_t x, uint8_t y, uint16_t color, const char* text)
  : m_x(x),
    m_y(y),
    m_color(color),
    m_text(text),
    m_pending(false) {
}

bool GUILabelComponent::Enter(void) {
  m_pending = true;
  return false;
}

bool GUILabelComponent::Process(void) {
  return false;
}

bool GUILabelComponent::ProcessAndSend(void) {
  if (!m_pending || !GUIIsLcdReady()) return false;
  const hmi_cmd_result_t rc = hmi_cmd_lcd_draw_text(m_x, m_y, m_color, m_text);
  if ((rc == HMI_CMD_OK) || (rc == HMI_CMD_ERR_INVALID_ARG)) {
    m_pending = false;
  }
  return true;
}

bool GUILabelComponent::Exit(void) {
  m_pending = false;
  return false;
}

GUIMenuItemComponent::GUIMenuItemComponent(uint8_t x, uint8_t y, const char* text, gui_scene_t* targetScene)
  : m_x(x),
    m_y(y),
    m_text(text),
    m_targetScene(targetScene),
    m_active(false),
    m_prevActive(false),
    m_pending(false) {
}

GUIMenuItemComponent* GUIMenuItemComponent::Cast(GUIComponent* component) {
  if ((component != nullptr) && (component->GetClassId() == GUIMenuItemComponent::ClassId())) {
    return static_cast<GUIMenuItemComponent*>(component);
  }
  return nullptr;
}

const GUIMenuItemComponent* GUIMenuItemComponent::Cast(const GUIComponent* component) {
  if ((component != nullptr) && (component->GetClassId() == GUIMenuItemComponent::ClassId())) {
    return static_cast<const GUIMenuItemComponent*>(component);
  }
  return nullptr;
}

GUIMenuItemComponent* GUIMenuItemComponent::FindFirst(gui_scene_t* scene) {
  if ((scene == nullptr) || (scene->components == nullptr)) return nullptr;
  for (size_t i = 0U; i < scene->componentCount; ++i) {
    GUIMenuItemComponent* item = Cast(scene->components[i]);
    if (item != nullptr) return item;
  }
  return nullptr;
}

GUIMenuItemComponent* GUIMenuItemComponent::FindActive(gui_scene_t* scene) {
  if ((scene == nullptr) || (scene->components == nullptr)) return nullptr;
  for (size_t i = 0U; i < scene->componentCount; ++i) {
    GUIMenuItemComponent* item = Cast(scene->components[i]);
    if ((item != nullptr) && item->m_active) return item;
  }
  return nullptr;
}

GUIMenuItemComponent* GUIMenuItemComponent::FindAdjacent(gui_scene_t* scene, const GUIMenuItemComponent* from, int step) {
  if ((scene == nullptr) || (scene->components == nullptr) || (from == nullptr) || (step == 0)) return nullptr;

  int startIndex = -1;
  for (size_t i = 0U; i < scene->componentCount; ++i) {
    if (scene->components[i] == from) {
      startIndex = (int)i;
      break;
    }
  }
  if (startIndex < 0) return nullptr;

  const int count = (int)scene->componentCount;
  int index = startIndex;
  for (int n = 0; n < count; ++n) {
    index += step;
    if (index < 0) {
      index = count - 1;
    } else if (index >= count) {
      index = 0;
    }

    GUIMenuItemComponent* item = Cast(scene->components[index]);
    if (item != nullptr) return item;
  }
  return nullptr;
}

void GUIMenuItemComponent::SetActive(bool active) {
  if (m_active != active) {
    m_active = active;
    m_pending = true;
  }
}

void GUIMenuItemComponent::SyncPrevActive(void) {
  m_prevActive = m_active;
}

void GUIMenuItemComponent::EnsureSceneSelection(gui_scene_t* scene) {
  if (FindActive(scene) != nullptr) return;
  GUIMenuItemComponent* first = FindFirst(scene);
  if (first != nullptr) {
    first->SetActive(true);
  }
}

bool GUIMenuItemComponent::ProcessNavigation(void) {
  if (!m_prevActive) return false;

  gui_scene_t* const scene = GUIGetActiveScene();
  if (scene == nullptr) return false;

  if (hmi_changed(HMI_DATA_BTN_UP) && (hmi_get(HMI_DATA_BTN_UP) != 0U)) {
    GUIMenuItemComponent* next = FindAdjacent(scene, this, -1);
    if ((next != nullptr) && (next != this)) {
      SetActive(false);
      next->SetActive(true);
    }
    return false;
  }

  if (hmi_changed(HMI_DATA_BTN_DOWN) && (hmi_get(HMI_DATA_BTN_DOWN) != 0U)) {
    GUIMenuItemComponent* next = FindAdjacent(scene, this, 1);
    if ((next != nullptr) && (next != this)) {
      SetActive(false);
      next->SetActive(true);
    }
    return false;
  }

  if (hmi_changed(HMI_DATA_BTN_OK) && (hmi_get(HMI_DATA_BTN_OK) != 0U) && (m_targetScene != nullptr)) {
    GUISwitchScene(m_targetScene);
    return (GUIGetActiveScene() != scene);
  }

  return false;
}

bool GUIMenuItemComponent::Draw(bool active) {
  char text[32];
  text[0] = active ? '>' : ' ';
  text[1] = ' ';

  if (m_text == nullptr) {
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

bool GUIMenuItemComponent::Enter(void) {
  m_prevActive = m_active;
  m_pending = true;
  return false;
}

bool GUIMenuItemComponent::Process(void) {
  if (m_active != m_prevActive) {
    m_pending = true;
  }
  (void)ProcessNavigation();
  SyncPrevActive();
  return false;
}

bool GUIMenuItemComponent::ProcessAndSend(void) {
  bool switchedScene = false;

  if (m_active != m_prevActive) {
    m_pending = true;
  }

  switchedScene = ProcessNavigation();
  if (!switchedScene && m_pending && GUIIsLcdReady()) {
    const bool handled = Draw(m_active);
    SyncPrevActive();
    return handled;
  }

  SyncPrevActive();
  return false;
}

bool GUIMenuItemComponent::Exit(void) {
  m_pending = false;
  m_prevActive = m_active;
  return false;
}

uint8_t GUIMapAxis(uint16_t value, uint8_t outMin, uint8_t outMax) {
  if (outMax <= outMin) return outMin;
  if (value > 4095U) value = 4095U;
  return (uint8_t)(outMin + (((uint32_t)value * (uint32_t)(outMax - outMin)) / 4095U));
}

void GUISwitchScene(gui_scene_t* scene) {
  if (s_guiActiveScene == scene) return;
  if (s_guiActiveScene != nullptr) {
    GUISceneLeave(s_guiActiveScene);
  }
  s_guiActiveScene = scene;
  if (s_guiActiveScene != nullptr) {
    GUISceneEnter(s_guiActiveScene);
    GUIMenuItemComponent::EnsureSceneSelection(s_guiActiveScene);
  }
}

gui_scene_t* GUIGetActiveScene(void) {
  return s_guiActiveScene;
}

bool GUIServiceActiveScene(void) {
  gui_scene_t* const scene = s_guiActiveScene;
  if (scene == nullptr) return false;
  return GUISceneService(scene);
}
