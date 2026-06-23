#include "gui.h"

#include <Arduino.h>
#include <Preferences.h>
#include <stdio.h>


static gui_scene_t* s_guiActiveScene = nullptr;

bool GUIBrightnessComponent::s_loaded = false;
uint8_t GUIBrightnessComponent::s_storedIndex = 5U;
uint8_t GUIBrightnessComponent::s_actualIndex = 5U;

static constexpr uint8_t GUI_J_VIEW_OUT_MIN_X = 45U;
static constexpr uint8_t GUI_J_VIEW_OUT_MAX_X = 114U;
static constexpr uint8_t GUI_J_VIEW_OUT_MIN_Y = 10U;
static constexpr uint8_t GUI_J_VIEW_OUT_MAX_Y = 79U;

static constexpr uint32_t GUI_CLS_KEEPALIVE_PERIOD_MS = 10000U;
static constexpr uint16_t GUI_CLS_KEEPALIVE_TIMEOUT_MS = 15000U;

static bool GUIIsLcdReady(void) {
  return (hmi_get(HMI_DATA_STAT_LCD_BUSY) == 0U);
}

static uint8_t GUIMapLinearWindow(uint16_t value,
                                  uint16_t inMin,
                                  uint16_t inMax,
                                  uint8_t outMin,
                                  uint8_t outMax) {
  if (outMax <= outMin) return outMin;
  if (inMax <= inMin) return (uint8_t)(outMin + ((outMax - outMin) / 2U));
  if (value <= inMin) return outMin;
  if (value >= inMax) return outMax;
  return (uint8_t)(outMin + ((((uint32_t)(value - inMin)) * (uint32_t)(outMax - outMin)) / (uint32_t)(inMax - inMin)));
}

static uint8_t GUIMapCalibratedValue(uint16_t value,
                                     const gui_axis_cal_t& cal,
                                     uint8_t outMin,
                                     uint8_t outMax) {
  if (outMax <= outMin) return outMin;
  const uint8_t outMid = (uint8_t)(outMin + ((outMax - outMin) / 2U));
  const uint16_t leftMin = cal.eMin;
  const uint16_t leftMax = (cal.cMin >= cal.eMin) ? cal.cMin : cal.eMin;
  const uint16_t rightMin = (cal.eMax >= cal.cMax) ? cal.cMax : cal.eMax;
  const uint16_t rightMax = cal.eMax;
  if (value <= leftMax) {
    return GUIMapLinearWindow(value, leftMin, leftMax, outMin, outMid);
  }
  if (value >= rightMin) {
    return GUIMapLinearWindow(value, rightMin, rightMax, outMid, outMax);
  }
  return outMid;
}

static void GUILoadCalibrationFromPreferences(gui_axis_cal_t* axisX, gui_axis_cal_t* axisY) {
  if ((axisX == nullptr) || (axisY == nullptr)) return;
  Preferences prefs;
  if (!prefs.begin("joycal", true)) return;
  axisX->eMin = prefs.getUShort("xemin", axisX->eMin);
  axisX->cMin = prefs.getUShort("xcmin", axisX->cMin);
  axisX->cMax = prefs.getUShort("xcmax", axisX->cMax);
  axisX->eMax = prefs.getUShort("xemax", axisX->eMax);
  axisY->eMin = prefs.getUShort("yemin", axisY->eMin);
  axisY->cMin = prefs.getUShort("ycmin", axisY->cMin);
  axisY->cMax = prefs.getUShort("ycmax", axisY->cMax);
  axisY->eMax = prefs.getUShort("yemax", axisY->eMax);
  prefs.end();
}


static constexpr uint8_t GUI_BRIGHTNESS_STEP_TABLE[10] = {
  1U, 2U, 3U, 5U, 8U, 13U, 22U, 36U, 61U, 127U
};

static uint8_t GUIBrightnessToStep(uint8_t index) {
  if (index > 9U) index = 9U;
  return GUI_BRIGHTNESS_STEP_TABLE[index];
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


GUIClsComponent::GUIClsComponent(uint16_t color, bool highlight)
  : m_color(color),
    m_highlight(highlight),
    m_pendingClear(false),
    m_nextKeepAliveMs(0U) {
}

bool GUIClsComponent::SendBacklightKeepOn(void) {
  if (!m_highlight) return false;
  const uint32_t now = millis();
  if ((int32_t)(now - m_nextKeepAliveMs) < 0) return false;
  hmi_cmd_set_backlight_timeout(GUI_CLS_KEEPALIVE_TIMEOUT_MS);
  m_nextKeepAliveMs = now + GUI_CLS_KEEPALIVE_PERIOD_MS;
  return true;
}

bool GUIClsComponent::Enter(void) {
  m_pendingClear = true;
  m_nextKeepAliveMs = millis();
  return false;
}

bool GUIClsComponent::Process(void) {
  return false;
}

bool GUIClsComponent::ProcessAndSend(void) {
  if (m_pendingClear) {
    if (!GUIIsLcdReady()) return false;
    const hmi_cmd_result_t rc = hmi_cmd_lcd_clear(m_color);
    if ((rc == HMI_CMD_OK) || (rc == HMI_CMD_ERR_INVALID_ARG)) {
      m_pendingClear = false;
    }
    return true;
  }
  return SendBacklightKeepOn();
}

bool GUIClsComponent::Exit(void) {
  m_pendingClear = false;
  return false;
}

GUIJViewComponent::GUIJViewComponent(gui_j_view_mode_t mode,
                                     gui_axis_cal_t* axisX,
                                     gui_axis_cal_t* axisY,
                                     gui_scene_t* targetScene)
  : m_mode(mode),
    m_axisX(axisX),
    m_axisY(axisY),
    m_targetScene(targetScene),
    m_currentX(0U),
    m_currentY(0U),
    m_nextX(0U),
    m_nextY(0U),
    m_visible(false),
    m_pending(false),
    m_hasSample(false),
    m_trackCalLoaded(false),
    m_rawX(0U),
    m_rawY(0U),
    m_minX(0U),
    m_minY(0U),
    m_maxX(0U),
    m_maxY(0U),
    m_windowMinX(0U),
    m_windowMaxX(4095U),
    m_windowMinY(0U),
    m_windowMaxY(4095U),
    m_phase(GUI_J_VIEW_PHASE_IDLE) {
  if ((m_mode == GUI_J_VIEW_MODE_TRACK) && ! m_trackCalLoaded){
    GUILoadCalibrationFromPreferences(m_axisX, m_axisY);
    m_trackCalLoaded = true;
  }
}

void GUIJViewComponent::UpdateWindow(void) {
  if ((m_axisX == nullptr) || (m_axisY == nullptr)) {
    m_windowMinX = 0U;
    m_windowMaxX = 4095U;
    m_windowMinY = 0U;
    m_windowMaxY = 4095U;
    return;
  }
  if (m_mode == GUI_J_VIEW_MODE_CAL_CENTER) {
    const uint16_t centerX = (uint16_t)(((uint32_t)m_axisX->cMin + (uint32_t)m_axisX->cMax) / 2U);
    const uint16_t centerY = (uint16_t)(((uint32_t)m_axisY->cMin + (uint32_t)m_axisY->cMax) / 2U);
    const uint16_t widthX = (uint16_t)(((m_axisX->cMax >= m_axisX->cMin) ? (m_axisX->cMax - m_axisX->cMin) : 0U) * 2U);
    const uint16_t widthY = (uint16_t)(((m_axisY->cMax >= m_axisY->cMin) ? (m_axisY->cMax - m_axisY->cMin) : 0U) * 2U);
    const uint16_t spanX = (widthX >= 70U) ? widthX : 70U;
    const uint16_t spanY = (widthY >= 70U) ? widthY : 70U;
    const uint16_t halfX = (uint16_t)(spanX / 2U);
    const uint16_t halfY = (uint16_t)(spanY / 2U);
    m_windowMinX = (centerX > halfX) ? (uint16_t)(centerX - halfX) : 0U;
    m_windowMaxX = (uint16_t)((centerX + halfX <= 4095U) ? (centerX + halfX) : 4095U);
    m_windowMinY = (centerY > halfY) ? (uint16_t)(centerY - halfY) : 0U;
    m_windowMaxY = (uint16_t)((centerY + halfY <= 4095U) ? (centerY + halfY) : 4095U);
    return;
  }
  m_windowMinX = m_axisX->eMin;
  m_windowMaxX = m_axisX->eMax;
  m_windowMinY = m_axisY->eMin;
  m_windowMaxY = m_axisY->eMax;
}

uint8_t GUIJViewComponent::MapAxisX(uint16_t value) const {
  if (m_mode == GUI_J_VIEW_MODE_CAL_CENTER) {
    return GUIMapLinearWindow(value, m_windowMinX, m_windowMaxX, GUI_J_VIEW_OUT_MIN_X, GUI_J_VIEW_OUT_MAX_X);
  }
  if (m_axisX == nullptr) return GUIMapAxis(value, GUI_J_VIEW_OUT_MIN_X, GUI_J_VIEW_OUT_MAX_X);
  return GUIMapCalibratedValue(value, *m_axisX, GUI_J_VIEW_OUT_MIN_X, GUI_J_VIEW_OUT_MAX_X);
}

uint8_t GUIJViewComponent::MapAxisY(uint16_t value) const {
  if (m_mode == GUI_J_VIEW_MODE_CAL_CENTER) {
    return GUIMapLinearWindow(value, m_windowMinY, m_windowMaxY, GUI_J_VIEW_OUT_MIN_Y, GUI_J_VIEW_OUT_MAX_Y);
  }
  if (m_axisY == nullptr) return GUIMapAxis(value, GUI_J_VIEW_OUT_MIN_Y, GUI_J_VIEW_OUT_MAX_Y);
  return GUIMapCalibratedValue(value, *m_axisY, GUI_J_VIEW_OUT_MIN_Y, GUI_J_VIEW_OUT_MAX_Y);
}

bool GUIJViewComponent::SaveCalibration(void) {
  if (!m_hasSample || (m_axisX == nullptr) || (m_axisY == nullptr)) return false;
  if (m_mode == GUI_J_VIEW_MODE_CAL_CENTER) {
    m_axisX->cMin = m_minX - 4;
    m_axisX->cMax = m_maxX + 4;
    m_axisY->cMin = m_minY - 4;
    m_axisY->cMax = m_maxY + 4;
  } else if (m_mode == GUI_J_VIEW_MODE_CAL_EDGE) {
    m_axisX->eMin = m_minX;
    m_axisX->eMax = m_maxX;
    m_axisY->eMin = m_minY;
    m_axisY->eMax = m_maxY;
  } else {
    return false;
  }
  Preferences prefs;
  if (!prefs.begin("joycal", false)) return false;
  prefs.putUShort("xemin", m_axisX->eMin);
  prefs.putUShort("xcmin", m_axisX->cMin);
  prefs.putUShort("xcmax", m_axisX->cMax);
  prefs.putUShort("xemax", m_axisX->eMax);
  prefs.putUShort("yemin", m_axisY->eMin);
  prefs.putUShort("ycmin", m_axisY->cMin);
  prefs.putUShort("ycmax", m_axisY->cMax);
  prefs.putUShort("yemax", m_axisY->eMax);
  prefs.end();
  UpdateWindow();
  return true;
}

bool GUIJViewComponent::HandleButtons(void) {
  if (m_mode == GUI_J_VIEW_MODE_TRACK) {
    if (hmi_changed(HMI_DATA_BTN_OK) && (hmi_get(HMI_DATA_BTN_OK) != 0U) && (m_targetScene != nullptr)) {
      GUISwitchScene(m_targetScene);
      return true;
    }
    return false;
  }
  if (hmi_changed(HMI_DATA_BTN_BACK) && (hmi_get(HMI_DATA_BTN_BACK) != 0U) && (m_targetScene != nullptr)) {
    GUISwitchScene(m_targetScene);
    return true;
  }
  if (hmi_changed(HMI_DATA_BTN_OK) && (hmi_get(HMI_DATA_BTN_OK) != 0U) && (m_targetScene != nullptr)) {
    (void)SaveCalibration();
    GUISwitchScene(m_targetScene);
    return true;
  }
  return false;
}

bool GUIJViewComponent::Update(void) {
  const bool backlightOn = (hmi_get(HMI_DATA_STAT_BL_ON) != 0U);
  if (!backlightOn) {
    if ((m_mode == GUI_J_VIEW_MODE_TRACK) && m_visible) {
      m_phase = GUI_J_VIEW_PHASE_ERASE;
      m_pending = true;
      return true;
    }
    m_phase = GUI_J_VIEW_PHASE_IDLE;
    m_pending = false;
    return false;
  }
  UpdateWindow();
  m_rawX = hmi_get(HMI_DATA_JOY_X);
  m_rawY = hmi_get(HMI_DATA_JOY_Y);
  if (!m_hasSample) {
    m_hasSample = true;
    m_minX = m_maxX = m_rawX;
    m_minY = m_maxY = m_rawY;
  } else {
    if (m_rawX < m_minX) m_minX = m_rawX;
    if (m_rawX > m_maxX) m_maxX = m_rawX;
    if (m_rawY < m_minY) m_minY = m_rawY;
    if (m_rawY > m_maxY) m_maxY = m_rawY;
  }
  m_nextX = MapAxisX(m_rawX);
  m_nextY = MapAxisY(m_rawY);
  if (m_mode == GUI_J_VIEW_MODE_TRACK) {
    if ((m_phase == GUI_J_VIEW_PHASE_DRAW) || (m_phase == GUI_J_VIEW_PHASE_ERASE)) return true;
    if (!m_visible) {
      m_phase = GUI_J_VIEW_PHASE_DRAW;
      m_pending = true;
      return true;
    }
    if ((m_currentX == m_nextX) && (m_currentY == m_nextY)) {
      m_phase = GUI_J_VIEW_PHASE_IDLE;
      m_pending = false;
      return false;
    }
    m_phase = GUI_J_VIEW_PHASE_ERASE;
    m_pending = true;
    return true;
  }
  if (!m_visible || (m_currentX != m_nextX) || (m_currentY != m_nextY)) {
    m_phase = GUI_J_VIEW_PHASE_DRAW;
    m_pending = true;
    return true;
  }
  m_phase = GUI_J_VIEW_PHASE_IDLE;
  m_pending = false;
  return false;
}

bool GUIJViewComponent::Enter(void) {
  m_currentX = 0U;
  m_currentY = 0U;
  m_nextX = 0U;
  m_nextY = 0U;
  m_visible = false;
  m_pending = false;
  m_hasSample = false;
  m_rawX = 0U;
  m_rawY = 0U;
  m_minX = 0U;
  m_minY = 0U;
  m_maxX = 0U;
  m_maxY = 0U;
  if (m_mode != GUI_J_VIEW_MODE_TRACK) {
    m_trackCalLoaded = false;
  }
  UpdateWindow();
  m_phase = GUI_J_VIEW_PHASE_IDLE;
  return false;
}

bool GUIJViewComponent::Process(void) {
  if (HandleButtons()) return false;
  (void)Update();
  return false;
}

bool GUIJViewComponent::ProcessAndSend(void) {
  if (HandleButtons()) return false;
  (void)Update();
  if (!m_pending || !GUIIsLcdReady()) return false;
  if ((m_mode == GUI_J_VIEW_MODE_TRACK) && (m_phase == GUI_J_VIEW_PHASE_ERASE)) {
    const hmi_cmd_result_t rc = hmi_cmd_lcd_draw_marker(m_currentX, m_currentY, 3U, GUI_COLOR_BLACK);
    if ((rc == HMI_CMD_OK) || (rc == HMI_CMD_ERR_INVALID_ARG)) {
      m_visible = false;
      if ((hmi_get(HMI_DATA_STAT_BL_ON) != 0U) && ((m_currentX != m_nextX) || (m_currentY != m_nextY))) {
        m_phase = GUI_J_VIEW_PHASE_DRAW;
      } else {
        m_phase = GUI_J_VIEW_PHASE_IDLE;
        m_pending = false;
      }
    }
    return true;
  }
  if ((m_mode == GUI_J_VIEW_MODE_TRACK) && (hmi_get(HMI_DATA_STAT_BL_ON) == 0U)) {
    m_phase = GUI_J_VIEW_PHASE_IDLE;
    m_pending = false;
    return false;
  }
  if (m_phase == GUI_J_VIEW_PHASE_DRAW) {
    const uint8_t markerIndex = (m_mode == GUI_J_VIEW_MODE_CAL_CENTER) ? 5U : 3U;
    const uint16_t markerColor = (m_mode == GUI_J_VIEW_MODE_TRACK) ? GUI_COLOR_WHITE
                               : (m_mode == GUI_J_VIEW_MODE_CAL_CENTER) ? GUI_COLOR_MAGENTA
                                                                        : GUI_COLOR_CYAN;
    const hmi_cmd_result_t rc = hmi_cmd_lcd_draw_marker(m_nextX, m_nextY, markerIndex, markerColor);
    if ((rc == HMI_CMD_OK) || (rc == HMI_CMD_ERR_INVALID_ARG)) {
      m_currentX = m_nextX;
      m_currentY = m_nextY;
      m_visible = true;
      m_phase = GUI_J_VIEW_PHASE_IDLE;
      m_pending = false;
    }
    return true;
  }
  return false;
}

bool GUIJViewComponent::Exit(void) {
  m_pending = false;
  m_phase = GUI_J_VIEW_PHASE_IDLE;
  m_visible = false;
  return false;
}

GUIHotKeyComponent::GUIHotKeyComponent(hmi_data_idx_t idx, gui_scene_t* targetScene)
  : m_idx(idx),
    m_targetScene(targetScene) {
}

bool GUIHotKeyComponent::Enter(void) {
  return false;
}

bool GUIHotKeyComponent::Process(void) {
  if (hmi_changed(m_idx) && (hmi_get(m_idx) != 0U) && (m_targetScene != nullptr)) {
    GUISwitchScene(m_targetScene);
  }
  return false;
}

bool GUIHotKeyComponent::ProcessAndSend(void) {
  return Process();
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
  const uint16_t color = active ? GUI_COLOR_WHITE : GUI_COLOR_GRAY;
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


GUIBrightnessComponent::GUIBrightnessComponent(uint8_t mode, uint8_t x, uint8_t y)
  : m_mode(mode),
    m_x(x),
    m_y(y),
    m_pendingSend(false),
    m_pendingDraw(false) {
  EnsureLoaded();
}

void GUIBrightnessComponent::EnsureLoaded(void) {
  if (s_loaded) return;

  Preferences prefs;
  if (prefs.begin("gui", true)) {
    const uint32_t value = prefs.getUInt("br_idx", 5U);
    s_storedIndex = (value <= 9U) ? (uint8_t)value : 5U;
    prefs.end();
  } else {
    s_storedIndex = 5U;
  }

  s_actualIndex = s_storedIndex;
  s_loaded = true;
}

bool GUIBrightnessComponent::SaveStoredIndex(void) {
  Preferences prefs;
  if (!prefs.begin("gui", false)) return false;
  prefs.putUInt("br_idx", s_actualIndex);
  prefs.end();
  s_storedIndex = s_actualIndex;
  return true;
}

bool GUIBrightnessComponent::SendBrightness(void) {
  uint8_t step = GUIBrightnessToStep(s_actualIndex);
  if (m_mode == 0U) {
    step = (uint8_t)((step + 1U) / 2U);
  }

  hmi_cmd_set_brightness(step);
  m_pendingSend = false;
  return true;
}

bool GUIBrightnessComponent::DrawValue(void) {
  char text[4];
  (void)snprintf(text, sizeof(text), "%u", (unsigned)s_actualIndex);
  text[sizeof(text) - 1U] = '\0';

  const hmi_cmd_result_t rc = hmi_cmd_lcd_draw_text(m_x, m_y, GUI_COLOR_CYAN, text);
  if ((rc == HMI_CMD_OK) || (rc == HMI_CMD_ERR_INVALID_ARG)) {
    m_pendingDraw = false;
  }
  return true;
}

bool GUIBrightnessComponent::ProcessInput(void) {
  if (m_mode != 1U) return false;

  bool changed = false;
  if (hmi_changed(HMI_DATA_BTN_LUP) && (hmi_get(HMI_DATA_BTN_LUP) != 0U) && (s_actualIndex < 9U)) {
    ++s_actualIndex;
    changed = true;
  }
  if (hmi_changed(HMI_DATA_BTN_LDN) && (hmi_get(HMI_DATA_BTN_LDN) != 0U) && (s_actualIndex > 0U)) {
    --s_actualIndex;
    changed = true;
  }

  if (changed) {
    m_pendingSend = true;
    m_pendingDraw = true;
  }
  return false;
}

bool GUIBrightnessComponent::Enter(void) {
  EnsureLoaded();
  s_actualIndex = s_storedIndex;
  m_pendingSend = true;
  m_pendingDraw = (m_mode == 1U);
  return false;
}

bool GUIBrightnessComponent::Process(void) {
  (void)ProcessInput();
  return false;
}

bool GUIBrightnessComponent::ProcessAndSend(void) {
  (void)ProcessInput();
  if (m_pendingSend) return SendBrightness();
  if ((m_mode == 1U) && m_pendingDraw && GUIIsLcdReady()) return DrawValue();
  return false;
}

bool GUIBrightnessComponent::Exit(void) {
  if (s_actualIndex != s_storedIndex) {
    (void)SaveStoredIndex();
  }
  m_pendingSend = false;
  m_pendingDraw = false;
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
  if ((scene == nullptr) || (scene->components == nullptr)) return false;
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
