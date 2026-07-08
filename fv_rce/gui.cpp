#include "gui.h"

#include <Arduino.h>
#include <Preferences.h>
#include <stdio.h>
#include <string.h>

namespace {
enum gui_class_id_t : uint8_t {
  GUI_CLASS_CLS = 1,
  GUI_CLASS_J_VIEW,
  GUI_CLASS_HOT_KEY,
  GUI_CLASS_LABEL,
  GUI_CLASS_VAR,
  GUI_CLASS_MENU_ITEM,
  GUI_CLASS_BRIGHTNESS
};

static gui_scene_t* s_guiActiveScene = nullptr;
static gui_scene_t* s_guiHomeScene = nullptr;
static uint32_t s_guiLastActivityMs = 0U;
static constexpr uint32_t GUI_IDLE_HOME_TIMEOUT_MS = 65000U;
static uint8_t s_guiIndicatorValue[2] = { 0U, 0U };
static bool s_guiIndicatorPending[2] = { true, true };
static uint8_t s_guiProgressValue[4] = { 0U, 0U, 0U, 0U };
}

// -----------------------------------------------------------------------------
// Section: GUIComponent
// -----------------------------------------------------------------------------

// Base interface only.

// -----------------------------------------------------------------------------
// Section: GUIClsComponent
// -----------------------------------------------------------------------------

uint8_t GUIClsComponent::GetClassId(void) const {
  return (uint8_t)GUI_CLASS_CLS;
}

GUIClsComponent::GUIClsComponent(uint16_t color)
  : m_color(color),
    m_pendingClear(false) {
}

void GUIClsComponent::Enter(void) {
  m_pendingClear = true;
}

void GUIClsComponent::Process(void) {
}

void GUIClsComponent::Draw(void) {
  if (m_pendingClear) {
    LCD_Clear(m_color);
    m_pendingClear = false;
  }
}

void GUIClsComponent::Exit(void) {
  m_pendingClear = false;
}

// -----------------------------------------------------------------------------
// Section: GUIJViewComponent
// -----------------------------------------------------------------------------

namespace {
static constexpr uint8_t GUI_J_VIEW_OUT_MIN_X = 48U;
static constexpr uint8_t GUI_J_VIEW_OUT_MAX_X = 111U;
static constexpr uint8_t GUI_J_VIEW_OUT_MIN_Y = 13U;
static constexpr uint8_t GUI_J_VIEW_OUT_MAX_Y = 76U;

static uint8_t GUIMapAxis(uint16_t value, uint8_t outMin, uint8_t outMax) {
  if (outMax <= outMin) return outMin;
  if (value > 4095U) return outMax;
  return (uint8_t)(outMin + (((uint32_t)value * (uint32_t)(outMax - outMin)) / 4095U));
}

static uint8_t GUIMapLinearWindow(uint16_t value, uint16_t inMin, uint16_t inMax, uint8_t outMin, uint8_t outMax) {
  if (value <= inMin) return outMin;
  if (value >= inMax) return outMax;
  return (uint8_t)(outMin + ((((uint32_t)(value - inMin)) * (uint32_t)(outMax - outMin)) / (uint32_t)(inMax - inMin)));
}

static uint8_t GUIMapCalibratedValue(uint16_t value, const gui_axis_cal_t& cal, uint8_t outMin, uint8_t outMax) {
  const uint8_t outMid = (uint8_t)(outMin + ((outMax - outMin) / 2U));
  const uint16_t leftMin = cal.eMin;
  const uint16_t leftMax = cal.cMin;
  const uint16_t rightMin = cal.cMax;
  const uint16_t rightMax = cal.eMax;

  if (value <= leftMax) return GUIMapLinearWindow(value, leftMin, leftMax, outMin, outMid);
  if (value >= rightMin) return GUIMapLinearWindow(value, rightMin, rightMax, outMid, outMax);
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
}

uint8_t GUIJViewComponent::GetClassId(void) const {
  return (uint8_t)GUI_CLASS_J_VIEW;
}

GUIJViewComponent::GUIJViewComponent(gui_j_view_mode_t mode, gui_axis_cal_t* axisX, gui_axis_cal_t* axisY)
  : m_mode(mode),
    m_axisX(axisX),
    m_axisY(axisY),
    m_currentX(0U),
    m_currentY(0U),
    m_visible(false),
    m_hasSample(false),
    m_trackCalLoaded(false),
    m_minX(0U),
    m_minY(0U),
    m_maxX(0U),
    m_maxY(0U),
    m_windowMinX(0U),
    m_windowMaxX(4095U),
    m_windowMinY(0U),
    m_windowMaxY(4095U) {
}

void GUIJViewComponent::UpdateWindow(void) {
  if ((m_axisX == nullptr) || (m_axisY == nullptr) || m_mode == GUI_J_VIEW_MODE_CAL_EDGE) {
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
    const uint16_t spanX = (widthX >= 64U) ? widthX : 64U;
    const uint16_t spanY = (widthY >= 64U) ? widthY : 64U;
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
  if (m_mode == GUI_J_VIEW_MODE_CAL_CENTER) return GUIMapLinearWindow(value, m_windowMinX, m_windowMaxX, GUI_J_VIEW_OUT_MIN_X, GUI_J_VIEW_OUT_MAX_X);
  if (m_axisX == nullptr) return GUIMapAxis(value, GUI_J_VIEW_OUT_MIN_X, GUI_J_VIEW_OUT_MAX_X);
  return GUIMapCalibratedValue(value, *m_axisX, GUI_J_VIEW_OUT_MIN_X, GUI_J_VIEW_OUT_MAX_X);
}

uint8_t GUIJViewComponent::MapAxisY(uint16_t value) const {
  if (m_mode == GUI_J_VIEW_MODE_CAL_CENTER) return GUIMapLinearWindow(value, m_windowMinY, m_windowMaxY, GUI_J_VIEW_OUT_MIN_Y, GUI_J_VIEW_OUT_MAX_Y);
  if (m_axisY == nullptr) return GUIMapAxis(value, GUI_J_VIEW_OUT_MIN_Y, GUI_J_VIEW_OUT_MAX_Y);
  return GUIMapCalibratedValue(value, *m_axisY, GUI_J_VIEW_OUT_MIN_Y, GUI_J_VIEW_OUT_MAX_Y);
}

bool GUIJViewComponent::SaveCalibration(void) {
  if (!m_hasSample || (m_axisX == nullptr) || (m_axisY == nullptr)) return false;

  if (m_mode == GUI_J_VIEW_MODE_CAL_CENTER) {
    m_axisX->cMin = (m_minX > 4U) ? (uint16_t)(m_minX - 4U) : 0U;
    m_axisX->cMax = (m_maxX < 4091U) ? (uint16_t)(m_maxX + 4U) : 4095U;
    m_axisY->cMin = (m_minY > 4U) ? (uint16_t)(m_minY - 4U) : 0U;
    m_axisY->cMax = (m_maxY < 4091U) ? (uint16_t)(m_maxY + 4U) : 4095U;
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

void GUIJViewComponent::Enter(void) {
  m_currentX = 0U;
  m_currentY = 0U;
  m_visible = false;
  m_hasSample = false;
  m_minX = 0U;
  m_minY = 0U;
  m_maxX = 0U;
  m_maxY = 0U;
  if (!m_trackCalLoaded) {
    GUILoadCalibrationFromPreferences(m_axisX, m_axisY);
    m_trackCalLoaded = true;
  }
}

void GUIJViewComponent::Process(void) {
  if ((m_mode != GUI_J_VIEW_MODE_TRACK) && rio_changed(RIO_DATA_BTN_OK) && (rio_get(RIO_DATA_BTN_OK) != 0U)) {
    SaveCalibration();
  }
}

void GUIJViewComponent::Draw(void) {
  const bool backlightOn = (rio_get(RIO_DATA_STAT_BL_ON) != 0U);
  uint16_t nextX, nextY;
  if (backlightOn) {
    UpdateWindow();
    const uint16_t rawX = rio_get(RIO_DATA_JOY_X);
    const uint16_t rawY = rio_get(RIO_DATA_JOY_Y);

    if (!m_hasSample) {
      m_hasSample = true;
      m_minX = m_maxX = rawX;
      m_minY = m_maxY = rawY;
    } else {
      if (rawX < m_minX) m_minX = rawX;
      if (rawX > m_maxX) m_maxX = rawX;
      if (rawY < m_minY) m_minY = rawY;
      if (rawY > m_maxY) m_maxY = rawY;
    }

    nextX = MapAxisX(rawX);
    nextY = MapAxisY(rawY);
    if (m_visible && (m_currentX == nextX) && (m_currentY == nextY)) return;
  } else {
    if (!m_visible) return;
  }

  if (m_mode == GUI_J_VIEW_MODE_TRACK && m_visible) {
    LCD_DrawMarker(m_currentX, m_currentY, 3U, GUI_COLOR_BLACK);
    m_visible = false;
  }
  if (!backlightOn) return;

  const uint8_t markerIndex = (m_mode == GUI_J_VIEW_MODE_CAL_CENTER) ? 5U : 3U;
  const uint16_t markerColor = (m_mode == GUI_J_VIEW_MODE_TRACK)        ? GUI_COLOR_WHITE
                               : (m_mode == GUI_J_VIEW_MODE_CAL_CENTER) ? GUI_COLOR_MAGENTA
                                                                        : GUI_COLOR_CYAN;
  LCD_DrawMarker(nextX, nextY, markerIndex, markerColor);

  m_currentX = nextX;
  m_currentY = nextY;
  m_visible = true;
}

void GUIJViewComponent::Exit(void) {
  m_visible = false;
}

// -----------------------------------------------------------------------------
// Section: GUIHotKeyComponent
// -----------------------------------------------------------------------------

uint8_t GUIHotKeyComponent::GetClassId(void) const {
  return (uint8_t)GUI_CLASS_HOT_KEY;
}

GUIHotKeyComponent::GUIHotKeyComponent(rio_data_idx_t idx, gui_scene_t* targetScene)
  : m_idx(idx),
    m_targetScene(targetScene) {
}

void GUIHotKeyComponent::Enter(void) {
}

void GUIHotKeyComponent::Process(void) {
  if (rio_changed(m_idx) && (rio_get(m_idx) != 0U) && (m_targetScene != nullptr)) {
    GUISwitchScene(m_targetScene);
  }
}

void GUIHotKeyComponent::Draw(void) {
}

void GUIHotKeyComponent::Exit(void) {
}

// -----------------------------------------------------------------------------
// Section: GUILabelComponent
// -----------------------------------------------------------------------------

uint8_t GUILabelComponent::GetClassId(void) const {
  return (uint8_t)GUI_CLASS_LABEL;
}

GUILabelComponent::GUILabelComponent(uint8_t x, uint8_t y, uint16_t color, const char* text)
  : m_x(x),
    m_y(y),
    m_color(color),
    m_text(text),
    m_pending(false) {
}

void GUILabelComponent::Enter(void) {
  m_pending = true;
}

void GUILabelComponent::Process(void) {
}

void GUILabelComponent::Draw(void) {
  if (!m_pending) return;

  (void)LCD_DrawText(m_x, m_y, m_color, m_text);
  m_pending = false;
}

void GUILabelComponent::Exit(void) {
  m_pending = false;
}

// -----------------------------------------------------------------------------
// Section: GUIVarComponent
// -----------------------------------------------------------------------------
uint8_t GUIVarComponent::GetClassId(void) const {
  return (uint8_t)GUI_CLASS_VAR;
}

GUIVarComponent::GUIVarComponent(uint8_t x, uint8_t y, uint16_t color, const int32_t* value)
  : m_x(x),
    m_y(y),
    m_color(color),
    m_type(VALUE_INT32),
    m_value(value),
    m_lastValue{0},
    m_hasDrawn(false),
    m_pendingDraw(false) {
  m_drawnText[0] = '\0';
  m_nextText[0] = '\0';
}

GUIVarComponent::GUIVarComponent(uint8_t x, uint8_t y, uint16_t color, const float* value)
  : m_x(x),
    m_y(y),
    m_color(color),
    m_type(VALUE_FLOAT),
    m_value(value),
    m_lastValue{0},
    m_hasDrawn(false),
    m_pendingDraw(false) {
  m_drawnText[0] = '\0';
  m_nextText[0] = '\0';
}

GUIVarComponent::GUIVarComponent(uint8_t x, uint8_t y, uint16_t color, const char* value)
  : m_x(x),
    m_y(y),
    m_color(color),
    m_type(VALUE_CSTR),
    m_value(value),
    m_lastValue{0},
    m_hasDrawn(false),
    m_pendingDraw(false) {
  m_drawnText[0] = '\0';
  m_nextText[0] = '\0';
}

GUIVarComponent::GUIVarComponent(uint8_t x, uint8_t y, uint16_t color, char* const* value)
  : m_x(x),
    m_y(y),
    m_color(color),
    m_type(VALUE_CSTR_PTR),
    m_value(value),
    m_lastValue{0},
    m_hasDrawn(false),
    m_pendingDraw(false) {
  m_drawnText[0] = '\0';
  m_nextText[0] = '\0';
}

GUIVarComponent::GUIVarComponent(uint8_t x, uint8_t y, uint16_t color, const char* const* value)
  : m_x(x),
    m_y(y),
    m_color(color),
    m_type(VALUE_CSTR_PTR),
    m_value(value),
    m_lastValue{0},
    m_hasDrawn(false),
    m_pendingDraw(false) {
  m_drawnText[0] = '\0';
  m_nextText[0] = '\0';
}

void GUIVarComponent::FormatValue(char* out, size_t outSize) const {
  if ((out == nullptr) || (outSize == 0U)) return;

  out[0] = '\0';
  if (m_value == nullptr) return;

  switch (m_type) {
    case VALUE_INT32:
      (void)snprintf(out, outSize, "%ld", (long)(*static_cast<const int32_t*>(m_value)));
      break;

    case VALUE_FLOAT:
      (void)snprintf(out, outSize, "%.2f", (double)(*static_cast<const float*>(m_value)));
      break;

    case VALUE_CSTR:
      (void)snprintf(out, outSize, "%s", static_cast<const char*>(m_value));
      break;

    case VALUE_CSTR_PTR:
      {
        const char* const* textPtr = static_cast<const char* const*>(m_value);
        const char* text = (textPtr != nullptr) ? *textPtr : nullptr;
        (void)snprintf(out, outSize, "%s", (text != nullptr) ? text : "");
        break;
      }

    default:
      break;
  }

  out[outSize - 1U] = '\0';
}

void GUIVarComponent::Enter(void) {
  m_drawnText[0] = '\0';
  m_nextText[0] = '\0';
  m_lastValue.i32 = 0;
  m_lastValue.f32 = 0.0f;

  if (m_value != nullptr) {
    if (m_type == VALUE_INT32) {
      m_lastValue.i32 = *static_cast<const int32_t*>(m_value);
    } else if (m_type == VALUE_FLOAT) {
      m_lastValue.f32 = *static_cast<const float*>(m_value);
    }
  }

  m_hasDrawn = false;
  m_pendingDraw = true;
}

void GUIVarComponent::Process(void) {
  if (m_pendingDraw) return;

  switch (m_type) {
    case VALUE_INT32:
      if (m_value == nullptr) return;
      {
        const int32_t value = *static_cast<const int32_t*>(m_value);
        if (m_hasDrawn && (value == m_lastValue.i32)) return;
        m_lastValue.i32 = value;
      }
      FormatValue(m_nextText, sizeof(m_nextText));
      m_pendingDraw = true;
      break;

    case VALUE_FLOAT:
      if (m_value == nullptr) return;
      {
        const float value = *static_cast<const float*>(m_value);
        if (m_hasDrawn && (value == m_lastValue.f32)) return;
        m_lastValue.f32 = value;
      }
      FormatValue(m_nextText, sizeof(m_nextText));
      m_pendingDraw = true;
      break;

    case VALUE_CSTR:
    case VALUE_CSTR_PTR:
      {
        char text[sizeof(m_nextText)];
        FormatValue(text, sizeof(text));
        if (!m_hasDrawn || (strcmp(text, m_drawnText) != 0)) {
          (void)snprintf(m_nextText, sizeof(m_nextText), "%s", text);
          m_nextText[sizeof(m_nextText) - 1U] = '\0';
          m_pendingDraw = true;
        }
      }
      break;

    default:
      break;
  }
}

void GUIVarComponent::Draw(void) {
  if (!m_pendingDraw) return;

  if (m_nextText[0] == '\0') {
    FormatValue(m_nextText, sizeof(m_nextText));
  }

  char drawText[sizeof(m_nextText)];
  (void)snprintf(drawText, sizeof(drawText), "%s", m_nextText);
  drawText[sizeof(drawText) - 1U] = '\0';

  const size_t nextLen = strlen(m_nextText);
  const size_t drawnLen = strlen(m_drawnText);
  if (drawnLen > nextLen) {
    size_t i = nextLen;
    while ((i < drawnLen) && (i < (sizeof(drawText) - 1U))) {
      drawText[i++] = ' ';
    }
    drawText[i] = '\0';
  }

  LCD_DrawText(m_x, m_y, m_color, drawText);

  (void)snprintf(m_drawnText, sizeof(m_drawnText), "%s", m_nextText);
  m_drawnText[sizeof(m_drawnText) - 1U] = '\0';
  m_hasDrawn = true;
  m_pendingDraw = false;
}

void GUIVarComponent::Exit(void) {
  m_pendingDraw = false;
}

// -----------------------------------------------------------------------------
// Section: GUIMenuItemComponent
// -----------------------------------------------------------------------------

uint8_t GUIMenuItemComponent::GetClassId(void) const {
  return (uint8_t)GUI_CLASS_MENU_ITEM;
}

GUIMenuItemComponent::GUIMenuItemComponent(uint8_t x, uint8_t y, const char* text, gui_scene_t* targetScene)
  : m_x(x),
    m_y(y),
    m_text(text),
    m_targetScene(targetScene),
    m_active(false),
    m_prevActive(false) {
}

namespace {
static GUIMenuItemComponent* GUIMenuItemCast(GUIComponent* component) {
  if ((component != nullptr) && (component->GetClassId() == (uint8_t)GUI_CLASS_MENU_ITEM)) {
    return static_cast<GUIMenuItemComponent*>(component);
  }
  return nullptr;
}

static GUIMenuItemComponent* GUIMenuFindFirst(void) {
  gui_scene_t* const scene = s_guiActiveScene;
  if ((scene == nullptr) || (scene->components == nullptr)) return nullptr;
  for (size_t i = 0U; i < scene->componentCount; ++i) {
    GUIMenuItemComponent* item = GUIMenuItemCast(scene->components[i]);
    if (item != nullptr) return item;
  }
  return nullptr;
}

static GUIMenuItemComponent* GUIMenuFindAdjacent(const GUIMenuItemComponent* from, int step) {
  gui_scene_t* const scene = s_guiActiveScene;
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
    if (index < 0) index = count - 1;
    else if (index >= count) index = 0;

    GUIMenuItemComponent* item = GUIMenuItemCast(scene->components[index]);
    if (item != nullptr) return item;
  }
  return nullptr;
}
}

GUIMenuItemComponent* GUIMenuFindActive(void) {
  gui_scene_t* const scene = s_guiActiveScene;
  if ((scene == nullptr) || (scene->components == nullptr)) return nullptr;
  for (size_t i = 0U; i < scene->componentCount; ++i) {
    GUIMenuItemComponent* item = GUIMenuItemCast(scene->components[i]);
    if ((item != nullptr) && item->IsActive()) return item;
  }
  return nullptr;
}

void GUIMenuItemComponent::SetActive(bool active) {
  if (m_active != active) {
    m_active = active;
  }
}
bool GUIMenuItemComponent::IsActive(void){
  return m_active;
}

void GUIMenuItemComponent::Enter(void) {
  if ((GUIMenuFindActive() == nullptr) && (GUIMenuFindFirst() == this)) {
    m_active = true;
  }
  m_prevActive = !m_active;  // redraw
}

void GUIMenuItemComponent::Process(void) {
  if (!m_prevActive) return;
  // m_prevActive is used here to protect against repeated DOWN button handling.

  if (rio_changed(RIO_DATA_BTN_UP) && (rio_get(RIO_DATA_BTN_UP) != 0U)) {
    GUIMenuItemComponent* next = GUIMenuFindAdjacent(this, -1);
    if ((next != nullptr) && (next != this)) {
      SetActive(false);
      next->SetActive(true);
    }
  } else if (rio_changed(RIO_DATA_BTN_DOWN) && (rio_get(RIO_DATA_BTN_DOWN) != 0U)) {
    GUIMenuItemComponent* next = GUIMenuFindAdjacent(this, 1);
    if ((next != nullptr) && (next != this)) {
      SetActive(false);
      next->SetActive(true);
    }
  } else if (rio_changed(RIO_DATA_BTN_OK) && (rio_get(RIO_DATA_BTN_OK) != 0U) && (m_targetScene != nullptr)) {
    GUISwitchScene(m_targetScene);
  }
}

void GUIMenuItemComponent::Draw(void) {
  if (m_active == m_prevActive) return;

  char text[LCD_MAX_TEXT_LEN + 1U];
  text[0] = m_active ? '>' : ' ';
  text[1] = ' ';
  if (m_text == nullptr) {
    text[2] = '\0';
  } else {
    (void)snprintf(&text[2], sizeof(text) - 2U, "%s", m_text);
    text[sizeof(text) - 1U] = '\0';
  }

  const uint16_t color = m_active ? GUI_COLOR_WHITE : GUI_COLOR_GRAY;
  LCD_DrawText(m_x, m_y, color, text);
  m_prevActive = m_active;
}

void GUIMenuItemComponent::Exit(void) {
}

// -----------------------------------------------------------------------------
// Section: GUIBrightnessComponent
// -----------------------------------------------------------------------------

namespace {
static constexpr uint8_t GUI_BRIGHTNESS_STEP_TABLE[10] = {
  1U, 2U, 3U, 5U, 8U, 13U, 22U, 36U, 61U, 127U
};

static uint8_t GUIBrightnessToStep(uint8_t index) {
  if (index > 9U) index = 9U;
  return GUI_BRIGHTNESS_STEP_TABLE[index];
}

static void GUIBrightnessApply(uint8_t mode, uint8_t index) {
  uint8_t step = GUIBrightnessToStep(index);
  if (mode == 0U) {
    step = (uint8_t)((step + 1U) / 2U);
  }
  rio_cmd_set_brightness(step);
}
}

bool GUIBrightnessComponent::s_loaded = false;
uint8_t GUIBrightnessComponent::s_storedIndex = 5U;

uint8_t GUIBrightnessComponent::GetClassId(void) const {
  return (uint8_t)GUI_CLASS_BRIGHTNESS;
}

GUIBrightnessComponent::GUIBrightnessComponent(uint8_t mode, uint8_t x, uint8_t y)
  : m_mode(mode),
    m_x(x),
    m_y(y),
    m_actualIndex(5U),
    m_pendingDraw(false) {
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
  s_loaded = true;
}

bool GUIBrightnessComponent::SaveStoredIndex(uint8_t index) {
  if (index > 9U) index = 9U;

  Preferences prefs;
  if (!prefs.begin("gui", false)) return false;

  prefs.putUInt("br_idx", index);
  prefs.end();
  s_storedIndex = index;
  return true;
}

void GUIBrightnessComponent::DrawValue(void) {
  char text[4];
  (void)snprintf(text, sizeof(text), "%u", (unsigned)m_actualIndex);
  text[sizeof(text) - 1U] = '\0';

  (void)LCD_DrawText(m_x, m_y, GUI_COLOR_ORANGE, text);
  m_pendingDraw = false;
}

bool GUIBrightnessComponent::ProcessInput(void) {
  if (m_mode != 1U) return false;

  bool changed = false;
  if (rio_changed(RIO_DATA_BTN_LUP) && (rio_get(RIO_DATA_BTN_LUP) != 0U) && (m_actualIndex < 9U)) {
    ++m_actualIndex;
    changed = true;
  }
  if (rio_changed(RIO_DATA_BTN_LDN) && (rio_get(RIO_DATA_BTN_LDN) != 0U) && (m_actualIndex > 0U)) {
    --m_actualIndex;
    changed = true;
  }

  if (changed) {
    GUIBrightnessApply(m_mode, m_actualIndex);
    m_pendingDraw = true;
  }
  return changed;
}

void GUIBrightnessComponent::Enter(void) {
  EnsureLoaded();
  m_actualIndex = s_storedIndex;
  GUIBrightnessApply(m_mode, m_actualIndex);
  m_pendingDraw = (m_mode == 1U);
}

void GUIBrightnessComponent::Process(void) {
  (void)ProcessInput();
}

void GUIBrightnessComponent::Draw(void) {
  if ((m_mode == 1U) && m_pendingDraw) {
    DrawValue();
  }
}

void GUIBrightnessComponent::Exit(void) {
  if (m_actualIndex != s_storedIndex) {
    (void)SaveStoredIndex(m_actualIndex);
  }
  m_pendingDraw = false;
}

// -----------------------------------------------------------------------------
// Section: Common
// -----------------------------------------------------------------------------

static bool GUIHandleIdleHomeTimeout(void) {
  if (rio_get(RIO_DATA_BTN_ANYKEY) != 0U) {
    s_guiLastActivityMs = millis();
    return false;
  }
  if ((s_guiHomeScene == nullptr) || (s_guiActiveScene == nullptr) || (s_guiActiveScene == s_guiHomeScene)) return false;
  const uint32_t now = millis();
  if ((uint32_t)(now - s_guiLastActivityMs) < GUI_IDLE_HOME_TIMEOUT_MS) return false;
  GUISwitchScene(s_guiHomeScene);
  return true;
}

static void GUISceneEnter(gui_scene_t* scene) {
  if ((scene == nullptr) || (scene->components == nullptr)) return;
  for (size_t i = 0U; i < scene->componentCount; ++i) {
    GUIComponent* component = scene->components[i];
    if (component != nullptr) component->Enter();
  }
}

static void GUISceneLeave(gui_scene_t* scene) {
  if ((scene == nullptr) || (scene->components == nullptr)) return;
  for (size_t i = 0U; i < scene->componentCount; ++i) {
    GUIComponent* component = scene->components[i];
    if (component != nullptr) component->Exit();
  }
}

void GUISwitchScene(gui_scene_t* scene) {
  if (s_guiActiveScene == scene) return;
  GUISceneLeave(s_guiActiveScene);
  s_guiActiveScene = scene;
  GUISceneEnter(s_guiActiveScene);
}

void GUISetHomeScene(gui_scene_t* scene) {
  s_guiHomeScene = scene;
  s_guiLastActivityMs = millis();
}

gui_scene_t* GUIGetActiveScene(void) {
  return s_guiActiveScene;
}

void GUIServiceActiveScene(void) {
  gui_scene_t* const scene = s_guiActiveScene;
  if ((scene != nullptr) && (scene->components != nullptr) && !GUIHandleIdleHomeTimeout()) {
    for (size_t i = 0U; i < scene->componentCount; ++i) {
      GUIComponent* component = scene->components[i];
      if (component == nullptr) continue;
      component->Process();
      if (s_guiActiveScene != scene) return;
    }

    for (size_t i = 0U; i < scene->componentCount; ++i) {
      GUIComponent* component = scene->components[i];
      if (component == nullptr) continue;
      component->Draw();
      if (s_guiActiveScene != scene) return;
    }
  }
  for (uint8_t i = 0U; i < 2U; ++i) {
    if (s_guiIndicatorPending[i]) {
      (void)LCD_DrawIndicator(i, s_guiIndicatorValue[i]);
      s_guiIndicatorPending[i] = false;
    }
  }
  for (uint8_t i = 0U; i < 4U; ++i) {
    (void)LCD_DrawProgressBar(i, s_guiProgressValue[i]);
  }
}

void GUISetIndicator(uint8_t index, uint8_t state) {
  if (index >= 2U) return;
  if ((s_guiIndicatorValue[index] != state) || s_guiIndicatorPending[index]) {
    s_guiIndicatorValue[index] = state;
    s_guiIndicatorPending[index] = true;
  }
}

void GUISetProgress(uint8_t index, uint8_t value) {
  if (index >= 4U) return;
  s_guiProgressValue[index] = value;
}
