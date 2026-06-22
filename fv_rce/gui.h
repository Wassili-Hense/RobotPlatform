#ifndef GUI_H
#define GUI_H

#include <stddef.h>
#include <stdint.h>
#include "hmi.h"

typedef enum {
  GUI_CALL_ENTER = 0,
  GUI_CALL_PROCESS,
  GUI_CALL_PROCESS_AND_SEND,
  GUI_CALL_EXIT
} gui_call_t;

typedef enum {
  GUI_J_VIEW_PHASE_IDLE = 0,
  GUI_J_VIEW_PHASE_ERASE,
  GUI_J_VIEW_PHASE_DRAW
} gui_j_view_phase_t;

struct gui_scene_t;

typedef struct gui_scene_t {
  class GUIComponent* const* components;
  size_t componentCount;
} gui_scene_t;

class GUIComponent {
public:
  virtual bool Handle(gui_call_t call) = 0;
  virtual bool IsMenuItem(void) const { return false; }
  virtual bool IsMenuActive(void) const { return false; }
  virtual void SetMenuActive(bool active) { (void)active; }
  virtual gui_scene_t* GetMenuTargetScene(void) const { return nullptr; }
  virtual ~GUIComponent() = default;
protected:
  GUIComponent() = default;
};

class GUIClsComponent final : public GUIComponent {
public:
  explicit GUIClsComponent(uint16_t color);
  bool Handle(gui_call_t call) override;
private:
  uint16_t m_color;
  bool m_pending;
};

class GUIJViewComponent final : public GUIComponent {
public:
  GUIJViewComponent();
  bool Handle(gui_call_t call) override;
private:
  bool Update();
  uint8_t m_currentX;
  uint8_t m_currentY;
  uint8_t m_nextX;
  uint8_t m_nextY;
  bool m_visible;
  gui_j_view_phase_t m_phase;
};

class GUIHotKeyComponent final : public GUIComponent {
public:
  GUIHotKeyComponent(hmi_data_idx_t idx, gui_scene_t* targetScene);
  bool Handle(gui_call_t call) override;
private:
  hmi_data_idx_t m_idx;
  gui_scene_t* m_targetScene;
};

class GUILabelComponent final : public GUIComponent {
public:
  GUILabelComponent(uint8_t x, uint8_t y, uint16_t color, const char* text);
  bool Handle(gui_call_t call) override;
private:
  uint8_t m_x;
  uint8_t m_y;
  uint16_t m_color;
  const char* m_text;
  bool m_pending;
};

class GUIMenuItemComponent final : public GUIComponent {
public:
  GUIMenuItemComponent(uint8_t x, uint8_t y, const char* text, gui_scene_t* targetScene);
  bool Handle(gui_call_t call) override;
  bool IsMenuItem(void) const override;
  bool IsMenuActive(void) const override;
  void SetMenuActive(bool active) override;
  gui_scene_t* GetMenuTargetScene(void) const override;
private:
  bool Draw(bool active);
  uint8_t m_x;
  uint8_t m_y;
  const char* m_text;
  gui_scene_t* m_targetScene;
  bool m_active;
  bool m_pending;
};

uint8_t GUIMapAxis(uint16_t value, uint8_t outMin, uint8_t outMax);
void GUISwitchScene(gui_scene_t* scene);
gui_scene_t* GUIGetActiveScene(void);
bool GUIServiceActiveScene(void);

#endif  // GUI_H
