#ifndef HMI_HELPERS_H
#define HMI_HELPERS_H

#include <stddef.h>
#include <stdint.h>
#include "hmi.h"

enum hmis_cmd_type_t {
  HMIS_CMD_PROGRESS = 0,
  HMIS_CMD_INDICATOR,
  HMIS_CMD_BEEP,
  HMIS_CMD_BRIGHTNESS,
  HMIS_CMD_BL_TIMEOUT
};

typedef struct {
  uint8_t index;
} hmis_cmd_config_t;

typedef union {
  struct { uint8_t value; } progress;
  struct { uint8_t value; } indicator;
  struct { uint16_t divider; uint16_t durationMs; } beep;
  struct { uint8_t level; } brightness;
  struct { uint32_t timeoutMs; } blTimeout;
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

hmi_cmd_result_t HmisSendProgress(hmis_cmd_t* cmd);
hmi_cmd_result_t HmisSendIndicator(hmis_cmd_t* cmd);
hmi_cmd_result_t HmisSendBeep(hmis_cmd_t* cmd);
hmi_cmd_result_t HmisSendBrightness(hmis_cmd_t* cmd);
hmi_cmd_result_t HmisSendBlTimeout(hmis_cmd_t* cmd);
bool HmisIsLcdReady(void);
bool HmisService(hmis_cmd_t* const* cmds, size_t count);

#define HMIS_PROGRESS(_index)   { HMIS_CMD_PROGRESS,   true,  false, HmisSendProgress,   { (uint8_t)(_index) }, { { 0U } } }
#define HMIS_INDICATOR(_index)  { HMIS_CMD_INDICATOR,  true,  false, HmisSendIndicator,  { (uint8_t)(_index) }, { { 0U } } }
#define HMIS_BEEP()             { HMIS_CMD_BEEP,       false, false, HmisSendBeep,       { 0U },               { { 0U } } }
#define HMIS_BRIGHTNESS()       { HMIS_CMD_BRIGHTNESS, false, false, HmisSendBrightness, { 0U },               { { 0U } } }
#define HMIS_BL_TIMEOUT()       { HMIS_CMD_BL_TIMEOUT, false, false, HmisSendBlTimeout,  { 0U },               { { 0U } } }

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

struct hmig_scene_t;

typedef struct hmig_scene_t {
  class HmigComponent* const* components;
  size_t componentCount;
} hmig_scene_t;

class HmigComponent {
public:
  virtual bool Handle(hmig_call_t call) = 0;
  virtual bool IsMenuItem(void) const { return false; }
  virtual bool IsMenuActive(void) const { return false; }
  virtual void SetMenuActive(bool active) { (void)active; }
  virtual hmig_scene_t* GetMenuTargetScene(void) const { return nullptr; }
  virtual ~HmigComponent() = default;
protected:
  HmigComponent() = default;
};

class HmigClsComponent final : public HmigComponent {
public:
  explicit HmigClsComponent(uint16_t color);
  bool Handle(hmig_call_t call) override;
private:
  uint16_t m_color;
  bool m_pending;
};

class HmigJViewComponent final : public HmigComponent {
public:
  HmigJViewComponent();
  bool Handle(hmig_call_t call) override;
private:
  bool Update();
  uint8_t m_currentX;
  uint8_t m_currentY;
  uint8_t m_nextX;
  uint8_t m_nextY;
  bool m_visible;
  hmig_j_view_phase_t m_phase;
};

class HmigHotKeyComponent final : public HmigComponent {
public:
  HmigHotKeyComponent(hmi_data_idx_t idx, hmig_scene_t* targetScene);
  bool Handle(hmig_call_t call) override;
private:
  hmi_data_idx_t m_idx;
  hmig_scene_t* m_targetScene;
};

class HmigLabelComponent final : public HmigComponent {
public:
  HmigLabelComponent(uint8_t x, uint8_t y, uint16_t color, const char* text);
  bool Handle(hmig_call_t call) override;
private:
  uint8_t m_x;
  uint8_t m_y;
  uint16_t m_color;
  const char* m_text;
  bool m_pending;
};

class HmigMenuItemComponent final : public HmigComponent {
public:
  HmigMenuItemComponent(uint8_t x, uint8_t y, const char* text, hmig_scene_t* targetScene);
  bool Handle(hmig_call_t call) override;
  bool IsMenuItem(void) const override;
  bool IsMenuActive(void) const override;
  void SetMenuActive(bool active) override;
  hmig_scene_t* GetMenuTargetScene(void) const override;
private:
  bool Draw(bool active);
  uint8_t m_x;
  uint8_t m_y;
  const char* m_text;
  hmig_scene_t* m_targetScene;
  bool m_active;
  bool m_pending;
};

uint8_t HmigMapAxis(uint16_t value, uint8_t outMin, uint8_t outMax);
void HmigSwitchScene(hmig_scene_t* scene);
hmig_scene_t* HmigGetActiveScene(void);
bool HmigServiceActiveScene(void);

#endif  // HMI_HELPERS_H
