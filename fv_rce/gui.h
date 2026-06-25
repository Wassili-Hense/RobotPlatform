#ifndef GUI_H
#define GUI_H

#include <stddef.h>
#include <stdint.h>

#include "hmi.h"
// -----------------------------------------------------------------------------
// Section: Common
// -----------------------------------------------------------------------------
static constexpr uint16_t GUI_COLOR_BLACK   = 0x0000U;
static constexpr uint16_t GUI_COLOR_WHITE   = 0xFFFFU;
static constexpr uint16_t GUI_COLOR_CYAN    = 0x07FFU;
static constexpr uint16_t GUI_COLOR_GREEN   = 0x07E0U;
static constexpr uint16_t GUI_COLOR_ORANGE  = 0xFD20U;
static constexpr uint16_t GUI_COLOR_MAGENTA = 0xF81FU;
static constexpr uint16_t GUI_COLOR_GRAY    = 0x7BEFU;

class GUIComponent;
class GUIClsComponent;
class GUIJViewComponent;
class GUIHotKeyComponent;
class GUILabelComponent;
class GUIMenuItemComponent;
class GUIBrightnessComponent;

typedef struct {
    GUIComponent** components;
    size_t componentCount;
} gui_scene_t;

#define GUI_SCENE(_items) { (_items), sizeof(_items) / sizeof((_items)[0]) }

void GUISwitchScene(gui_scene_t* scene);
gui_scene_t* GUIGetActiveScene(void);
bool GUIServiceActiveScene(void);

// -----------------------------------------------------------------------------
// Section: GUIComponent
// -----------------------------------------------------------------------------

class GUIComponent {
public:
    virtual ~GUIComponent() = default;

    virtual uint8_t GetClassId(void) const = 0;
    virtual void Enter(void) = 0;
    virtual void Process(void) = 0;
    virtual bool Send(void) = 0;
    virtual void Exit(void) = 0;
};

// -----------------------------------------------------------------------------
// Section: GUIClsComponent
// -----------------------------------------------------------------------------

class GUIClsComponent : public GUIComponent {
public:
    GUIClsComponent(uint16_t color, bool highlight);

    uint8_t GetClassId(void) const override;
    void Enter(void) override;
    void Process(void) override;
    bool Send(void) override;
    void Exit(void) override;

private:
    bool SendBacklightKeepOn(void);

    uint16_t m_color;
    bool m_highlight;
    bool m_pendingClear;
    uint32_t m_nextKeepAliveMs;
};

// -----------------------------------------------------------------------------
// Section: GUIJViewComponent
// -----------------------------------------------------------------------------
typedef struct {
    uint16_t eMin;
    uint16_t cMin;
    uint16_t cMax;
    uint16_t eMax;
} gui_axis_cal_t;

enum gui_j_view_mode_t {
    GUI_J_VIEW_MODE_TRACK = 1,
    GUI_J_VIEW_MODE_CAL_CENTER = 2,
    GUI_J_VIEW_MODE_CAL_EDGE = 3
};

enum gui_j_view_phase_t {
    GUI_J_VIEW_PHASE_IDLE = 0,
    GUI_J_VIEW_PHASE_ERASE,
    GUI_J_VIEW_PHASE_DRAW
};

class GUIJViewComponent : public GUIComponent {
public:
    GUIJViewComponent(gui_j_view_mode_t mode,
                      gui_axis_cal_t* axisX,
                      gui_axis_cal_t* axisY);

    uint8_t GetClassId(void) const override;
    void Enter(void) override;
    void Process(void) override;
    bool Send(void) override;
    void Exit(void) override;

private:
    bool Update(void);
    bool HandleButtons(void);
    bool SaveCalibration(void);
    void UpdateWindow(void);
    uint8_t MapAxisX(uint16_t value) const;
    uint8_t MapAxisY(uint16_t value) const;

    gui_j_view_mode_t m_mode;
    gui_axis_cal_t* m_axisX;
    gui_axis_cal_t* m_axisY;
    gui_scene_t* m_targetScene;
    uint8_t m_currentX;
    uint8_t m_currentY;
    uint8_t m_nextX;
    uint8_t m_nextY;
    bool m_visible;
    bool m_pending;
    bool m_hasSample;
    bool m_trackCalLoaded;
    uint16_t m_rawX;
    uint16_t m_rawY;
    uint16_t m_minX;
    uint16_t m_minY;
    uint16_t m_maxX;
    uint16_t m_maxY;
    uint16_t m_windowMinX;
    uint16_t m_windowMaxX;
    uint16_t m_windowMinY;
    uint16_t m_windowMaxY;
    gui_j_view_phase_t m_phase;
};

// -----------------------------------------------------------------------------
// Section: GUIHotKeyComponent
// -----------------------------------------------------------------------------

class GUIHotKeyComponent : public GUIComponent {
public:
    GUIHotKeyComponent(hmi_data_idx_t idx, gui_scene_t* targetScene);

    uint8_t GetClassId(void) const override;
    void Enter(void) override;
    void Process(void) override;
    bool Send(void) override;
    void Exit(void) override;

private:
    hmi_data_idx_t m_idx;
};

// -----------------------------------------------------------------------------
// Section: GUILabelComponent
// -----------------------------------------------------------------------------

class GUILabelComponent : public GUIComponent {
public:
    GUILabelComponent(uint8_t x, uint8_t y, uint16_t color, const char* text);

    uint8_t GetClassId(void) const override;
    void Enter(void) override;
    void Process(void) override;
    bool Send(void) override;
    void Exit(void) override;

private:
    uint8_t m_x;
    uint8_t m_y;
    uint16_t m_color;
    const char* m_text;
    bool m_pending;
};

// -----------------------------------------------------------------------------
// Section: GUIMenuItemComponent
// -----------------------------------------------------------------------------

class GUIMenuItemComponent : public GUIComponent {
public:
    GUIMenuItemComponent(uint8_t x, uint8_t y, const char* text, gui_scene_t* targetScene);

    uint8_t GetClassId(void) const override;
    void Enter(void) override;
    void Process(void) override;
    bool Send(void) override;
    void Exit(void) override;

    void SetActive(bool active);

private:
    friend GUIMenuItemComponent* GUIMenuFindActive(void);
    bool Draw(bool active);
    bool ProcessNavigation(void);


    uint8_t m_x;
    uint8_t m_y;
    const char* m_text;
    bool m_active;
    bool m_prevActive;
    bool m_pending;
};

// -----------------------------------------------------------------------------
// Section: GUIBrightnessComponent
// -----------------------------------------------------------------------------

class GUIBrightnessComponent : public GUIComponent {
public:
    GUIBrightnessComponent(uint8_t mode, uint8_t x, uint8_t y);

    uint8_t GetClassId(void) const override;
    void Enter(void) override;
    void Process(void) override;
    bool Send(void) override;
    void Exit(void) override;

private:
    static void EnsureLoaded(void);
    static bool SaveStoredIndex(uint8_t index);

    bool ProcessInput(void);
    bool DrawValue(void);

    uint8_t m_mode;
    uint8_t m_x;
    uint8_t m_y;
    uint8_t m_actualIndex;
    bool m_pendingDraw;

    static bool s_loaded;
    static uint8_t s_storedIndex;
};

#endif // GUI_H
