#ifndef GUI_H
#define GUI_H

#include <stddef.h>
#include <stdint.h>

#include "rio.h"
#include "st7735.h"
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
class GUIVarComponent;
class GUIMenuItemComponent;
class GUIBrightnessComponent;

typedef struct {
    GUIComponent** components;
    size_t componentCount;
} gui_scene_t;

#define GUI_SCENE(_items) { (_items), sizeof(_items) / sizeof((_items)[0]) }

void GUISwitchScene(gui_scene_t* scene);
void GUISetHomeScene(gui_scene_t* scene);
gui_scene_t* GUIGetActiveScene(void);
void GUIServiceActiveScene(void);
void GUISetIndicator(uint8_t index, bool state);
void GUISetProgress(uint8_t index, uint8_t value);

// -----------------------------------------------------------------------------
// Section: GUIComponent
// -----------------------------------------------------------------------------

class GUIComponent {
public:
    virtual ~GUIComponent() = default;

    virtual uint8_t GetClassId(void) const = 0;
    virtual void Enter(void) = 0;
    virtual void Process(void) = 0;
    virtual void Draw(void) = 0;
    virtual void Exit(void) = 0;
};

// -----------------------------------------------------------------------------
// Section: GUIClsComponent
// -----------------------------------------------------------------------------

class GUIClsComponent : public GUIComponent {
public:
    GUIClsComponent(uint16_t color);

    uint8_t GetClassId(void) const override;
    void Enter(void) override;
    void Process(void) override;
    void Draw(void) override;
    void Exit(void) override;

private:
    uint16_t m_color;
    bool m_pendingClear;
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

class GUIJViewComponent : public GUIComponent {
public:
    GUIJViewComponent(gui_j_view_mode_t mode,
                      gui_axis_cal_t* axisX,
                      gui_axis_cal_t* axisY);

    uint8_t GetClassId(void) const override;
    void Enter(void) override;
    void Process(void) override;
    void Draw(void) override;
    void Exit(void) override;

private:
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
    bool m_visible;
    bool m_hasSample;
    bool m_trackCalLoaded;
    uint16_t m_minX;
    uint16_t m_minY;
    uint16_t m_maxX;
    uint16_t m_maxY;
    uint16_t m_windowMinX;
    uint16_t m_windowMaxX;
    uint16_t m_windowMinY;
    uint16_t m_windowMaxY;
};

// -----------------------------------------------------------------------------
// Section: GUIHotKeyComponent
// -----------------------------------------------------------------------------

class GUIHotKeyComponent : public GUIComponent {
public:
    GUIHotKeyComponent(rio_data_idx_t idx, gui_scene_t* targetScene);

    uint8_t GetClassId(void) const override;
    void Enter(void) override;
    void Process(void) override;
    void Draw(void) override;
    void Exit(void) override;

private:
    rio_data_idx_t m_idx;
    gui_scene_t* m_targetScene;
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
    void Draw(void) override;
    void Exit(void) override;

private:
    uint8_t m_x;
    uint8_t m_y;
    uint16_t m_color;
    const char* m_text;
    bool m_pending;
};

// -----------------------------------------------------------------------------
// Section: GUIVarComponent
// -----------------------------------------------------------------------------
class GUIVarComponent : public GUIComponent {
public:
    GUIVarComponent(uint8_t x, uint8_t y, uint16_t color, const int32_t* value);
    GUIVarComponent(uint8_t x, uint8_t y, uint16_t color, const float* value);
    GUIVarComponent(uint8_t x, uint8_t y, uint16_t color, const char* value);
    GUIVarComponent(uint8_t x, uint8_t y, uint16_t color, char* const* value);
    GUIVarComponent(uint8_t x, uint8_t y, uint16_t color, const char* const* value);

    uint8_t GetClassId(void) const override;
    void Enter(void) override;
    void Process(void) override;
    void Draw(void) override;
    void Exit(void) override;
private:
    enum value_type_t : uint8_t {
        VALUE_INT32,
        VALUE_FLOAT,
        VALUE_CSTR,
        VALUE_CSTR_PTR
    };
    void FormatValue(char* out, size_t outSize) const;

    uint8_t m_x;
    uint8_t m_y;
    uint16_t m_color;
    value_type_t m_type;
    const void* m_value;
    char m_drawnText[LCD_MAX_TEXT_LEN + 1U];
    char m_nextText[LCD_MAX_TEXT_LEN + 1U];
    union {
        int32_t i32;
        float f32;
    } m_lastValue;
    bool m_hasDrawn;
    bool m_pendingDraw;
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
    void Draw(void) override;
    void Exit(void) override;

    void SetActive(bool active);
    bool IsActive(void);

private:
    uint8_t m_x;
    uint8_t m_y;
    const char* m_text;
    gui_scene_t* m_targetScene;
    bool m_active;
    bool m_prevActive;
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
    void Draw(void) override;
    void Exit(void) override;

private:
    static void EnsureLoaded(void);
    static bool SaveStoredIndex(uint8_t index);

    bool ProcessInput(void);
    void DrawValue(void);

    uint8_t m_mode;
    uint8_t m_x;
    uint8_t m_y;
    uint8_t m_actualIndex;
    bool m_pendingDraw;

    static bool s_loaded;
    static uint8_t s_storedIndex;
};

#endif // GUI_H
