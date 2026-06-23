#ifndef GUI_H
#define GUI_H

#include <stddef.h>
#include <stdint.h>
#include "hmi.h"

class GUIComponent;
typedef const void* gui_class_id_t;

template<typename T>
inline gui_class_id_t GUIClassIdOf(void) {
    return reinterpret_cast<gui_class_id_t>(&GUIClassIdOf<T>);
}

typedef struct {
    GUIComponent** components;
    size_t componentCount;
} gui_scene_t;

#define GUI_SCENE(_items) { (_items), sizeof(_items) / sizeof((_items)[0]) }

static constexpr uint16_t GUI_COLOR_BLACK   = 0x0000U;
static constexpr uint16_t GUI_COLOR_WHITE   = 0xFFFFU;
static constexpr uint16_t GUI_COLOR_CYAN    = 0x07FFU;
static constexpr uint16_t GUI_COLOR_GREEN   = 0x07E0U;
static constexpr uint16_t GUI_COLOR_ORANGE  = 0xFD20U;
static constexpr uint16_t GUI_COLOR_MAGENTA = 0xF81FU;
static constexpr uint16_t GUI_COLOR_GRAY    = 0x7BEFU;

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

class GUIComponent {
public:
    virtual ~GUIComponent() = default;
    virtual gui_class_id_t GetClassId(void) const = 0;
    virtual bool Enter(void) = 0;
    virtual bool Process(void) = 0;
    virtual bool ProcessAndSend(void) = 0;
    virtual bool Exit(void) = 0;
};

template<typename T>
class GUIComponentTyped : public GUIComponent {
public:
    static gui_class_id_t ClassId(void) {
        return GUIClassIdOf<T>();
    }
    gui_class_id_t GetClassId(void) const override {
        return ClassId();
    }
};

class GUIClsComponent : public GUIComponentTyped<GUIClsComponent> {
public:
    GUIClsComponent(uint16_t color, bool highlight);
    bool Enter(void) override;
    bool Process(void) override;
    bool ProcessAndSend(void) override;
    bool Exit(void) override;
private:
    bool SendBacklightKeepAlive(void);
    uint16_t m_color;
    bool m_highlight;
    bool m_pendingClear;
    uint32_t m_nextKeepAliveMs;
};

enum gui_j_view_phase_t {
    GUI_J_VIEW_PHASE_IDLE = 0,
    GUI_J_VIEW_PHASE_ERASE,
    GUI_J_VIEW_PHASE_DRAW
};

class GUIJViewComponent : public GUIComponentTyped<GUIJViewComponent> {
public:
    GUIJViewComponent(gui_j_view_mode_t mode,
                      gui_axis_cal_t* axisX,
                      gui_axis_cal_t* axisY,
                      gui_scene_t* targetScene);
    bool Enter(void) override;
    bool Process(void) override;
    bool ProcessAndSend(void) override;
    bool Exit(void) override;
private:
    bool Update(void);
    bool HandleButtons(void);
    bool SaveCalibration(void);
    void UpdateWindow(void);
    void EnsureTrackCalibrationLoaded(void);
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

class GUIHotKeyComponent : public GUIComponentTyped<GUIHotKeyComponent> {
public:
    GUIHotKeyComponent(hmi_data_idx_t idx, gui_scene_t* targetScene);
    bool Enter(void) override;
    bool Process(void) override;
    bool ProcessAndSend(void) override;
    bool Exit(void) override;
private:
    bool ProcessImpl(void);
    hmi_data_idx_t m_idx;
    gui_scene_t* m_targetScene;
};

class GUILabelComponent : public GUIComponentTyped<GUILabelComponent> {
public:
    GUILabelComponent(uint8_t x, uint8_t y, uint16_t color, const char* text);
    bool Enter(void) override;
    bool Process(void) override;
    bool ProcessAndSend(void) override;
    bool Exit(void) override;
private:
    uint8_t m_x;
    uint8_t m_y;
    uint16_t m_color;
    const char* m_text;
    bool m_pending;
};

class GUIMenuItemComponent : public GUIComponentTyped<GUIMenuItemComponent> {
public:
    GUIMenuItemComponent(uint8_t x, uint8_t y, const char* text, gui_scene_t* targetScene);
    bool Enter(void) override;
    bool Process(void) override;
    bool ProcessAndSend(void) override;
    bool Exit(void) override;
    static void EnsureSceneSelection(gui_scene_t* scene);
private:
    bool Draw(bool active);
    bool ProcessNavigation(void);
    void SetActive(bool active);
    void SyncPrevActive(void);
    static GUIMenuItemComponent* Cast(GUIComponent* component);
    static const GUIMenuItemComponent* Cast(const GUIComponent* component);
    static GUIMenuItemComponent* FindFirst(gui_scene_t* scene);
    static GUIMenuItemComponent* FindActive(gui_scene_t* scene);
    static GUIMenuItemComponent* FindAdjacent(gui_scene_t* scene, const GUIMenuItemComponent* from, int step);
    uint8_t m_x;
    uint8_t m_y;
    const char* m_text;
    gui_scene_t* m_targetScene;
    bool m_active;
    bool m_prevActive;
    bool m_pending;
};


class GUIBrightnessComponent : public GUIComponentTyped<GUIBrightnessComponent> {
public:
    GUIBrightnessComponent(uint8_t mode, uint8_t x, uint8_t y);
    bool Enter(void) override;
    bool Process(void) override;
    bool ProcessAndSend(void) override;
    bool Exit(void) override;

private:
    static void EnsureLoaded(void);
    static bool SaveStoredIndex(void);

    bool ProcessInput(void);
    bool SendBrightness(void);
    bool DrawValue(void);

    uint8_t m_mode;
    uint8_t m_x;
    uint8_t m_y;
    bool m_pendingSend;
    bool m_pendingDraw;

    static bool s_loaded;
    static uint8_t s_storedIndex;
    static uint8_t s_actualIndex;
};

uint8_t GUIMapAxis(uint16_t value, uint8_t outMin, uint8_t outMax);
void GUISwitchScene(gui_scene_t* scene);
gui_scene_t* GUIGetActiveScene(void);
bool GUIServiceActiveScene(void);

#endif // GUI_H
