#pragma once
#include <deque>
#include <string>
#include <vector>

#include "../physics/world.h"
#include "gpu_sim.h"
#include "raylib.h"

// Run state of the simulator. Only Running advances time.
//   Running - simulation advances; Grab / Attract / Repel act on it
//   Paused  - frozen; Step advances one frame; Grab moves objects directly
//   Editing - frozen; edit tools add, draw and erase objects
enum class RunState { Running, Paused, Editing };

enum class Tool { Grab, Attract, Repel, Ball, Rope, Blob, Box, Polygon, Wall, Erase, Count };
enum class ColorMode { Base, Speed, Density };
enum class Tab { Physics, Performance, View };

struct AppOptions {
    int width = 1600;
    int height = 900;
    int scene = 0;
    int threads = 0;            // 0 = all hardware threads
    std::string recordPath;     // non-empty: render a fixed 60 fps timeline into this video via ffmpeg
    std::string captionFile;    // optional: text shown as a caption banner (re-read every frame)
    std::string statusFile;     // optional: current frame number written here every frame
    int recordFrames = 0;       // stop after this many frames when recording (0 = until closed)
    bool gpuSelfTest = false;   // run the GPU path headless against the CPU and exit
};

struct BroadphaseComparison {
    std::string name;
    long long tests;
    int pairs;
    double ms;
    bool matches;
};

class App {
public:
    explicit App(const AppOptions& options);
    ~App();
    int run();

private:
    // ---- frame
    void frame();
    void updateLayout();
    void handleShortcuts();
    void handleWorldMouse();
    void advanceSimulation(float frameTime);
    void stepOnce();

    // ---- state
    void setState(RunState s);
    void selectTool(Tool t);
    bool toolAvailable(Tool t, std::string* reason) const;
    void loadCurrentScene();
    void markWorldEdited();
    void setGpuEnabled(bool on);
    bool gpuAllowed(std::string* reason) const;
    void toast(const std::string& text);
    const char* hintText() const;

    // ---- tools
    // Ball tool pours continuously (fluid, molecules, or Rigid with Spray ticked)
    bool ballSprays() const {
        PhysicsModel m = world_.params.model;
        return m == PhysicsModel::SPH || m == PhysicsModel::LennardJones || (m == PhysicsModel::Rigid && ballSpray_);
    }
    void spawnBall(Vec2 pos, Vec2 vel);
    void sprayParticles(Vec2 pos);
    void eraseAt(Vec2 pos);
    void spawnBlob(Vec2 pos);
    void spawnRope(Vec2 a, Vec2 b);
    void spawnShape(Vec2 pos, bool polygon);

    // ---- drawing (render.cpp)
    void drawWorld();
    void drawStructureOverlay();
    void drawToolPreview();
    void drawHintBar();
    void drawToast();
    void drawCaption();
    void drawHelpOverlay();
    void drawCursor();
    Color particleColor(int i, float speedScale) const;
    void text(const std::string& s, float x, float y, float size, Color c, bool bold = false) const;
    float textWidth(const std::string& s, float size, bool bold = false) const;

    // ---- panel & toolbar (ui.cpp)
    void drawToolbar();
    void drawPanel();
    float drawHeader(float y);
    float drawSceneSection(float y);
    float drawToolSection(float y);
    float drawPhysicsTab(float y);
    float drawPerformanceTab(float y);
    float drawViewTab(float y);
    float drawEnergyGraph(float y, float height);
    float drawSpeedHistogram(float y, float height);
    void compareBroadphases();
    void drawComparisonCard();
    Rectangle comparisonRect_{};
    void applyStyle();
    // panel row helpers; each returns the y below the row
    float sliderRow(float y, const char* label, float* value, float lo, float hi, const char* fmt, bool enabled = true,
                    bool logScale = false);
    float intSliderRow(float y, const char* label, int* value, int lo, int hi, bool enabled = true);
    float checkRow(float y, const char* label, bool* value, bool enabled = true);
    float noteRow(float y, const std::string& note, Color c);
    float panelX_ = 0, panelW_ = 0;
    std::string hoverHint_;  // overrides the hint bar (e.g. why a tool is disabled)
    Rectangle sceneDropdownRect_{}, broadphaseDropdownRect_{};
    bool broadphaseDropdownVisible_ = false;

    // ---- recording
    void beginRecording();
    void captureFrame();
    void endRecording();

    AppOptions options_;
    World world_;
    GpuSim gpu_;
    bool gpuActive_ = false;
    bool gpuDirty_ = true;

    RunState state_ = RunState::Running;
    Tool tool_ = Tool::Grab;
    Tab tab_ = Tab::Physics;
    ColorMode colorMode_ = ColorMode::Base;
    bool showStructure_ = false;
    bool showHelp_ = false;
    bool showLinks_ = true;
    bool overlayStale_ = true;  // broad phase needs a rebuild before drawing the overlay

    int scene_ = 0;
    int sceneCount_ = 0;        // particle count slider value
    bool sceneCountDirty_ = false;
    unsigned seed_ = 1;

    // tool options
    float ballRadius_ = 8;
    bool ballRandomSize_ = true;
    bool ballSpray_ = false;
    float forceStrength_ = 6000;
    float forceRadius_ = 180;
    float shapeSize_ = 40;
    int polygonSides_ = 5;
    bool shapeStatic_ = false;
    float blobRadius_ = 50;
    float eraseRadius_ = 30;
    bool ropePinned_ = true;
    float starMass_ = 200;

    // mouse interaction state
    bool dragging_ = false;
    Vec2 dragStart_;
    int movingParticle_ = -1;   // paused/edit-mode "move" of a particle
    int movingBody_ = -1;
    Vec2 moveOffset_;
    float sprayAccumulator_ = 0;

    // layout (pixels)
    static constexpr float kPanelWidth = 380;
    static constexpr float kToolbarHeight = 46;
    static constexpr float kHintHeight = 28;
    Rectangle worldRect_{};
    Rectangle panelRect_{};
    bool mouseOverUi_ = false;
    bool dropdownOpen_ = false;
    // Who owns the current left-button press. A press that starts in the world never
    // drives panel controls (and vice versa); a press that closes the help overlay
    // or an open dropdown is swallowed until the button is released.
    enum class Press { None, World, Ui, Swallow };
    Press press_ = Press::None;
    bool uiLocked() const { return press_ == Press::World || press_ == Press::Swallow || showHelp_; }
    void applyGuiLock() const;
    bool sceneDropdownOpen_ = false;
    bool broadphaseDropdownOpen_ = false;

    // timing
    double accumulator_ = 0;
    bool lagging_ = false;
    std::deque<float> stepMsHistory_;
    struct EnergySample { float kinetic, potential; };
    std::deque<EnergySample> energyHistory_;
    std::vector<BroadphaseComparison> comparison_;
    std::string comparisonNote_;

    // fonts
    Font font_{};
    Font fontBold_{};
    Texture2D circleTex_{};

    std::string toastText_;
    double toastUntil_ = 0;
    std::string caption_;

    // recording
    FILE* recordPipe_ = nullptr;
    long long frameIndex_ = 0;
    bool recording() const { return recordPipe_ != nullptr; }
    bool quit_ = false;
};
