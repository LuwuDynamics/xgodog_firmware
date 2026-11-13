#pragma once
#include <array>
#include <cstdint>

namespace xgo {
struct WalkFoot { // mm, negative Z lifts
    float x = 0, y = 0, z = 0;
    WalkFoot() = default;
    WalkFoot(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}
};
using WalkFeet = std::array<WalkFoot, 4>;

// Tuning lives here; planner has no robot globals, IK, protocol or allocation.
struct WalkConfig {
    std::array<uint8_t, 4> order{{0, 2, 1, 3}};
    float period_scale = 6.0f; // requested trot period -> complete four-leg walk cycle
    float min_cycle_s = 2.0f;
    float transfer_fraction = 0.55f; // fraction of each quarter-cycle, all feet down
    float max_stride_x_mm = 16.0f; // half-stride; scale all legs together
    float max_stride_y_mm = 10.0f;
    float shift_scale = 0.5f; // 0 = no lean, 1 = full support-triangle centroid
    float max_lift_mm = 25.0f;
    float landing_s = 0.5f;
    float yaw_scale = 1.0f;
};

// Higher step and a longer cycle than WalkConfig's defaults.
// Firmware currently builds only this config, for GAIT_HIGH_WALK.
inline WalkConfig HighWalkConfig() {
    WalkConfig config;
    config.period_scale = 12.0f;
    config.min_cycle_s = 4.5f;
    // Keep the full cycle, but allow the folded swing enough time without
    // increasing peak joint speed. All four feet stay down during transfer.
    config.transfer_fraction = 0.55f;
    config.max_stride_x_mm = 18.0f;
    config.max_stride_y_mm = 10.0f;
    config.shift_scale = 0.8f;
    config.max_lift_mm = 50.0f;
    config.landing_s = 0.5f;
    return config;
}

struct WalkInput {
    bool moving = false;
    WalkFeet stride{};
    WalkFeet nominal{}; // nominal foot positions relative to body/COM projection
    float cycle_s = 2.0f;
    float lift_mm = 22.0f;
};
enum class WalkStage : uint8_t { Idle, Landing, Transfer, Swing, Center };

class WalkPlanner {
public:
    explicit WalkPlanner(const WalkConfig &config = {});
    void reset();
    void begin(const WalkFeet &current);
    void setLeadPair(bool lead02);
    const WalkFeet &update(const WalkInput &input, float dt_s);
    bool active() const { return stage_ != WalkStage::Idle; }
    const WalkConfig &config() const { return config_; }
private:
    void transfer(const WalkInput &input);
    void beginStop();
    void render();
    WalkConfig config_;
    WalkStage stage_ = WalkStage::Idle;
    WalkFeet feet_{}, from_{}, target_{}, output_{};
    WalkFoot shift_{}, shift_from_{}, shift_target_{};
    WalkFoot landing_target_{};
    // Integer time prevents gait phase drift from repeated float additions.
    uint32_t time_us_ = 0;
    uint32_t duration_us_ = 1000000, swing_us_ = 200000;
    float lift_ = 0;
    uint8_t slot_ = 0;
};
}
