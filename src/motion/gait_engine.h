#pragma once
#include "motion/walk_planner.h"
#include <array>
#include <cstdint>

namespace xgo {
struct GaitSample { WalkFoot position{}, velocity{}, acceleration{}; };

// A normalized power-basis quintic. Keep float operation order shared by the
// executor and candidate planner: a separately reimplemented curve can certify
// one path while the controller emits another.
struct GaitPolynomial {
    float c[6]{};
    void set(float p, float v, float a, float end, float end_v, float seconds) {
        c[0] = p; c[1] = v*seconds; c[2] = a*seconds*seconds*0.5f;
        const float d = end-c[0]-c[1]-c[2];
        const float dv = end_v*seconds-c[1]-2*c[2];
        const float da = -2*c[2];
        c[3] = 10*d-4*dv+0.5f*da;
        c[4] = -15*d+7*dv-da;
        c[5] = 6*d-3*dv+0.5f*da;
    }
    void sample(float u, float seconds, float &p, float &v, float &a) const {
        p = c[0]+u*(c[1]+u*(c[2]+u*(c[3]+u*(c[4]+u*c[5]))));
        v = (c[1]+u*(2*c[2]+u*(3*c[3]+u*(4*c[4]+u*5*c[5]))))/seconds;
        a = (2*c[2]+u*(6*c[3]+u*(12*c[4]+u*20*c[5])))/(seconds*seconds);
    }
};

// One immutable-by-convention segment value. The engine owns elapsed time;
// constraint evaluation can inspect this exact trajectory without advancing it.
// Position units are mm; derivatives use seconds. Swing replaces Z with a
// two-half quintic and is constructed only at grounded exchanges.
struct GaitTrajectory {
    GaitPolynomial x, y, z;
    WalkFoot target{}, target_velocity{};
    uint32_t duration = 1;
    float lift = 0;
    bool swing = false;

    void set(const GaitSample &start, const WalkFoot &end, const WalkFoot &end_v,
             uint32_t microseconds) {
        duration = microseconds ? microseconds : 1;
        target = end; target_velocity = end_v;
        const float seconds = duration*0.000001f;
        x.set(start.position.x, start.velocity.x, start.acceleration.x, end.x, end_v.x, seconds);
        y.set(start.position.y, start.velocity.y, start.acceleration.y, end.y, end_v.y, seconds);
        z.set(start.position.z, start.velocity.z, start.acceleration.z, end.z, end_v.z, seconds);
        swing = false; lift = 0;
    }
    static void applyLift(float u, float seconds, float height, GaitSample &s) {
        const float q = u <= 0.5f ? u*2 : (1-u)*2;
        const float sign = u <= 0.5f ? 1.f : -1.f;
        const float q2 = q*q;
        // Short-side evaluation avoids cancellation near the swing apex.
        const float r = q <= .5f ? q : 1-q;
        const float h = r*r*r*(10+r*(-15+6*r));
        s.position.z = -height*(q <= .5f ? h : 1-h);
        s.velocity.z = -height*30*q2*(1-q)*(1-q)*sign*2/seconds;
        s.acceleration.z = -height*60*q*(1-q)*(1-2*q)*4/(seconds*seconds);
    }
    GaitSample sample(uint32_t elapsed) const {
        GaitSample value;
        if (elapsed >= duration) {
            // Exact endpoint is part of the executed trajectory contract.
            value.position = target;
            value.velocity = target_velocity;
            return value;
        }
        const float u = static_cast<float>(elapsed)/duration;
        const float seconds = duration*0.000001f;
        x.sample(u, seconds, value.position.x, value.velocity.x, value.acceleration.x);
        y.sample(u, seconds, value.position.y, value.velocity.y, value.acceleration.y);
        z.sample(u, seconds, value.position.z, value.velocity.z, value.acceleration.z);
        if (swing) applyLift(u, seconds, lift, value);
        return value;
    }
};
} // namespace xgo

namespace xgo {
// Millimetres and seconds. Negative Z lifts a foot. Timing changes at cycle
// boundaries; stride/lift changes at each grounded exchange before liftoff.
struct GaitInput {
    bool moving = false;
    bool walk = false;
    uint8_t kind = 0;
    float period_s = 0.37f;
    float swing_ratio = 0.48f;
    // Body-frame fore/aft support placement, mm. Latched at grounded exchanges;
    // this shifts the step centre without changing nominal stride or timing.
    float support_x_mm = 0;
    WalkFeet stride{};
    std::array<float, 4> lift{{22, 22, 22, 22}};
};

class GaitEngine {
public:
    static constexpr uint32_t BrakeUs = 100000;
    static constexpr uint32_t CenterUs = 250000;
    void reset();
    void begin(const WalkFeet &current, bool lead02);
    const WalkFeet &update(const GaitInput &input, uint32_t elapsed_us);
    bool active() const { return mode_ != Mode::Idle; }
private:
    enum class Mode : uint8_t { Idle, Starting, Running, Stopping, Center };
    struct Leg {
        GaitSample sample{};
        GaitTrajectory trajectory{};
        uint32_t elapsed = 0;
    };
    void latchTiming(const GaitInput &input);
    void startStep(const GaitInput &input);
    void segment(unsigned leg, const WalkFoot &target, const WalkFoot &velocity, uint32_t duration);
    void swing(unsigned leg);
    void stance(unsigned leg, uint32_t duration);
    void brake(unsigned leg);
    void requestStop();
    void center();
    void advance(uint32_t dt);
    void events(const GaitInput &input);
    Mode mode_ = Mode::Idle;
    std::array<Leg, 4> legs_{};
    WalkFeet output_{};
    GaitInput command_{};
    std::array<uint32_t, 4> lift_at_{};
    uint32_t cycle_us_ = 370000, swing_us_ = 177600, clock_us_ = 0;
    bool lead02_ = true;
};
}
