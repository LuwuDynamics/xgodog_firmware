#include "motion/gait_engine.h"
#include <cmath>

namespace xgo {
namespace {
float bound(float x, float low, float high) {
    return std::isfinite(x) ? (x < low ? low : x > high ? high : x) : low;
}
uint32_t us(float seconds) { return static_cast<uint32_t>(seconds * 1000000.f + 0.5f); }
uint32_t smaller(uint32_t a, uint32_t b) { return a < b ? a : b; }
bool finite(const WalkFoot &p) { return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z); }
}

GaitSample GaitTrajectory::sampleLiteTrot(float u, float seconds) const {
    GaitSample value;
    const float quarterRate = 4.0f/seconds;
    if (u < 0.25f) {
        const float q = 4.0f*u;
        value.position = {x.c[0] + swing_x_offset*q, y.c[0], -lift*q};
        value.velocity = {swing_x_offset*quarterRate, 0, -lift*quarterRate};
    } else if (u < 0.75f) {
        const float q = 2.0f*u - 0.5f;
        const float dx = target.x - x.c[0];
        const float dy = target.y - y.c[0];
        value.position = {x.c[0] + swing_x_offset + dx*q, y.c[0] + dy*q, -lift};
        value.velocity = {dx*quarterRate/2.0f, dy*quarterRate/2.0f, 0};
    } else {
        const float q = 4.0f*(1.0f-u);
        value.position = {target.x + swing_x_offset*q, target.y, -lift*q};
        value.velocity = {-swing_x_offset*quarterRate, 0, lift*quarterRate};
    }
    return value;
}

void GaitEngine::reset() { *this = GaitEngine{}; }
void GaitEngine::begin(const WalkFeet &current, bool lead02) {
    reset(); lead02_ = lead02; mode_ = Mode::Starting;
    for (unsigned i=0; i<4; ++i) {
        legs_[i].sample.position = finite(current[i]) ? current[i] : WalkFoot{};
        output_[i] = legs_[i].sample.position;
    }
}
void GaitEngine::latchTiming(const GaitInput &input) {
    gait_kind_ = input.kind;
    const float period = bound(input.period_s, 0.2f, 4.f);
    const float swingRatio = bound(input.swing_ratio, 0.05f, input.walk ? 0.25f : 0.5f);
    cycle_us_ = us(period);
    // The shortest integer event spacing bounds swing duration. Independently
    // rounding T*ratio could overlap adjacent swings by 1 us for odd periods.
    swing_us_ = smaller(us(period * swingRatio),
                        cycle_us_ / (input.walk ? 4u : 2u));
    const unsigned order[4] = {lead02_ ? 0u : 1u, lead02_ ? 2u : 3u,
                               lead02_ ? 1u : 0u, lead02_ ? 3u : 2u};
    for (unsigned slot=0; slot<4; ++slot)
        lift_at_[order[slot]] = input.walk ? cycle_us_*slot/4 : (slot/2)*(cycle_us_/2);
}
void GaitEngine::startRunning(const GaitInput &input) {
    latchTiming(input);
    mode_ = Mode::Running;
    clock_us_ = 0;
    lift_cycle_ = input.full_lift_from_start ? kStartupLiftScales.size()-1 : 0;
    startStep(input);
}
void GaitEngine::startStep(const GaitInput &input) {
    // Every preceding swing has landed at this scheduled exchange. Adopt the
    // latest stride for all feet together; never redirect an airborne target.
    // Timing is independent, so a continuous stream cannot postpone adoption.
    command_.stride_center_y = input.stride_center_y;
    command_.swing_x_offset = input.swing_x_offset;
    command_.lite_swing = input.lite_swing;
    for (unsigned i=0; i<4; ++i) {
        // Bounds exceed every shipped robot profile but prevent malformed
        // internal values from producing non-finite polynomial coefficients.
        command_.stride[i].x = std::isfinite(input.stride[i].x) ? bound(input.stride[i].x, -100, 100) : 0;
        command_.stride[i].y = std::isfinite(input.stride[i].y) ? bound(input.stride[i].y, -100, 100) : 0;
        command_.lift[i] = bound(input.lift[i], 0, 60);
        command_.stride_center_x[i] = input.stride_center_x[i];
    }
    for (unsigned i=0; i<4; ++i) {
        const uint32_t liftAt = lift_at_[i];
        if (liftAt == clock_us_) swing(i);
        else stance(i, liftAt + (liftAt < clock_us_ ? cycle_us_ : 0u) - clock_us_);
    }
}
void GaitEngine::segment(unsigned i, const WalkFoot &target, const WalkFoot &velocity, uint32_t duration) {
    auto &leg = legs_[i];
    leg.elapsed = 0;
    leg.trajectory.set(leg.sample, target, velocity, duration);
}
void GaitEngine::swing(unsigned i) {
    const auto &stride = command_.stride[i];
    const WalkFoot target{stride.x + command_.stride_center_x[i],
                          stride.y + command_.stride_center_y, 0};
    // The quintic starts at rest. Lite Trot supplies its own linear velocity.
    auto &sample = legs_[i].sample;
    sample.velocity = {};
    sample.acceleration = {};
    segment(i, target, {}, swing_us_);
    auto &trajectory = legs_[i].trajectory;
    trajectory.swing = true;
    trajectory.lite_swing = command_.lite_swing;
    trajectory.swing_x_offset = command_.swing_x_offset;
    trajectory.lift = command_.lift[i]*kStartupLiftScales[lift_cycle_];
}
void GaitEngine::stance(unsigned i, uint32_t duration) {
    const float stance_s = (cycle_us_-swing_us_)*0.000001f;
    const auto &stride = command_.stride[i];
    const WalkFoot target{-stride.x + command_.stride_center_x[i],
                          -stride.y + command_.stride_center_y, 0};
    const WalkFoot velocity{-2*stride.x/stance_s, -2*stride.y/stance_s, 0};
    // Match ground travel at touchdown. Grounded replans keep their sample.
    if (legs_[i].trajectory.swing) legs_[i].sample.velocity = velocity;
    segment(i, target, velocity, duration);
}
void GaitEngine::brake(unsigned i) {
    const auto &s = legs_[i].sample;
    const float t = BrakeUs*0.000001f;
    const WalkFoot target{s.position.x+s.velocity.x*t/2+s.acceleration.x*t*t/12,
                          s.position.y+s.velocity.y*t/2+s.acceleration.y*t*t/12, 0};
    segment(i, target, {}, BrakeUs);
}
void GaitEngine::requestStop() {
    mode_ = Mode::Stopping;
    for (unsigned i=0; i<4; ++i) if (!legs_[i].trajectory.swing) brake(i);
}
void GaitEngine::center() {
    mode_ = Mode::Center;
    for (unsigned i=0; i<4; ++i) segment(i, {}, {}, CenterUs);
}
void GaitEngine::advance(uint32_t dt) {
    for (unsigned i=0; i<4; ++i) {
        auto &leg = legs_[i];
        leg.elapsed += smaller(dt, leg.trajectory.duration-leg.elapsed);
        leg.sample = leg.trajectory.sample(leg.elapsed);
        output_[i] = leg.sample.position;
    }
    if (mode_ == Mode::Running) clock_us_ += dt;
}
void GaitEngine::events(const GaitInput &input) {
    if (mode_ == Mode::Running) {
        // Process touchdown before cycle/liftoff events, including low trot's
        // exact diagonal exchange. No sampled all-feet-down window is needed.
        for (unsigned i=0; i<4; ++i) if (legs_[i].trajectory.swing && legs_[i].elapsed == legs_[i].trajectory.duration)
            stance(i, cycle_us_-swing_us_);
        if (clock_us_ == cycle_us_) {
            if (lift_cycle_ < kStartupLiftScales.size()-1) ++lift_cycle_;
            clock_us_ = 0; latchTiming(input);
        }
        bool exchange = false;
        for (unsigned i=0; i<4; ++i)
            exchange |= clock_us_ == lift_at_[i];
        if (exchange) startStep(input);
    } else if (mode_ == Mode::Stopping) {
        bool done = true;
        for (unsigned i=0; i<4; ++i) {
            if (legs_[i].elapsed == legs_[i].trajectory.duration && legs_[i].trajectory.swing) brake(i);
            done &= legs_[i].elapsed == legs_[i].trajectory.duration;
        }
        if (done) {
            // A fresh command can start from the grounded, stationary feet.
            if (input.moving && input.kind == gait_kind_) startRunning(input);
            else center();
        }
    } else if (mode_ == Mode::Center) {
        bool done = true;
        for (const auto &leg : legs_) done &= leg.elapsed == leg.trajectory.duration;
        if (done) reset();
    }
}
const WalkFeet &GaitEngine::update(const GaitInput &input, uint32_t elapsed_us) {
    if (!elapsed_us || !active()) return output_;
    if (mode_ == Mode::Starting) {
        bool displaced = false;
        for (const auto &leg : legs_) displaced |= std::fabs(leg.sample.position.z)>0.001f;
        if (displaced || !input.moving) center();
        else startRunning(input);
    }
    if (mode_ == Mode::Running && (!input.moving || input.kind != gait_kind_)) requestStop();
    uint32_t remaining = smaller(elapsed_us, 20000);
    while (remaining && active()) {
        uint32_t dt = remaining;
        if (mode_ == Mode::Running) {
            dt = smaller(dt, cycle_us_-clock_us_);
            for (unsigned i=0; i<4; ++i) if (lift_at_[i]>clock_us_)
                dt = smaller(dt, lift_at_[i]-clock_us_);
        }
        for (const auto &leg : legs_) if (leg.elapsed < leg.trajectory.duration)
            dt = smaller(dt, leg.trajectory.duration-leg.elapsed);
        advance(dt); remaining -= dt; events(input);
    }
    return output_;
}
}
