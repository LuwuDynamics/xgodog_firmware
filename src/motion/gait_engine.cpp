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

void GaitEngine::reset() { *this = GaitEngine{}; }
void GaitEngine::begin(const WalkFeet &current, bool lead02) {
    reset(); lead02_ = lead02; mode_ = Mode::Starting;
    for (unsigned i=0; i<4; ++i) {
        legs_[i].sample.position = finite(current[i]) ? current[i] : WalkFoot{};
        output_[i] = legs_[i].sample.position;
    }
}
void GaitEngine::latchTiming(const GaitInput &input) {
    command_.kind = input.kind;
    command_.walk = input.walk;
    command_.period_s = bound(input.period_s, 0.2f, 4.f);
    command_.swing_ratio = bound(input.swing_ratio, 0.05f, input.walk ? 0.25f : 0.5f);
    cycle_us_ = us(command_.period_s);
    // The shortest integer event spacing bounds swing duration. Independently
    // rounding T*ratio could overlap adjacent swings by 1 us for odd periods.
    swing_us_ = smaller(us(command_.period_s * command_.swing_ratio),
                        cycle_us_ / (input.walk ? 4u : 2u));
    const unsigned order[4] = {lead02_ ? 0u : 1u, lead02_ ? 2u : 3u,
                               lead02_ ? 1u : 0u, lead02_ ? 3u : 2u};
    for (unsigned slot=0; slot<4; ++slot)
        lift_at_[order[slot]] = input.walk ? cycle_us_*slot/4 : (slot/2)*(cycle_us_/2);
}
void GaitEngine::startStep(const GaitInput &input) {
    // Every preceding swing has landed at this scheduled exchange. Adopt the
    // latest stride for all feet together; never redirect an airborne target.
    // Timing is independent, so a continuous stream cannot postpone adoption.
    command_.support_x_mm = std::isfinite(input.support_x_mm)
        ? bound(input.support_x_mm, -6, 6) : 0;
    for (unsigned i=0; i<4; ++i) {
        // Bounds exceed every shipped robot profile but prevent malformed
        // internal values from producing non-finite polynomial coefficients.
        command_.stride[i].x = std::isfinite(input.stride[i].x) ? bound(input.stride[i].x, -100, 100) : 0;
        command_.stride[i].y = std::isfinite(input.stride[i].y) ? bound(input.stride[i].y, -100, 100) : 0;
        command_.lift[i] = bound(input.lift[i], 0, 60);
    }
    for (unsigned i=0; i<4; ++i) {
        const uint32_t until_lift = (lift_at_[i] + cycle_us_ - clock_us_) % cycle_us_;
        if (until_lift == 0) swing(i);
        else stance(i, until_lift);
    }
}
void GaitEngine::segment(unsigned i, const WalkFoot &target, const WalkFoot &velocity, uint32_t duration) {
    auto &leg = legs_[i];
    leg.elapsed = 0;
    leg.trajectory.set(leg.sample, target, velocity, duration);
}
void GaitEngine::swing(unsigned i) {
    const float stance_s = (cycle_us_-swing_us_)*0.000001f;
    const auto &stride = command_.stride[i];
    WalkFoot target{stride.x, stride.y, 0};
    if (command_.support_x_mm != 0) target.x += command_.support_x_mm;
    segment(i, target, {-2*stride.x/stance_s, -2*stride.y/stance_s, 0}, swing_us_);
    legs_[i].trajectory.swing = true; legs_[i].trajectory.lift = command_.lift[i];
}
void GaitEngine::stance(unsigned i, uint32_t duration) {
    const float stance_s = (cycle_us_-swing_us_)*0.000001f;
    const auto &stride = command_.stride[i];
    WalkFoot target{-stride.x, -stride.y, 0};
    if (command_.support_x_mm != 0) target.x += command_.support_x_mm;
    segment(i, target, {-2*stride.x/stance_s, -2*stride.y/stance_s, 0}, duration);
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
        if (done) center();
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
        else {
            latchTiming(input); mode_ = Mode::Running; clock_us_ = 0;
            startStep(input);
        }
    }
    if (mode_ == Mode::Running && (!input.moving || input.kind != command_.kind)) requestStop();
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
