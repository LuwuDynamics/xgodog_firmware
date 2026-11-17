#include "motion/walk_planner.h"
#include <algorithm>
#include <cmath>
#ifdef min
#undef min
#endif
#ifdef max
#undef max
#endif

namespace xgo {
namespace {
float smooth(float u) { return u*u*u*(10 + u*(-15 + 6*u)); }
// Only finite, bounded durations pass here. Round to the Motion Tick clock;
// float conversion plus rounding differs by less than 1 us per stage.
uint32_t microseconds(float seconds) { return (uint32_t)(seconds * 1000000.f + 0.5f); }
float bounded(float x, float low, float high) {
    return std::isfinite(x) ? std::max(low, std::min(high, x)) : low;
}
WalkFoot mix(const WalkFoot &a, const WalkFoot &b, float s) {
    return {a.x+(b.x-a.x)*s, a.y+(b.y-a.y)*s, a.z+(b.z-a.z)*s};
}
}
WalkPlanner::WalkPlanner(const WalkConfig &config) : config_(config) {
    unsigned mask = 0;
    for (auto leg : config_.order) if (leg < 4) mask |= 1u << leg;
    if (mask != 15) config_.order = {{0, 2, 1, 3}};
    config_.period_scale = bounded(config_.period_scale, 1, 16);
    config_.min_cycle_s = bounded(config_.min_cycle_s, 1.2f, 10);
    config_.transfer_fraction = bounded(config_.transfer_fraction, 0.3f, 0.75f);
    config_.max_stride_x_mm = bounded(config_.max_stride_x_mm, 1, 20);
    config_.max_stride_y_mm = bounded(config_.max_stride_y_mm, 1, 12);
    config_.shift_scale = bounded(config_.shift_scale, 0, 1);
    config_.max_lift_mm = bounded(config_.max_lift_mm, 1, 60);
    config_.landing_s = bounded(config_.landing_s, 0.1f, 1);
    config_.yaw_scale = bounded(config_.yaw_scale, 0, 1);
}
void WalkPlanner::reset() {
    stage_ = WalkStage::Idle; feet_ = from_ = target_ = output_ = {};
    shift_ = shift_from_ = shift_target_ = {};
    time_us_ = 0; slot_ = 0;
}
void WalkPlanner::setLeadPair(bool lead02) {
    config_.order = lead02 ? std::array<uint8_t, 4>{{0, 2, 1, 3}}
                           : std::array<uint8_t, 4>{{1, 3, 0, 2}};
}
void WalkPlanner::begin(const WalkFeet &current) {
    reset(); feet_ = from_ = target_ = current;
    for (auto &foot : target_) foot.z = 0;
    stage_ = WalkStage::Landing; duration_us_ = microseconds(config_.landing_s);
    render();
}
void WalkPlanner::transfer(const WalkInput &input) {
    const bool stopping = !input.moving;
    from_ = feet_; target_ = feet_; shift_from_ = shift_;
    float scale = 1;
    for (const auto &stride : input.stride) {
        if (!std::isfinite(stride.x) || !std::isfinite(stride.y)) { scale = 0; break; }
        scale = std::min(scale, config_.max_stride_x_mm / std::max(0.001f, std::fabs(stride.x)));
        scale = std::min(scale, config_.max_stride_y_mm / std::max(0.001f, std::fabs(stride.y)));
    }
    if (!stopping && scale > 0) {
        // Four transfers advance the body by one full stride. All support feet
        // move together in body coordinates; the world support positions stay fixed.
        for (int i=0; i<4; ++i) {
            target_[i].x -= 0.5f*input.stride[i].x*scale;
            target_[i].y -= 0.5f*input.stride[i].y*scale;
        }
    }
    shift_target_ = {};
    const auto swing = config_.order[slot_];
    for (int i=0; i<4; ++i) if (i != swing) {
        shift_target_.x += (input.nominal[i].x + target_[i].x)/3;
        shift_target_.y += (input.nominal[i].y + target_[i].y)/3;
    }
    shift_target_.x *= config_.shift_scale;
    shift_target_.y *= config_.shift_scale;
    // Latch the landing target and timing once per step: changes mid-swing
    // cannot move its endpoint or restart its trajectory.
    landing_target_ = {};
    if (!stopping && scale > 0) {
        landing_target_.x = input.stride[swing].x*scale;
        landing_target_.y = input.stride[swing].y*scale;
    }
    const float quarter = bounded(input.cycle_s, config_.min_cycle_s, 20)/4;
    duration_us_ = microseconds(quarter*config_.transfer_fraction);
    swing_us_ = microseconds(quarter*(1-config_.transfer_fraction));
    lift_ = bounded(input.lift_mm, 0, config_.max_lift_mm);
    time_us_ = 0; stage_ = WalkStage::Transfer;
}
void WalkPlanner::beginStop() {
    constexpr float kStopCenterSeconds = 0.18f;
    from_ = feet_; target_ = {};
    shift_from_ = shift_; shift_target_ = {};
    time_us_ = 0; duration_us_ = microseconds(std::min(config_.landing_s, kStopCenterSeconds));
    stage_ = WalkStage::Center;
}
void WalkPlanner::render() {
    for (int i=0; i<4; ++i) output_[i] = {feet_[i].x-shift_.x, feet_[i].y-shift_.y, feet_[i].z};
}
const WalkFeet &WalkPlanner::update(const WalkInput &input, float dt_s) {
    if (!active() || !std::isfinite(dt_s) || dt_s <= 0) return output_;
    if (!input.moving && stage_ == WalkStage::Landing) {
        bool neutral = std::fabs(shift_.x)<0.001f && std::fabs(shift_.y)<0.001f;
        for (const auto &foot : feet_)
            neutral &= std::fabs(foot.x)<0.001f && std::fabs(foot.y)<0.001f && std::fabs(foot.z)<0.001f;
        if (neutral) { reset(); return output_; }
    }
    // Keep the scheduling-stall cap, but carry a tick's remainder through a
    // stage boundary instead of slowing the cycle by up to one tick per stage.
    // Validated stage durations are >=75 ms, so a <=20 ms call crosses at most
    // one boundary (two evaluations); this loop cannot create catch-up bursts.
    // Stage durations are latched in this same clock; trajectory arithmetic
    // stays single precision without repeated float time accumulation.
    uint32_t remaining = microseconds(std::min(dt_s, 0.02f));
    do {
        const uint32_t advance = std::min(remaining, duration_us_ - time_us_);
        time_us_ += advance;
        remaining -= advance;
        const float u = (float)time_us_ / duration_us_, s = smooth(u);
        for (int i=0; i<4; ++i) feet_[i] = mix(from_[i], target_[i], s);
        shift_ = mix(shift_from_, shift_target_, s);
        if (stage_ == WalkStage::Swing) {
            // Two quintics: zero velocity/acceleration at lift/apex/touchdown.
            const float lift_shape = u < 0.5f ? smooth(2*u) : smooth(2*(1-u));
            const auto swing = config_.order[slot_];
            // Retract toward the nominal fore/aft neutral position at maximum
            // lift, then extend to the unchanged landing point. Keeping X far
            // behind the hip at full lift over-folds Lite's hip joint.
            // Reuse the lift shape: X and Z have zero V/A at apex and contacts.
            const float end = u < 0.5f ? from_[swing].x : target_[swing].x;
            feet_[swing].x = end + (shift_.x-end)*lift_shape;
            feet_[swing].z = -lift_*lift_shape;
        }
        render();
        if (time_us_ < duration_us_) break;
        // Stop at an existing zero-velocity endpoint. Never redirect an airborne
        // foot or start another swing after a stop request. Centering remains
        // bounded by the remaining current stage plus the stop centering time.
        if (stage_ == WalkStage::Landing) {
            bool neutral = true;
            for (const auto &foot : feet_)
                neutral &= std::fabs(foot.x)<0.001f && std::fabs(foot.y)<0.001f;
            if (!input.moving && neutral) { reset(); break; }
            if (input.moving) transfer(input);
            else beginStop();
        } else if (stage_ == WalkStage::Transfer) {
            if (!input.moving) {
                beginStop();
            } else {
                from_ = target_ = feet_;
                target_[config_.order[slot_]] = landing_target_;
                shift_from_ = shift_target_ = shift_;
                time_us_ = 0; duration_us_ = swing_us_; stage_ = WalkStage::Swing;
            }
        } else if (stage_ == WalkStage::Swing) {
            slot_ = (slot_+1)%4;
            if (!input.moving) beginStop();
            else transfer(input);
        } else if (stage_ == WalkStage::Center) {
            if (input.moving) transfer(input);
            else reset();
        }
    } while (remaining > 0 && active());
    return output_;
}
}
