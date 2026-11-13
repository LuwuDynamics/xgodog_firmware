#pragma once
#include <stdint.h>
#include "motion/config.h"

namespace motion_time {
constexpr uint32_t kNominalUs = 1000000u / SYSTEM_TICK_HZ;
constexpr uint32_t kMaximumStepUs = 20000u;
static_assert(1000000u % SYSTEM_TICK_HZ == 0, "Motion Tick must have integral microseconds");

// Unsigned subtraction handles micros() wrap; a long stall is deliberately
// capped at 20 ms rather than extrapolating across the entire gap.
class SampleClock {
public:
    uint32_t sample(uint32_t now) {
        if (!started_) { previous_ = now; started_ = true; return kNominalUs; }
        const uint32_t elapsed = now - previous_;
        previous_ = now;
        return elapsed > kMaximumStepUs ? kMaximumStepUs : elapsed;
    }
private:
    uint32_t previous_ = 0;
    bool started_ = false;
};

// Only the control task owns this context. Legacy direct calls see one normal
// 5 ms step. Actions, gait, oscillators and heading consume the same interval.
void beginStep(uint32_t elapsedUs);
void endStep();
float stepTicks();
float stepSeconds();
uint32_t stepMicroseconds();
uint32_t wholeTicks();
} // namespace motion_time
