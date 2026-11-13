#pragma once
#include "robot/types.h"

// Sensor drivers own register setup, byte order, axes and physical scales.
// The estimator owns one-shot calibration, sample timing and attitude.
class ImuEstimator {
public:
    enum class Model { Icm42670, Icm20948, Qmi8658c };
    explicit ImuEstimator(Model model) : model_(model) {}
    // Returns true only after a new, finite attitude observation is accepted;
    // duplicate timestamps and calibration-only samples are not publishable.
    bool update(imu_t &state, volatile uint8_t &ready,
                uint32_t nowUs, uint32_t uptimeMs);
    void calibrate(imu_t &state, volatile uint8_t &ready, uint32_t nowUs, uint32_t uptimeMs);
    void reset();
private:
    bool gravityAttitude(const float acc[3], float &roll, float &pitch) const;
    Model model_;
    uint32_t previousUs_ = 0;
    bool clockStarted_ = false;
    bool previouslyReady_ = false;
    bool calibrationPublished_ = false;
    bool initialTiltPending_ = false;
    bool calibrationClockStarted_ = false;
    uint32_t calibrationUs_ = 0;
    uint16_t samples_ = 0;
    float accSum_[3] = {};
    float gyroMean_[3] = {};
};

#include <stdint.h>

namespace imu_health {
// Approximately 33 nominal samples may be missed before feedback is stale.
// This is an availability timeout, independent of the 20 ms integration gap.
constexpr uint32_t kMaximumAgeMs = 100;

// One producer (IMU task). Publish only after a valid attitude update/frame.
void reset();
void publish(uint32_t nowMs);
bool observed();
bool fresh(uint32_t nowMs);
} // namespace imu_health
