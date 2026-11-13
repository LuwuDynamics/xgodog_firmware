#include "sensor/attitude.h"
#include <cmath>
#include <atomic>

namespace {
constexpr uint16_t kCalibrationSamples = 250;
constexpr uint32_t kCalibrationHoldoffMs = 4000;
constexpr uint32_t kMaximumSampleGapUs = 20000;
constexpr float kGravity = 9.8f;
// Reject an excessive averaged bias without holding startup in calibration.
constexpr float kMaximumGyroBias = 5.0f; // degree/s, applied per axis
constexpr float kRadToDeg = 57.2957795f;
constexpr float kDegToRad = 0.0174532925f;

// Backward-Euler low-pass weights retain deployed gains at the nominal 3 ms
// sample interval and retain comparable response when the scheduler jitters.
float correctionWeight(float dt, bool complementary) {
    const float nominal = complementary ? (1.0f - 0.98f) : 0.12f;
    if (dt == 0.003f) return nominal;
    const float timeConstant = 0.003f * (1.0f - nominal) / nominal;
    return dt / (timeConstant + dt);
}

bool finiteSample(const imu_t &state) {
    for (uint8_t i = 0; i < 3; ++i)
        if (!std::isfinite(state.acc[i]) || !std::isfinite(state.gyro[i])) return false;
    return true;
}

bool hasExpectedGravity(const imu_t &state) {
    float gravitySquared = 0;
    for (uint8_t i = 0; i < 3; ++i) {
        gravitySquared += state.acc[i] * state.acc[i];
    }
    return gravitySquared >= (0.8f * kGravity) * (0.8f * kGravity) &&
           gravitySquared <= (1.2f * kGravity) * (1.2f * kGravity);
}

// Retain the deployed +/-90 degree tilt branch of atan(numerator/denominator).
// atan2 avoids zero division; folding preserves the negative-Z board mounting.
float ratioAngle(float numerator, float denominator) {
    float angle = atan2f(numerator, denominator);
    constexpr float pi = 3.14159265358979323846f;
    if (angle > pi / 2) angle -= pi;
    if (angle < -pi / 2) angle += pi;
    return angle * 57.2958f;
}

bool usableGravity(const float acc[3]) {
    const float norm = acc[0] * acc[0] + acc[1] * acc[1] + acc[2] * acc[2];
    return std::isfinite(norm) && norm > 1.0e-6f;
}
}

void ImuEstimator::reset() {
    samples_ = 0;
    for (uint8_t i = 0; i < 3; ++i) accSum_[i] = gyroMean_[i] = 0;
    clockStarted_ = previouslyReady_ = calibrationPublished_ = false;
    previousUs_ = 0;
    calibrationClockStarted_ = initialTiltPending_ = false;
    calibrationUs_ = 0;
}

void ImuEstimator::calibrate(imu_t &state, volatile uint8_t &ready,
                             uint32_t nowUs, uint32_t uptimeMs) {
    if (calibrationPublished_) {
        if (!ready) ready = 1;
        return;
    }
    if (calibrationClockStarted_ && nowUs == calibrationUs_) return;
    calibrationClockStarted_ = true;
    calibrationUs_ = nowUs;
    if (ready != 0) {
        calibrationPublished_ = true;
        return;
    }
    if (uptimeMs <= kCalibrationHoldoffMs) return;
    // Skip invalid observations without discarding samples already collected.
    if (!finiteSample(state) || !hasExpectedGravity(state)) return;
    ++samples_;
    for (uint8_t i = 0; i < 3; ++i) {
        accSum_[i] += state.acc[i];
        gyroMean_[i] += (state.gyro[i] - gyroMean_[i]) / samples_;
    }
    if (samples_ < kCalibrationSamples) return;
    // Publish only complete averages. Control observes readiness after all bias
    // fields are initialized; samples never temporarily replace the final bias.
    for (uint8_t i = 0; i < 3; ++i) {
        state.acc_bias[i] = accSum_[i] / kCalibrationSamples;
        state.gyro_bias[i] = fabsf(gyroMean_[i]) <= kMaximumGyroBias ? gyroMean_[i] : 0.0f;
    }
    // Complete this window even when a bias was rejected; do not retry it.
    calibrationPublished_ = true;
    initialTiltPending_ = true;
    std::atomic_thread_fence(std::memory_order_release);
    ready = 1;
}

bool ImuEstimator::gravityAttitude(const float acc[3], float &roll, float &pitch) const {
    if (!usableGravity(acc)) return false;
    // QMI keeps chip axes in telemetry; rotate only the estimator's local
    // sample into the same roll/pitch frame as the two ICM adapters.
    const bool qmi = model_ == Model::Qmi8658c;
    const float ax = qmi ? -acc[1] : acc[0];
    const float ay = qmi ? -acc[0] : acc[1];
    const float az = qmi ? -acc[2] : acc[2];
    const float pitchDenominator = sqrtf(ay * ay + az * az);
    if (model_ == Model::Icm42670) {
        roll = atan2f(-ay, -az) * kRadToDeg;
        pitch = atan2f(ax, pitchDenominator) * kRadToDeg;
    } else {
        // Retain the deployed folded tilt branch for these two models.
        // The full transverse magnitude removes roll from the pitch estimate.
        roll = ratioAngle(-ay, -az);
        pitch = ratioAngle(ax, copysignf(pitchDenominator, -az));
    }
    return std::isfinite(roll) && std::isfinite(pitch);
}

bool ImuEstimator::update(imu_t &state, volatile uint8_t &ready,
                          uint32_t nowUs, uint32_t uptimeMs) {
    const uint32_t elapsedUs = nowUs - previousUs_; // deliberate uint32 wrap
    // The same acquisition timestamp is not a new observation. Reject it
    // before bias subtraction, recovery, or health publication can run again.
    if (clockStarted_ && elapsedUs == 0) return false;
    previousUs_ = nowUs;
    float dt = clockStarted_ && previouslyReady_ && elapsedUs <= kMaximumSampleGapUs
        ? elapsedUs * 0.000001f : 0.0f;
    clockStarted_ = true;
    if (!finiteSample(state)) {
        previouslyReady_ = false;
        return false;
    }
    // Calibration owns its timestamp contract, including legacy entry points.
    calibrate(state, ready, nowUs, uptimeMs);
    if (!ready) {
        previouslyReady_ = false;
        return false;
    }
    const bool firstAttitude = !previouslyReady_;
    if (firstAttitude) dt = 0; // no integration of discovery/calibration time
    previouslyReady_ = true;
    // Permit one initial gravity correction, but repeated timestamps and stalled
    // intervals do not repeatedly advance the filter without elapsed time.
    const float correctionDt = firstAttitude ? 0.003f : dt;
    const float correction = initialTiltPending_ ? 0.0f :
        correctionWeight(correctionDt, model_ == Model::Icm42670);
    float correctedGyro[3];
    for (uint8_t i = 0; i < 3; ++i) {
        correctedGyro[i] = state.gyro[i] - state.gyro_bias[i];
        if (!std::isfinite(correctedGyro[i])) return false;
        state.gyro[i] = correctedGyro[i];
        // Preserve public telemetry shaping, without discarding real slow
        // rotations in the attitude estimator.
        if (model_ == Model::Icm42670 && fabsf(state.gyro[i]) < 2.0f) state.gyro[i] = 0;
    }

    float roll = state.rol, pitch = state.pit, yaw = state.yaw;
    // Only a newly completed stationary calibration establishes initial tilt.
    // Ordinary read failures or invalid-sample recovery do not snap attitude.
    if (initialTiltPending_) {
        if (!gravityAttitude(state.acc_bias, roll, pitch)) return false;
        dt = 0;
    }
    float accRoll = 0, accPitch = 0;
    const bool gravityAvailable = gravityAttitude(state.acc, accRoll, accPitch);
    const bool qmi = model_ == Model::Qmi8658c;
    const float gx = qmi ? -correctedGyro[1] : correctedGyro[0];
    const float gy = qmi ? -correctedGyro[0] : correctedGyro[1];
    const float gz = qmi ? -correctedGyro[2] : correctedGyro[2];
    const float rollRad = roll * kDegToRad;
    const float pitchRad = pitch * kDegToRad;
    const float sinRoll = sinf(rollRad), cosRoll = cosf(rollRad);
    const float cosPitch = cosf(pitchRad);
    float rollRate = 0, pitchRate = 0, yawRate = 0;
    // Local gravity is (g*sin(theta), -g*sin(phi)*cos(theta),
    // -g*cos(phi)*cos(theta)). All models use the corresponding Euler rates;
    // body Z angular velocity alone is not world yaw rate when tilted.
    // Near gimbal lock freeze propagation; gravity can still recover tilt.
    if (fabsf(cosPitch) >= 0.05f) {
        const float projected = gy * sinRoll + gz * cosRoll;
        yawRate = projected / cosPitch;
        rollRate = gx + sinf(pitchRad) * yawRate;
        pitchRate = gy * cosRoll - gz * sinRoll;
    }
    const float predictedRoll = roll + rollRate * dt;
    const float predictedPitch = pitch + pitchRate * dt;
    yaw += yawRate * dt;
    if (gravityAvailable) {
        roll = (1.0f - correction) * predictedRoll + correction * accRoll;
        pitch = (1.0f - correction) * predictedPitch + correction * accPitch;
    } else {
        roll = predictedRoll;
        pitch = predictedPitch;
    }
    if (!std::isfinite(roll) || !std::isfinite(pitch) || !std::isfinite(yaw)) return false;
    state.rol = roll;
    state.pit = pitch;
    state.yaw = yaw;
    initialTiltPending_ = false;
    return true;
}

namespace imu_health {
static_assert(ATOMIC_INT_LOCK_FREE == 2 && ATOMIC_BOOL_LOCK_FREE == 2,
              "IMU sample health requires lock-free scalar publication");
namespace {
std::atomic<uint32_t> lastSampleMs{0};
std::atomic<bool> haveSample{false};
}

void reset() { haveSample.store(false, std::memory_order_release); }
void publish(uint32_t nowMs) {
    lastSampleMs.store(nowMs, std::memory_order_release);
    haveSample.store(true, std::memory_order_release);
}
bool observed() { return haveSample.load(std::memory_order_acquire); }
bool fresh(uint32_t nowMs) {
    if (!observed()) return false;
    const uint32_t sampledAt = lastSampleMs.load(std::memory_order_acquire);
    // A concurrent publication may be newer than the caller's captured time.
    // Signed modular age tolerates that race and the uint32 millis rollover.
    const int32_t age = static_cast<int32_t>(nowMs - sampledAt);
    return age >= -static_cast<int32_t>(kMaximumAgeMs) &&
           age <= static_cast<int32_t>(kMaximumAgeMs);
}
} // namespace imu_health
