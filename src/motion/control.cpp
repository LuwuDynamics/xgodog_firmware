#include <cmath>
#include <stdint.h>
#include "motion/config.h"
#include "motion/time.h"
#include "motion/control.h"
#include "motor/motor.h"
#include "action/action.h"
#include "robot/math.h"
#include "robot/state.h"
#include "motion/kinematics.h"
#include "sensor/imu.h"
#include "sensor/attitude.h"
#include "motion/gait.h"
#include <atomic>
#include <Arduino.h>

static void updateBodyXYZ();
static void updateBodyRPY();
static void updateBodyState();
static void updateYawSpeed(bool stopRequested);

namespace motion {
static_assert(Period_T_MIN > 0 && Period_T_MAX >= Period_T_MIN &&
              Period_T_MAX <= UINT32_MAX / 1000u, "Pose period must fit microseconds");
// One commanded pose axis. Host period units stay 0..255 (0 finishes the
// current cycle). Integer microseconds avoid drift from partitioning time.
class PoseOscillator {
public:
    bool active() const { return commanded_ || elapsedUs_ != 0; }

    // True means publish sample(), including the final neutral stop sample.
    bool advance(float command, uint32_t dtUs) {
        if (!std::isfinite(command) || command < 0) command = 0;
        if (command > 255) command = 255;
        const bool update = command != 0 || active();
        commanded_ = command != 0;
        if (commanded_ && command != command_) {
            const uint32_t nextPeriod = (uint32_t)(1000.0 *
                (Period_T_MIN + command / 255.0 * (Period_T_MAX - Period_T_MIN)) + 0.5);
            // Changing frequency preserves phase, not elapsed milliseconds.
            // Wide arithmetic runs only on command changes; error is <1 us.
            elapsedUs_ = (uint32_t)((uint64_t)elapsedUs_ * nextPeriod / periodUs_);
            periodUs_ = nextPeriod;
            command_ = command;
        }
        if (!update) return false;
        if (!commanded_ && elapsedUs_ == 0) return true;
        const uint32_t untilWrap = periodUs_ - elapsedUs_;
        if (dtUs < untilWrap) elapsedUs_ += dtUs;
        else elapsedUs_ = commanded_ ? (dtUs - untilWrap) % periodUs_ : 0;
        return true;
    }

    float sample() const {
        constexpr float twoPi = 6.2831853071795864769f;
        return sinf(((float)elapsedUs_ / periodUs_) * twoPi);
    }

private:
    uint32_t elapsedUs_ = 0;
    uint32_t periodUs_ = Period_T_MAX * 1000u;
    float command_ = 0;
    bool commanded_ = false;
};
} // namespace motion

namespace motion_time {
namespace {
uint32_t stepUs = kNominalUs;
uint32_t ticks = 1;
uint32_t remainderUs = 0;
}
void beginStep(uint32_t elapsedUs) {
    stepUs = elapsedUs > kMaximumStepUs ? kMaximumStepUs : elapsedUs;
    const uint32_t accumulated = remainderUs + stepUs;
    ticks = accumulated / kNominalUs;
    remainderUs = accumulated % kNominalUs;
}
void endStep() { stepUs = kNominalUs; ticks = 1; }
float stepTicks() { return (float)stepUs / kNominalUs; }
float stepSeconds() { return (float)stepUs / 1000000.0f; }
uint32_t stepMicroseconds() { return stepUs; }
uint32_t wholeTicks() { return ticks; }
} // namespace motion_time

static PID_t pit_pid = {.fpKp = 0.016f, .fpKd = 0.0001f, .fpUMax = 5.0f, .fpEiMax = 5.0f, .fpEMin = 3.0f};
static PID_t rol_pid = {.fpKp = 0.012f, .fpKd = 0.0001f, .fpUMax = 3.0f, .fpEiMax = 3.0f, .fpEMin = 1.0f};
namespace {
static_assert(ATOMIC_BOOL_LOCK_FREE == 2, "Motion reset requires a lock-free flag");
motion::PoseOscillator xCycle, yCycle, zCycle, yawCycle, pitchCycle, rollCycle;
bool headingFeedbackLost = true;
bool wheelHeadingActive = false;
float wheelLegYaw = 0;
constexpr float kWheelHeadingDeadbandDeg = 1.0f;
constexpr float kWheelHeadingLimitDeg = 10.0f;
constexpr float kWheelHeadingGain = 0.05f;
constexpr float kWheelLegHeadingGain = 0.33f;
bool stableFeedbackLost = true;
std::atomic<bool> resetPending{false};
std::atomic<bool> stopPending{false};

void clearPidHistory(PID_t &pid) {
    pid.fpE = pid.fpPreE = pid.fpSumE = pid.fpU = 0;
}
}

void MotionControl_RequestReset() {
    resetPending.store(true, std::memory_order_release);
}

static bool translationIdle()
{
    return fabsf(body.vx) <= 5.0f && fabsf(body.vy) <= 1.0f;
}

static bool turnInputIdle()
{
    return fabsf(imu.yaw_speed) <= 0.1f;
}

void MotionControl_NotifyMotionCommand()
{
    if (yaw_track_flag == 0 && translationIdle() && turnInputIdle())
        stopPending.store(true, std::memory_order_release);
}

static bool CommandSpeedIdle(void)
{
    if (MoveMode == MOVE_MODE_WHEEL2)
        return translationIdle() && turnInputIdle();
    const bool headingAvailable = isIMUInit && imu_health::fresh((uint32_t)millis()) && isfinite(imu.yaw);
    return translationIdle() && (!headingAvailable || (turnInputIdle() && yaw_track_flag == 0));
}

static void updateWheelSpeed(){
    // Open-loop wheel output; pure wheel mode mixes the heading correction
    // into steering without using measured wheel speed.
    const bool idle = translationIdle() && turnInputIdle();
    double wheelDrive = 20.0 * body.vx;
    double wheelTurn = 40.0 * body.vyaw;
    if (MoveMode == MOVE_MODE_WHEEL2) {
        if (!isfinite(wheelDrive)) wheelDrive = 0;
        if (!isfinite(wheelTurn)) wheelTurn = 0;
        if (wheelTurn > WHEEL_ORDER_MAX) wheelTurn = WHEEL_ORDER_MAX;
        if (wheelTurn < -WHEEL_ORDER_MAX) wheelTurn = -WHEEL_ORDER_MAX;
        // Reserve differential steering before limiting forward/reverse drive.
        // Clipping each wheel independently can erase steering at full drive.
        const double driveLimit = WHEEL_ORDER_MAX - fabs(wheelTurn);
        if (wheelDrive > driveLimit) wheelDrive = driveLimit;
        if (wheelDrive < -driveLimit) wheelDrive = -driveLimit;
    }
    for (int i = 0; i < 4; ++i) {
        double command = 0;
        if (MoveMode == MOVE_MODE_WHEEL2) {
            if (!idle) command = wheel[i].sign * wheelDrive - wheelTurn;
        } else if (MoveMode == MOVE_MODE_HYBRID || MoveMode == MOVE_MODE_WHEEL) {
            if (!idle) command = wheel[i].sign * (20.0 * body.vx);
        } else {
            // Keep the deployed feedforward scale; no accumulated correction.
            // Keep this product in float so the represented endpoint 2.3f
            // reaches 230 instead of truncating a double result to 229.
            command = wheel[i].sign * (wheel[i].des_vel * WHEEL_OPEN_LOOP_GAIN);
        }
        if (!isfinite(command)) command = 0;
        // Saturate before conversion, including oversized host commands.
        if (command > WHEEL_ORDER_MAX) command = WHEEL_ORDER_MAX;
        if (command < -WHEEL_ORDER_MAX) command = -WHEEL_ORDER_MAX;
        wheel[i].order_vel = (short)command;
    }
}

static void updateMoveSpeed(bool stopRequested) {
    body.vx = Saturation(body.vx, bodyConfig.vx_max, bodyConfig.vx_min);
    body.vy = Saturation(body.vy, bodyConfig.vy_max, bodyConfig.vy_min);
    body.vyaw = Saturation(body.vyaw, bodyConfig.vyaw_max, bodyConfig.vyaw_min);
    // The gait adapter owns contact, phase, transitions and the wheel handoff.
    const float legYaw = MoveMode == MOVE_MODE_WHEEL2 ? wheelLegYaw : body.vyaw;
    // Process a short stop pulse even when the next motion command has arrived.
    const bool finishStep = stopRequested && yaw_track_flag == 0 && GaitIsActive();
    GaitUpdateMotion(!finishStep && !CommandSpeedIdle(), legYaw);
}

static void controlStep(){
    // Handle startup folding in the same task as ArmIK and the delayed fold
    // stage. A loop-task write could be overwritten by an already-running IK.
    if (isIMUInit == 1 && motorInitializationComplete) {
        FoldArm();
        isIMUInit = 2;
    }
    ActionManager();
    UpdateMotorMoveToMid();

    // Reset can originate in a protocol task or ActionManager. Only this task
    // writes the private cycle/PID state; clearing public periods alone merely
    // requests a graceful cycle finish and is insufficient for an explicit reset.
    if (resetPending.exchange(false, std::memory_order_acq_rel)) {
        xCycle = yCycle = zCycle = motion::PoseOscillator{};
        yawCycle = pitchCycle = rollCycle = motion::PoseOscillator{};
        headingFeedbackLost = stableFeedbackLost = true;
        wheelHeadingActive = false;
        clearPidHistory(move_yaw_pid);
        clearPidHistory(pit_pid);
        clearPidHistory(rol_pid);
    }

    if(xgo_mode == XGO_MODE_BODY){
        // Zero means unlimited speed. Preserve Reset's recovery limit at rest;
        // release it only for movement or a gait that is still finishing.
        if (Action_Ctx.id == 0 && isIMUInit != 0 && (!CommandSpeedIdle() || GaitIsActive()))
            SetLegMotorSpeed(0);
        const bool stopRequested = stopPending.exchange(false, std::memory_order_acq_rel);
        updateBodyState();
        updateYawSpeed(stopRequested);
        updateMoveSpeed(stopRequested);
        updateWheelSpeed();
        WholeIK();
    }else if (xgo_mode == XGO_MODE_LEG || xgo_mode == XGO_MODE_MOTOR) {
        // Body feedback is suspended in direct-control modes. Reuse the
        // existing recovery path on return: anchor relative heading to the
        // current observation and prime tilt D, retaining explicit yaw targets.
        headingFeedbackLost = stableFeedbackLost = true;
        wheelHeadingActive = false;
        GaitResetWalk();
        if (xgo_mode == XGO_MODE_LEG)
            for(int i = 0; i < 4; i++) LegIK(&leg[i]);
    }
    ArmIK();
    if(xgo_mode != XGO_MODE_MOTOR){
        for(int i = 0; i < 4; i++){
            motor[3*i].des_angle = leg[i].theta1;
            motor[3*i+1].des_angle = leg[i].theta2;
            motor[3*i+2].des_angle = leg[i].theta3;
        }
    }
    UpdateMotorDesPos();
}

void MotionControl_Step(uint32_t elapsedUs) {
    if (elapsedUs == 0) return;
    motion_time::beginStep(elapsedUs);
    controlStep();
    motion_time::endStep();
}

static void updateYawSpeed(bool stopRequested){
    if (stopRequested) {
        headingFeedbackLost = true;
        wheelHeadingActive = false;
    }
    wheelLegYaw = 0;
    if (!isfinite(imu.yaw_speed)) imu.yaw_speed = 0;
    if (MoveMode == MOVE_MODE_WHEEL2) {
        body.vyaw = 0;
        yaw_track_flag = 0;
        clearPidHistory(move_yaw_pid);
        headingFeedbackLost = true;
        // Manual yaw is a rate command, in degrees per nominal 5 ms step.
        // Stop cancels the reference immediately; stale feedback cannot accrue
        // a turn that would be replayed when observations recover.
        if (CommandSpeedIdle() || !isIMUInit ||
            !imu_health::fresh((uint32_t)millis()) || !isfinite(imu.yaw)) {
            wheelHeadingActive = false;
            return;
        }
        if (!wheelHeadingActive || !isfinite(imu.des_yaw)) {
            imu.des_yaw = imu.yaw;
            wheelHeadingActive = true;
        }
        const float target = imu.des_yaw + imu.yaw_speed * motion_time::stepTicks();
        imu.des_yaw = isfinite(target) ? target : imu.yaw;
        const float errorDeg = imu.des_yaw - imu.yaw;
        // Separate wheel steering from leg stride: a small correction must
        // not start a gait, and a large one must not saturate wheel steering.
        if (fabsf(errorDeg) > kWheelHeadingLimitDeg)
            wheelLegYaw = Saturation(kWheelLegHeadingGain * errorDeg, bodyConfig.vyaw_max, bodyConfig.vyaw_min);
        else if (fabsf(errorDeg) > kWheelHeadingDeadbandDeg)
            body.vyaw = kWheelHeadingGain * errorDeg;
        return;
    }
    wheelHeadingActive = false;
    if (yaw_track_flag && !isfinite(imu.yaw_des)) yaw_track_flag = 0;
    if (!isIMUInit || !imu_health::fresh((uint32_t)millis()) || !isfinite(imu.yaw)) {
        body.vyaw = 0;
        clearPidHistory(move_yaw_pid);
        headingFeedbackLost = true;
        return;
    }
    if (CommandSpeedIdle()) {
        imu.des_yaw = imu.yaw;
        body.vyaw = 0;
        clearPidHistory(move_yaw_pid);
        headingFeedbackLost = true;
        return;
    }
    if (headingFeedbackLost) {
        // Anchor the next motion at the current observation after idle or lost IMU feedback.
        imu.des_yaw = imu.yaw;
        clearPidHistory(move_yaw_pid);
        headingFeedbackLost = false;
    }
    if (!isfinite(imu.des_yaw)) imu.des_yaw = imu.yaw;
	move_yaw_pid.fpEMin = 1.5f;
    move_yaw_pid.fpKp = 0.33f; // 0.3 * 1.1
	move_yaw_pid.fpKi = 0.0f;
	move_yaw_pid.fpKd = 0.02f;
	float track_err = 0.0;
	if (yaw_track_flag) {
		gm.T = bodyConfig.NORMAL_T;
		SetLegMotorSpeed(0);
		track_err = imu.yaw_des - imu.yaw;
		if(abs(track_err)<1.5){
			yaw_track_flag = 0;
			imu.des_yaw = imu.yaw;
			body.vyaw = 0;
		}else{
			body.vyaw = 0.1*track_err;
			if (body.vyaw<0&&body.vyaw>-2.5f){
				body.vyaw = -2.5f;
			}else if (body.vyaw>0&&body.vyaw<2.5f){
				body.vyaw = 2.5f;
			}
			body.vyaw = Saturation(body.vyaw,bodyConfig.vyaw_max,-bodyConfig.vyaw_max);
		}
		return;
	}

    const float target = imu.des_yaw + imu.yaw_speed * motion_time::stepTicks();
    imu.des_yaw = isfinite(target) ? target : imu.yaw;
	move_yaw_pid.fpDes = imu.des_yaw;
	move_yaw_pid.fpFB = imu.yaw;
	CalISeparatedPID(&move_yaw_pid, motion_time::stepTicks());
	body.vyaw = move_yaw_pid.fpU;
	body.vyaw = Saturation(body.vyaw,bodyConfig.vyaw_max,-bodyConfig.vyaw_max);
}

static void updateBodyXYZ(){
    if (body.x_T || body.y_T || body.z_T || xCycle.active() || yCycle.active() || zCycle.active()) {
        if (xCycle.advance(body.x_T, motion_time::stepMicroseconds())) body.x = 0.8f * bodyConfig.xmax * xCycle.sample();
        if (yCycle.advance(body.y_T, motion_time::stepMicroseconds())) body.y = 0.8f * bodyConfig.ymax * yCycle.sample();
        if (zCycle.advance(body.z_T, motion_time::stepMicroseconds())) body.z = (bodyConfig.zmax + bodyConfig.zmin) / 2.0f
            + (bodyConfig.zmax - bodyConfig.zmin) / 2.0f * 0.8f * zCycle.sample();
    }
}

static void updateBodyRPY(){

	if(imu.stable == 1)//自稳定
	{
		body.yaw = 0;//航向角为0
		if (!isIMUInit || !imu_health::fresh((uint32_t)millis()) ||
            !isfinite(imu.pit) || !isfinite(imu.rol)) {
            clearPidHistory(pit_pid);
            clearPidHistory(rol_pid);
            stableFeedbackLost = true;
            return; // Hold the last compensation, never integrate stale tilt.
        }
        if (stableFeedbackLost) {
            clearPidHistory(pit_pid);
            clearPidHistory(rol_pid);
            pit_pid.fpPreE = -imu.pit;
            rol_pid.fpPreE = -imu.rol;
            stableFeedbackLost = false;
        }
		//陀螺仪加PID
		pit_pid.fpDes = 0;
		pit_pid.fpFB = imu.pit;
		CalISeparatedPID(&pit_pid, motion_time::stepTicks());
		body.pit = body.pit + pit_pid.fpU * motion_time::stepTicks();

		rol_pid.fpDes = 0;
		rol_pid.fpFB = imu.rol;
		CalISeparatedPID(&rol_pid, motion_time::stepTicks());
		body.rol = body.rol + rol_pid.fpU * motion_time::stepTicks();
	}
    else {
        stableFeedbackLost = true;
        if (body.yaw_T || body.pit_T || body.rol_T || pitchCycle.active() || rollCycle.active() || yawCycle.active()) {
            if (yawCycle.advance(body.yaw_T, motion_time::stepMicroseconds())) body.yaw = 0.8f * bodyConfig.yawmax * yawCycle.sample();
            if (pitchCycle.advance(body.pit_T, motion_time::stepMicroseconds())) body.pit = 0.8f * bodyConfig.pitmax * pitchCycle.sample();
            if (rollCycle.advance(body.rol_T, motion_time::stepMicroseconds())) body.rol = 0.8f * bodyConfig.rolmax * rollCycle.sample();
        }
    }
}

static void updateBodyState(){
    updateBodyXYZ();
    updateBodyRPY();
    body.x = Saturation(body.x,bodyConfig.xmax, bodyConfig.xmin);
    body.y = Saturation(body.y,bodyConfig.ymax, bodyConfig.ymin);
    body.z = Saturation(body.z,bodyConfig.zmax, bodyConfig.zmin);
    body.rol = Saturation(body.rol,bodyConfig.rolmax, bodyConfig.rolmin);
	body.pit = Saturation(body.pit,bodyConfig.pitmax, bodyConfig.pitmin);
	body.yaw = Saturation(body.yaw,bodyConfig.yawmax, bodyConfig.yawmin);
}

void controlTask(void *pvParameter){
    (void)pvParameter;
    TickType_t lastWakeTime = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(1000 / SYSTEM_TICK_HZ);
    static_assert(pdMS_TO_TICKS(1000 / SYSTEM_TICK_HZ) > 0, "RTOS tick is too coarse for Motion Tick");
    motion_time::SampleClock clock;

    for(;;){
        MotionControl_Step(clock.sample(micros()));
        // Do not replay stale deadlines in a tight burst after preemption or
        // a slow tick. Nominal scheduling remains absolute at 200 Hz.
        const TickType_t now = xTaskGetTickCount();
        if ((TickType_t)(now - lastWakeTime) >= period) lastWakeTime = now;
        vTaskDelayUntil(&lastWakeTime, period);
    }
}
