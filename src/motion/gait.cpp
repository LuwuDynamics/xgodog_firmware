#include "motion/gait.h"
#include "motion/walk_planner.h"
#include "motion/gait_engine.h"
#include "motion/kinematics.h"
#include "motion/time.h"
#include <cmath>

uint8_t gait_reinit = 0;
namespace {
xgo::WalkPlanner high_walk{xgo::HighWalkConfig()};
xgo::GaitEngine ordinary_gait;
float periodBeforeOverride = 0;
bool lead_02 = true;

bool takeLead02() {
    const bool now = lead_02;
    lead_02 = !lead_02;
    return now;
}

void applyFeet(const xgo::WalkFeet &feet) {
    for (int i=0; i<4; ++i) {
        leg[i].x_step = feet[i].x;
        leg[i].y_step = feet[i].y;
        leg[i].z_step = feet[i].z;
    }
}

float reverseStrideOffset(float strideX, float maxReverseStride) {
    if (xgo_version != XGOMINI || !(strideX < 0) || !(maxReverseStride > 0))
        return 0;
    return xgo::kMaxSupportOffsetMm * (-strideX / maxReverseStride);
}

bool isWheelLeggedMini() {
    return xgo_version == XGOMINI && hasWheel == 1;
}

xgo::WalkInput fillStride(float period_scale, float yaw_scale, float lift_mm, bool moving, float legYaw) {
    xgo::WalkInput input;
    input.moving = moving;
    input.full_lift_from_start = isWheelLeggedMini();
    input.cycle_s = gm.T / SYSTEM_TICK_HZ * period_scale;
    input.lift_mm = lift_mm;
    const float vx = MoveMode == MOVE_MODE_WHEEL || MoveMode == MOVE_MODE_WHEEL2 ? 0 : body.vx;
    if (moving) input.support_x_mm = reverseStrideOffset(vx, high_walk.config().max_stride_x_mm);
    const float yaw = legYaw * yaw_scale;
    const float footHalfWidth = kinematics::neutralFootHalfWidth(bodyConfig);
    for (int i=0; i<4; ++i) {
        input.stride[i] = {vx + YAW_STRIDE_SCALE*sign.nppn[i]*yaw,
            body.vy + sign.ppnn[i]*yaw, 0};
        input.nominal[i] = {sign.ppnn[i]*bodyConfig.length-body.x,
            -sign.nppn[i]*footHalfWidth-body.y, 0};
    }
    return input;
}

float lowTrotPeriod() {
    return xgo_version == XGOLITE ? LOW_TROT_PERIOD_LITE : LOW_TROT_PERIOD;
}

float lowTrotStrideX() {
    return xgo_version == XGOLITE ? LOW_TROT_STRIDE_X_LITE : LOW_TROT_STRIDE_X;
}
}

void GaitResetWalk() {
    high_walk.reset();
    ordinary_gait.reset();
}

static void prepareLowTrot(float *vx) {
    if (gm.gait_type == GAIT_WALK) {
        gm.T = SEC_2_TICK(WALK_PERIOD);
        return;
    }
    if (gm.gait_type != GAIT_LOW_TROT) return;
    gm.T = SEC_2_TICK(lowTrotPeriod());
    const float xmax = lowTrotStrideX();
    if (vx) {
        if (*vx > xmax) *vx = xmax;
        if (*vx < -xmax) *vx = -xmax;
    }
}

static bool updateWalk(bool moving, float legYaw) {
    const bool want_high = gm.gait_type == GAIT_HIGH_WALK;
    if (!want_high && !high_walk.active())
        return false;

    xgo::WalkFeet current;
    bool displaced = false;
    for (int i=0; i<4; ++i) {
        current[i] = {leg[i].x_step, leg[i].y_step, leg[i].z_step};
        displaced |= std::fabs(current[i].x)+std::fabs(current[i].y)+std::fabs(current[i].z) > 0.001f;
    }

    const float dt = motion_time::stepSeconds();
    xgo::WalkInput stop;
    stop.moving = false;
    if (!want_high && high_walk.active()) { applyFeet(high_walk.update(stop, dt)); return true; }

    if (want_high && !high_walk.active() && (moving || (gait_reinit && displaced))) {
        high_walk.setLeadPair(takeLead02());
        high_walk.begin(current);
    }
    if (want_high) gait_reinit = 0;
    const float lift = gm.z_step > 50.0f ? gm.z_step : 50.0f;
    applyFeet(high_walk.update(
        fillStride(high_walk.config().period_scale, high_walk.config().yaw_scale, lift, moving, legYaw), dt));
    return true;
}

bool GaitSetType(uint8_t type)
{
    if (type != GAIT_TROT && type != GAIT_WALK && type != GAIT_HIGH_WALK
        && type != GAIT_LOW_TROT && type != GAIT_MARKTIME)
        return false;
    if (gm.gait_type != type) {
        const bool override_period = (type == GAIT_LOW_TROT || type == GAIT_WALK);
        const bool was_override = (gm.gait_type == GAIT_LOW_TROT || gm.gait_type == GAIT_WALK);
        if (override_period && !was_override)
            periodBeforeOverride = gm.T;
        else if (!override_period && was_override && periodBeforeOverride > 0)
            gm.T = periodBeforeOverride;
        gm.gait_type = type;
        gait_reinit = 1;
        if (type == GAIT_LOW_TROT)
            gm.T = SEC_2_TICK(lowTrotPeriod());
        else if (type == GAIT_WALK)
            gm.T = SEC_2_TICK(WALK_PERIOD);
    }
    return true;
}

namespace {
xgo::GaitInput ordinaryInput(bool moving, float strideX) {
    xgo::GaitInput input;
    input.moving = moving;
    input.full_lift_from_start = isWheelLeggedMini();
    input.kind = gm.gait_type;
    input.walk = gm.gait_type == GAIT_WALK;
    input.period_s = gm.T / SYSTEM_TICK_HZ;
    input.swing_ratio = input.walk ? WALK_SR : gm.gait_type == GAIT_LOW_TROT ? LOW_TROT_SWING : sr;
    const bool reversing = gm.gait_type != GAIT_MARKTIME && body.vx < 0;
    const bool slowLiteTrot = xgo_version == XGOLITE && gm.gait_type == GAIT_TROT
        && std::fabs(gm.T-bodyConfig.SLOW_T)<0.5f;
    if (moving && reversing) {
        const float maxReverseStride = gm.gait_type == GAIT_LOW_TROT
            ? fminf(-bodyConfig.vx_min, lowTrotStrideX()) : -bodyConfig.vx_min;
        input.support_x_mm = reverseStrideOffset(strideX, maxReverseStride);
    }
    for (int i=0; i<4; ++i) {
        input.stride[i] = {gm.x_step[i], gm.y_step[i], 0};
        if (gm.gait_type == GAIT_MARKTIME) input.stride[i] = {};
        input.lift[i] = gm.gait_type == GAIT_LOW_TROT
            ? (i < 2 || reversing ? LOW_TROT_LIFT_FRONT : LOW_TROT_LIFT_REAR)
            : input.walk ? WALK_LIFT
            : slowLiteTrot ? TROT_SLOW_LIFT : gm.z_step;
    }
    return input;
}
xgo::WalkFeet currentFeet() {
    xgo::WalkFeet feet;
    for (int i=0; i<4; ++i) feet[i] = {leg[i].x_step, leg[i].y_step, leg[i].z_step};
    return feet;
}
}

bool GaitIsActive() { return ordinary_gait.active() || high_walk.active(); }

void GaitUpdateMotion(bool moving, float legYaw) {
    const bool wheelMode = MoveMode == MOVE_MODE_WHEEL2;
    const bool want_high = gm.gait_type == GAIT_HIGH_WALK;
    const bool mark = gm.gait_type == GAIT_MARKTIME;
    // Wheel mode uses the legs for lateral motion and large heading errors.
    // Forward wheel travel and small differential corrections do not step.
    moving = wheelMode ? moving && (std::fabs(body.vy) > 1.0f || std::fabs(legYaw) > 1.0f)
                       : moving || mark;
    float vx = MoveMode == MOVE_MODE_WHEEL || wheelMode ? 0 : body.vx;
    prepareLowTrot(&vx);
    for (int i=0; i<4; ++i) {
        gm.x_step[i] = mark ? 0 : vx + YAW_STRIDE_SCALE*sign.nppn[i]*legYaw;
        gm.y_step[i] = mark ? 0 : body.vy + sign.ppnn[i]*legYaw;
    }
    bool has_stride = mark;
    for (int i=0; i<4; ++i)
        has_stride |= std::fabs(gm.x_step[i])>1 || std::fabs(gm.y_step[i])>1;
    // High walk retains its deployed wheel/translation dispatch; its own
    // planner applies stride limits and the support-centroid transfer.
    if (!want_high) moving &= has_stride;

    // Only one planner owns the feet. Finish its stop before switching gait types.
    if (ordinary_gait.active()) {
        applyFeet(ordinary_gait.update(ordinaryInput(moving && !want_high, vx),
                                      motion_time::stepMicroseconds()));
        gait_reinit = 0;
        return;
    }
    if (high_walk.active() || (want_high && moving)) {
        updateWalk(moving && want_high, legYaw);
        return;
    }
    if (moving && !want_high) {
        ordinary_gait.begin(currentFeet(), takeLead02());
        applyFeet(ordinary_gait.update(ordinaryInput(true, vx), motion_time::stepMicroseconds()));
        gait_reinit = 0;
    }
}
