#include "motion/gait.h"
#include "motion/walk_planner.h"
#include "motion/gait_engine.h"
#include "motion/kinematics.h"
#include "motion/time.h"
#include <cmath>

namespace {
constexpr float kLiteHighWalkPeriod = 3.2f;
constexpr float kLiteHighWalkLift = 35.0f;
constexpr float kLiteTrotSwingRatio = 3.0f/8.0f;
constexpr float kLiteTrotSwingX = 8.5f;
constexpr float kLiteTrotStrideY = 16.0f;
constexpr float kTrotSwingRatio = 0.48f;

xgo::WalkConfig liteHighWalkConfig() {
    auto config = xgo::HighWalkConfig();
    config.min_cycle_s = kLiteHighWalkPeriod;
    config.transfer_fraction = 0.5f;
    config.max_lift_mm = kLiteHighWalkLift;
    return config;
}

xgo::WalkPlanner high_walk{xgo::HighWalkConfig()};
xgo::WalkPlanner lite_high_walk{liteHighWalkConfig()};

xgo::WalkPlanner &highWalk() {
    return xgo_version == XGOLITE ? lite_high_walk : high_walk;
}
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

xgo::WalkFeet currentFeet() {
    xgo::WalkFeet feet;
    for (int i=0; i<4; ++i) feet[i] = {leg[i].x_step, leg[i].y_step, leg[i].z_step};
    return feet;
}

float forwardStride() {
    return MoveMode == MOVE_MODE_WHEEL || MoveMode == MOVE_MODE_WHEEL2 ? 0 : body.vx;
}

xgo::WalkInput highWalkInput(const xgo::WalkConfig &config, bool moving, float legYaw) {
    xgo::WalkInput input;
    input.moving = moving;
    input.full_lift_from_start = isWheelLeggedMini();
    const bool lite = xgo_version == XGOLITE;
    input.cycle_s = lite ? kLiteHighWalkPeriod : gm.T / SYSTEM_TICK_HZ * config.period_scale;
    input.lift_mm = lite ? kLiteHighWalkLift : fmaxf(gm.z_step, 50.0f);
    const float vx = forwardStride();
    if (moving) input.support_x_mm = reverseStrideOffset(vx, config.max_stride_x_mm);
    const float yaw = legYaw * config.yaw_scale;
    const float footHalfWidth = kinematics::neutralFootHalfWidth(bodyConfig);
    for (int i=0; i<4; ++i) {
        input.stride[i] = {vx + YAW_STRIDE_SCALE*sign.nppn[i]*yaw,
            body.vy + sign.ppnn[i]*yaw, 0};
        input.nominal[i] = {kinematics::neutralFootX(bodyConfig, sign.ppnn[i])-body.x,
            -sign.nppn[i]*footHalfWidth-body.y, 0};
    }
    return input;
}

uint32_t lowTrotTicks() {
    return xgo_version == XGOLITE ? bodyConfig.MID_T : SEC_2_TICK(LOW_TROT_PERIOD);
}

float lowTrotStrideX() {
    return xgo_version == XGOLITE ? LOW_TROT_STRIDE_X_LITE : LOW_TROT_STRIDE_X;
}

void applyFixedGaitPeriod() {
    if (gm.gait_type == GAIT_WALK) gm.T = SEC_2_TICK(WALK_PERIOD);
    else if (gm.gait_type == GAIT_LOW_TROT) gm.T = lowTrotTicks();
}
}

void GaitResetWalk() {
    high_walk.reset();
    lite_high_walk.reset();
    ordinary_gait.reset();
    applyFeet({});
}

static void updateHighWalk(xgo::WalkPlanner &planner, bool selected, bool moving, float legYaw) {
    const float dt = motion_time::stepSeconds();
    if (!selected) {
        applyFeet(planner.update({}, dt));
        return;
    }
    if (!planner.active()) {
        planner.setLeadPair(takeLead02());
        planner.begin(currentFeet());
    }
    applyFeet(planner.update(highWalkInput(planner.config(), moving, legYaw), dt));
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
        applyFixedGaitPeriod();
    }
    return true;
}

namespace {
float liteLift(unsigned legIndex, bool reversing) {
    switch (gm.gait_type) {
    case GAIT_TROT:
        if (reversing) return legIndex < 2 ? 15.75f : 12.6f;
        return gm.T > bodyConfig.NORMAL_T ? 8.0f : 16.0f;
    case GAIT_LOW_TROT:
        return reversing ? 3.4f : 4.0f;
    case GAIT_WALK:
        return 25.0f;
    default:
        return gm.z_step;
    }
}

xgo::GaitInput ordinaryInput(bool moving, float legYaw) {
    xgo::GaitInput input;
    input.full_lift_from_start = isWheelLeggedMini();
    input.kind = gm.gait_type;
    input.walk = gm.gait_type == GAIT_WALK;
    input.period_s = gm.T / SYSTEM_TICK_HZ;
    const bool lowTrot = gm.gait_type == GAIT_LOW_TROT;
    const bool mark = gm.gait_type == GAIT_MARKTIME;
    const bool liteTrot = xgo_version == XGOLITE && gm.gait_type == GAIT_TROT;
    const bool reversing = !mark && body.vx < 0;
    input.swing_ratio = input.walk ? WALK_SR : lowTrot ? LOW_TROT_SWING : kTrotSwingRatio;
    input.lite_swing = xgo_version == XGOLITE;
    float vx = forwardStride();
    float vy = body.vy;
    const float lowTrotLimit = lowTrot ? lowTrotStrideX() : 0;
    if (lowTrot) {
        if (vx > lowTrotLimit) vx = lowTrotLimit;
        if (vx < -lowTrotLimit) vx = -lowTrotLimit;
    } else if (liteTrot) {
        // Scale translation at the gait layer before adding the turn stride.
        const float xLimit = gm.T <= bodyConfig.FAST_T ? 20.0f : 25.0f;
        vx *= xLimit / (vx < 0 ? -bodyConfig.vx_min : bodyConfig.vx_max);
        vy *= kLiteTrotStrideY / bodyConfig.vy_max;
    }
    if (input.lite_swing) {
        if (liteTrot) input.swing_ratio = kLiteTrotSwingRatio;
        else if (input.walk) input.swing_ratio = 0.25f;
        else if (mark) input.swing_ratio = 0.5f;
    }
    if (liteTrot) {
        input.swing_x_offset = reversing ? -kLiteTrotSwingX : kLiteTrotSwingX;
        input.stride_center_y = body.vy > 0 ? -6.0f : body.vy < 0 ? 6.0f : 0.0f;
    } else if (moving && reversing && xgo_version == XGOMINI) {
        const float maxReverseStride = lowTrot ? fminf(-bodyConfig.vx_min, lowTrotLimit) : -bodyConfig.vx_min;
        input.stride_center_x.fill(fminf(reverseStrideOffset(vx, maxReverseStride),
                                         xgo::kMaxSupportOffsetMm));
    }

    const float forwardOffset = gm.T > bodyConfig.NORMAL_T ? 2.0f : 0.0f;
    bool hasStride = mark;
    for (int i=0; i<4; ++i) {
        auto &stride = input.stride[i];
        if (!mark) stride = {vx + YAW_STRIDE_SCALE*sign.nppn[i]*legYaw,
                             vy + sign.ppnn[i]*legYaw, 0};
        hasStride |= std::fabs(stride.x)>1 || std::fabs(stride.y)>1;
        if (liteTrot) input.stride_center_x[i] = reversing ? 2.0f : forwardOffset - stride.x/7.0f;
        input.lift[i] = input.lite_swing ? liteLift(i, reversing)
            : lowTrot ? (i < 2 || reversing ? LOW_TROT_LIFT_FRONT : LOW_TROT_LIFT_REAR)
                      : input.walk ? WALK_LIFT : gm.z_step;
    }
    input.moving = moving && hasStride;
    return input;
}
}

bool GaitIsActive() { return ordinary_gait.active() || highWalk().active(); }

void GaitUpdateMotion(bool moving, float legYaw) {
    const bool wheelMode = MoveMode == MOVE_MODE_WHEEL2;
    const bool want_high = gm.gait_type == GAIT_HIGH_WALK;
    const bool mark = gm.gait_type == GAIT_MARKTIME;
    // Wheel mode uses the legs for lateral motion and large heading errors.
    // Forward wheel travel and small differential corrections do not step.
    moving = wheelMode ? moving && (std::fabs(body.vy) > 1.0f || std::fabs(legYaw) > 1.0f)
                       : moving || mark;
    applyFixedGaitPeriod();

    // Finish the active planner's stop before handing the feet to another gait.
    const bool ordinaryActive = ordinary_gait.active();
    if (!ordinaryActive) {
        auto &planner = highWalk();
        if (planner.active() || (want_high && moving)) {
            updateHighWalk(planner, want_high, moving, legYaw);
            return;
        }
        if (!moving || want_high) return;
    }

    const auto input = ordinaryInput(moving && !want_high, legYaw);
    if (!ordinaryActive) {
        if (!input.moving) return;
        ordinary_gait.begin(currentFeet(), takeLead02());
    }
    applyFeet(ordinary_gait.update(input, motion_time::stepMicroseconds()));
}
