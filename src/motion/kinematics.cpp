#include "motion/kinematics.h"
#include <math.h>
#include "robot/state.h"
#include <Arduino.h>

namespace kinematics {
namespace {
// Keep the original double PI literal and evaluation order. Rounding here is
// visible in servo targets; changing literals to float changes nominal traces.
const double kPi = 3.1415926535897932384626433832795;

bool finitePoint(const Point &point)
{
    return isfinite(point.x) && isfinite(point.y) && isfinite(point.z);
}

bool safeAcos(float value, float &result)
{
    const float tolerance = 1.0e-4f;
    if (!isfinite(value) || value < -1.0f - tolerance || value > 1.0f + tolerance)
        return false;
    if (value < -1.0f) value = -1.0f;
    if (value > 1.0f) value = 1.0f;
    result = acosf(value);
    return isfinite(result);
}

void sinCos(float angle, float &sine, float &cosine)
{
    if (angle == 0.0f) {
        sine = angle; // retain sin(-0) as -0, without calling libm
        cosine = 1.0f;
    } else {
        sine = sinf(angle);
        cosine = cosf(angle);
    }
}
} // namespace

Geometry::Geometry(const BodyParam_t &profile)
    : upper(profile.L_b), lower(profile.L_c), lateral(profile.L_d),
      halfWidth(profile.width), halfLength(profile.length),
      upperSquared(upper * upper), lowerSquared(lower * lower),
      lateralSquared(lateral * lateral), twiceUpper(2.0f * upper), kneeDenominator(twiceUpper * lower),
      valid(isfinite(upperSquared) && isfinite(lowerSquared) &&
            isfinite(lateralSquared) && isfinite(kneeDenominator) &&
            isfinite(halfWidth) && isfinite(halfLength) && upper > 0.0f &&
            lower > 0.0f && lateral >= 0.0f && kneeDenominator > 0.0f)
{
}

bool solveLeg(const Geometry &geometry, const Point &foot, float side,
              float heightBias, LegAngles &angles)
{
    if (!geometry.valid || !finitePoint(foot) || (side != -1.0f && side != 1.0f) || !isfinite(heightBias))
        return false;
    const float x = foot.x;
    const float y = side * foot.y;
    const float z = foot.z + heightBias;
    const float legZSquared = z * z + y * y - geometry.lateralSquared;
    if (!isfinite(legZSquared) || legZSquared <= 0.0f || z <= 0.0f)
        return false;
    const float legZ = sqrtf(legZSquared);
    // Retain sqrt-then-square rounding for compatibility near workspace limits.
    const float length = sqrtf(legZ * legZ + x * x);
    const float hipDenominator = geometry.twiceUpper * length;
    if (!isfinite(length) || length <= 0.0f || hipDenominator == 0.0f)
        return false;
    float kneeAcos, hipAcos;
    const float kneeArg = (geometry.upperSquared + geometry.lowerSquared - length * length)
        / geometry.kneeDenominator;
    const float hipArg = (geometry.upperSquared + length * length - geometry.lowerSquared)
        / hipDenominator;
    if (!safeAcos(kneeArg, kneeAcos) || !safeAcos(hipArg, hipAcos))
        return false;
    // This solver supports feet below the hip only (effective z > 0).
    // Reject the upper half-space instead of silently reflecting its y/z point.
    const float lateral = atanf(geometry.lateral / legZ) - atanf(y / z);
    const float knee = kneeAcos - kPi / 2.0f;
    const float hip = hipAcos - atanf(x / legZ);
    if (!isfinite(knee) || !isfinite(hip) || !isfinite(lateral))
        return false;
    angles = { (float)(knee * 180.0f / kPi), (float)(hip * 180.0f / kPi),
               (float)(lateral * 180.0f / kPi) };
    return true;
}

bool solveBody(const Geometry &geometry, const BodyPose &pose, const Sign &sides,
               Point (&bases)[4])
{
    if (!geometry.valid || !finitePoint({pose.x, pose.y, pose.z}) ||
        !isfinite(pose.roll) || !isfinite(pose.pitch) || !isfinite(pose.yaw))
        return false;
    const float width = geometry.halfWidth + geometry.lateral;
    const float length = geometry.halfLength;
    const float roll = pose.roll * kPi / 180.0f;
    const float pitch = pose.pitch * kPi / 180.0f;
    const float yaw = pose.yaw * kPi / 180.0f;
    Point candidate[4];
    if (roll == 0.0f && pitch == 0.0f && yaw == 0.0f) {
        for (uint8_t i = 0; i < 4; ++i) {
            const float x = sides.ppnn[i] * length;
            const float y = -1 * sides.nppn[i] * width;
            candidate[i] = {x - pose.x - geometry.halfLength * sides.ppnn[i],
                            y - pose.y + geometry.halfWidth * sides.nppn[i], pose.z};
        }
    } else {
        float sr, cr, sp, cp, sy, cy;
        sinCos(roll, sr, cr);
        sinCos(pitch, sp, cp);
        sinCos(yaw, sy, cy);
        // R = Rz(yaw) Ry(pitch) Rx(roll). Inverting the body pose requires
        // reverse-order inverse rotations, not another forward rotation.
        // x/y translation is expressed in the yaw-aligned horizontal plane;
        // z is body height above the fixed ground anchors.
        for (uint8_t i = 0; i < 4; ++i) {
            const float anchorX = sides.ppnn[i] * length;
            const float anchorY = -sides.nppn[i] * width;
            // Rz(-yaw) F - (x,y,z): neutral world foot anchors stay fixed.
            const float headingX = cy*anchorX + sy*anchorY - pose.x;
            const float headingY = -sy*anchorX + cy*anchorY - pose.y;
            const float headingZ = -pose.z;
            // Ry(-pitch), then Rx(-roll), then subtract the body-frame hip.
            const float pitchX = cp*headingX - sp*headingZ;
            const float pitchZ = sp*headingX + cp*headingZ;
            const float localY = cr*headingY + sr*pitchZ;
            const float localZ = -sr*headingY + cr*pitchZ;
            candidate[i] = {pitchX - geometry.halfLength*sides.ppnn[i],
                            localY + geometry.halfWidth*sides.nppn[i], -localZ};
        }
    }
    for (const Point &point : candidate) if (!finitePoint(point)) return false;
    for (uint8_t i = 0; i < 4; ++i) bases[i] = candidate[i];
    return true;
}

bool compensateArm(const BodyPose &pose, float standingHeight, const Point &target,
                   Point &compensated)
{
    const float baseX = 70.0f, baseZ = 55.0f;
    if (!finitePoint(target) || !isfinite(pose.pitch) || !isfinite(pose.x) ||
        !isfinite(pose.z) || !isfinite(standingHeight)) return false;
    const float pitch = pose.pitch * kPi / 180.0f;
    float sinPitch, cosPitch;
    sinCos(pitch, sinPitch, cosPitch);
    const float worldX = target.x + baseX;
    const float worldZ = target.z + baseZ;
    const float armBaseX = baseX*cosPitch + baseZ*sinPitch;
    const float armBaseZ = -baseX*sinPitch + baseZ*cosPitch;
    const float dx = worldX - armBaseX - pose.x;
    const float dz = worldZ - armBaseZ - (pose.z - standingHeight);
    const Point candidate = {dx*cosPitch - dz*sinPitch, target.y, dx*sinPitch + dz*cosPitch};
    if (!finitePoint(candidate)) return false;
    compensated = candidate;
    return true;
}

ArmGeometry::ArmGeometry(float upperLength, float lowerLength)
    : upperSquared(upperLength * upperLength), lowerSquared(lowerLength * lowerLength),
      twiceUpper(2.0f * upperLength), elbowDenominator(twiceUpper * lowerLength),
      valid(isfinite(upperSquared) && isfinite(lowerSquared) &&
            isfinite(elbowDenominator) && upperLength > 0.0f && lowerLength > 0.0f &&
            elbowDenominator > 0.0f)
{
}

bool solveArm(const ArmGeometry &geometry, const Point &target, ArmAngles &angles)
{
    if (!geometry.valid || !finitePoint(target) || target.y != 0.0f) return false;
    const float x = target.x;
    const float z = target.z;
    const float length = sqrtf(x*x + z*z);
    if (!isfinite(length) || length <= 0.0f) return false;
    float elbowAcos, shoulderAcos;
    const float elbowArg = (length*length - geometry.upperSquared - geometry.lowerSquared)
        / geometry.elbowDenominator;
    const float shoulderArg = (geometry.upperSquared + length*length - geometry.lowerSquared)
        / (geometry.twiceUpper * length);
    if (!safeAcos(elbowArg, elbowAcos) || !safeAcos(shoulderArg, shoulderAcos)) return false;
    const float elbow = elbowAcos - kPi / 2.0f;
    // Principal planar angle, defined on the vertical axis without an epsilon
    // target offset or a spurious full shoulder revolution for x<0,z<0.
    const float targetAngle = atan2f(z, x);
    const float shoulder = -shoulderAcos - targetAngle + kPi / 2.0f;
    if (!isfinite(shoulder) || !isfinite(elbow)) return false;
    angles = {(float)(elbow * 180.0f / kPi), (float)(shoulder * 180.0f / kPi)};
    return true;
}
} // namespace kinematics

namespace {
const kinematics::ArmGeometry s_armGeometry(L1, L2);

kinematics::BodyPose bodyPose()
{
    return {body.x, body.y, body.z, body.rol, body.pit, body.yaw};
}

kinematics::Point footTarget(const Leg_t &limb, const kinematics::Point &base)
{
    return {base.x + limb.x_step, base.y + limb.y_step, base.z + limb.z_step};
}

void commitLeg(Leg_t &limb, const kinematics::Point &target,
               const kinematics::LegAngles &angles)
{
    limb.x = target.x;
    limb.y = target.y;
    limb.z = target.z;
    limb.theta1 = angles.knee;
    limb.theta2 = angles.hip;
    limb.theta3 = angles.lateral;
}

void commitBase(Leg_t &limb, const kinematics::Point &base)
{
    limb.x_base = base.x;
    limb.y_base = base.y;
    limb.z_base = base.z;
}
}

void Init_Kinematic(){
    for(int i=0;i<4;i++){
        leg[i].x = 0;
        leg[i].y = 0;
        leg[i].z = bodyConfig.z_init;
        leg[i].sign = -sign.nppn[i];
        leg[i].x_step = 0;
        leg[i].y_step = 0;
        leg[i].z_step = 0;
    }
    body.x = 0;
    body.y = 0;
    body.z = bodyConfig.z_init;
    body.pit = 0;
    body.rol = 0;
    body.yaw = 0;
    bodyConfig.xmax = bodyConfig.ymax = bodyConfig.zmax = 30.0f;
    bodyConfig.rolmax = bodyConfig.pitmax = bodyConfig.yawmax = 30.0f;
    body.yaw_T = body.pit_T = body.rol_T = 0.0f;
}

void LegIK(Leg_t *limb)
{
    if (limb == NULL) {
        return;
    }
    const kinematics::Geometry geometry(bodyConfig);
    const kinematics::Point target = footTarget(*limb, {limb->x_base, limb->y_base, limb->z_base});
    kinematics::LegAngles angles;
    if (!kinematics::solveLeg(geometry, target, limb->sign, limb->z_bias, angles)) {
        return;
    }
    commitLeg(*limb, target, angles);
}

static uint32_t armOverloadStartedMs = 0;

static void checkArmOverload(){
    if(abs(motor[12].fb_tor)<250){
        armOverloadStartedMs = millis();
    }

    if(millis()-armOverloadStartedMs>6000){
        motor[12].des_angle = motor[12].fb_angle;
    }
}

void ArmIK()
{
    checkArmOverload();
    if (armCtrlMode == 0) return;
    kinematics::Point target = {arm_x, 0, arm_z};
    if (armPosMode == 1 &&
        !kinematics::compensateArm(bodyPose(), bodyConfig.z_init, target, target)) {
        return;
    }
    kinematics::ArmAngles angles;
    if (!kinematics::solveArm(s_armGeometry, target, angles)) {
        return;
    }
    motor[13].des_angle = angles.elbow;
    motor[14].des_angle = angles.shoulder;
}

void WholeIK()
{
    const kinematics::Geometry geometry(bodyConfig);
    kinematics::Point bases[4], targets[4];
    kinematics::LegAngles angles[4];
    if (!kinematics::solveBody(geometry, bodyPose(), sign, bases)) {
        return;
    }
    // A rejected foot must not combine old angles on one leg with new angles on
    // the other three. Stage the complete result, then commit after validation.
    // This is solve-level consistency, not cross-task atomic state publication.
    for (uint8_t i = 0; i < 4; ++i) {
        targets[i] = footTarget(leg[i], bases[i]);
        if (!kinematics::solveLeg(geometry, targets[i], leg[i].sign, leg[i].z_bias, angles[i])) {
            return;
        }
    }
    for (uint8_t i = 0; i < 4; ++i) {
        commitBase(leg[i], bases[i]);
        commitLeg(leg[i], targets[i], angles[i]);
    }
}
