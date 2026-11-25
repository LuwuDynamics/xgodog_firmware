#pragma once

#include "robot/types.h"

// Pure kinematics: lengths retain the deployed millimetre convention and all
// public angles are degrees. No global state, motor output, clock or RTOS calls.
namespace kinematics {
struct Point { float x, y, z; };
struct LegAngles { float knee, hip, lateral; };
struct ArmAngles { float elbow, shoulder; };
// x/y: yaw-aligned horizontal displacement; z: height above the fixed ground.
// Rotation from body to ground: Rz(yaw) Ry(pitch) Rx(roll).
struct BodyPose { float x, y, z, roll, pitch, yaw; };

inline float neutralFootHalfWidth(const BodyParam_t &profile) {
    return profile.width + profile.L_d - profile.stance_y_inset;
}

// Construct once per solve, sharing invariant terms across all four legs.
struct Geometry {
    const float upper, lower, lateral, halfWidth, halfLength, footHalfWidth;
    const float upperSquared, lowerSquared, lateralSquared, twiceUpper, kneeDenominator;
    const bool valid;
    explicit Geometry(const BodyParam_t &profile);
};

struct ArmGeometry {
    const float upperSquared, lowerSquared, twiceUpper, elbowDenominator;
    const bool valid;
    ArmGeometry(float upperLength, float lowerLength);
};

// Output arguments remain unchanged on any invalid input or unreachable target.
// A bounded acos tolerance (1e-4) admits sub-millimetre boundary roundoff;
// farther unreachable points are rejected instead of projected onto a trajectory.
// Leg domain: side is +/-1 and foot.z > 0 (below the hip).
bool solveLeg(const Geometry &geometry, const Point &foot, float side, LegAngles &angles);
bool solveBody(const Geometry &geometry, const BodyPose &pose, const Sign &sides,
               Point (&bases)[4]);
bool compensateArm(const BodyPose &pose, float standingHeight, const Point &target,
                   Point &compensated);
// Arm domain: target.y == 0; turntable rotation is a separate actuator.
// These pure geometric solvers do not impose servo joint limits.
bool solveArm(const ArmGeometry &geometry, const Point &target, ArmAngles &angles);
} // namespace kinematics



void Init_Kinematic(void);
// Invalid solves retain the previous output. WholeIK stages all four legs and
// commits only if every target is reachable; it does not lock shared task state.
void LegIK(Leg_t *leg);
void WholeIK(void);
void ArmIK();
