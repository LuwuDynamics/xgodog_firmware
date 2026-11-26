#pragma once
#include "robot/types.h"

// Shared Motion Tick units and deployed gait tuning.
#define SYSTEM_TICK_HZ 200
#define YAW_RATE_LIMIT_DEG_S 120.0f
#define SEC_2_TICK(sec) ((uint32_t)((sec) * SYSTEM_TICK_HZ))
#define Period_T_MIN 1000
#define Period_T_MAX 4000

#define NORMAL_PERIOD 0.42f
#define FAST_PERIOD 0.37f
#define SLOW_PERIOD 0.52f
#define MID_PERIOD 0.45f

#define NORMAL_SPEED 0
#define SLOW_SPEED 1
#define FAST_SPEED 2
#define MID_SPEED 3

#define STEP_HEIGHT 22.0f
#define STEP_HEIGHT_MINI 20.0f
#define STEP_HEIGHT_WHEEL 18.0f
#define GAIT_TROT 0
#define GAIT_WALK 1
#define GAIT_HIGH_WALK 2
#define GAIT_LOW_TROT 3
#define GAIT_MARKTIME 255
#define WALK_LIFT 32.0f
#define WALK_SWING_S 0.20f
#define WALK_TRANSFER_S 0.15f
#define WALK_PERIOD (4.0f * (WALK_SWING_S + WALK_TRANSFER_S))
#define LOW_TROT_PERIOD 0.50f
#define LOW_TROT_LIFT_FRONT 20.0f
#define LOW_TROT_LIFT_REAR 15.0f
#define LOW_TROT_SWING 0.5f
#define LOW_TROT_STRIDE_X 18.7f
#define LOW_TROT_STRIDE_X_LITE 15.4f
