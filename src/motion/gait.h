#pragma once

#include "robot/types.h"
#include "robot/state.h"

#define WALK_SR (WALK_SWING_S / WALK_PERIOD)
#define YAW_STRIDE_SCALE 0.721f
extern uint8_t gait_reinit;

// legYaw is the turn stride input; wheel differential steering is separate.
void GaitUpdateMotion(bool moving, float legYaw);
bool GaitIsActive();
bool GaitSetType(uint8_t type);
void GaitResetWalk();
