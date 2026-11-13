#pragma once

#include <stdint.h>

enum RobotMotionAxis : uint8_t {
    ROBOT_MOTION_VX = 1u << 0,
    ROBOT_MOTION_VY = 1u << 1,
    ROBOT_MOTION_YAW = 1u << 2,
    ROBOT_MOTION_ALL = ROBOT_MOTION_VX | ROBOT_MOTION_VY | ROBOT_MOTION_YAW,
};

bool RobotCommand_SetBaud(uint8_t port, uint8_t baudIndex);
void RobotCommand_SetFeedback(uint8_t port, uint8_t enabled);
void RobotCommand_SetCalibration(uint8_t enabled);
void RobotCommand_SetYawOrigin(void);
void RobotCommand_SetActionLoop(uint8_t enabled);
void RobotCommand_ApplyMotion(uint8_t axisMask);
bool RobotCommand_SetGaitPeriod(uint8_t periodIndex);
bool RobotCommand_SetGaitType(uint8_t type);
bool RobotCommand_SetImuStable(uint8_t mode);
void RobotCommand_ApplyMoveMode(uint8_t mode);
void RobotCommand_ApplyArmPolar(void);
// Legacy motor-group command entry point retained for both Host Protocols.
void parseEnableMotor(uint8_t mode);
