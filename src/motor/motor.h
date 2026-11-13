#pragma once

#include "robot/types.h"

extern MotorConfig_t g_motorConfig;

// Startup, periodic target encoding, and nonblocking command requests.
void MotorTasks_Start();
void Init_Motor();
void UpdateMotorDesPos();
void RequestMotorEnable(uint8_t id, uint8_t state);
// Raw single write used by the host protocols; does not change cached enable.
void Motor_WriteEnable(uint8_t id, uint8_t state, uint8_t version);
void EnableAllMotor(uint8_t state);
void SetLegMotorSpeed(uint8_t speed);
void SetArmMotorSpeed(uint8_t speed);
void MotorMoveToMid();
void UpdateMotorMoveToMid();
