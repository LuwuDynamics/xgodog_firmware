#include "motion/commands.h"
#include "robot/math.h"
#include "action/action.h"
#include "motion/gait.h"
#include "motion/control.h"
#include "board/calibration.h"
#include "robot/state.h"
#include "motor/motor.h"
#include "protocol/protocol.h"
#include <math.h>

bool RobotCommand_SetBaud(uint8_t port, uint8_t baudIndex)
{
    static const uint32_t kBaudRates[] = {
        115200, 500000, 921600, 1000000, 1500000, 2000000
    };
    if (baudIndex >= sizeof(kBaudRates) / sizeof(kBaudRates[0]))
        return false;
    ProtocolTransport_SetBaud(port, kBaudRates[baudIndex]);
    return true;
}

void RobotCommand_SetFeedback(uint8_t port, uint8_t enabled)
{
    ProtocolTransport_SelectFeedbackPort(port);
    feedback_flag = enabled;
}

void RobotCommand_SetCalibration(uint8_t enabled)
{
    if (!inCalibration && enabled)
        StartCalibration();
    else if (inCalibration && !enabled)
        EndCalibration();
}

void RobotCommand_SetYawOrigin(void)
{
    imu.yaw_origin = imu.yaw;
}

void RobotCommand_SetActionLoop(uint8_t enabled)
{
    Action_Ctx.loop = enabled != 0;
    if (Action_Ctx.loop)
        Action_Ctx.loop_id = 0;
    else
        Reset(2);
}

void RobotCommand_ApplyMotion(uint8_t axisMask)
{
    // The control task restores movement speed; a zero command must not
    // cancel the speed limit used for returning from an action.
    imu.stable = 0;
    xgo_mode = XGO_MODE_BODY;
    if ((axisMask & ROBOT_MOTION_VX) != 0 && fabsf(body.vx) < 0.1f * bodyConfig.vx_max)
        body.vx = 0;
    if ((axisMask & ROBOT_MOTION_VY) != 0 && fabsf(body.vy) < 0.1f * bodyConfig.vy_max)
        body.vy = 0;
    if ((axisMask & ROBOT_MOTION_YAW) != 0 && fabsf(imu.yaw_speed) < 0.1f)
        imu.yaw_speed = 0;
    MotionControl_NotifyMotionCommand();
}

bool RobotCommand_SetGaitPeriod(uint8_t periodIndex)
{
    if (periodIndex == 0)
        gm.T = bodyConfig.NORMAL_T;
    else if (periodIndex == 1)
        gm.T = bodyConfig.SLOW_T;
    else if (periodIndex == 2)
        gm.T = bodyConfig.FAST_T;
    else
        return false;
    xgo_mode = XGO_MODE_BODY;
    return true;
}

bool RobotCommand_SetGaitType(uint8_t type)
{
    if (!GaitSetType(type))
        return false;
    xgo_mode = XGO_MODE_BODY;
    return true;
}

bool RobotCommand_SetImuStable(uint8_t mode)
{
    if (mode > 2)
        return false;
    imu.stable = mode;
    if (mode == 0) {
        body.pit = 0;
        body.rol = 0;
        body.yaw = 0;
    }
    return true;
}

void RobotCommand_ApplyMoveMode(uint8_t mode)
{
    MoveMode = mode;
    body.vx = 0;
    body.vy = 0;
    body.vyaw = 0;
    for (uint8_t i = 0; i < 4; ++i) {
        wheel[i].des_vel = 0;
        wheel[i].order_vel = 0;
    }
}

void RobotCommand_ApplyArmPolar(void)
{
    armCtrlMode = 1;
    arm_x = Saturation(-arm_r * cosf(arm_theta), ARM_X_MAX, ARM_X_MIN);
    arm_z = Saturation( arm_r * sinf(arm_theta), ARM_Z_MAX, ARM_Z_MIN);
}

void parseEnableMotor(uint8_t mode){
    if(mode == 0x00){
        RequestMotorEnable(0xFE, 1);
    }else if(mode == 0x01){
        RequestMotorEnable(0xFE, 0);
    }else if((mode & 0xF0) == 0x10 || (mode & 0xF0) == 0x20){
        const uint8_t group = mode & 0x0F;
        if (group < 1 || group > 5)
            return;
        const uint8_t state = ((mode & 0xF0) == 0x20) ? 1 : 0;
        const uint8_t count = (group == 5) ? 4 : 3;
        const uint8_t firstId = group * 10 + 1;
        for (uint8_t i = 0; i < count; ++i)
            RequestMotorEnable(firstId + i, state);
    }
}
