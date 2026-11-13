#include <Arduino.h>
#include "robot/state.h"
#include "motion/gait.h"
#include "motion/control.h"

/* 版本 */

char firmwareVersion[11] = XGO_FIRMWARE_VERSION;

/* Debug（协议 main 128） */
float debug_var[10] = {0};

uint8_t motor_version=MOTOR_UNKNOWN;
uint8_t hasWheel = 0;

/* 状态 */
uint8_t xgo_mode = 0;
uint8_t isReceiveMsg = true;
uint8_t isIMUErr = 0;
uint8_t isMotorErr = 0;
long motorRevTime = 0;
uint8_t inCalibration = 0;
Sign sign = {{-1,1,1,-1},
			 {1,1,-1,-1}};


Odom odom;

uint8_t feedback_flag = 0;
uint8_t MoveMode = MOVE_MODE_HYBRID;
/*运动*/
uint8_t yaw_track_flag = 0; // yaw指定跟踪标志位
PID_t move_yaw_pid = {.fpKp = 3.0f, .fpUMax = 255.0f, .fpEiMax = 8.0f, .fpEMin = 1.0f};

float sr = 0.48; // swing phase ratio 摆动相占比

GaitManager_t gm = {
    .x_step = {0, 0, 0, 0},
    .y_step = {0, 0, 0, 0},
    .z_step = STEP_HEIGHT,
    .T = SEC_2_TICK(NORMAL_PERIOD),
    .gait_type = GAIT_TROT,
};


/* 动作 */
ActionContext_t Action_Ctx = {0, 0, 0, false};


/* 机械臂 */
uint8_t armCtrlMode = 1;
uint8_t armPosMode = 0;
float arm_x = L2;
float arm_z = L1;
float arm_theta = 135.0/180.0*PI;
float arm_r = 100.0;


/* 机器人构型 */
uint8_t xgo_version = XGOMINI;

Body_t body;
Leg_t leg[4];
BodyParam_t bodyConfig = {
    .width = 45.0f/2,
    .length = 150.0f/2,
    .L_b = 60.0f,
    .L_c = 75.0f,
    .L_d = 31.6f,
    .z_init = 95.0f,
    .NORMAL_T = SEC_2_TICK(NORMAL_PERIOD),
    .MID_T = SEC_2_TICK(MID_PERIOD),
    .SLOW_T = SEC_2_TICK(SLOW_PERIOD),
    .FAST_T = SEC_2_TICK(FAST_PERIOD),
};


/* 电机 */
Motor_t motor[MOTOR_NUM];
Motor_t ex_motor[10];

/* 轮子 */
Wheel_t wheel[4];

/* IMU */
imu_t imu;

uint8_t imu_type = 0;
volatile uint8_t isIMUInit = 0;
volatile bool motorInitializationComplete = false;

/*外设*/
uint8_t user_rgb[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

/* BLE */
std::string BLEName("XGOMINI_");

void Reset_Body(){
    GaitResetWalk();
    xgo_mode = XGO_MODE_BODY;
    MoveMode = MOVE_MODE_HYBRID;
    body.x = 0;
    body.y = 0;
    body.z = bodyConfig.z_init;
    body.rol = 0;
    body.pit = 0;
    body.yaw = 0;
    body.vx = 0;
    body.vy = 0;
    body.vyaw = 0;
    imu.yaw_speed = 0;
    body.x_T = 0;
    body.y_T = 0;
    body.z_T = 0;
    body.rol_T = 0;
    body.yaw_T = 0;
    body.pit_T = 0;
    gm.T = bodyConfig.NORMAL_T;
    gm.gait_type = GAIT_TROT;
    gait_reinit = 1;
    MotionControl_RequestReset();
}


void Init_Config(){
    const RobotProfile profile = RobotProfile_Create(xgo_version, hasWheel);
    odom.x = 0;
    odom.y = 0;
    bodyConfig = profile.body;
    gm.z_step = profile.stepHeight;
    memcpy(firmwareVersion, profile.firmwareVersion, sizeof(firmwareVersion));
}

void Init_Globals(){
    Reset_Body();

    imu.yaw_origin = 0;
    imu.yaw_speed_min = -0.75;
    imu.yaw_speed_max = 0.75;
}
