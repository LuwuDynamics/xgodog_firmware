#pragma once

#include <stdint.h>

typedef struct
{
	float fpDes;//控制变量目标值
	float fpFB;//控制变量反馈值

	float fpKp;//比例系数Kp
	float fpKi;//积分系数Ki
	float fpKd;//微分系数Kd

	float fpE;//本次偏差
	float fpPreE;//上次偏差
    float fpSumE;//总偏差
	float fpU;//本次PID运算结果

	float fpUMax;//PID运算后输出最大值
	float fpEiMax;//积分分离限制 偏差过大
	float fpEMin;//死区
}PID_t;

typedef struct{
    float stable;
    float yaw_origin;
    float yaw_des;
    float des_yaw;
    float yaw_speed;
    float yaw_speed_min;
    float yaw_speed_max;
    float acc[3];
    float gyro[3];
    float yaw;
    float pit;
    float rol;
    float gyro_bias[3];
    float acc_bias[3];
}imu_t;

typedef struct {
    uint8_t id;
    int position;
    short velocity;
    short torque;
    uint8_t order_len;
} MotorState_t;

typedef struct {
    float M_N; // 范围
    float M_A; // 角度范围
    float M_V; // 速度范围
    uint8_t  ENDIAN; // 0:大端序 1:小端序
    uint8_t  ADDR_POS; // 位置地址
    uint8_t  ADDR_POS_READ; // 位置读取地址
    uint8_t  ADDR_ENABLE; // 使能地址
} MotorConfig_t;

typedef struct {
    float width;
    float length;
    float L_b;
    float L_c;
    float L_d;
    float z_init;
    uint32_t NORMAL_T;
    uint32_t MID_T;
    uint32_t SLOW_T;
    uint32_t FAST_T;
    float xmax, ymax, zmax;
    float xmin, ymin, zmin;
    float rolmax, pitmax, yawmax;
    float rolmin, pitmin, yawmin;
    float vx_max, vy_max, vyaw_max;
    float vx_min, vy_min, vyaw_min;
    float stance_y_inset; // Inward offset of each neutral foot anchor, mm.

} BodyParam_t;

typedef struct{
	float nppn[4];
	float ppnn[4];
}Sign;

typedef struct {
    float x;
    float y;
    float z;
    float x_base;
    float y_base;
    float z_base;
    float x_step;
    float y_step;
    float z_step;
    float theta1;
    float theta2;
    float theta3;
    float sign;
} Leg_t;

typedef struct {
    float x,y,z;
    float rol,pit,yaw;
    float x_T, y_T, z_T;
    float yaw_T, pit_T, rol_T;
    float vx, vy, vyaw;
} Body_t;

typedef struct
{
	uint8_t    id;
    uint16_t   des_pos;    // 目标位置
    uint16_t   zero_pos;   // 零点位置
	uint16_t   max_pos;    // 最大位置
    uint16_t   min_pos;    // 最小位置
    uint16_t   fb_pos;     // 反馈位置
    short      fb_vel;     // 反馈速度
    short      fb_tor;     // 反馈扭矩
    uint8_t    enable;     // 使能
    uint16_t   speed;      // 速度

    float      des_angle;   // 角度
	float      fb_angle;    // 角度
	float      max_angle;   // 最大角度
	float      min_angle;   // 最小角度
    float      sign;        // 正负号
} Motor_t;

typedef struct
{
	uint8_t    id;
    uint16_t   fb_pos;     // 反馈位置
    float      des_vel;    // 目标速度
    short      order_vel;    // 发送速度
    float      fb_vel;     // 反馈速度
    long       odom;       // 里程计
    uint8_t    enable;     // 使能
    short      sign;       // 正负号
} Wheel_t;

typedef struct {
    uint8_t id;       // 当前正在执行的动作 ID
    uint32_t cur_tick;    // 当前动作已运行的系统 Tick 数
    uint32_t next_tick;    // 下一个关键帧的系统 Tick 数
    uint8_t frame;    // 当前关键帧序号
    bool loop;          // 是否循环播放
    uint8_t loop_id;    // 循环播放的ID
    uint16_t loop_tick;
} ActionContext_t;

typedef struct {
    float x_step[4];
    float y_step[4];
    float z_step;
    float T;
    uint8_t gait_type;
} GaitManager_t;

typedef struct {
    float x;
    float y;
} Odom;
