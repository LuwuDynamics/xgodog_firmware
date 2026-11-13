#pragma once
#include <stdint.h>
/* 引脚定义：电源/校准/LED/开关/电池/扩展IO */
#define volt_5_pin 12
#define calibrate_pin 21
#define LED_RED 22
#define LED_BLUE 23
#define sw_led_pin 25

#define sw_ctrl_pin 32
#define battery_adc 33
#define sw_state_pin 34

#define ana_input_pin 36
#define digital_input_pin 39

#define ana_output_pin 26
#define digital_output_pin 27

/* 协议与任务共享：模拟/数字输出控制量，数字/模拟输入采样值 */
extern uint8_t swLED;
extern uint8_t swCtrl;

extern uint8_t anaCtrl;
extern uint8_t digCtrl;
extern uint8_t digital_in_state;
extern uint8_t analog_in_state;

void Init_GPIO(void);       /* 仅做引脚模式与初始电平 */
void GPIO_Task_Start(void); /* 创建周期任务：读入→标定/开关/电量更新→刷新输出 */

/* 开关状态灯、电源控制、长按阶段计数（供协议/低电逻辑使用） */

void Switch_Off(void);

extern float battery;      /* 电量百分比 0~100 */
extern uint16_t isBattLow; /* 0=正常，非零为低电处理阶段 */
