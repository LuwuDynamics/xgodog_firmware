#include "board/calibration.h"
#include "robot/state.h"
#include "motor/motor.h"
#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static uint8_t cal_btn_count = 0;

void StartCalibration(void) {
    // Open the gripper before releasing torque for zero calibration.
    motor[12].des_angle = -120;
    motor[12].min_angle = -120;
    vTaskDelay(pdMS_TO_TICKS(1500));
    EnableAllMotor(0);
    vTaskDelay(pdMS_TO_TICKS(100));

    inCalibration = 1;
}

void EndCalibration(void) {
    uint16_t zero_pos[MOTOR_NUM];
    for(int i=0;i<MOTOR_NUM;i++){
        zero_pos[i] = motor[i].fb_pos;
        if(i==12){
            zero_pos[i] = zero_pos[i] + motor[i].max_angle*g_motorConfig.M_N/g_motorConfig.M_A;
        }else if(i==13){
            zero_pos[i] = zero_pos[i] + motor[i].max_angle*g_motorConfig.M_N/g_motorConfig.M_A;
        }else if(i==14){
            zero_pos[i] = zero_pos[i] - motor[i].min_angle*g_motorConfig.M_N/g_motorConfig.M_A;
        }
        motor[i].zero_pos = zero_pos[i];
    }
    FlashZeroPosWrite(zero_pos, MOTOR_NUM);
    vTaskDelay(pdMS_TO_TICKS(200));  // 确保 flash 写完
    ESP.restart();
}

static uint8_t cal_btn_last_state = 0;

/* 状态变化计数防抖，超过阈值后根据当前电平决定进入/退出标定 */
void Calibration_Update(uint8_t calibrate_btn_state) {
    if(millis()<3000){
        cal_btn_last_state = calibrate_btn_state;
    }

    if (calibrate_btn_state != cal_btn_last_state) {
        cal_btn_count++;
    } else {
        cal_btn_last_state = calibrate_btn_state;
    }

    if (cal_btn_count > 50) {
        if (calibrate_btn_state == 1 && inCalibration == 1) {
            EndCalibration();
        } else if (calibrate_btn_state == 0 && inCalibration == 0) {
            StartCalibration();
        }
        cal_btn_count = 0;
        cal_btn_last_state = calibrate_btn_state;
    }
}

#define FLASH_SECTOR_SIZE 4096

void FlashZeroPosRead(uint16_t *data, uint8_t motor_num) {
    uint32_t storedPositions[motor_num];
    ESP.flashRead(FLASH_ADDR_ZEROPOS, storedPositions, 4u * motor_num);
    for (int i = 0; i < motor_num; i++)
    {
        data[i] = (uint16_t)storedPositions[i];
    }
}

void FlashZeroPosWrite(uint16_t *data, uint8_t motor_num) {
    uint32_t storedPositions[motor_num];
    for (int i = 0; i < motor_num; i++)
    {
        storedPositions[i] = (uint32_t)data[i];
    }
    ESP.flashEraseSector(FLASH_ADDR_ZEROPOS / FLASH_SECTOR_SIZE);
    ESP.flashWrite(FLASH_ADDR_ZEROPOS, storedPositions, 4*motor_num);
}
