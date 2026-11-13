#pragma once
#include "robot/types.h"
/* 标定键防抖，稳定后触发 Start/End 标定 */
void Calibration_Update(uint8_t calibrate_btn_state);
void StartCalibration(void);
void EndCalibration(void);

#define FLASH_ADDR_ZEROPOS  0x330000
#define FLASH_ADDR_NAME     0x331000
#define FLASH_NAME_MAX_LEN  30

void FlashZeroPosWrite(uint16_t *data, uint8_t motor_num);
void FlashZeroPosRead(uint16_t *data, uint8_t motor_num);
