#pragma once
#include <cmath>
#include "robot/types.h"

void CalISeparatedPID(PID_t *pid, float elapsedTicks);

uint16_t Saturation(uint16_t data,uint16_t upper_limit,uint16_t lower_limit);
float Saturation(float data,float upper_limit,float lower_limit);
void Float2Byte(uint8_t* Byte,float data);
float Byte2Float(uint8_t* data);
void Int2Byte(uint8_t* Byte,int data);
float u8_to_float(uint8_t data,float data_min,float data_max);
uint8_t float_to_u8(float data,float data_min,float data_max);
