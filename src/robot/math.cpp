#include "robot/math.h"
#include <cstring>
#include <cmath>
#include <utility>

/*******************************************************************
函数名称：CalISeparatedPID(PID_t *pid)
函数功能：积分分离式PID算法计算PID量
备    注：积分分离式PID改进算法可减小启动、停止或大幅度增减时较大偏差
          对积分项的积累，从而避免出现较大的超调及振荡现象。
********************************************************************/
void CalISeparatedPID(PID_t *pid, float elapsedTicks)
{
    if (pid == nullptr) return;
    if (!std::isfinite(elapsedTicks) || elapsedTicks <= 0 ||
        !std::isfinite(pid->fpDes) || !std::isfinite(pid->fpFB)) {
        pid->fpU = 0;
        return;
    }
    pid->fpE = pid->fpDes - pid->fpFB;
    if (fabsf(pid->fpE) <= pid->fpEMin) {
        pid->fpE = 0;
        pid->fpSumE = 0;
    }
    if (!std::isfinite(pid->fpPreE)) pid->fpPreE = pid->fpE;
    if (!std::isfinite(pid->fpSumE)) pid->fpSumE = 0;
    // Gains remain in deployed 5 ms units. D scales inversely with elapsed
    // time, I with elapsed time; the nominal period uses the same expression.
    const float derivative = (pid->fpE - pid->fpPreE) / elapsedTicks;
    const bool integrate = fabsf(pid->fpE) <= pid->fpEiMax && pid->fpKi != 0;
    const float candidate = integrate ? pid->fpSumE + pid->fpE * elapsedTicks : pid->fpSumE;
    const float integral = integrate ? pid->fpKi * candidate : 0;
    const float proposed = pid->fpKp * pid->fpE + integral + pid->fpKd * derivative;
    // Integral separation must stop accumulation, not merely hide the output.
    // Permit integration out of saturation, but never further into it.
    if (integrate && std::isfinite(candidate) &&
        !((proposed > pid->fpUMax && pid->fpE * pid->fpKi > 0) ||
          (proposed < -pid->fpUMax && pid->fpE * pid->fpKi < 0)))
        pid->fpSumE = candidate;
    pid->fpU = pid->fpKp * pid->fpE
        + (integrate ? pid->fpKi * pid->fpSumE : 0)
        + pid->fpKd * derivative;
    pid->fpPreE = pid->fpE;
    if (!std::isfinite(pid->fpU)) pid->fpU = 0;
    if (pid->fpU > pid->fpUMax) pid->fpU = pid->fpUMax;
    else if (pid->fpU < -pid->fpUMax) pid->fpU = -pid->fpUMax;
}


uint8_t float_to_u8(float data,float data_min,float data_max){
	if(data<data_min){
		return 0x00;
	}
	else if(data>data_max){
		return 0xff;
	}
	else{
		return (uint8_t)((data-data_min)/(data_max-data_min)*255);
	}
}

float u8_to_float(uint8_t data,float data_min,float data_max){
	return (float)data/(float)255.0f*(data_max-data_min)+data_min;
}


/*******************************************************************
float Saturation(float data,float upper_limit,float lower_limit)
函数功能：限制幅度
备    注：
********************************************************************/
float Saturation(float data,float upper_limit,float lower_limit){
	if(data>upper_limit)
		data = upper_limit;
	if (data<lower_limit)
		data = lower_limit;
	return data;
}

uint16_t Saturation(uint16_t data,uint16_t upper_limit,uint16_t lower_limit){
	if(upper_limit<lower_limit){
		std::swap(upper_limit, lower_limit);
	}
	if(data>upper_limit)
		data = upper_limit;
	if (data<lower_limit)
		data = lower_limit;
	return data;
}

void Float2Byte(uint8_t* Byte,float data){
    memcpy(Byte, &data, 4);
}

float Byte2Float(uint8_t* data) {
    float f;
    memcpy(&f, data, 4);
    return f;
}

void Int2Byte(uint8_t* Byte,int data){
    memcpy(Byte, &data, 4);
}
