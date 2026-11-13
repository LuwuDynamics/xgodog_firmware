#include "sensor/drivers.h"
#include "robot/state.h"
#include "sensor/attitude.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace { ImuEstimator estimator(ImuEstimator::Model::Icm42670); }

void imu_init_icm(){
    estimator.reset();
    I2C_WriteByte(ICM42670P_I2C_ADDRESS,ICM42670_PWR_MGMT0, 0x9F);
    vTaskDelay(pdMS_TO_TICKS(50));
    I2C_WriteByte(ICM42670P_I2C_ADDRESS,0x20, 0x08);  // 2000dps
    I2C_WriteByte(ICM42670P_I2C_ADDRESS,0x21, 0x68);
    I2C_WriteByte(ICM42670P_I2C_ADDRESS,0x23, 0x01);
    I2C_WriteByte(ICM42670P_I2C_ADDRESS,0x24, 0x01);  //0X42
}

void imu_read_icm(){

    uint8_t data[12]={0};
    if(!I2C_ReadIMU_Fast(ICM42670P_I2C_ADDRESS,0x0B, data, 12)){
        return;
    }
    short accx,accy,accz,gyrox,gyroy,gyroz;
    accx = data[0]<<8 | data[1];
    accy = data[2]<<8 | data[3];
    accz = data[4]<<8 | data[5];
    gyrox = data[6]<<8 | data[7];
    gyroy = data[8]<<8 | data[9];
    gyroz = data[10]<<8 | data[11];
    imu.acc[0] = (float)accy/32768.0f*9.8*2.0f;
    imu.acc[1] = (float)accx/32768.0f*9.8*2.0f;
    imu.acc[2] = (float)accz/32768.0f*9.8*2.0f;
    imu.gyro[0] = -(float)gyroy/32768.0f*2000.0f;
    imu.gyro[1] = -(float)gyrox/32768.0f*2000.0f;
    imu.gyro[2] = -(float)gyroz/32768.0f*2000.0f;

    const uint32_t nowMs = (uint32_t)millis();
    if (estimator.update(imu, isIMUInit, (uint32_t)micros(), nowMs))
        imu_health::publish(nowMs);
}

uint8_t getChipID(void)
{
    uint8_t data[2] = {0};
    I2C_slave_read(ICM42670P_I2C_ADDRESS, ICM42670_WHO_AM_I, data, 1);
    return data[0];
}
