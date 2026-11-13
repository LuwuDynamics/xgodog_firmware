#include "sensor/drivers.h"
#include "robot/state.h"
#include "sensor/attitude.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace { ImuEstimator estimator(ImuEstimator::Model::Icm20948); }

static void selectBank(uint8_t bank)
{
    I2C_WriteByte(ICM20948_I2C_ADDRESS, ICM20948_REG_BANK_SEL, bank);
}

uint8_t getChipID_ICM20948(void)
{
    selectBank(ICM20948_VAL_BANK_0);
    uint8_t r[2] = {0};
    I2C_slave_read(ICM20948_I2C_ADDRESS, ICM20948_WHO_AM_I, r, 1);
    return r[0];
}

void imu_init_icm20948(void)
{
    estimator.reset();
    selectBank(ICM20948_VAL_BANK_0);
    I2C_WriteByte(ICM20948_I2C_ADDRESS, ICM20948_PWR_MGMT_1, 0x80); /* reset */
    vTaskDelay(pdMS_TO_TICKS(10));
    I2C_WriteByte(ICM20948_I2C_ADDRESS, ICM20948_PWR_MGMT_1, 0x01); /* run, clock PLL */
    vTaskDelay(pdMS_TO_TICKS(10));

    /* Bank 2: gyro/accel config */
    selectBank(ICM20948_VAL_BANK_2);
    I2C_WriteByte(ICM20948_I2C_ADDRESS, ICM20948_GYRO_SMPLRT_DIV, 0x07);
    /* 2000dps, DLPF on */
    I2C_WriteByte(ICM20948_I2C_ADDRESS, ICM20948_GYRO_CONFIG_1,
                  0x30 | ICM20948_GYRO_FS_2000DPS | ICM20948_GYRO_DLPF_EN);
    I2C_WriteByte(ICM20948_I2C_ADDRESS, ICM20948_ACCEL_SMPLRT_DIV_2, 0x07);
    /* 2g, DLPF on */
    I2C_WriteByte(ICM20948_I2C_ADDRESS, ICM20948_ACCEL_CONFIG,
                  0x30 | ICM20948_ACCEL_FS_2G | ICM20948_ACCEL_DLPF_EN);

    selectBank(ICM20948_VAL_BANK_0);
    vTaskDelay(pdMS_TO_TICKS(50));
}

void imu_read_icm20948(void)
{
    uint8_t data[12] = {0};

    selectBank(ICM20948_VAL_BANK_0);
    if (!I2C_ReadIMU_Fast(ICM20948_I2C_ADDRESS, ICM20948_ACCEL_XOUT_H, data, 12)) {
        return;
    }

    /* ICM20948: ACCEL_XOUT_H,L, ACCEL_YOUT_H,L, ACCEL_ZOUT_H,L, GYRO_XOUT_H,L, ... (big-endian) */
    short accx = (short)(data[0] << 8 | data[1]);
    short accy = (short)(data[2] << 8 | data[3]);
    short accz = (short)(data[4] << 8 | data[5]);
    short gyrox = (short)(data[6] << 8 | data[7]);
    short gyroy = (short)(data[8] << 8 | data[9]);
    short gyroz = (short)(data[10] << 8 | data[11]);

    /* Accel 2g: 16384 LSB/g -> m/s^2 = raw/16384*9.8 */
    /* Gyro 2000dps: 16.4 LSB/(°/s) -> dps = raw/16.4 */

    imu.acc[0] = (float)accy / 16384.0f * 9.8f;
    imu.acc[1] = -(float)accx / 16384.0f * 9.8f;
    imu.acc[2] = -(float)accz / 16384.0f * 9.8f;



    imu.gyro[0] = -(float)gyroy / 16.4f;
    imu.gyro[1] = (float)gyrox / 16.4f;
    imu.gyro[2] = (float)gyroz / 16.4f;
    const uint32_t nowMs = (uint32_t)millis();
    if (estimator.update(imu, isIMUInit, (uint32_t)micros(), nowMs))
        imu_health::publish(nowMs);
}
