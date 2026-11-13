#include "sensor/drivers.h"
#include "robot/state.h"
#include "sensor/attitude.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace { ImuEstimator estimator(ImuEstimator::Model::Qmi8658c); }

static uint8_t qmi_i2c_addr = 0;

static uint8_t QMI8658C_ReadWhoAmI(uint8_t addr)
{
    uint8_t id = 0;
    if (!I2C_ReadIMU_Fast(addr, QMI8658C_REG_WHO_AM_I, &id, 1)) {
        return 0;
    }
    return id;
}

uint8_t getChipID_QMI8658C(void)
{
    static const uint8_t addrs[] = { QMI8658C_I2C_ADDR_6A, QMI8658C_I2C_ADDR_6B };
    for (unsigned i = 0; i < sizeof(addrs); i++) {
        if (QMI8658C_ReadWhoAmI(addrs[i]) == QMI8658C_WHO_AM_I_VALUE) {
            qmi_i2c_addr = addrs[i];
            return QMI8658C_WHO_AM_I_VALUE;
        }
    }
    qmi_i2c_addr = 0;
    return 0;
}

void imu_init_qmi8658c(void)
{
    estimator.reset();
    if (qmi_i2c_addr == 0) {
        return;
    }
    vTaskDelay(pdMS_TO_TICKS(10));
    I2C_WriteByte(qmi_i2c_addr, QMI8658C_REG_CTRL1, 0x40); /* 接口与传感器使能 */
    I2C_WriteByte(qmi_i2c_addr, QMI8658C_REG_CTRL7, 0x03); /* 使能陀螺仪和加速度计 */
    /* ±2g; ODR code 4 is nominal 470 Hz with accel+gyro enabled. */
    I2C_WriteByte(qmi_i2c_addr, QMI8658C_REG_CTRL2, 0x04);
    /* gFS=110: ±1024 dps; gODR=0100: nominal 470 Hz (QST Rev 0.9 pp 18, 31). */
    I2C_WriteByte(qmi_i2c_addr, QMI8658C_REG_CTRL3, 0x64);
    /* 低通滤波等数据处理设置 */
    I2C_WriteByte(qmi_i2c_addr, QMI8658C_REG_CTRL5, 0x11);

    vTaskDelay(pdMS_TO_TICKS(50));
}

void imu_read_qmi8658c(void)
{
    uint8_t data[12] = {0};

    if (qmi_i2c_addr == 0) {
        return;
    }
    /* 从 AccX_L 开始连续读取 6 轴数据 */
    if (!I2C_ReadIMU_Fast(qmi_i2c_addr, QMI8658C_REG_ACC_X_L, data, 12)) {
        return;
    }

    short accx = (short)(data[1] << 8 | data[0]);
    short accy = (short)(data[3] << 8 | data[2]);
    short accz = (short)(data[5] << 8 | data[4]);
    short gyrox = (short)(data[7] << 8 | data[6]);
    short gyroy = (short)(data[9] << 8 | data[8]);
    short gyroz = (short)(data[11] << 8 | data[10]);

    /* CTRL2 full-scale 000: ±2g, 16384 LSB/g. */
    imu.acc[0] = (float)accx / 16384.0f * 9.8f;
    imu.acc[1] = (float)accy / 16384.0f * 9.8f;
    imu.acc[2] = (float)accz / 16384.0f * 9.8f;

    /* CTRL3=0x64 selects ±1024 dps: 32 LSB/(degree/s), not ±2048. */
    imu.gyro[0] = (float)gyrox / 32.0f;
    imu.gyro[1] = (float)gyroy / 32.0f;
    imu.gyro[2] = (float)gyroz / 32.0f;

    const uint32_t nowMs = (uint32_t)millis();
    if (estimator.update(imu, isIMUInit, (uint32_t)micros(), nowMs))
        imu_health::publish(nowMs);
}
