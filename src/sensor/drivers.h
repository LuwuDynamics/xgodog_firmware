#pragma once
#include "Arduino.h"
#include "math.h"
#include "board/i2c.h"
#include "robot/types.h"
#include "robot/state.h"

/* ICM20948 I2C address (AD0=L: 0x68, AD0=H: 0x69) */
#define ICM20948_I2C_ADDRESS      0x68

/* Bank 0 - User bank 0 */
#define ICM20948_REG_BANK_SEL     0x7F
#define ICM20948_VAL_BANK_0       0x00
#define ICM20948_VAL_BANK_2       0x20

#define ICM20948_WHO_AM_I         0x00   /* WIA, value = 0xEA */
#define ICM20948_PWR_MGMT_1       0x06
#define ICM20948_ACCEL_XOUT_H     0x2D   /* Accel + Gyro block: 0x2D~0x38, 12 bytes */

/* Bank 2 - Configuration */
#define ICM20948_GYRO_SMPLRT_DIV  0x00
#define ICM20948_GYRO_CONFIG_1    0x01
#define ICM20948_ACCEL_SMPLRT_DIV_2 0x11
#define ICM20948_ACCEL_CONFIG     0x14

/* Gyro full scale: 250/500/1000/2000 dps */
#define ICM20948_GYRO_FS_2000DPS  0x06
#define ICM20948_GYRO_DLPF_EN     0x01
/* Accel full scale: 2/4/8/16 g */
#define ICM20948_ACCEL_FS_2G      0x00
#define ICM20948_ACCEL_DLPF_EN    0x01

#define ICM20948_WHO_AM_I_VALUE   0xEA

uint8_t getChipID_ICM20948(void);
void imu_init_icm20948(void);
void imu_read_icm20948(void);

#define ICM42670P_I2C_ADDRESS 0x69

// Bank 0
#define ICM42670_PWR_MGMT0                 0x1F
#define ICM42670_WHO_AM_I                  0x75

uint8_t getChipID(void);

void imu_init_icm();
void imu_read_icm();

/* SA0 high/float -> 0x6A, SA0 low -> 0x6B */
#define QMI8658C_I2C_ADDR_6A     0x6A
#define QMI8658C_I2C_ADDR_6B     0x6B

/* Configuration Registers */
#define QMI8658C_REG_WHO_AM_I    0x00
#define QMI8658C_REG_CTRL1       0x02
#define QMI8658C_REG_CTRL2       0x03
#define QMI8658C_REG_CTRL3       0x04
#define QMI8658C_REG_CTRL5       0x06
#define QMI8658C_REG_CTRL7       0x08

/* Sensor Data Output Registers */
#define QMI8658C_REG_ACC_X_L     0x35

/* WHO_AM_I expected value (per QMI8658C datasheet) */
#define QMI8658C_WHO_AM_I_VALUE  0x05

uint8_t getChipID_QMI8658C(void);
void imu_init_qmi8658c(void);
void imu_read_qmi8658c(void);

void imu_init_fdi();
void imu_read_fdi();
