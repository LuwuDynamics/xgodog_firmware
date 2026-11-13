#pragma once

enum ImuType { ICM42670 = 0, ICM20948 = 1, QMI8658C = 2 };

void imu_init();
void imu_read();
void taskIMU(void *parameter);
