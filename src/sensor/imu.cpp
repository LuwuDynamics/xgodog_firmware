#include "sensor/imu.h"
#include "sensor/attitude.h"
#include "robot/state.h"
#include "sensor/drivers.h"
#include "board/i2c.h"

void imu_init(){
    imu_health::reset();
    I2C_Init();
    imu_type = 0xFF; /* unknown */
    for (int i = 0; i < 50; i++) {
        if (getChipID() == 0x67) {
            imu_type = ICM42670;
            break;
        }
        if (getChipID_ICM20948() == ICM20948_WHO_AM_I_VALUE) {
            imu_type = ICM20948;
            break;
        }

        if (getChipID_QMI8658C() == QMI8658C_WHO_AM_I_VALUE) {
            imu_type = QMI8658C;
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }

    if (imu_type == ICM42670) {
        imu_init_icm();
    } else if (imu_type == ICM20948) {
        imu_init_icm20948();
    } else if (imu_type == QMI8658C) {
        imu_init_qmi8658c();
    }else{
        imu_init_fdi();
    }

}

void imu_read(){
    if (imu_type == ICM42670) {
        imu_read_icm();
    } else if (imu_type == ICM20948) {
        imu_read_icm20948();
    } else if (imu_type == QMI8658C) {
        imu_read_qmi8658c();
    }else{
        imu_read_fdi();
    }
}


void taskIMU(void *parameter){
    (void)parameter;
    imu_init();
    const TickType_t period = pdMS_TO_TICKS(3);
    static_assert(pdMS_TO_TICKS(3) > 0, "IMU sampling requires a millisecond RTOS tick");
    TickType_t lastWake = xTaskGetTickCount();
    for(;;){
        imu_read();
        const TickType_t now = xTaskGetTickCount();
        // Do not issue a burst of stale catch-up samples after a driver stall.
        if ((TickType_t)(now - lastWake) >= period) lastWake = now;
        vTaskDelayUntil(&lastWake, period);
    }
}
