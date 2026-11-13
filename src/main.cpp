#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "motion/control.h"
#include "protocol/protocol.h"
#include "motion/kinematics.h"
#include "motor/motor.h"
#include "board/board.h"
#include "board/ble.h"
#include "sensor/imu.h"
#include "robot/state.h"

void setup() {
  setCpuFrequencyMhz(240);
  ProtocolTransport_Init();
  Init_GPIO();
  Init_Kinematic();
  Init_Motor();
  GPIO_Task_Start();

  xTaskCreate(taskIMU, "imu", 4096, NULL, 0, NULL);
  Protocol_Start();
  MotorTasks_Start();
  // Wait for motor discovery. Control folds the arm once both the IMU and
  // final motor setup are ready.
  while(motor_version == MOTOR_UNKNOWN){
    vTaskDelay(pdMS_TO_TICKS(50));
  }
  vTaskDelay(pdMS_TO_TICKS(100));
  Init_Globals();
  xTaskCreate(controlTask, "control", 4096, NULL, 2, NULL);
  vTaskDelay(pdMS_TO_TICKS(500));
  xTaskCreate(taskBLE, "ble", 4096, NULL, 1, NULL);
}

void loop() {
  if(isIMUInit == 0){
    swLED = !swLED;
    digitalWrite(sw_led_pin, swLED);
    vTaskDelay(pdMS_TO_TICKS(90));
  }else{
      swLED = 1;
  }
  vTaskDelay(pdMS_TO_TICKS(10));
}
