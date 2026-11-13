#include "sensor/drivers.h"
#include "sensor/attitude.h"
#include "robot/math.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "board/i2c.h"
#include <cmath>
static int yawTurns = 0;
static float previousYaw = 0.0;

static int parseState = 0;
static int frameByteCount = 0;
static uint8_t frameData[53];

void imu_init_fdi(){
    Switch_I2C_To_Serial();
    vTaskDelay(pdMS_TO_TICKS(100));
}

void imu_read_fdi(){
    // Bound sensor UART work so an uninterrupted stream cannot starve tasks.
    uint16_t remaining = 512;
    while(remaining-- != 0 && Serial.available()){
        const uint8_t receivedByte = (uint8_t)Serial.read();
        switch(parseState){
            case 0:
                if (receivedByte== 0xFC)
                    parseState = 1;
                break;
            case 1:
                if(receivedByte== 0x41)
                    parseState = 2;
                else
                    parseState = 0;
                break;
            case 2:
                if(receivedByte== 48){
                    parseState = 3;
                    frameByteCount = 0;
                }
                else
                    parseState = 0;
                break;
            case 3:
                frameData[frameByteCount] = receivedByte;
                frameByteCount++;
                if(frameByteCount==53){
                    if(receivedByte== 0xFD){
                        parseState = 0;
                        const float roll = -Byte2Float(frameData+4+16) / PI * 180.0;
                        const float pitch = Byte2Float(frameData+4+12) / PI * 180.0;
                        const float yaw = -Byte2Float(frameData+4+20) / PI * 180.0;
                        // A malformed float must not poison feedback or yaw unwrap
                        // history; the next complete frame remains recoverable.
                        if (!std::isfinite(roll) || !std::isfinite(pitch) || !std::isfinite(yaw)) break;
                        imu.rol = roll;
                        // Keep the mounted sensor's half-turn offset, including
                        // signed zero: raw +/-0 represents body pitch +/-180 deg.
                        imu.pit = std::signbit(pitch) ? -180.0 - pitch : 180.0 - pitch;

                        if((yaw-previousYaw)>300){
                            yawTurns = yawTurns - 1;
                        }
                        if((yaw-previousYaw)<-300){
                            yawTurns = yawTurns + 1;
                        }
                        previousYaw = yaw;
                        imu.yaw = yawTurns * 360.0 + yaw;
                        if (isIMUInit==0)
                        {
                            isIMUInit = 1;
                        }
                        imu_health::publish((uint32_t)millis());
                    }else{
                        parseState = 0;
                    }
                }
                break;
            default:
                parseState = 0;
                break;
        }
    }
}
