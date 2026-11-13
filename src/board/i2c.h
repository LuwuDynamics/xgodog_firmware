#pragma once
#include "robot/types.h"
void I2C_Init();
void Switch_I2C_To_Serial();
void I2C_WriteByte(uint8_t slaveAddr, uint8_t regAddr, uint8_t data);
bool I2C_slave_read(uint8_t slave_addr, uint8_t data, uint8_t *buf, uint32_t len);
bool I2C_ReadIMU_Fast(uint8_t slave_addr, uint8_t reg_addr, uint8_t *buf, uint32_t len);
