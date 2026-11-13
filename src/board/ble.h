#pragma once

#include <stdint.h>

void BLENameRead();
void taskBLE( void * parameter );
void BLE_Send(uint8_t* dataAddr,uint8_t dataLen);
void BLENameWrite(uint8_t* data);
void InitBLE();
