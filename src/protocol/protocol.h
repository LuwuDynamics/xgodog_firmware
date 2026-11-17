#pragma once

#include <stdint.h>

enum ProtocolPort : uint8_t {
    PORT_SERIAL0 = 0,
    PORT_SERIAL1 = 1,
    PORT_BLE = 2,
    PROTOCOL_PORT_COUNT = 3,
};

// Configure host UARTs before starting the protocol task.
void ProtocolTransport_Init();
void Protocol_Start();
void Protocol_SendShutdownNotice();

// Each UART/BLE stream owns an independent parser; dispatch is immediate.
void Protocol_ProcessBytes(const uint8_t *data, uint16_t length, uint8_t port);
void Protocol_ResetPort(uint8_t port);
void ProtocolTransport_SetBaud(uint8_t port, uint32_t baudRate);
void ProtocolTransport_SelectFeedbackPort(uint8_t port);
