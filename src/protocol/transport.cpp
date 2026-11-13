#include "protocol/protocol_internal.h"
#include "board/ble.h"
#include <Arduino.h>
#include <driver/uart.h>

static volatile uint8_t feedbackPort = PORT_SERIAL0;
constexpr size_t kHostUartBufferSize = 512;

static void configureHostUart(HardwareSerial &serial)
{
    serial.setRxFIFOFull(32);
    serial.setRxTimeout(1);
}

void ProtocolTransport_Init(void)
{
    // Configure before begin(): ESP32's default TX buffer is zero, which makes
    // a 254-byte feedback write wait for the 128-byte hardware FIFO to drain.
    Serial.setRxBufferSize(kHostUartBufferSize);
    Serial.setTxBufferSize(kHostUartBufferSize);
    Serial1.setRxBufferSize(kHostUartBufferSize);
    Serial1.setTxBufferSize(kHostUartBufferSize);
    Serial.begin(115200);
    Serial1.begin(115200, SERIAL_8N1, 4, 5);
    configureHostUart(Serial);
    configureHostUart(Serial1);
}

bool ProtocolTransport_CanSendFeedback(uint8_t port)
{
    if (port == PORT_BLE) return true;
    if (port != PORT_SERIAL0 && port != PORT_SERIAL1) return false;
    HardwareSerial &serial = port == PORT_SERIAL0 ? Serial : Serial1;
    const uart_port_t uart = port == PORT_SERIAL0 ? UART_NUM_0 : UART_NUM_1;
    // Read responses take precedence. Never queue a second telemetry frame
    // behind a busy wire; at 115200 baud one frame already takes 22.05 ms.
    return serial.available() == 0 && serial.availableForWrite() >= kHostUartBufferSize &&
           uart_wait_tx_done(uart, 0) == ESP_OK;
}

void ProtocolTransport_Send(uint8_t port, uint8_t *data, uint8_t length)
{
    if (data == nullptr || length == 0)
        return;

    if (port == PORT_SERIAL1)
        Serial1.write(data, length);
    else if (port == PORT_SERIAL0)
        Serial.write(data, length);
    else if (port == PORT_BLE)
        BLE_Send(data, length);
}

void ProtocolTransport_SetBaud(uint8_t port, uint32_t baudRate)
{
    if (port != PORT_SERIAL0 && port != PORT_SERIAL1) return;
    HardwareSerial &serial = port == PORT_SERIAL0 ? Serial : Serial1;
    // Complete already accepted bytes at the old rate, retaining RX/TX buffers
    // and pin ownership instead of tearing down and reinstalling the driver.
    const uart_port_t uart = port == PORT_SERIAL0 ? UART_NUM_0 : UART_NUM_1;
    uart_wait_tx_done(uart, portMAX_DELAY);
    serial.updateBaudRate(baudRate);
    configureHostUart(serial);
}

void ProtocolTransport_SelectFeedbackPort(uint8_t port)
{
    if (port < PROTOCOL_PORT_COUNT)
        feedbackPort = port;
}

uint8_t ProtocolTransport_GetFeedbackPort(void)
{
    return feedbackPort;
}
