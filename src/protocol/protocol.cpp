#include "protocol/protocol_internal.h"
#include "robot/state.h"
#include "board/board.h"
#include "robot/math.h"
#include <Arduino.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static bool encodeResponse(uint8_t command, uint8_t address,
                           const uint8_t *payload, uint8_t payloadLen,
                           uint8_t *frame, uint16_t frameCapacity,
                           uint8_t *frameLen)
{
    if (frame == nullptr || frameLen == nullptr)
        return false;
    if (payloadLen == 0 || payload == nullptr)
        return false;

    const uint16_t totalLen = (uint16_t)payloadLen + PROTOCOL_FRAME_OVERHEAD;
    if (totalLen > PROTOCOL_FRAME_MAX_SIZE || totalLen > frameCapacity)
        return false;

    for (uint8_t i = 0; i < payloadLen; ++i)
        frame[5 + i] = payload[i];
    return ProtocolFrame_FinalizeResponse(command, address, payloadLen,
                                          frame, frameCapacity, frameLen);
}

bool ProtocolFrame_FinalizeResponse(uint8_t command, uint8_t address,
                                    uint8_t payloadLen,
                                    uint8_t *frame, uint16_t frameCapacity,
                                    uint8_t *frameLen)
{
    if (frame == nullptr || frameLen == nullptr)
        return false;
    const uint16_t totalLen = (uint16_t)payloadLen + PROTOCOL_FRAME_OVERHEAD;
    if (totalLen > PROTOCOL_FRAME_MAX_SIZE || totalLen > frameCapacity)
        return false;

    frame[0] = 0x55;
    frame[1] = 0x00;
    frame[2] = (uint8_t)totalLen;
    frame[3] = command;
    frame[4] = address;

    uint8_t checksum = (uint8_t)(frame[2] + command + address);
    for (uint8_t i = 0; i < payloadLen; ++i) {
        checksum = (uint8_t)(checksum + frame[5 + i]);
    }

    frame[5 + payloadLen] = (uint8_t)(~checksum);
    frame[6 + payloadLen] = 0x00;
    frame[7 + payloadLen] = 0xAA;
    *frameLen = (uint8_t)totalLen;
    return true;
}

void Protocol_SendShutdownNotice()
{
    constexpr uint8_t payload = 0xFF;
    uint8_t frame[PROTOCOL_FRAME_OVERHEAD + sizeof(payload)];
    uint8_t frameLength = 0;
    if (encodeResponse(0x12, 0x00, &payload, sizeof(payload), frame, sizeof(frame), &frameLength)) {
        ProtocolTransport_Send(PORT_SERIAL0, frame, frameLength);
        ProtocolTransport_Send(PORT_SERIAL1, frame, frameLength);
    }
}

namespace {
constexpr uint16_t kUartRxBudget = 512;
constexpr uint16_t kTelemetryFrameSize = 254;

struct ProtocolParser {
    MsgPacket packet = {};
    uint16_t messageIndex = 0;
    uint16_t messageLength = 0;
    uint8_t checksum = 0;
    uint8_t state = 0;

    bool push(uint8_t byte);
    void reset();
};

void ProtocolParser::reset()
{
    state = 0;
    messageIndex = 0;
    messageLength = 0;
    checksum = 0;
}

bool ProtocolParser::push(uint8_t byte)
{
    switch (state) {
    case 0:
        if (byte == 0x55) state = 1;
        break;
    case 1:
        state = byte == 0x00 ? 2 : (byte == 0x55 ? 1 : 0);
        break;
    case 2:
        if (byte < PROTOCOL_FRAME_OVERHEAD) {
            reset();
            break;
        }
        packet.length = byte;
        messageLength = byte - 6; // command + address + payload
        messageIndex = 0;
        checksum = byte;
        state = 3;
        break;
    case 3:
        packet.message[messageIndex++] = byte;
        checksum = (uint8_t)(checksum + byte);
        if (messageIndex == messageLength) state = 4;
        break;
    case 4:
        state = byte == (uint8_t)~checksum ? 5 : (byte == 0x55 ? 1 : 0);
        break;
    case 5:
        state = byte == 0x00 ? 6 : (byte == 0x55 ? 1 : 0);
        break;
    case 6:
        state = byte == 0x55 ? 1 : 0;
        return byte == 0xAA;
    default:
        reset();
        break;
    }
    return false;
}

static ProtocolParser parsers[PROTOCOL_PORT_COUNT];
static MsgRead registerResponses[PROTOCOL_PORT_COUNT];

// All offsets are relative to the 250-byte payload, not the envelope.
constexpr uint16_t kPayloadSize = 250;
constexpr uint16_t kImuOffset = 20;
constexpr uint16_t kMotorAngleOffset = kImuOffset + 9 * 4;
constexpr uint16_t kMotorVelocityOffset = kMotorAngleOffset + 16 * 4;
constexpr uint16_t kWheelVelocityOffset = kMotorVelocityOffset + 16 * 4;
constexpr uint16_t kWheelOdometryOffset = kWheelVelocityOffset + 4 * 4;
constexpr uint16_t kReservedOffset = kWheelOdometryOffset + 4 * 4;
static_assert(kReservedOffset == 216, "deployed telemetry field offsets changed");
static_assert(kPayloadSize + 4 == kTelemetryFrameSize,
              "deployed telemetry envelope changed");
static_assert(MOTOR_NUM >= 16, "telemetry requires the deployed sixteen motor slots");

static void encodeTelemetry(uint8_t (&frame)[kTelemetryFrameSize])
{
    // Reserved payload bytes are transmitted as zero.
    memset(frame, 0, sizeof(frame));
    frame[0] = 0x55;
    frame[kTelemetryFrameSize - 1] = 0xAA;
    uint8_t *payload = frame + 2;
    payload[0] = (uint8_t)battery;
    Float2Byte(payload + 4, body.vx / 74.0);
    Float2Byte(payload + 8, body.vy);
    Float2Byte(payload + 12, odom.x);
    Float2Byte(payload + 16, odom.y);
    Float2Byte(payload + kImuOffset, imu.rol);
    Float2Byte(payload + kImuOffset + 4, imu.pit);
    Float2Byte(payload + kImuOffset + 8, imu.yaw);
    for (uint8_t i = 0; i < 3; ++i) {
        Float2Byte(payload + kImuOffset + 12 + i * 4, imu.acc[i]);
        Float2Byte(payload + kImuOffset + 24 + i * 4, imu.gyro[i]);
    }
    for (uint8_t i = 0; i < 16; ++i) {
        Float2Byte(payload + kMotorAngleOffset + i * 4, motor[i].fb_angle);
        // Preserve the original promotion/rounding, including its double literals.
        Float2Byte(payload + kMotorVelocityOffset + i * 4,
                   (float)motor[i].fb_vel / 4096.0 * 360.0);
    }
    for (uint8_t i = 0; i < 4; ++i) {
        Float2Byte(payload + kWheelVelocityOffset + i * 4, wheel[i].fb_vel);
        Int2Byte(payload + kWheelOdometryOffset + i * 4, (int)wheel[i].odom);
    }
}

static void sendRegisterResponse(uint8_t port)
{
    uint8_t frame[PROTOCOL_FRAME_MAX_SIZE];
    const MsgRead &response = registerResponses[port];
    uint8_t length = 0;
    if (response.length > 0 &&
        encodeResponse(0x12, response.addr, response.message,
                                     response.length, frame, sizeof(frame), &length)) {
        ProtocolTransport_Send(port, frame, length);
    }
}

static void dispatchPacket(uint8_t port)
{
    MsgPacket &packet = parsers[port].packet;
    const uint8_t command = packet.message[0];
    if (command == 0x02) {
        if (packet.length < 9) return;
        MsgRead &response = registerResponses[port];
        response.addr = packet.message[1];
        response.length = packet.message[2];
        if (response.length > PROTOCOL_FRAME_MAX_SIZE - PROTOCOL_FRAME_OVERHEAD) return;
        RegisterProtocol_Read(response);
        sendRegisterResponse(port);
        return;
    }
    if (command == 0x04) {
        if (packet.length >= 8) {
            CommandProtocol_Read(port, packet.message[1], packet.message + 2,
                                 (uint8_t)(packet.length - 8));
        }
        return;
    }
    if (!isReceiveMsg) return;
    if (command == 0x00 || command == 0x01) {
        RegisterProtocol_Write(port, packet);
    } else if (command == 0x03 && packet.length >= 8) {
        CommandProtocol_Write(port, packet.message[1], packet.message + 2,
                              (uint8_t)(packet.length - 8));
    }
}

static void pollSerial(HardwareSerial &serial, uint8_t port)
{
    uint8_t chunk[64];
    uint16_t remaining = kUartRxBudget;
    while (remaining != 0) {
        const size_t count = serial.read(chunk, min((size_t)remaining, sizeof(chunk)));
        if (count == 0) break;
        Protocol_ProcessBytes(chunk, (uint16_t)count, port);
        remaining -= (uint16_t)count;
    }
}

static void protocolTask(void *)
{
    uint32_t lastFeedbackMs = millis();
    for (;;) {
        if (imu_type != 0xFF) pollSerial(Serial, PORT_SERIAL0); // FDI owns Serial0.
        pollSerial(Serial1, PORT_SERIAL1);
        const uint32_t now = millis();
        if (feedback_flag != 1) {
            lastFeedbackMs = now;
        } else if (now - lastFeedbackMs >= 20u &&
                   ProtocolTransport_CanSendFeedback(ProtocolTransport_GetFeedbackPort())) {
            lastFeedbackMs = now;
            uint8_t frame[kTelemetryFrameSize];
            encodeTelemetry(frame);
            ProtocolTransport_Send(ProtocolTransport_GetFeedbackPort(), frame, sizeof(frame));
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}
} // namespace

void Protocol_ProcessBytes(const uint8_t *data, uint16_t length, uint8_t port)
{
    if (data == nullptr || port >= PROTOCOL_PORT_COUNT) return;
    for (uint16_t i = 0; i < length; ++i) {
        if (parsers[port].push(data[i])) dispatchPacket(port);
    }
}

void Protocol_ResetPort(uint8_t port)
{
    if (port < PROTOCOL_PORT_COUNT) parsers[port].reset();
}

void Protocol_Start()
{
    xTaskCreatePinnedToCore(protocolTask, "Protocol", 4096, nullptr, 3, nullptr, 0);
}
