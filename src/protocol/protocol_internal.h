#pragma once

#include "protocol/protocol.h"

constexpr uint16_t PROTOCOL_FRAME_MAX_SIZE = 255u;
constexpr uint8_t PROTOCOL_FRAME_OVERHEAD = 8u;

struct MsgPacket {
    uint16_t length;
    uint8_t message[256];
};

struct MsgRead {
    uint8_t addr;
    uint8_t length;
    uint8_t message[256];
};

// Register commands use byte-scaled values; command groups use typed subfields.
void RegisterProtocol_Write(uint8_t port, const MsgPacket &packet);
void RegisterProtocol_Read(MsgRead &response);
void CommandProtocol_Write(uint8_t port, uint8_t address,
                           const uint8_t *data, uint8_t length);
void CommandProtocol_Read(uint8_t port, uint8_t address,
                          const uint8_t *subfields, uint8_t length);

// Payload is already stored at frame + 5.
bool ProtocolFrame_FinalizeResponse(uint8_t command, uint8_t address,
                                    uint8_t payloadLength, uint8_t *frame,
                                    uint16_t capacity, uint8_t *frameLength);
void ProtocolTransport_Send(uint8_t port, uint8_t *data, uint8_t length);
bool ProtocolTransport_CanSendFeedback(uint8_t port);
uint8_t ProtocolTransport_GetFeedbackPort();
