/*
 * serial.cpp
 * Implementation of UBX protocol classes
 */

#include "MiniUBX.hpp"
#include <cstdio>
#include <cstring>
#include <algorithm>
#include <functional>


// ============================================================================
// UbxMessage Implementation
// ============================================================================

UbxMessage::UbxMessage(uint8_t msgClass, uint8_t msgId, const uint8_t* payloadData, size_t length)
    : msgClass(msgClass), msgId(msgId), actual_length(0)
{
    // Check if payload fits in buffer
    if (length > UbxMessage::MAX_PAYLOAD_SIZE) {
        // Invalid message, reset everything
        this->msgClass = 0;
        this->msgId = 0;
        actual_length = 0;
    } else if (payloadData != nullptr && length > 0) {
        // Copy payload data
        std::memcpy(payload, payloadData, length);
        actual_length = static_cast<uint16_t>(length);
    } else {
        // No payload data
        actual_length = 0;
    }
}

// ============================================================================
// UbxProtocol Implementation
// ============================================================================

UbxProtocol::UbxProtocol()
{
    reset();
}

void UbxProtocol::reset()
{
    state = State::SYNC1;
    currentClass = 0;
    currentId = 0;
    currentLength = 0;
    payloadIndex = 0;
    currentCk_a = 0;
    currentCk_b = 0;
    expectedCk_a = 0;
    expectedCk_b = 0;
    actualLength = 0;
    std::memset(payloadBuffer, 0, sizeof(payloadBuffer));
}

void UbxProtocol::calculateChecksum(uint8_t msgClass, uint8_t msgId,
                                   const uint8_t* payload, size_t length,
                                   uint8_t& ck_a, uint8_t& ck_b) const
{
    ck_a = 0;
    ck_b = 0;
    
    // Add class and ID to checksum
    ck_a += msgClass;
    ck_b += ck_a;
    
    ck_a += msgId;
    ck_b += ck_a;
    
    // Add length (little-endian)
    ck_a += (length & 0xFF);
    ck_b += ck_a;
    
    ck_a += ((length >> 8) & 0xFF);
    ck_b += ck_a;
    
    // Add payload
    if (payload != nullptr) {
        for (size_t i = 0; i < length; ++i) {
            ck_a += payload[i];
            ck_b += ck_a;
        }
    }
}

size_t UbxProtocol::generateMessage(uint8_t msgClass, uint8_t msgId,
                                   const uint8_t* payload, size_t length,
                                   uint8_t* buffer, size_t bufferSize) const
{
    // Calculate required size: 2 sync + class + id + 2 length + payload + 2 checksum
    const size_t requiredSize = 8 + length;
    
    if (buffer == nullptr || bufferSize < requiredSize) {
        return 0;  // Buffer too small or null
    }
    
    // Calculate checksum
    uint8_t ck_a = 0, ck_b = 0;
    calculateChecksum(msgClass, msgId, payload, length, ck_a, ck_b);
    
    // Build message
    size_t index = 0;
    
    // Sync bytes
    buffer[index++] = UBX_SYNC1;
    buffer[index++] = UBX_SYNC2;
    
    // Class and ID
    buffer[index++] = msgClass;
    buffer[index++] = msgId;
    
    // Length (little-endian)
    buffer[index++] = static_cast<uint8_t>(length & 0xFF);
    buffer[index++] = static_cast<uint8_t>((length >> 8) & 0xFF);
    
    // Payload
    if (payload != nullptr && length > 0) {
        std::memcpy(&buffer[index], payload, length);
        index += length;
    }
    
    // Checksum
    buffer[index++] = ck_a;
    buffer[index++] = ck_b;
    
    return index;
}

void UbxProtocol::processData(const uint8_t* data, size_t length, std::function<void(const UbxMessage&)> callback)
{
    if (data == nullptr || length == 0) {
        return;
    }
    
    for (size_t i = 0; i < length; ++i) {
        const uint8_t currentByte = data[i];
        
        switch (state) {
            case State::SYNC1:
                //printf("State: SYNC1\n");
                if (currentByte == UBX_SYNC1) {
                    state = State::SYNC2;
                }
                break;
                
            case State::SYNC2:
                //printf("State: SYNC2\n");
                if (currentByte == UBX_SYNC2) {
                    state = State::CLASS;
                    // Reset checksum calculation
                    currentCk_a = 0;
                    currentCk_b = 0;
                } else {
                    state = State::SYNC1;  // Reset if second sync byte isn't correct
                }
                break;
                
            case State::CLASS:
                //printf("State: CLASS\n");
                currentClass = currentByte;
                currentCk_a += currentByte;
                currentCk_b += currentCk_a;
                state = State::ID;
                break;
                
            case State::ID:
                //printf("State: ID\n");
                currentId = currentByte;
                currentCk_a += currentByte;
                currentCk_b += currentCk_a;
                state = State::LENGTH1;
                break;
                
            case State::LENGTH1:
                //printf("State: LENGTH1\n");
                currentLength = currentByte;
                currentCk_a += currentByte;
                currentCk_b += currentCk_a;
                state = State::LENGTH2;
                break;
                
            case State::LENGTH2:
                //printf("State: LENGTH2\n");
                currentLength |= (static_cast<uint16_t>(currentByte) << 8);
                currentCk_a += currentByte;
                currentCk_b += currentCk_a;
                
                // Check if payload size is reasonable
                if (currentLength <= UbxMessage::MAX_PAYLOAD_SIZE) {
                    actualLength = currentLength;
                    payloadIndex = 0;
                    state = (currentLength > 0) ? State::PAYLOAD : State::CHECKSUM1;
                } else {
                    // Payload too large, reset
                    state = State::SYNC1;
                }
                break;
                
            case State::PAYLOAD:
                //printf("State: PAYLOAD\n");
                payloadBuffer[payloadIndex] = currentByte;
                currentCk_a += currentByte;
                currentCk_b += currentCk_a;
                payloadIndex++;
                
                if (payloadIndex >= currentLength) {
                    state = State::CHECKSUM1;
                }
                break;
                
            case State::CHECKSUM1:
                //printf("State: CHECKSUM1\n");
                expectedCk_a = currentByte;
                state = State::CHECKSUM2;
                break;
                
            case State::CHECKSUM2:
                //printf("State: CHECKSUM2\n");
                expectedCk_b = currentByte;
                state = State::SYNC1;  // Reset state for next message
                
                // Check if checksum is correct
                if (currentCk_a == expectedCk_a && currentCk_b == expectedCk_b) 
                {
                    //printf("State: VALID - processing data...\n");
                    // Valid message, create UbxMessage and call callback
                    if (callback) {
                        UbxMessage msg(currentClass, currentId, payloadBuffer, actualLength);
                        callback(msg);
                    }
                }
                // If checksum fails, message is silently discarded
                break;
        }
    }
}