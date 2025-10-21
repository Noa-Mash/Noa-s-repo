/*
 * UbxProtocol.cpp
 *
 *  Created on: May 14, 2025
 *      Author: David Michaeli
 */


#include "UbxGnssDriver.hpp"

#pragma GCC push_options
#pragma GCC optimize("O3,unroll-loops,prefetch-loop-arrays")
__attribute__((hot))                  // Mark as hot path
__attribute__((flatten))              // Inline all calls

UbxProtocol::UbxProtocol()
{
    reset();
}

size_t UbxProtocol::generateMessage(uint8_t msgClass, uint8_t msgId,
                                    const uint8_t* payload, size_t length,
                                    uint8_t* buffer, size_t bufferSize) const
{
    // Calculate required buffer size (2 sync bytes + class + id + length(2) + payload + checksum(2))
    const size_t requiredSize = 2 + 1 + 1 + 2 + length + 2;

    if (buffer == nullptr || bufferSize < requiredSize) {
        return 0; // Buffer too small or null
    }

    // Calculate checksum
    uint8_t ck_a = 0;
    uint8_t ck_b = 0;
    calculateChecksum(msgClass, msgId, payload, length, ck_a, ck_b);

    // Fill buffer
    size_t index = 0;
    buffer[index++] = UBX_SYNC1;
    buffer[index++] = UBX_SYNC2;
    buffer[index++] = msgClass;
    buffer[index++] = msgId;
    buffer[index++] = length & 0xFF;        // Length LSB
    buffer[index++] = (length >> 8) & 0xFF; // Length MSB

    // Copy payload
    if (payload != nullptr && length > 0) {
        std::memcpy(buffer + index, payload, length);
        index += length;
    }

    // Add checksum
    buffer[index++] = ck_a;
    buffer[index++] = ck_b;

    return index; // Return the actual message size
}

void UbxProtocol::processData(const uint8_t* data, size_t length, std::function<void(const UbxMessage&)> callback)
{
    for (size_t i = 0; i < length; ++i)
    {
        const uint8_t currentByte = data[i];

        switch (state)
        {
            case State::SYNC1:
                if (currentByte == UBX_SYNC1) {
                    state = State::SYNC2;
                }
                break;

            case State::SYNC2:
                if (currentByte == UBX_SYNC2) {
                    state = State::CLASS;
                } else {
                    state = State::SYNC1; // Reset if second sync byte isn't correct
                }
                break;

            case State::CLASS:
                currentClass = currentByte;
                currentCk_a = currentByte;
                currentCk_b = currentByte;
                state = State::ID;
                break;

            case State::ID:
                currentId = currentByte;
                currentCk_a += currentByte;
                currentCk_b += currentCk_a;
                state = State::LENGTH1;
                break;

            case State::LENGTH1:
                currentLength = currentByte;
                currentCk_a += currentByte;
                currentCk_b += currentCk_a;
                state = State::LENGTH2;
                break;

            case State::LENGTH2:
                currentLength |= (static_cast<uint16_t>(currentByte) << 8);
                currentCk_a += currentByte;
                currentCk_b += currentCk_a;

                // Check if payload size is reasonable
                if (currentLength <= MAX_PAYLOAD_SIZE)
                {
                	actualLength = currentLength;
                    //payloadBuffer.resize(currentLength);
                    payloadIndex = 0;
                    state = (currentLength > 0) ? State::PAYLOAD : State::CHECKSUM1;
                }
                else
                {
                    state = State::SYNC1; // Payload too large, reset
                }
                break;

            case State::PAYLOAD:
                payloadBuffer[payloadIndex] = currentByte;
                currentCk_a += currentByte;
                currentCk_b += currentCk_a;
                payloadIndex++;

                if (payloadIndex >= currentLength)
                {
                    state = State::CHECKSUM1;
                }
                break;

            case State::CHECKSUM1:
                expectedCk_a = currentByte;
                state = State::CHECKSUM2;
                break;

            case State::CHECKSUM2:
                expectedCk_b = currentByte;
                state = State::SYNC1; // Reset state for next message

                // Check if checksum is correct
                if (currentCk_a == expectedCk_a && currentCk_b == expectedCk_b) {
                    // Valid message, create UbxMessage and call callback
                    UbxMessage msg(currentClass, currentId, payloadBuffer, actualLength);
                    if (callback)
                    {
                        //callback(msg);
                    }
                }
                break;
        }
    }
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
    actualLength = 0;;
}

void UbxProtocol::calculateChecksum(uint8_t msgClass, uint8_t msgId,
                                  const uint8_t* payload, size_t length,
                                  uint8_t& ck_a, uint8_t& ck_b) const
{
    ck_a = 0;
    ck_b = 0;

    // Class and ID
    ck_a += msgClass;
    ck_b += ck_a;

    ck_a += msgId;
    ck_b += ck_a;

    // Length (LSB first)
    ck_a += length & 0xFF;
    ck_b += ck_a;

    ck_a += (length >> 8) & 0xFF;
    ck_b += ck_a;

    // Payload
    for (size_t i = 0; i < length; ++i) {
        ck_a += payload[i];
        ck_b += ck_a;
    }
}

// UbxMessage implementation
UbxMessage::UbxMessage(uint8_t msgClass, uint8_t msgId, const uint8_t* payloadData, size_t length)
    : msgClass(msgClass), msgId(msgId)
{
	if (length > max_payload_leng)
	{
		msgClass = 0;
		msgId = 0;
		length = 0;
	}
	else if (payloadData != nullptr && length > 0)
    {
    	memcpy(payload, payloadData, length);
    	actual_length = length;
    }
}

#pragma GCC pop_options

