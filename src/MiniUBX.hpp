#pragma once
#include <cstdint>
#include <cstring>
#include <functional>



// UBX Protocol Constants
namespace UbxConstants {
    // Message Classes
    constexpr uint8_t UBX_CLASS_NAV = 0x01;  // Navigation Results
    constexpr uint8_t UBX_CLASS_RXM = 0x02;  // Receiver Manager Messages
    constexpr uint8_t UBX_CLASS_INF = 0x04;  // Information Messages
    constexpr uint8_t UBX_CLASS_ACK = 0x05;  // Acknowledgements
    constexpr uint8_t UBX_CLASS_CFG = 0x06;  // Configuration
    constexpr uint8_t UBX_CLASS_UPD = 0x09;  // Firmware Update
    constexpr uint8_t UBX_CLASS_MON = 0x0A;  // Monitoring
    constexpr uint8_t UBX_CLASS_AID = 0x0B;  // AssistNow Aiding
    constexpr uint8_t UBX_CLASS_TIM = 0x0D;  // Timing
    constexpr uint8_t UBX_CLASS_ESF = 0x10;  // External Sensor Fusion
    constexpr uint8_t UBX_CLASS_MGA = 0x13;  // Multiple GNSS Assistance
    constexpr uint8_t UBX_CLASS_LOG = 0x21;  // Logging
    constexpr uint8_t UBX_CLASS_SEC = 0x27;  // Security
    constexpr uint8_t UBX_CLASS_HNR = 0x28;  // High Rate Navigation

    // Message IDs for NAV class
    constexpr uint8_t UBX_NAV_PVT = 0x07;    // Navigation Position Velocity Time Solution
    constexpr uint8_t UBX_NAV_SAT = 0x35;    // Satellite Information
    constexpr uint8_t UBX_NAV_CLOCK = 0x22;  // Clock Solution
    constexpr uint8_t UBX_NAV_DOP = 0x04;    // Dilution of Precision
    constexpr uint8_t UBX_NAV_EOE = 0x61;    // End of Epoch
    constexpr uint8_t UBX_NAV_STATUS = 0x03; // Receiver Navigation Status
    constexpr uint8_t UBX_NAV_POSECEF = 0x01; // Position Solution in ECEF
    constexpr uint8_t UBX_NAV_VELECEF = 0x11; // Velocity Solution in ECEF

    // Message IDs for RXM class
    constexpr uint8_t UBX_RXM_RAWX = 0x15;   // Raw Measurement Data
    constexpr uint8_t UBX_RXM_SFRBX = 0x13;  // Broadcast Navigation Data Subframe
    constexpr uint8_t UBX_RXM_MEASX = 0x14;  // Satellite Measurements for RRLP

    // Message IDs for ACK class
    constexpr uint8_t UBX_ACK_ACK = 0x01;    // Acknowledged
    constexpr uint8_t UBX_ACK_NAK = 0x00;    // Not Acknowledged

    // Message IDs for CFG class
    constexpr uint8_t UBX_CFG_PRT = 0x00;    // Port Configuration
    constexpr uint8_t UBX_CFG_MSG = 0x01;    // Message Configuration
    constexpr uint8_t UBX_CFG_RST = 0x04;    // Reset Receiver
    constexpr uint8_t UBX_CFG_RATE = 0x08;   // Navigation Rate
    constexpr uint8_t UBX_CFG_CFG = 0x09;    // Save/Load/Reset Configuration
    constexpr uint8_t UBX_CFG_RXM = 0x11;    // Receiver Manager Configuration
    constexpr uint8_t UBX_CFG_SBAS = 0x16;   // SBAS Configuration
    constexpr uint8_t UBX_CFG_GNSS = 0x3E;   // GNSS Configuration
    constexpr uint8_t UBX_CFG_PM2 = 0x3B;    // Power Management 2 Configuration
    constexpr uint8_t UBX_CFG_NAVX5 = 0x23;  // Navigation Engine Expert Settings
    constexpr uint8_t UBX_CFG_SIG = 0x8A;    // Signal configuration

    // Message IDs for MON class
    constexpr uint8_t UBX_MON_VER = 0x04;    // Receiver/Software Version
    constexpr uint8_t UBX_MON_HW = 0x09;     // Hardware Status
    constexpr uint8_t UBX_MON_RF = 0x38;     // RF Information

    // Message IDs for AID class
    constexpr uint8_t UBX_AID_EPH = 0x31;    // Ephemeris Data
    constexpr uint8_t UBX_AID_ALM = 0x30;    // Almanac Data
    constexpr uint8_t UBX_AID_HUI = 0x02;    // Health, UTC, Ionosphere Parameters

    // Reset types
    constexpr uint16_t UBX_RESET_HOT = 0x0000;    // Hot start
    constexpr uint16_t UBX_RESET_WARM = 0x0001;   // Warm start
    constexpr uint16_t UBX_RESET_COLD = 0x0002;   // Cold start
    constexpr uint16_t UBX_RESET_HW = 0x0000;     // Hardware reset
    constexpr uint16_t UBX_RESET_SW = 0x0001;     // Software reset
    constexpr uint16_t UBX_RESET_GNSS = 0x0002;   // GNSS only reset

    // GNSS IDs
    constexpr uint8_t UBX_GNSS_ID_GPS = 0;        // GPS
    constexpr uint8_t UBX_GNSS_ID_SBAS = 1;       // SBAS
    constexpr uint8_t UBX_GNSS_ID_GALILEO = 2;    // Galileo
    constexpr uint8_t UBX_GNSS_ID_BEIDOU = 3;     // BeiDou
    constexpr uint8_t UBX_GNSS_ID_IMES = 4;       // IMES
    constexpr uint8_t UBX_GNSS_ID_QZSS = 5;       // QZSS
    constexpr uint8_t UBX_GNSS_ID_GLONASS = 6;    // GLONASS
    constexpr uint8_t UBX_GNSS_ID_NAVIC = 7;      // NavIC

    // Signal identifiers
    constexpr uint32_t UBX_SIG_GPS_L1CA = 0x10310001;
    constexpr uint32_t UBX_SIG_GPS_L2C = 0x10310003;
    constexpr uint32_t UBX_SIG_GPS_L5 = 0x10310004;
    constexpr uint32_t UBX_SIG_SBAS_L1 = 0x10310005;
    constexpr uint32_t UBX_SIG_GAL_E1 = 0x10310007;
    constexpr uint32_t UBX_SIG_GAL_E5A = 0x10310009;
    constexpr uint32_t UBX_SIG_GAL_E5B = 0x1031000A;
    constexpr uint32_t UBX_SIG_BDS_B1I = 0x1031000D;
    constexpr uint32_t UBX_SIG_BDS_B2I = 0x1031000E;
    constexpr uint32_t UBX_SIG_BDS_B2A = 0x10310028;
    constexpr uint32_t UBX_SIG_QZSS_L1CA = 0x10310012;
    constexpr uint32_t UBX_SIG_QZSS_L1S = 0x10310014;
    constexpr uint32_t UBX_SIG_QZSS_L2C = 0x10310015;
    constexpr uint32_t UBX_SIG_QZSS_L5 = 0x10310017;
    constexpr uint32_t UBX_SIG_GLO_L1 = 0x10310018;
    constexpr uint32_t UBX_SIG_GLO_L2 = 0x10310019;
    constexpr uint32_t UBX_SIG_NAVIC_L5 = 0x1031001D;

    // GPS constellation enable
    constexpr uint32_t UBX_SIG_GPS_ENA = 0x1031001F;
    constexpr uint32_t UBX_SIG_SBAS_ENA = 0x10310020;
    constexpr uint32_t UBX_SIG_GAL_ENA = 0x10310021;
    constexpr uint32_t UBX_SIG_BDS_ENA = 0x10310022;
    constexpr uint32_t UBX_SIG_QZSS_ENA = 0x10310024;
    constexpr uint32_t UBX_SIG_GLO_ENA = 0x10310025;
    constexpr uint32_t UBX_SIG_NAVIC_ENA = 0x10310026;
}

// Forward declaration of the UbxMessage class
class UbxMessage
{
public:
    static constexpr size_t MAX_PAYLOAD_SIZE = 4096;
public:
    enum class Errors
    {
        NoError = 0,
        FieldOutOfBound = 1,
    };

    UbxMessage() : msgClass(0), msgId(0), actual_length(0) {}
    UbxMessage(uint8_t msgClass, uint8_t msgId, const uint8_t* payload, size_t length);

    uint8_t getClass() const { return msgClass; }
    uint8_t getId() const { return msgId; }
    const uint8_t* getPayload() const { return payload; }
    size_t getLength() const { return actual_length; }

    template<typename T>
    T getPayloadField(size_t offset) const
    {
        T value = T{};
        if (offset + sizeof(T) <= actual_length) 
        {
            memcpy(&value, payload + offset, sizeof(T));
        }
        return value;
    }

    template<typename T>
    void getPayloadField(size_t offset, T& value) const
    {
        if (offset + sizeof(T) <= actual_length) 
        {
            memcpy(&value, payload + offset, sizeof(T));
        }
    }

private:
    uint8_t msgClass{0};
    uint8_t msgId{0};
    uint8_t payload[MAX_PAYLOAD_SIZE]; // Maximum payload size
    uint16_t actual_length;
};

/**
 * @brief UBX Protocol Handler Class
 */
class UbxProtocol {
private:
    // Constants
    static constexpr uint8_t UBX_SYNC1 = 0xB5;
    static constexpr uint8_t UBX_SYNC2 = 0x62;
    
public:
    /**
     * @brief Constructor
     */
    UbxProtocol();

    /**
     * @brief Generate a UBX message packet
     * @param msgClass Message class
     * @param msgId Message ID
     * @param payload Payload data
     * @param length Payload length
     * @param buffer Output buffer for the complete message
     * @param bufferSize Size of the output buffer
     * @return Size of the complete message or 0 if buffer too small
     */
    size_t generateMessage(uint8_t msgClass, uint8_t msgId,
                          const uint8_t* payload, size_t length,
                          uint8_t* buffer, size_t bufferSize) const;

    /**
     * @brief Process input data looking for UBX messages
     * @param data Input data buffer
     * @param length Length of input data
     * @param callback Callback function for complete messages
     */
    void processData(const uint8_t* data, size_t length,
                    std::function<void(const UbxMessage&)> callback);

    /**
     * @brief Reset the parser state
     */
    void reset();

private:
    // Parser state
    enum class State {
        SYNC1,      // Waiting for first sync char (0xB5)
        SYNC2,      // Waiting for second sync char (0x62)
        CLASS,      // Waiting for class byte
        ID,         // Waiting for ID byte
        LENGTH1,    // Waiting for length byte 1
        LENGTH2,    // Waiting for length byte 2
        PAYLOAD,    // Collecting payload
        CHECKSUM1,  // Waiting for checksum byte 1
        CHECKSUM2   // Waiting for checksum byte 2
    };

    // Calculate UBX checksum
    void calculateChecksum(uint8_t msgClass, uint8_t msgId,
                         const uint8_t* payload, size_t length,
                         uint8_t& ck_a, uint8_t& ck_b) const;

    // Parser state variables
    State state{State::SYNC1};
    uint8_t currentClass{0};
    uint8_t currentId{0};
    uint16_t currentLength{0};
    uint16_t payloadIndex{0};
    uint8_t currentCk_a{0};
    uint8_t currentCk_b{0};
    uint8_t expectedCk_a{0};
    uint8_t expectedCk_b{0};
    uint8_t payloadBuffer[UbxMessage::MAX_PAYLOAD_SIZE];
    uint16_t actualLength{0};
};