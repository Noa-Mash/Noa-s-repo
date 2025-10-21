/*
 * UbxGnssDriver.h
 *
 *  Created on: May 14, 2025
 *      Author: David Michaeli
 */

#ifndef __UBXGNSSDRIVER_HPP_
#define __UBXGNSSDRIVER_HPP_

//#include "tx_api.h"
#include <cstdint>
#include <functional>
#include <memory>
#include <vector>
#include <array>
#include <mutex>
#include <string>
#include <unordered_map>
//#include "../HighLevelDrivers/uart.hpp"
//#include "../HighLevelDrivers/gpio.hpp"

// Forward declarations
class UbxProtocol;
class UbxMessage;

/**
 * @brief Main class for the u-blox GNSS driver
 */
class UbxGnssDriver {
public:
    // Constellation types
    enum class Constellation {
        GPS,
        SBAS,
        GALILEO,
        BEIDOU,
        IMES,
        QZSS,
        GLONASS,
        NAVIC
    };

    // Signal types (for each constellation)
    enum class GpsSignal {
        L1CA = 0x01,
        L2C = 0x10,
        L5 = 0x20
    };

    enum class GalileoSignal {
        E1 = 0x01,
        E5a = 0x10,
        E5b = 0x20
    };

    enum class BeidouSignal {
        B1I = 0x01,
        B2I = 0x10,
        B2A = 0x80
    };

    enum class GlonassSignal {
        L1 = 0x01,
        L2 = 0x10
    };

    enum class SbasSignal {
        L1 = 0x01
    };

    enum class QzssSignal {
        L1CA = 0x01,
        L1S = 0x04,
        L2C = 0x10,
        L5 = 0x20
    };

    enum class NavicSignal {
        L5 = 0x01
    };

    // Fix type definitions
    enum class FixType {
        NO_FIX = 0,
        DEAD_RECKONING = 1,
        FIX_2D = 2,
        FIX_3D = 3,
        GNSS_DEAD_RECKONING = 4,
        TIME_ONLY = 5
    };

    // Power modes
    enum class PowerMode {
        FULL_POWER = 0,
        BALANCED = 1,
        INTERVAL = 2,
        AGGRESSIVE_1HZ = 3,
        AGGRESSIVE_2HZ = 4,
        AGGRESSIVE_4HZ = 5
    };

    // Error states
    enum class Errors
    	{
    		NoError = 0,
    		PvtParsingFailed = 1,
    	};

    // Configuration structure for the driver
    struct Config {
        Config(uint32_t baudRate = 230400,
               bool useDma = true,
               uint32_t rxBufferSize = 2048,
               uint32_t txBufferSize = 256)
            : baudRate(baudRate),
              useDma(useDma),
              rxBufferSize(rxBufferSize),
              txBufferSize(txBufferSize) {}

        uint32_t baudRate;
        bool useDma;
        uint32_t rxBufferSize;
        uint32_t txBufferSize;
    };

    // Position, Velocity, Time (PVT) data structure
    struct __attribute__((packed)) PvtData {
        uint32_t iTOW;        // GPS time of week (ms)
        uint16_t year;        // Year (UTC)
        uint8_t month;        // Month (UTC) 1..12
        uint8_t day;          // Day of month (UTC) 1..31
        uint8_t hour;         // Hour (UTC) 0..23
        uint8_t minute;       // Minute (UTC) 0..59
        uint8_t second;       // Second (UTC) 0..60
        bool validDate;       // Valid UTC date flag
        bool validTime;       // Valid UTC time flag
        bool fullyResolved;   // UTC time of day fully resolved
        uint32_t tAccNs;      // Time accuracy estimate (ns)
        int32_t nanoSecond;   // Fraction of second (-1e9..1e9)
        FixType fixType;      // GNSS fix type
        bool gnssFixOK;       // Valid fix flag
        bool diffSoln;        // Differential corrections applied
        uint8_t numSV;        // Number of satellites used
        double latitude;      // Latitude (deg)
        double longitude;     // Longitude (deg)
        double height;        // Height above ellipsoid (m)
        double heightMSL;     // Height above mean sea level (m)
        double hAcc;          // Horizontal accuracy (m)
        double vAcc;          // Vertical accuracy (m)
        double velN;          // North velocity (m/s)
        double velE;          // East velocity (m/s)
        double velD;          // Down velocity (m/s)
        double groundSpeed;   // Ground speed (m/s)
        double headingMotion; // Heading of motion (deg)
        double sAcc;          // Speed accuracy (m/s)
        double headingAcc;    // Heading accuracy (deg)
        double pDOP;          // Position DOP
        double magDec;        // Magnetic declination (deg)
        double magAcc;        // Magnetic declination accuracy (deg)
    };

    // Satellite data structure
    struct __attribute__((packed)) SatelliteData {
        uint8_t gnssId;       // GNSS identifier
        uint8_t svId;         // Satellite identifier
        uint8_t cno;          // Carrier-to-noise ratio (dBHz)
        int8_t elevation;     // Elevation (deg)
        int16_t azimuth;      // Azimuth (deg)
        int16_t prRes;        // Pseudorange residual (m * 0.1)
        bool used;            // Satellite used in navigation solution
        uint8_t signalQuality; // Signal quality indicator
        bool healthyFlag;     // Satellite health flag
        bool diffCorr;        // Differential correction data available
        bool orbitAvail;      // Ephemeris or almanac orbit data available
        bool ephAvail;        // Ephemeris orbit data available
        bool almAvail;        // Almanac orbit data available
    };

    // Receiver hardware status
    struct __attribute__((packed)) ReceiverStatus {
        uint8_t antStatus;    // Antenna status (0=INIT, 1=UNKNOWN, 2=OK, 3=SHORT, 4=OPEN)
        uint8_t antPower;     // Antenna power status (0=OFF, 1=ON, 2=UNKNOWN)
        uint16_t noisePerMS;  // Noise level
        uint16_t agcCnt;      // AGC monitor value
        uint8_t jamState;     // Jamming state (0=unknown/disabled, 1=ok, 2=warning, 3=critical)
        uint8_t cwJamming;    // CW jamming indicator (0-255)
        bool rtcCalib;        // RTC is calibrated flag
        bool safeboot;        // Safeboot mode flag
    };

    // Raw measurement data
    struct __attribute__((packed)) RawMeasurement {
        uint8_t gnssId;       // GNSS identifier
        uint8_t svId;         // Satellite identifier
        uint8_t sigId;        // Signal identifier
        uint8_t cno;          // Carrier-to-noise ratio (dBHz)
        double pseudorange;   // Pseudorange (m)
        double carrierPhase;  // Carrier phase (cycles)
        double doppler;       // Doppler (Hz)
        uint8_t lockTime;     // Carrier phase locktime counter
        uint8_t prStdev;      // Pseudorange standard deviation
        uint8_t cpStdev;      // Carrier phase standard deviation
        uint8_t doStdev;      // Doppler standard deviation
        bool prValid;         // Pseudorange valid flag
        bool cpValid;         // Carrier phase valid flag
        bool halfCycle;       // Half cycle valid flag
        bool subHalfCycle;    // Half cycle subtracted from phase
    };

    // Ephemeris data
    struct __attribute__((packed)) EphemerisData {
        uint8_t gnssId;       // GNSS identifier
        uint8_t svId;         // Satellite identifier
        uint8_t sigId;        // Signal identifier
        uint16_t week;        // GPS/Galileo/BeiDou week number
        uint32_t toe;         // Time of Ephemeris
        uint32_t toc;         // Clock data reference time
        double af0;           // Clock bias (s)
        double af1;           // Clock drift (s/s)
        double af2;           // Clock drift rate (s/s²)
        double tgd;           // Group delay (s)
        double ura;           // SV accuracy (m)
        double health;        // Satellite health
        double e;             // Eccentricity
        double sqrtA;         // Square root of semi-major axis (m^1/2)
        double omega0;        // Longitude of ascending node (rad)
        double i0;            // Inclination (rad)
        double omega;         // Argument of perigee (rad)
        double m0;            // Mean anomaly (rad)
        double deltaN;        // Mean motion difference (rad/s)
        double idot;          // Rate of inclination angle (rad/s)
        double omegaDot;      // Rate of right ascension (rad/s)
        double cuc;           // Cos latitude - argument correction (rad)
        double cus;           // Sin latitude - argument correction (rad)
        double crc;           // Cos orbit radius correction (m)
        double crs;           // Sin orbit radius correction (m)
        double cic;           // Cos inclination correction (rad)
        double cis;           // Sin inclination correction (rad)
        bool valid;           // Valid ephemeris flag
    };

    // Almanac data
    struct __attribute__((packed)) AlmanacData {
        uint8_t gnssId;       // GNSS identifier
        uint8_t svId;         // Satellite identifier
        uint16_t week;        // GPS/Galileo/BeiDou week number
        uint32_t toa;         // Reference time of almanac
        double e;             // Eccentricity
        double deltai;        // Inclination delta from reference
        double omegaDot;      // Rate of right ascension
        double svHealth;      // Satellite health information
        double sqrtA;         // Square root of semi-major axis
        double omega0;        // Longitude of ascending node
        double omega;         // Argument of perigee
        double m0;            // Mean anomaly
        double af0;           // Clock bias (s)
        double af1;           // Clock drift (s/s)
        bool valid;           // Valid almanac flag
    };

    // Receiver version information
    struct ReceiverVersion {
        std::string swVersion;      // Software version string
        std::string hwVersion;      // Hardware version string
        std::string extension;      // Extension string
        std::vector<std::string> extModules; // Extended module information
    };

    struct __attribute__((packed)) PortConfiguration
    {
    	uint8_t port_id;
    	uint8_t res1;
    	uint16_t tx_ready_bitmask;
    	uint32_t mode_bitmask;
    	uint32_t baudrate;
    	uint16_t in_proto_mask;
    	uint16_t out_proto_mask;
    	uint16_t flags;
    	uint8_t res2;
    	uint8_t res3;
    };

    // Callback types for different message types
    using PvtCallback = std::function<void(const PvtData&)>;
    using SatelliteCallback = std::function<void(const std::vector<SatelliteData>&)>;
    using RawMeasurementCallback = std::function<void(const std::vector<RawMeasurement>&)>;
    using ReceiverStatusCallback = std::function<void(const ReceiverStatus&)>;
    using EphemerisCallback = std::function<void(const EphemerisData&)>;
    using AlmanacCallback = std::function<void(const AlmanacData&)>;
    using ErrorCallback = std::function<void(const std::string&, int)>;
    using VersionCallback = std::function<void(const ReceiverVersion&)>;

    /**
     * @brief Constructor for the GNSS driver
     * @param uart Pointer to the UART driver instance
     * @param config Configuration settings
     */
    UbxGnssDriver();

    /**
     * @brief Destructor
     */
    ~UbxGnssDriver();

    /**
     * @brief Initialize the GNSS receiver
     * @param timeout Timeout in system ticks
     * @return True if initialization was successful
     */
    bool init(UART* uart, GPIO* power_output_pin, const Config& config = Config(), ULONG timeout = 1000/*TX_WAIT_FOREVER*/);

    /**
     * @brief Register callback for PVT data
     * @param callback Function to call when PVT data is received
     */
    void registerPvtCallback(PvtCallback callback);

    /**
     * @brief Register callback for satellite data
     * @param callback Function to call when satellite data is received
     */
    void registerSatelliteCallback(SatelliteCallback callback);

    /**
     * @brief Register callback for raw measurement data
     * @param callback Function to call when raw measurement data is received
     */
    void registerRawMeasurementCallback(RawMeasurementCallback callback);

    /**
     * @brief Register callback for receiver status
     * @param callback Function to call when receiver status is received
     */
    void registerReceiverStatusCallback(ReceiverStatusCallback callback);

    /**
     * @brief Register callback for ephemeris data
     * @param callback Function to call when ephemeris data is received
     */
    void registerEphemerisCallback(EphemerisCallback callback);

    /**
     * @brief Register callback for almanac data
     * @param callback Function to call when almanac data is received
     */
    void registerAlmanacCallback(AlmanacCallback callback);

    /**
     * @brief Register callback for version information
     * @param callback Function to call when version info is received
     */
    void registerVersionCallback(VersionCallback callback);

    /**
     * @brief Register callback for error conditions
     * @param callback Function to call when errors occur
     */
    void registerErrorCallback(ErrorCallback callback);

    /**
     * @brief Configure which constellations to enable/disable
     * @param constellation Constellation to configure
     * @param enable True to enable, false to disable
     * @param timeout Timeout in system ticks
     * @return True if successful
     */
    bool configureConstellation(Constellation constellation, bool enable, ULONG timeout = TX_WAIT_FOREVER);

    /**
     * @brief Configure specific signal for a constellation
     * @param constellation Constellation to configure
     * @param signalMask Bit mask of signals to enable
     * @param timeout Timeout in system ticks
     * @return True if successful
     */
    bool configureSignals(Constellation constellation, uint8_t signalMask, ULONG timeout = TX_WAIT_FOREVER);

    /**
     * @brief Configure GPS signals
     * @param l1ca Enable L1CA signal
     * @param l2c Enable L2C signal
     * @param l5 Enable L5 signal
     * @param timeout Timeout in system ticks
     * @return True if successful
     */
    bool configureGpsSignals(bool l1ca, bool l2c, bool l5, ULONG timeout = TX_WAIT_FOREVER);

    /**
     * @brief Configure Galileo signals
     * @param e1 Enable E1 signal
     * @param e5a Enable E5a signal
     * @param e5b Enable E5b signal
     * @param timeout Timeout in system ticks
     * @return True if successful
     */
    bool configureGalileoSignals(bool e1, bool e5a, bool e5b, ULONG timeout = TX_WAIT_FOREVER);

    /**
     * @brief Configure BeiDou signals
     * @param b1i Enable B1I signal
     * @param b2i Enable B2I signal
     * @param b2a Enable B2A signal
     * @param timeout Timeout in system ticks
     * @return True if successful
     */
    bool configureBeidouSignals(bool b1i, bool b2i, bool b2a, ULONG timeout = TX_WAIT_FOREVER);

    /**
     * @brief Configure GLONASS signals
     * @param l1 Enable L1 signal
     * @param l2 Enable L2 signal
     * @param timeout Timeout in system ticks
     * @return True if successful
     */
    bool configureGlonassSignals(bool l1, bool l2, ULONG timeout = TX_WAIT_FOREVER);

    /**
     * @brief Configure message rates
     * @param pvtRate Rate for PVT messages (0=disable, 1=once per navigation solution)
     * @param satRate Rate for satellite info messages
     * @param rawMeasRate Rate for raw measurement messages
     * @param timeout Timeout in system ticks
     * @return True if successful
     */
    bool configureMessageRates(uint8_t pvtRate, uint8_t satRate, uint8_t rawMeasRate, ULONG timeout = TX_WAIT_FOREVER);

    /**
     * @brief Configure navigation rate
     * @param measRateMs Measurement rate in milliseconds
     * @param timeout Timeout in system ticks
     * @return True if successful
     */
    bool configureNavigationRate(uint16_t measRateMs, ULONG timeout = TX_WAIT_FOREVER);

    /**
     * @brief Configure power mode
     * @param mode Power mode to set
     * @param cycleTimeSeconds For interval mode, cycle time in seconds
     * @param onTimeSeconds For interval mode, on time in seconds
     * @param timeout Timeout in system ticks
     * @return True if successful
     */
    bool configurePowerMode(PowerMode mode, uint16_t cycleTimeSeconds = 0, uint16_t onTimeSeconds = 0, ULONG timeout = TX_WAIT_FOREVER);

    /**
     * @brief Request ephemeris data for specific satellite
     * @param gnssId GNSS identifier
     * @param svId Satellite identifier
     * @param timeout Timeout in system ticks
     * @return True if request was sent successfully
     */
    bool requestEphemeris(uint8_t gnssId, uint8_t svId, ULONG timeout = TX_WAIT_FOREVER);

    /**
     * @brief Request almanac data for specific satellite
     * @param gnssId GNSS identifier
     * @param svId Satellite identifier
     * @param timeout Timeout in system ticks
     * @return True if request was sent successfully
     */
    bool requestAlmanac(uint8_t gnssId, uint8_t svId, ULONG timeout = TX_WAIT_FOREVER);

    /**
     * @brief Request receiver version information
     * @param timeout Timeout in system ticks
     * @return True if request was sent successfully
     */
    bool requestReceiverVersion(ULONG timeout = TX_WAIT_FOREVER);

    /**
     * @brief Reset the receiver
     * @param hardReset True for hardware reset, false for software reset
     * @param timeout Timeout in system ticks
     * @return True if reset command was sent successfully
     */
    bool resetReceiver(bool hardReset = false, ULONG timeout = TX_WAIT_FOREVER, bool usingIO = true);

    /**
     * @brief Start the receiver task
     * @return True if task was started successfully
     */
    bool startReceiverTask();

    /**
     * @brief Stop the receiver task
     */
    void stopReceiverTask();

    /**
     * @brief Get the latest PVT data
     * @return The latest PVT data
     */
    PvtData getLatestPvtData() const;

    /**
     * @brief Get the latest satellite data
     * @return The latest satellite data
     */
    std::vector<SatelliteData> getLatestSatelliteData() const;

    /**
     * @brief Get the latest receiver status
     * @return The latest receiver status
     */
    ReceiverStatus getLatestReceiverStatus() const;

    /**
     * @brief Get the receiver version information (if available)
     * @return The receiver version information
     */
    ReceiverVersion getReceiverVersion() const;

    /**
     * @brief Process incoming data from the UART
     * @param data Pointer to the data buffer
     * @param length Length of the data
     */
    void processUartData(const uint8_t* data, size_t length);

private:
    // Receiver task
    static void receiverTaskFunction(ULONG parameter);

    // Message handling methods
    void handlePvtMessage(const UbxMessage& msg);
    void handleSatMessage(const UbxMessage& msg);
    void handleRawMeasMessage(const UbxMessage& msg);
    void handleHwStatusMessage(const UbxMessage& msg);
    void handleVersionMessage(const UbxMessage& msg);
    void handleAckMessage(const UbxMessage& msg);
    void handleNakMessage(const UbxMessage& msg);
    void handleEphemerisMessage(const UbxMessage& msg);
    void handleAlmanacMessage(const UbxMessage& msg);

    // Helper methods
    bool sendUbxMessage(uint8_t msgClass, uint8_t msgId, const uint8_t* payload, size_t length, ULONG timeout);
    bool waitForAck(uint8_t msgClass, uint8_t msgId, ULONG timeout);
    void reportError(const std::string& errorMsg, int errorCode);

    // Private data
    UART* uart;
    GPIO* power_pin;
    Config config;
    std::unique_ptr<UbxProtocol> protocol;

    // Callback storage
    PvtCallback pvtCallback;
    SatelliteCallback satelliteCallback;
    RawMeasurementCallback rawMeasCallback;
    ReceiverStatusCallback statusCallback;
    EphemerisCallback ephemerisCallback;
    AlmanacCallback almanacCallback;
    VersionCallback versionCallback;
    ErrorCallback errorCallback;

    // Message waiting mechanisms
    TX_SEMAPHORE ackSemaphore;
    TX_SEMAPHORE nakSemaphore;

    // Latest data storage
    mutable TX_MUTEX dataMutex;
    PvtData latestPvt;
    std::vector<SatelliteData> latestSatellites;
    ReceiverStatus latestStatus;
    ReceiverVersion receiverVersion;
    std::unordered_map<uint16_t, EphemerisData> ephemerisData; // key = gnssId << 8 | svId
    std::unordered_map<uint16_t, AlmanacData> almanacData;     // key = gnssId << 8 | svId

    // Last command info for ACK/NAK handling
    uint8_t lastCmdClass;
    uint8_t lastCmdId;
    bool ackReceived;
    bool nakReceived;

    // Task information
    TX_THREAD receiverTaskHandle;
    bool taskRunning;
    static constexpr size_t STACK_SIZE = 4096;
    uint8_t receiverTaskStack[STACK_SIZE];

    // Message reception buffer
    static constexpr size_t RX_BUFFER_SIZE = 1024;
    uint8_t rxBuffer[RX_BUFFER_SIZE];
};

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
private:
	static const size_t max_payload_leng = 2048;
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
        T value;
        memcpy(&value, payload + offset, sizeof(T));
        return value;
    }

    template<typename T>
    void getPayloadField(size_t offset, T& value) const
    {
        memcpy(&value, payload + offset, sizeof(T));
    }

private:
    uint8_t msgClass{0};
    uint8_t msgId{0};
    uint8_t payload[max_payload_leng]; // Maximum payload size;
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
	static constexpr size_t MAX_PAYLOAD_SIZE = 4096;
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
    uint8_t payloadBuffer[MAX_PAYLOAD_SIZE];
    uint16_t actualLength{0};
};


#endif /* __UBXGNSSDRIVER_HPP_ */
