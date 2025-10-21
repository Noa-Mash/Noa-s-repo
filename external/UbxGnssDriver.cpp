/*
 * UbxGnssDriver.cpp
 *
 *  Created on: May 14, 2025
 *      Author: David Michaeli
 */

#include "UbxGnssDriver.hpp"

UbxGnssDriver::UbxGnssDriver()
    : uart(nullptr), config(), taskRunning(false) {
    // Create the protocol handler using direct constructor instead of make_unique
    protocol = std::unique_ptr<UbxProtocol>(new UbxProtocol());

    // Initialize ThreadX objects
    tx_semaphore_create(&ackSemaphore, const_cast<char*>("ubx_ack_sem"), 0);
    tx_semaphore_create(&nakSemaphore, const_cast<char*>("ubx_nak_sem"), 0);
    tx_mutex_create(&dataMutex, const_cast<char*>("ubx_data_mutex"), TX_NO_INHERIT);

    // Initialize private data
    ackReceived = false;
    nakReceived = false;
    lastCmdClass = 0;
    lastCmdId = 0;
    power_pin = nullptr;
}

UbxGnssDriver::~UbxGnssDriver() {
    // Stop task if running
    stopReceiverTask();

    // Delete ThreadX objects
    tx_semaphore_delete(&ackSemaphore);
    tx_semaphore_delete(&nakSemaphore);
    tx_mutex_delete(&dataMutex);
}

bool UbxGnssDriver::init(UART* uart, GPIO* power_output_pin, const Config& config, ULONG timeout)
{
	// assign the used UART instance
	this->uart = uart;
	this->power_pin = power_output_pin;

	// Make sure the initial configuration is 9600
	UART::Config uartConfig1(9600, config.useDma, false, config.rxBufferSize, config.txBufferSize);
	uart->set_config(uartConfig1);
	tx_thread_sleep(10);

	protocol->reset();

	// Reset the GNSS receiver (soft reset) at this point the device will be reset into 9600 bps
	if (!resetReceiver(false, timeout, true))
	{
		reportError("Failed to reset receiver", 1);
		return false;
	}

	// Set port configuration (UART)
	PortConfiguration prt_cfg;
	prt_cfg.port_id = 1;
	prt_cfg.res1 = 0;
	prt_cfg.tx_ready_bitmask = 0;
	prt_cfg.mode_bitmask = (1<<4) | (0x3 << 6) | (0x4 << 9) | (0x00 << 12),
	prt_cfg.baudrate = config.baudRate;
	prt_cfg.in_proto_mask = 0x01;
	prt_cfg.out_proto_mask = 0x01;
	prt_cfg.flags = 0x00;
	prt_cfg.res2 = 0;
	prt_cfg.res3 = 0;
	if (!sendUbxMessage(UbxConstants::UBX_CLASS_CFG, UbxConstants::UBX_CFG_PRT, (const uint8_t*)&prt_cfg, sizeof(prt_cfg), timeout))
	{
		reportError("Failed to configure port", 2);
		return false;
	}
	// wait for the configuration to act
	tx_thread_sleep(100);

    // Re-Configure UART new baudrate
    UART::Config uartConfig2(config.baudRate, config.useDma, false, config.rxBufferSize, config.txBufferSize);
    uart->set_config(uartConfig2);

    startReceiverTask();
    tx_thread_sleep(30);

    // Configure message rates
    if (!configureMessageRates(1, 1, 1, timeout))
    {
    	reportError("Failed to configure message rates", 6);
    	//return false;
    }
    tx_thread_sleep(50);

    // Set navigation rate
    if (!configureNavigationRate(100, timeout))
    { // 10 Hz (100ms)
        reportError("Failed to set navigation rate", 4);
        //return false;
    }

    // Enable default constellations (GPS, GALILEO, GLONASS)
//    if (!configureConstellation(Constellation::GPS, true, timeout) ||
//        !configureConstellation(Constellation::GALILEO, true, timeout) ||
//        !configureConstellation(Constellation::GLONASS, true, timeout)) {
//        reportError("Failed to configure constellations", 5);
//        //return false;
//    }

    // Request receiver version
    if (!requestReceiverVersion(timeout))
    {
        reportError("Failed to request receiver version", 7);
        // Continue anyway
    }

    return true;
}

bool UbxGnssDriver::requestEphemeris(uint8_t gnssId, uint8_t svId, ULONG timeout) {
    uint8_t aidEphPayload[8] = {
        gnssId,
        svId,
        0, // Reserved
        0, // Reserved
        0, 0, 0, 0 // Reserved
    };

    return sendUbxMessage(UbxConstants::UBX_CLASS_AID, UbxConstants::UBX_AID_EPH,
                         aidEphPayload, sizeof(aidEphPayload), timeout);
}

bool UbxGnssDriver::requestAlmanac(uint8_t gnssId, uint8_t svId, ULONG timeout) {
    uint8_t aidAlmPayload[8] = {
        gnssId,
        svId,
        0, // Reserved
        0, // Reserved
        0, 0, 0, 0 // Reserved
    };

    return sendUbxMessage(UbxConstants::UBX_CLASS_AID, UbxConstants::UBX_AID_ALM,
                         aidAlmPayload, sizeof(aidAlmPayload), timeout);
}

bool UbxGnssDriver::requestReceiverVersion(ULONG timeout) {
    // MON-VER message doesn't require a payload
    return sendUbxMessage(UbxConstants::UBX_CLASS_MON, UbxConstants::UBX_MON_VER,
                         nullptr, 0, timeout);
}

bool UbxGnssDriver::configureNavigationRate(uint16_t measRateMs, ULONG timeout) {
    // CFG-RATE message payload
    uint8_t cfgRatePayload[6] = {0};

    // measRate (measurement rate in ms)
    cfgRatePayload[0] = measRateMs & 0xFF;
    cfgRatePayload[1] = (measRateMs >> 8) & 0xFF;

    // navRate (navigation rate, ratio to measurement rate, typically 1)
    cfgRatePayload[2] = 0x01;
    cfgRatePayload[3] = 0x00;

    // timeRef (time reference: 0=UTC, 1=GPS time)
    cfgRatePayload[4] = 0x01; // GPS time
    cfgRatePayload[5] = 0x00;

    if (!sendUbxMessage(UbxConstants::UBX_CLASS_CFG, UbxConstants::UBX_CFG_RATE,
                        cfgRatePayload, sizeof(cfgRatePayload), timeout)) {
        reportError("Failed to send navigation rate configuration", 400);
        return false;
    }
    return true;
}

bool UbxGnssDriver::configureConstellation(Constellation constellation, bool enable, ULONG timeout) {
    // CFG-GNSS message payload structure
    uint8_t cfgGnssPayload[12] = {0};

    // Header
    cfgGnssPayload[0] = 0x00; // msgVer
    cfgGnssPayload[1] = 0x20; // numTrkChHw (32 tracking channels)
    cfgGnssPayload[2] = 0xFF; // numTrkChUse (255 = use all available)
    cfgGnssPayload[3] = 0x01; // numConfigBlocks (1 block)

    // Configuration block for the specified constellation
    uint8_t gnssId = 0;
    uint8_t resTrkCh = 8;  // Reserved tracking channels
    uint8_t maxTrkCh = 16; // Maximum tracking channels
    uint32_t flags = 0x01010001; // Default flags: enable L1C/A

    switch (constellation) {
        case Constellation::GPS:
            gnssId = UbxConstants::UBX_GNSS_ID_GPS;
            resTrkCh = 8;
            maxTrkCh = 16;
            flags = enable ? 0x01010001 : 0x01010000; // Enable/disable GPS with L1C/A
            break;

        case Constellation::SBAS:
            gnssId = UbxConstants::UBX_GNSS_ID_SBAS;
            resTrkCh = 1;
            maxTrkCh = 4;
            flags = enable ? 0x01010001 : 0x01010000;
            break;

        case Constellation::GALILEO:
            gnssId = UbxConstants::UBX_GNSS_ID_GALILEO;
            resTrkCh = 4;
            maxTrkCh = 8;
            flags = enable ? 0x01010001 : 0x01010000; // Enable/disable Galileo with E1
            break;

        case Constellation::BEIDOU:
            gnssId = UbxConstants::UBX_GNSS_ID_BEIDOU;
            resTrkCh = 2;
            maxTrkCh = 16;
            flags = enable ? 0x01010001 : 0x01010000;
            break;

        case Constellation::IMES:
            gnssId = UbxConstants::UBX_GNSS_ID_IMES;
            resTrkCh = 0;
            maxTrkCh = 8;
            flags = enable ? 0x01010001 : 0x01010000;
            break;

        case Constellation::QZSS:
            gnssId = UbxConstants::UBX_GNSS_ID_QZSS;
            resTrkCh = 0;
            maxTrkCh = 3;
            flags = enable ? 0x01010001 : 0x01010000;
            break;

        case Constellation::GLONASS:
            gnssId = UbxConstants::UBX_GNSS_ID_GLONASS;
            resTrkCh = 8;
            maxTrkCh = 14;
            flags = enable ? 0x01010001 : 0x01010000; // Enable/disable GLONASS with L1
            break;

        case Constellation::NAVIC:
            gnssId = UbxConstants::UBX_GNSS_ID_NAVIC;
            resTrkCh = 0;
            maxTrkCh = 8;
            flags = enable ? 0x01010001 : 0x01010000;
            break;

        default:
            reportError("Unknown constellation type", 401);
            return false;
    }

    // Fill configuration block
    cfgGnssPayload[4] = gnssId;
    cfgGnssPayload[5] = resTrkCh;
    cfgGnssPayload[6] = maxTrkCh;
    cfgGnssPayload[7] = 0x00; // Reserved

    // Flags (little endian)
    cfgGnssPayload[8] = flags & 0xFF;
    cfgGnssPayload[9] = (flags >> 8) & 0xFF;
    cfgGnssPayload[10] = (flags >> 16) & 0xFF;
    cfgGnssPayload[11] = (flags >> 24) & 0xFF;

    if (!sendUbxMessage(UbxConstants::UBX_CLASS_CFG, UbxConstants::UBX_CFG_GNSS,
                        cfgGnssPayload, sizeof(cfgGnssPayload), timeout)) {
        reportError("Failed to send constellation configuration", 402);
        return false;
    }

    return true;
}

bool UbxGnssDriver::configureMessageRates(uint8_t pvtRate, uint8_t satRate, uint8_t rawMeasRate, ULONG timeout) {
    bool success = true;

    // Configure NAV-PVT message rate
    uint8_t pvtMsgConfig[3] =
    {
        UbxConstants::UBX_CLASS_NAV, // Message class
        UbxConstants::UBX_NAV_PVT,   // Message ID
        pvtRate                       // Rate (0=disabled, 1=once per nav solution)
    };

    if (!sendUbxMessage(UbxConstants::UBX_CLASS_CFG, UbxConstants::UBX_CFG_MSG,
                        pvtMsgConfig, sizeof(pvtMsgConfig), timeout))
    {
        reportError("Failed to configure PVT message rate", 403);
        success = false;
    }

    // Configure NAV-SAT message rate
    uint8_t satMsgConfig[3] =
    {
        UbxConstants::UBX_CLASS_NAV, // Message class
        UbxConstants::UBX_NAV_SAT,   // Message ID
        satRate                       // Rate
    };

    if (success)
    {
        if (!sendUbxMessage(UbxConstants::UBX_CLASS_CFG, UbxConstants::UBX_CFG_MSG,
                            satMsgConfig, sizeof(satMsgConfig), timeout))
        {
            reportError("Failed to configure SAT message rate", 405);
            success = false;
        }
    }

    // Configure RXM-RAWX message rate (raw measurements)
    uint8_t rawMsgConfig[3] = {
        UbxConstants::UBX_CLASS_RXM, // Message class
        UbxConstants::UBX_RXM_RAWX,  // Message ID
        rawMeasRate                   // Rate
    };

    if (success)
    {
        if (!sendUbxMessage(UbxConstants::UBX_CLASS_CFG, UbxConstants::UBX_CFG_MSG,
                            rawMsgConfig, sizeof(rawMsgConfig), timeout))
        {
            reportError("Failed to configure RAWX message rate", 407);
            success = false;
        }
    }

    return success;
}

// Also implement the callback registration functions that might be missing
void UbxGnssDriver::registerPvtCallback(PvtCallback callback)
{
    pvtCallback = callback;
}

void UbxGnssDriver::registerSatelliteCallback(SatelliteCallback callback)
{
    satelliteCallback = callback;
}

void UbxGnssDriver::registerRawMeasurementCallback(RawMeasurementCallback callback)
{
    rawMeasCallback = callback;
}

void UbxGnssDriver::registerReceiverStatusCallback(ReceiverStatusCallback callback)
{
    statusCallback = callback;
}

void UbxGnssDriver::registerEphemerisCallback(EphemerisCallback callback)
{
    ephemerisCallback = callback;
}

void UbxGnssDriver::registerAlmanacCallback(AlmanacCallback callback)
{
    almanacCallback = callback;
}

void UbxGnssDriver::registerVersionCallback(VersionCallback callback)
{
    versionCallback = callback;
}

void UbxGnssDriver::registerErrorCallback(ErrorCallback callback)
{
    errorCallback = callback;
}

// Additional configuration functions that might be needed
bool UbxGnssDriver::configureSignals(Constellation constellation, uint8_t signalMask, ULONG timeout)
{
    // This would configure specific signals for a constellation
    // Implementation depends on the specific UBX receiver version and capabilities

    // For now, return success (placeholder implementation)
    // You would need to implement CFG-SIG message handling for newer receivers
    // or use CFG-GNSS with appropriate signal flags for older receivers

    reportError("configureSignals not fully implemented", 409);
    return true; // Placeholder
}

bool UbxGnssDriver::configureGpsSignals(bool l1ca, bool l2c, bool l5, ULONG timeout)
{
    uint8_t signalMask = 0;

    if (l1ca) signalMask |= static_cast<uint8_t>(GpsSignal::L1CA);
    if (l2c) signalMask |= static_cast<uint8_t>(GpsSignal::L2C);
    if (l5) signalMask |= static_cast<uint8_t>(GpsSignal::L5);

    return configureSignals(Constellation::GPS, signalMask, timeout);
}

bool UbxGnssDriver::configureGalileoSignals(bool e1, bool e5a, bool e5b, ULONG timeout)
{
    uint8_t signalMask = 0;

    if (e1) signalMask |= static_cast<uint8_t>(GalileoSignal::E1);
    if (e5a) signalMask |= static_cast<uint8_t>(GalileoSignal::E5a);
    if (e5b) signalMask |= static_cast<uint8_t>(GalileoSignal::E5b);

    return configureSignals(Constellation::GALILEO, signalMask, timeout);
}

bool UbxGnssDriver::configureBeidouSignals(bool b1i, bool b2i, bool b2a, ULONG timeout)
{
    uint8_t signalMask = 0;

    if (b1i) signalMask |= static_cast<uint8_t>(BeidouSignal::B1I);
    if (b2i) signalMask |= static_cast<uint8_t>(BeidouSignal::B2I);
    if (b2a) signalMask |= static_cast<uint8_t>(BeidouSignal::B2A);

    return configureSignals(Constellation::BEIDOU, signalMask, timeout);
}

bool UbxGnssDriver::configureGlonassSignals(bool l1, bool l2, ULONG timeout)
{
    uint8_t signalMask = 0;

    if (l1) signalMask |= static_cast<uint8_t>(GlonassSignal::L1);
    if (l2) signalMask |= static_cast<uint8_t>(GlonassSignal::L2);

    return configureSignals(Constellation::GLONASS, signalMask, timeout);
}

bool UbxGnssDriver::configurePowerMode(PowerMode mode, uint16_t cycleTimeSeconds, uint16_t onTimeSeconds, ULONG timeout)
{
    // CFG-PM2 message for power management configuration
    uint8_t cfgPm2Payload[44] = {0}; // Initialize all to zero

    // Version and reserved fields
    cfgPm2Payload[0] = 0x02; // version = 2
    cfgPm2Payload[1] = 0x00; // reserved1
    cfgPm2Payload[2] = 0x00; // reserved2
    cfgPm2Payload[3] = 0x00; // reserved3

    // Power mode configuration
    uint32_t flags = 0x00000000;
    uint32_t updatePeriod = 0;
    uint32_t searchPeriod = 0;
    uint32_t gridOffset = 0;
    uint16_t onTime = 0;
    uint16_t minAcqTime = 0;

    switch (mode) {
        case PowerMode::FULL_POWER:
            // Full power mode - continuous operation
            flags = 0x00000000;
            break;

        case PowerMode::BALANCED:
            // Balanced power mode
            flags = 0x00000001; // Power save mode enabled
            updatePeriod = 1000; // 1 second
            searchPeriod = 10000; // 10 seconds
            onTime = 1000; // 1 second on time
            break;

        case PowerMode::INTERVAL:
            // Interval mode with custom timing
            flags = 0x00000001; // Power save mode enabled
            updatePeriod = cycleTimeSeconds * 1000; // Convert to ms
            searchPeriod = cycleTimeSeconds * 1000;
            onTime = onTimeSeconds * 1000; // Convert to ms
            break;

        case PowerMode::AGGRESSIVE_1HZ:
            // Aggressive 1Hz mode
            flags = 0x00000001;
            updatePeriod = 1000; // 1 second
            searchPeriod = 2000; // 2 seconds
            onTime = 500; // 0.5 second on time
            break;

        case PowerMode::AGGRESSIVE_2HZ:
            // Aggressive 2Hz mode
            flags = 0x00000001;
            updatePeriod = 500; // 0.5 seconds
            searchPeriod = 1000; // 1 second
            onTime = 250; // 0.25 second on time
            break;

        case PowerMode::AGGRESSIVE_4HZ:
            // Aggressive 4Hz mode
            flags = 0x00000001;
            updatePeriod = 250; // 0.25 seconds
            searchPeriod = 500; // 0.5 seconds
            onTime = 125; // 0.125 second on time
            break;
    }

    // Fill the payload
    // Flags (bytes 4-7)
    cfgPm2Payload[4] = flags & 0xFF;
    cfgPm2Payload[5] = (flags >> 8) & 0xFF;
    cfgPm2Payload[6] = (flags >> 16) & 0xFF;
    cfgPm2Payload[7] = (flags >> 24) & 0xFF;

    // Update period (bytes 8-11)
    cfgPm2Payload[8] = updatePeriod & 0xFF;
    cfgPm2Payload[9] = (updatePeriod >> 8) & 0xFF;
    cfgPm2Payload[10] = (updatePeriod >> 16) & 0xFF;
    cfgPm2Payload[11] = (updatePeriod >> 24) & 0xFF;

    // Search period (bytes 12-15)
    cfgPm2Payload[12] = searchPeriod & 0xFF;
    cfgPm2Payload[13] = (searchPeriod >> 8) & 0xFF;
    cfgPm2Payload[14] = (searchPeriod >> 16) & 0xFF;
    cfgPm2Payload[15] = (searchPeriod >> 24) & 0xFF;

    // Grid offset (bytes 16-19)
    cfgPm2Payload[16] = gridOffset & 0xFF;
    cfgPm2Payload[17] = (gridOffset >> 8) & 0xFF;
    cfgPm2Payload[18] = (gridOffset >> 16) & 0xFF;
    cfgPm2Payload[19] = (gridOffset >> 24) & 0xFF;

    // On time (bytes 20-21)
    cfgPm2Payload[20] = onTime & 0xFF;
    cfgPm2Payload[21] = (onTime >> 8) & 0xFF;

    // Min acquisition time (bytes 22-23)
    cfgPm2Payload[22] = minAcqTime & 0xFF;
    cfgPm2Payload[23] = (minAcqTime >> 8) & 0xFF;

    // Remaining bytes are reserved (already initialized to 0)

    if (!sendUbxMessage(UbxConstants::UBX_CLASS_CFG, UbxConstants::UBX_CFG_PM2, cfgPm2Payload, sizeof(cfgPm2Payload), timeout))
    {
        reportError("Failed to send power mode configuration", 410);
        return false;
    }

    return waitForAck(UbxConstants::UBX_CLASS_CFG, UbxConstants::UBX_CFG_PM2, timeout);
}

bool UbxGnssDriver::resetReceiver(bool hardReset, ULONG timeout, bool usingIO)
{
	if (usingIO && power_pin != nullptr)
	{
		power_pin->reset();
		if (tx_thread_identify() != TX_NULL) tx_thread_sleep(10);
		else HAL_Delay(10);
		power_pin->set();
		if (tx_thread_identify() != TX_NULL) tx_thread_sleep(500);
		else HAL_Delay(500);
		return true;
	}
	else
	{
		uint8_t cfgRstPayload[4] = {
			0x00, 0x00, // Watchdog reset (hot start)
			0x01, // Reset mode - controlled software reset
			0x00  // Reserved
		};

		if (hardReset) {
			// Hardware reset
			cfgRstPayload[2] = 0x00; // Hardware reset
		}

		return sendUbxMessage(UbxConstants::UBX_CLASS_CFG, UbxConstants::UBX_CFG_RST,
							 cfgRstPayload, sizeof(cfgRstPayload), timeout);
	}
}

bool UbxGnssDriver::startReceiverTask()
{
    if (taskRunning)
    {
        return true; // Task already running
    }

    // Create and start the receiver task
    UINT result = tx_thread_create(&receiverTaskHandle,
                                  const_cast<char*>("ubx_rcvr_task"),
                                  receiverTaskFunction,
                                  reinterpret_cast<ULONG>(this),
                                  receiverTaskStack,
                                  STACK_SIZE,
                                  10, // Priority
                                  10, // Preempt threshold
                                  TX_NO_TIME_SLICE,
                                  TX_AUTO_START);

    if (result != TX_SUCCESS)
    {
        reportError("Failed to create receiver task", static_cast<int>(result));
        return false;
    }

    taskRunning = true;
    return true;
}

void UbxGnssDriver::stopReceiverTask()
{
    if (!taskRunning)
    {
        return; // Task not running
    }

    // Terminate the task
    tx_thread_terminate(&receiverTaskHandle);
    tx_thread_delete(&receiverTaskHandle);

    taskRunning = false;
}

UbxGnssDriver::PvtData UbxGnssDriver::getLatestPvtData() const
{
    tx_mutex_get(&dataMutex, TX_WAIT_FOREVER);
    PvtData data = latestPvt;
    tx_mutex_put(&dataMutex);
    return data;
}

std::vector<UbxGnssDriver::SatelliteData> UbxGnssDriver::getLatestSatelliteData() const
{
    tx_mutex_get(&dataMutex, TX_WAIT_FOREVER);
    std::vector<SatelliteData> data = latestSatellites;
    tx_mutex_put(&dataMutex);
    return data;
}

UbxGnssDriver::ReceiverStatus UbxGnssDriver::getLatestReceiverStatus() const
{
    tx_mutex_get(&dataMutex, TX_WAIT_FOREVER);
    ReceiverStatus status = latestStatus;
    tx_mutex_put(&dataMutex);
    return status;
}

UbxGnssDriver::ReceiverVersion UbxGnssDriver::getReceiverVersion() const
{
    tx_mutex_get(&dataMutex, TX_WAIT_FOREVER);
    ReceiverVersion version = receiverVersion;
    tx_mutex_put(&dataMutex);
    return version;
}

void UbxGnssDriver::processUartData(const uint8_t* data, size_t length)
{
    if (data == nullptr || length == 0)
    {
        return;
    }

    protocol->processData(data, length, [this](const UbxMessage& msg)
    		{
				// Process parsed UBX message
				uint8_t msgClass = msg.getClass();
				uint8_t msgId = msg.getId();

				switch (msgClass) {
					case UbxConstants::UBX_CLASS_NAV:
						if (msgId == UbxConstants::UBX_NAV_PVT)
						{
							handlePvtMessage(msg);
						}
						else if (msgId == UbxConstants::UBX_NAV_SAT)
						{
							handleSatMessage(msg);
						}
						break;

					case UbxConstants::UBX_CLASS_RXM:
						if (msgId == UbxConstants::UBX_RXM_RAWX)
						{
							handleRawMeasMessage(msg);
						}
						break;

					case UbxConstants::UBX_CLASS_MON:
						if (msgId == UbxConstants::UBX_MON_HW)
						{
							handleHwStatusMessage(msg);
						}
						else if (msgId == UbxConstants::UBX_MON_VER)
						{
							handleVersionMessage(msg);
						}
						break;

					case UbxConstants::UBX_CLASS_ACK:
						if (msgId == UbxConstants::UBX_ACK_ACK)
						{
							handleAckMessage(msg);
						}
						else if (msgId == UbxConstants::UBX_ACK_NAK)
						{
							handleNakMessage(msg);
						}
						break;

					case UbxConstants::UBX_CLASS_AID:
						if (msgId == UbxConstants::UBX_AID_EPH)
						{
							handleEphemerisMessage(msg);
						}
						else if (msgId == UbxConstants::UBX_AID_ALM)
						{
							handleAlmanacMessage(msg);
						}
						break;
				}
			});
}

void UbxGnssDriver::receiverTaskFunction(ULONG parameter)
{
	UbxGnssDriver* driver = reinterpret_cast<UbxGnssDriver*>(parameter);
	uint8_t buffer[512];

	while (driver->taskRunning)
	{
		// Check how much data is available
		size_t available = driver->uart->available();

		if (available > 0)
		{
			// Read available data (up to buffer size)
			size_t toRead = (available > sizeof(buffer)) ? sizeof(buffer) : available;

			StreamIO::Result result = driver->uart->read(0, buffer, toRead, 100); // Short timeout

			if (result == StreamIO::Result::SUCCESS)
			{
				// Process the received data
				driver->processUartData(buffer, toRead);
			}
			else if (result != StreamIO::Result::TIMEOUT)
			{
				// Report error if not timeout
				driver->reportError("UART read error", static_cast<int>(result));
				tx_thread_sleep(100); // Short delay before retrying
			}
		}
		tx_thread_sleep(10);
	}
}

void UbxGnssDriver::handlePvtMessage(const UbxMessage& msg)
{
    if (msg.getLength() < 92)
    {
        // Invalid message length
        return;
    }

    // Lock mutex for data access
    tx_mutex_get(&dataMutex, TX_WAIT_FOREVER);

	// Extract PVT data from message
	latestPvt.iTOW = msg.getPayloadField<uint32_t>(0);
	latestPvt.year = msg.getPayloadField<uint16_t>(4);
	latestPvt.month = msg.getPayloadField<uint8_t>(6);
	latestPvt.day = msg.getPayloadField<uint8_t>(7);
	latestPvt.hour = msg.getPayloadField<uint8_t>(8);
	latestPvt.minute = msg.getPayloadField<uint8_t>(9);
	latestPvt.second = msg.getPayloadField<uint8_t>(10);

	uint8_t validFlags = msg.getPayloadField<uint8_t>(11);
	latestPvt.validDate = (validFlags & 0x01) != 0;
	latestPvt.validTime = (validFlags & 0x02) != 0;
	latestPvt.fullyResolved = (validFlags & 0x04) != 0;

	latestPvt.tAccNs = msg.getPayloadField<uint32_t>(12);
	latestPvt.nanoSecond = msg.getPayloadField<int32_t>(16);

	uint8_t fixType = msg.getPayloadField<uint8_t>(20);
	latestPvt.fixType = static_cast<FixType>(fixType);

	uint8_t flags = msg.getPayloadField<uint8_t>(21);
	latestPvt.gnssFixOK = (flags & 0x01) != 0;
	latestPvt.diffSoln = (flags & 0x02) != 0;

	latestPvt.numSV = msg.getPayloadField<uint8_t>(23);

	int32_t lon = msg.getPayloadField<int32_t>(24);
	int32_t lat = msg.getPayloadField<int32_t>(28);
	int32_t height = msg.getPayloadField<int32_t>(32);
	int32_t heightMSL = msg.getPayloadField<int32_t>(36);

	latestPvt.longitude = static_cast<float>(lon) * 1e-7;
	latestPvt.latitude = static_cast<float>(lat) * 1e-7;
	latestPvt.height = static_cast<float>(height) * 1e-3;
	latestPvt.heightMSL = static_cast<float>(heightMSL) * 1e-3;

	uint32_t hAcc = msg.getPayloadField<uint32_t>(40);
	uint32_t vAcc = msg.getPayloadField<uint32_t>(44);

	latestPvt.hAcc = static_cast<float>(hAcc) * 1e-3;
	latestPvt.vAcc = static_cast<float>(vAcc) * 1e-3;

	int32_t velN = msg.getPayloadField<int32_t>(48);
	int32_t velE = msg.getPayloadField<int32_t>(52);
	int32_t velD = msg.getPayloadField<int32_t>(56);
	int32_t gSpeed = msg.getPayloadField<int32_t>(60);
	int32_t headMotion = msg.getPayloadField<int32_t>(64);

	latestPvt.velN = static_cast<float>(velN) * 1e-3;
	latestPvt.velE = static_cast<float>(velE) * 1e-3;
	latestPvt.velD = static_cast<float>(velD) * 1e-3;
	latestPvt.groundSpeed = static_cast<float>(gSpeed) * 1e-3;
	latestPvt.headingMotion = static_cast<float>(headMotion) * 1e-5;

	uint32_t sAcc = msg.getPayloadField<uint32_t>(68);
	uint32_t headAcc = msg.getPayloadField<uint32_t>(72);

	latestPvt.sAcc = static_cast<float>(sAcc) * 1e-3;
	latestPvt.headingAcc = static_cast<float>(headAcc) * 1e-5;

	uint16_t pDOP = msg.getPayloadField<uint16_t>(76);
	latestPvt.pDOP = static_cast<float>(pDOP) * 0.01;

	// Optional: magnetic declination if available (not all receivers provide this)
	if (msg.getLength() >= 88)
	{
		int16_t magDec = msg.getPayloadField<int16_t>(84);
		uint16_t magAcc = msg.getPayloadField<uint16_t>(86);

		latestPvt.magDec = static_cast<float>(magDec) * 1e-2;
		latestPvt.magAcc = static_cast<float>(magAcc) * 1e-2;
	}

    // Make a copy of the data for the callback
    PvtData pvtCopy = latestPvt;

    // Release mutex
    tx_mutex_put(&dataMutex);

    // Notify callback
    if (pvtCallback) {
        pvtCallback(pvtCopy);
    }
}

void UbxGnssDriver::handleSatMessage(const UbxMessage& msg)
{
    if (msg.getLength() < 8)
    {
        // Invalid message length
        return;
    }

	uint8_t numSvs = msg.getPayloadField<uint8_t>(5);
	size_t expectedLength = 8 + (numSvs * 12);

	if (msg.getLength() < expectedLength)
	{
		// Invalid message length
		return;
	}

	// Prepare satellite data vector
	std::vector<SatelliteData> satellites;
	satellites.reserve(numSvs);

	// Process each satellite
	for (uint8_t i = 0; i < numSvs; ++i)
	{
		size_t offset = 8 + (i * 12);

		SatelliteData sat;
		sat.gnssId = msg.getPayloadField<uint8_t>(offset);
		sat.svId = msg.getPayloadField<uint8_t>(offset + 1);
		sat.cno = msg.getPayloadField<uint8_t>(offset + 2);
		sat.elevation = msg.getPayloadField<int8_t>(offset + 3);
		sat.azimuth = msg.getPayloadField<int16_t>(offset + 4);
		sat.prRes = msg.getPayloadField<int16_t>(offset + 6);

		uint32_t flags = msg.getPayloadField<uint32_t>(offset + 8);
		sat.used = (flags & 0x08) != 0; // Bit 3: Used for navigation
		sat.signalQuality = (flags & 0x07); // Bits 0-2: Signal quality indicator
		sat.healthyFlag = ((flags >> 4) & 0x03) == 1; // Bits 4-5: SV health (1=healthy)
		sat.diffCorr = ((flags >> 6) & 0x01) != 0; // Bit 6: Differential correction available
		sat.orbitAvail = ((flags >> 8) & 0x07) != 0; // Bits 8-10: Orbit source
		sat.ephAvail = ((flags >> 11) & 0x01) != 0; // Bit 11: Ephemeris available
		sat.almAvail = ((flags >> 12) & 0x01) != 0; // Bit 12: Almanac available

		satellites.push_back(sat);
	}

	// Lock mutex for data access
	tx_mutex_get(&dataMutex, TX_WAIT_FOREVER);

	// Update latest satellite data
	latestSatellites = satellites;

	// Make a copy for the callback
	std::vector<SatelliteData> satCopy = latestSatellites;

	// Release mutex
	tx_mutex_put(&dataMutex);

	// Notify callback
	if (satelliteCallback)
	{
		satelliteCallback(satCopy);
	}
}

void UbxGnssDriver::handleRawMeasMessage(const UbxMessage& msg)
{
    if (msg.getLength() < 16)
    {
        // Invalid message length
        return;
    }

	uint8_t numMeas = msg.getPayloadField<uint8_t>(11);
	size_t expectedLength = 16 + (numMeas * 32);

	if (msg.getLength() < expectedLength)
	{
		// Invalid message length
		return;
	}

	// Prepare raw measurement data vector
	std::vector<RawMeasurement> measurements;
	measurements.reserve(numMeas);

	// Process each measurement
	for (uint8_t i = 0; i < numMeas; ++i)
	{
		size_t offset = 16 + (i * 32);

		RawMeasurement meas;
		meas.gnssId = msg.getPayloadField<uint8_t>(offset);
		meas.svId = msg.getPayloadField<uint8_t>(offset + 1);
		meas.sigId = msg.getPayloadField<uint8_t>(offset + 2);

		uint8_t flags = msg.getPayloadField<uint8_t>(offset + 3);
		meas.prValid = (flags & 0x01) != 0; // Bit 0: Pseudorange valid
		meas.cpValid = (flags & 0x02) != 0; // Bit 1: Carrier phase valid
		meas.halfCycle = (flags & 0x04) != 0; // Bit 2: Half cycle valid
		meas.subHalfCycle = (flags & 0x08) != 0; // Bit 3: Half cycle subtracted

		meas.cno = msg.getPayloadField<uint8_t>(offset + 4);
		meas.lockTime = msg.getPayloadField<uint8_t>(offset + 5);

		meas.prStdev = (msg.getPayloadField<uint8_t>(offset + 6) & 0x0F); // Bits 0-3
		meas.cpStdev = (msg.getPayloadField<uint8_t>(offset + 6) >> 4); // Bits 4-7
		meas.doStdev = (msg.getPayloadField<uint8_t>(offset + 7) & 0x0F); // Bits 0-3

		// Double precision values
		float pseudorange = msg.getPayloadField<float>(offset + 8);
		float carrierPhase = msg.getPayloadField<float>(offset + 16);
		float doppler = msg.getPayloadField<float>(offset + 24);

		// Only store valid measurements
		if (meas.prValid || meas.cpValid)
		{
			meas.pseudorange = meas.prValid ? pseudorange : 0.0;
			meas.carrierPhase = meas.cpValid ? carrierPhase : 0.0;
			meas.doppler = doppler;

			measurements.push_back(meas);
		}
	}

	// Notify callback
	if (!measurements.empty() && rawMeasCallback)
	{
		rawMeasCallback(measurements);
	}
}

void UbxGnssDriver::handleHwStatusMessage(const UbxMessage& msg)
{
    if (msg.getLength() < 28)
    {
        // Invalid message length
        return;
    }

	// Lock mutex for data access
	tx_mutex_get(&dataMutex, TX_WAIT_FOREVER);

	// Extract hardware status
	latestStatus.antStatus = msg.getPayloadField<uint8_t>(20);
	latestStatus.antPower = msg.getPayloadField<uint8_t>(21);

	uint8_t flags = msg.getPayloadField<uint8_t>(22);
	latestStatus.rtcCalib = (flags & 0x01) != 0;
	latestStatus.safeboot = (flags & 0x02) != 0;
	latestStatus.jamState = (flags >> 2) & 0x03;

	latestStatus.noisePerMS = msg.getPayloadField<uint16_t>(16);
	latestStatus.agcCnt = msg.getPayloadField<uint16_t>(18);

	// CW jamming indicator if available
	if (msg.getLength() >= 68)
	{
		latestStatus.cwJamming = msg.getPayloadField<uint8_t>(52);
	}

	// Make a copy for the callback
	ReceiverStatus statusCopy = latestStatus;

	// Release mutex
	tx_mutex_put(&dataMutex);

	// Notify callback
	if (statusCallback)
	{
		statusCallback(statusCopy);
	}
}

void UbxGnssDriver::handleVersionMessage(const UbxMessage& msg)
{
    if (msg.getLength() < 40)
    {
        // Invalid message length
        return;
    }

	ReceiverVersion version;

	// Extract fixed-length fields (SW and HW version)
	const uint8_t* payload = msg.getPayload();

	// Extract null-terminated strings
	char swVer[30] = {0};
	char hwVer[30] = {0};
	std::strncpy(swVer, reinterpret_cast<const char*>(payload), 30);
	std::strncpy(hwVer, reinterpret_cast<const char*>(payload + 30), 10);

	version.swVersion = swVer;
	version.hwVersion = hwVer;

	// Extract extension fields (variable number)
	size_t offset = 40; // Start of extension section
	while (offset + 30 <= msg.getLength())
	{
		char extStr[30] = {0};
		std::strncpy(extStr, reinterpret_cast<const char*>(payload + offset), 30);
		version.extModules.push_back(std::string(extStr));
		offset += 30;
	}

	// Lock mutex for data access
	tx_mutex_get(&dataMutex, TX_WAIT_FOREVER);

	// Update version information
	receiverVersion = version;

	// Make a copy for the callback
	ReceiverVersion versionCopy = receiverVersion;

	// Release mutex
	tx_mutex_put(&dataMutex);

	// Notify callback
	if (versionCallback)
	{
		versionCallback(versionCopy);
	}
}

void UbxGnssDriver::handleAckMessage(const UbxMessage& msg)
{
    if (msg.getLength() < 2)
    {
        // Invalid message length
        return;
    }

    uint8_t ackClass = msg.getPayloadField<uint8_t>(0);
    uint8_t ackId = msg.getPayloadField<uint8_t>(1);

    // Check if this ACK matches the last command
    if (ackClass == lastCmdClass && ackId == lastCmdId)
    {
        ackReceived = true;
        tx_semaphore_put(&ackSemaphore);
    }
}

void UbxGnssDriver::handleNakMessage(const UbxMessage& msg)
{
    if (msg.getLength() < 2)
    {
        // Invalid message length
        return;
    }

    uint8_t nakClass = msg.getPayloadField<uint8_t>(0);
    uint8_t nakId = msg.getPayloadField<uint8_t>(1);

    // Check if this NAK matches the last command
    if (nakClass == lastCmdClass && nakId == lastCmdId)
    {
        nakReceived = true;
        tx_semaphore_put(&nakSemaphore);
    }

    // Report the NAK as an error
    std::string errorMsg = "Command NAK received: 0x" +
                          std::to_string(nakClass) + ",0x" + std::to_string(nakId);
    reportError(errorMsg, 200);
}

void UbxGnssDriver::handleEphemerisMessage(const UbxMessage& msg)
{
    if (msg.getLength() < 104)
    {
        // Invalid message length (minimum size for ephemeris)
        return;
    }

	uint8_t gnssId = msg.getPayloadField<uint8_t>(0);
	uint8_t svId = msg.getPayloadField<uint8_t>(1);

	// Check if ephemeris is valid
	bool valid = true;

	// Create ephemeris data structure
	EphemerisData eph;
	eph.gnssId = gnssId;
	eph.svId = svId;
	eph.valid = valid;

	// Extract ephemeris parameters based on GNSS type
	// Different GNSS systems have different ephemeris formats
	if (gnssId == UbxConstants::UBX_GNSS_ID_GPS)
	{
		// GPS ephemeris parameters
		eph.sigId = 0; // L1CA
		eph.toe = msg.getPayloadField<uint32_t>(8);
		eph.toc = eph.toe; // Same as toe for GPS
		eph.week = msg.getPayloadField<uint16_t>(4);

		// Clock correction parameters
		eph.af0 = msg.getPayloadField<float>(12);
		eph.af1 = msg.getPayloadField<float>(20);
		eph.af2 = msg.getPayloadField<float>(28);

		// Orbit parameters
		eph.e = msg.getPayloadField<float>(36);
		eph.sqrtA = msg.getPayloadField<float>(44);
		eph.omega0 = msg.getPayloadField<float>(52);
		eph.i0 = msg.getPayloadField<float>(60);
		eph.omega = msg.getPayloadField<float>(68);
		eph.m0 = msg.getPayloadField<float>(76);
		eph.deltaN = msg.getPayloadField<float>(84);
		eph.idot = msg.getPayloadField<float>(92);
		eph.omegaDot = msg.getPayloadField<float>(100);

		// Correction terms
		if (msg.getLength() >= 132)
		{
			eph.cuc = msg.getPayloadField<float>(108);
			eph.cus = msg.getPayloadField<float>(116);
			eph.crc = msg.getPayloadField<float>(124);
		}

		if (msg.getLength() >= 160)
		{
			eph.crs = msg.getPayloadField<float>(132);
			eph.cic = msg.getPayloadField<float>(140);
			eph.cis = msg.getPayloadField<float>(148);
			eph.tgd = msg.getPayloadField<float>(156);
		}

		// Health and accuracy if available
		if (msg.getLength() >= 168)
		{
			eph.ura = msg.getPayloadField<float>(164);
			eph.health = static_cast<float>(msg.getPayloadField<uint8_t>(168));
		}
	}
	else if (gnssId == UbxConstants::UBX_GNSS_ID_GALILEO)
	{
		// Galileo ephemeris has a different format
		// Similar extraction logic but with different offsets/fields
		// ...
	}
	else if (gnssId == UbxConstants::UBX_GNSS_ID_BEIDOU)
	{
		// BeiDou ephemeris has a different format
		// ...
	}
	else if (gnssId == UbxConstants::UBX_GNSS_ID_GLONASS)
	{
		// GLONASS ephemeris has a very different format
		// ...
	}

	// Lock mutex for data access
	tx_mutex_get(&dataMutex, TX_WAIT_FOREVER);

	// Store ephemeris data in map (using gnssId + svId as key)
	uint16_t satKey = (static_cast<uint16_t>(gnssId) << 8) | svId;
	ephemerisData[satKey] = eph;

	// Make a copy for the callback
	EphemerisData ephCopy = eph;

	// Release mutex
	tx_mutex_put(&dataMutex);

	// Notify callback
	if (ephemerisCallback)
	{
		ephemerisCallback(ephCopy);
	}
}

void UbxGnssDriver::handleAlmanacMessage(const UbxMessage& msg)
{
    if (msg.getLength() < 8)
    {
        // Invalid message length
        return;
    }

	uint8_t gnssId = msg.getPayloadField<uint8_t>(0);
	uint8_t svId = msg.getPayloadField<uint8_t>(1);

	// Check if almanac is valid
	bool valid = (msg.getLength() > 8);

	// Create almanac data structure
	AlmanacData alm;
	alm.gnssId = gnssId;
	alm.svId = svId;
	alm.valid = valid;

	if (valid) {
		// Extract almanac parameters based on GNSS type
		if (gnssId == UbxConstants::UBX_GNSS_ID_GPS)
		{
			// GPS almanac
			alm.week = msg.getPayloadField<uint16_t>(4);
			alm.toa = msg.getPayloadField<uint32_t>(8);
			alm.e = msg.getPayloadField<float>(12);
			alm.deltai = msg.getPayloadField<float>(20);
			alm.omegaDot = msg.getPayloadField<float>(28);
			alm.svHealth = msg.getPayloadField<float>(36);
			alm.sqrtA = msg.getPayloadField<float>(44);
			alm.omega0 = msg.getPayloadField<float>(52);
			alm.omega = msg.getPayloadField<float>(60);
			alm.m0 = msg.getPayloadField<float>(68);

			// Clock parameters
			if (msg.getLength() >= 84)
			{
				alm.af0 = msg.getPayloadField<float>(76);
				alm.af1 = msg.getPayloadField<float>(84);
			}
		}
		else if (gnssId == UbxConstants::UBX_GNSS_ID_GALILEO)
		{
			// Galileo almanac has a different format
			// ...
		}
		else if (gnssId == UbxConstants::UBX_GNSS_ID_BEIDOU)
		{
			// BeiDou almanac has a different format
			// ...
		}
		else if (gnssId == UbxConstants::UBX_GNSS_ID_GLONASS)
		{
			// GLONASS almanac has a very different format
			// ...
		}
	}

	// Lock mutex for data access
	tx_mutex_get(&dataMutex, TX_WAIT_FOREVER);

	// Store almanac data in map (using gnssId + svId as key)
	uint16_t satKey = (static_cast<uint16_t>(gnssId) << 8) | svId;
	almanacData[satKey] = alm;

	// Make a copy for the callback
	AlmanacData almCopy = alm;

	// Release mutex
	tx_mutex_put(&dataMutex);

	// Notify callback
	if (almanacCallback)
	{
		almanacCallback(almCopy);
	}
}

bool UbxGnssDriver::sendUbxMessage(uint8_t msgClass, uint8_t msgId,
                                  const uint8_t* payload, size_t length, ULONG timeout)
{
    // Prepare buffer for UBX message
    uint8_t buffer[1024]; // Max message size

    // Generate UBX message
    size_t msgSize = protocol->generateMessage(msgClass, msgId, payload, length,
                                             buffer, sizeof(buffer));
    if (msgSize == 0)
    {
        reportError("Failed to generate UBX message", 300);
        return false;
    }

    // Store command info for ACK/NAK handling
    lastCmdClass = msgClass;
    lastCmdId = msgId;
    ackReceived = false;
    nakReceived = false;

    // Send message via UART
    StreamIO::Result result = uart->write(0, buffer, msgSize, timeout);
    if (result != StreamIO::Result::SUCCESS)
    {
        reportError("Failed to send UBX message", static_cast<int>(result));
        return false;
    }

    return true;
}

bool UbxGnssDriver::waitForAck(uint8_t msgClass, uint8_t msgId, ULONG timeout)
{
    // Check if command requires ACK
    if (msgClass == UbxConstants::UBX_CLASS_CFG)
    {
        // Wait for ACK or NAK
        UINT status = tx_semaphore_get(&ackSemaphore, timeout);

        if (status != TX_SUCCESS)
        {
            reportError("Timeout waiting for acknowledgment", status);
            return false;
        }

        // Check if ACK was received
        return ackReceived && !nakReceived;
    }

    // Commands in other classes might not require ACK
    return true;
}

void UbxGnssDriver::reportError(const std::string& errorMsg, int errorCode)
{
    if (errorCallback)
    {
        errorCallback(errorMsg, errorCode);
    }
}


