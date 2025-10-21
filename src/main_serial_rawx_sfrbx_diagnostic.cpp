/*
 * main_serial_rawx_sfrbx_diagnostic.cpp
 * 
 * Enhanced version with UbxParser integration and diagnostics
 */

#include <stdio.h>
#include <signal.h>
#include <stdint.h>
#include <string.h>
#include <time.h>
#include <stdlib.h>
#include "ceserial.h"
#include "MiniUBX.hpp"
#include "updated_UBX_parser.hpp"

using namespace std;

volatile bool running = true;

// Color codes
#define COLOR_RESET   "\033[0m"
#define COLOR_RED     "\033[31m"
#define COLOR_GREEN   "\033[32m"
#define COLOR_YELLOW  "\033[33m"
#define COLOR_BLUE    "\033[34m"
#define COLOR_MAGENTA "\033[35m"
#define COLOR_CYAN    "\033[36m"

void signalHandler(int signum) {
    printf("\n%sReceived Ctrl+C, exiting...%s\n", COLOR_YELLOW, COLOR_RESET);
    running = false;
}

void printHexDump(const uint8_t* data, size_t length, const char* prefix = "") {
    printf("%s", prefix);
    for (size_t i = 0; i < length; i++) {
        printf("%02X ", data[i]);
        if ((i + 1) % 16 == 0) printf("\n%s", prefix);
    }
    if (length % 16 != 0) printf("\n");
}

const char* getMessageTypeName(uint8_t msgClass, uint8_t msgId) {
    if (msgClass == UbxConstants::UBX_CLASS_NAV) {
        switch (msgId) {
            case UbxConstants::UBX_NAV_PVT: return "NAV-PVT";
            case UbxConstants::UBX_NAV_SAT: return "NAV-SAT";
            case UbxConstants::UBX_NAV_STATUS: return "NAV-STATUS";
            default: return "NAV-OTHER";
        }
    } else if (msgClass == UbxConstants::UBX_CLASS_RXM) {
        switch (msgId) {
            case UbxConstants::UBX_RXM_RAWX: return "RXM-RAWX";
            case UbxConstants::UBX_RXM_SFRBX: return "RXM-SFRBX";
            default: return "RXM-OTHER";
        }
    } else if (msgClass == UbxConstants::UBX_CLASS_ACK) {
        switch (msgId) {
            case UbxConstants::UBX_ACK_ACK: return "ACK-ACK";
            case UbxConstants::UBX_ACK_NAK: return "ACK-NAK";
            default: return "ACK-OTHER";
        }
    } else if (msgClass == UbxConstants::UBX_CLASS_MON) {
        return "MON-*";
    } else if (msgClass == UbxConstants::UBX_CLASS_CFG) {
        if (msgId == 0x08) return "CFG-RATE";
        if (msgId == 0x01) return "CFG-MSG";
        return "CFG-OTHER";
    }
    return "UNKNOWN";
}

const char* getSystemName(int sys) {
    if (sys & SYS_GPS) return "GPS";
    if (sys & SYS_GLO) return "GLO";
    if (sys & SYS_GAL) return "GAL";
    if (sys & SYS_CMP) return "BDS";
    if (sys & SYS_QZS) return "QZS";
    if (sys & SYS_SBS) return "SBS";
    return "UNK";
}

void displayRawxSummary(const UbxMessage& msg, const obs_t& obs) {
    if (msg.getLength() < 16) {
        printf("%sInvalid RAWX length%s\n", COLOR_RED, COLOR_RESET);
        return;
    }
    
    double rcvTow = msg.getPayloadField<double>(0);
    uint16_t week = msg.getPayloadField<uint16_t>(8);
    uint8_t numMeas = msg.getPayloadField<uint8_t>(11);
    
    printf("%s=== RXM-RAWX ===%s\n", COLOR_CYAN, COLOR_RESET);
    printf("  Week: %u, TOW: %.3f s, Measurements: %u\n", week, rcvTow, numMeas);
    printf("  %sParsed Observations: %d%s\n", COLOR_GREEN, obs.n, COLOR_RESET);
    
    if (obs.n > 0) {
        printf("  %sFirst 5 observations:%s\n", COLOR_YELLOW, COLOR_RESET);
        for (int i = 0; i < obs.n && i < 5; i++) {
            int prn;
            int sys = satsys(obs.data[i].sat, &prn);
            printf("    [%d] %s%3d: PR=%13.3f L=%13.3f D=%7.2f SNR=%4.1f LLI=%d\n",
                   i, getSystemName(sys), prn,
                   obs.data[i].P[0], obs.data[i].L[0], obs.data[i].D[0],
                   obs.data[i].SNR[0] * 0.25, obs.data[i].LLI[0]);
        }
    }
}

void displaySfrbxSummary(const UbxMessage& msg, int parseResult) {
    if (msg.getLength() < 8) {
        printf("%sInvalid SFRBX length%s\n", COLOR_RED, COLOR_RESET);
        return;
    }
    
    uint8_t gnssId = msg.getPayloadField<uint8_t>(0);
    uint8_t svId = msg.getPayloadField<uint8_t>(1);
    uint8_t numWords = msg.getPayloadField<uint8_t>(2);
    
    const char* sysName = "UNK";
    switch (gnssId) {
        case 0: sysName = "GPS"; break;
        case 1: sysName = "SBS"; break;
        case 2: sysName = "GAL"; break;
        case 3: sysName = "BDS"; break;
        case 5: sysName = "QZS"; break;
        case 6: sysName = "GLO"; break;
    }
    
    printf("%s=== RXM-SFRBX ===%s\n", COLOR_CYAN, COLOR_RESET);
    printf("  System: %s, SV: %u, Words: %u\n", sysName, svId, numWords);
    
    if (parseResult == 2) {
        printf("  %s✓ Ephemeris decoded successfully%s\n", COLOR_GREEN, COLOR_RESET);
    } else if (parseResult == 9) {
        printf("  %s✓ Almanac/Iono decoded successfully%s\n", COLOR_GREEN, COLOR_RESET);
    } else if (parseResult > 0) {
        printf("  %sPartial decode (result=%d)%s\n", COLOR_YELLOW, parseResult, COLOR_RESET);
    } else {
        printf("  %s○ Waiting for more data...%s\n", COLOR_YELLOW, COLOR_RESET);
    }
}

void displayCfgRate(const UbxMessage& msg) {
    if (msg.getLength() >= 6) {
        uint16_t measRate = msg.getPayloadField<uint16_t>(0);
        uint16_t navRate = msg.getPayloadField<uint16_t>(2);
        uint16_t timeRef = msg.getPayloadField<uint16_t>(4);
        
        printf("%s=== CFG-RATE ===%s\n", COLOR_GREEN, COLOR_RESET);
        printf("  Measurement Rate: %u ms\n", measRate);
        printf("  Navigation Rate: %u cycles\n", navRate);
        printf("  Time Reference: %u (0=UTC, 1=GPS)\n", timeRef);
    }
}

void displayCfgMsg(const UbxMessage& msg) {
    if (msg.getLength() >= 8) {
        uint8_t msgClass = msg.getPayloadField<uint8_t>(0);
        uint8_t msgId = msg.getPayloadField<uint8_t>(1);
        uint8_t rate[6];
        for (int i = 0; i < 6; i++) {
            rate[i] = msg.getPayloadField<uint8_t>(2 + i);
        }
        
        printf("%s=== CFG-MSG Poll Response ===%s\n", COLOR_GREEN, COLOR_RESET);
        printf("  Message: Class=0x%02X, ID=0x%02X (%s)\n", 
               msgClass, msgId, getMessageTypeName(msgClass, msgId));
        printf("  Rates: DDC=%u, UART1=%u, UART2=%u, USB=%u, SPI=%u, Reserved=%u\n",
               rate[0], rate[1], rate[2], rate[3], rate[4], rate[5]);
    }
}

void displayMonVer(const UbxMessage& msg) {
    if (msg.getLength() < 40) return;
    
    char swVersion[31] = {0};
    char hwVersion[11] = {0};
    
    for (int i = 0; i < 30; i++) swVersion[i] = msg.getPayloadField<char>(i);
    for (int i = 0; i < 10; i++) hwVersion[i] = msg.getPayloadField<char>(30 + i);
    
    printf("%s=== MON-VER ===%s\n", COLOR_GREEN, COLOR_RESET);
    printf("  Software: %s\n", swVersion);
    printf("  Hardware: %s\n", hwVersion);
}

void displayAckNak(const UbxMessage& msg) {
    if (msg.getLength() >= 2) {
        uint8_t ackClass = msg.getPayloadField<uint8_t>(0);
        uint8_t ackId = msg.getPayloadField<uint8_t>(1);
        
        if (msg.getId() == UbxConstants::UBX_ACK_ACK) {
            printf("%s✓ ACK for: Class=0x%02X, ID=0x%02X (%s)%s\n",
                   COLOR_GREEN, ackClass, ackId, 
                   getMessageTypeName(ackClass, ackId), COLOR_RESET);
        } else {
            printf("%s✗ NAK for: Class=0x%02X, ID=0x%02X (%s)%s\n",
                   COLOR_RED, ackClass, ackId,
                   getMessageTypeName(ackClass, ackId), COLOR_RESET);
        }
    }
}

void displayNavStatus(const nav_t& nav) {
    int gps_count = 0, glo_count = 0, gal_count = 0, bds_count = 0, qzs_count = 0;
    
    // Count GPS/GAL/BDS/QZS ephemerides
    for (int i = 0; i < MAXSAT; i++) {
        if (nav.eph[i].sat > 0) {
            int prn;
            int sys = satsys(nav.eph[i].sat, &prn);
            if (sys == SYS_GPS) gps_count++;
            else if (sys == SYS_GAL) gal_count++;
            else if (sys == SYS_CMP) bds_count++;
            else if (sys == SYS_QZS) qzs_count++;
        }
    }
    
    // Count GLONASS ephemerides
    for (int i = 0; i < NSATGLO; i++) {
        if (nav.geph[i].sat > 0) glo_count++;
    }
    
    printf("%s=== Navigation Data Status ===%s\n", COLOR_MAGENTA, COLOR_RESET);
    printf("  GPS Ephemerides: %d\n", gps_count);
    printf("  GLO Ephemerides: %d\n", glo_count);
    printf("  GAL Ephemerides: %d\n", gal_count);
    printf("  BDS Ephemerides: %d\n", bds_count);
    printf("  QZS Ephemerides: %d\n", qzs_count);
    printf("  %sTotal: %d%s\n", COLOR_GREEN, 
           gps_count + glo_count + gal_count + bds_count + qzs_count, COLOR_RESET);
    
    if (nav.leaps != 0) {
        printf("  Leap Seconds: %d\n", nav.leaps);
    }
}

class MessageHandler {
private:
    unsigned long validMessages = 0;
    unsigned long rawxCount = 0;
    unsigned long sfrbxCount = 0;
    unsigned long otherCount = 0;
    FILE* logFile = nullptr;
    bool showAll = false;
    UbxParser* parser = nullptr;
    time_t lastNavDisplay = 0;

    MessageHandler() = default;

public:
    MessageHandler(const MessageHandler&) = delete;
    MessageHandler& operator=(const MessageHandler&) = delete;

    static MessageHandler& instance() {
        static MessageHandler handler;
        return handler;
    }

    void initialize(UbxParser* parserRef, FILE* log = nullptr, bool show = false) {
        parser = parserRef;
        logFile = log;
        showAll = show;
        validMessages = rawxCount = sfrbxCount = otherCount = 0;
        lastNavDisplay = 0;
    }

    void handle(const UbxMessage& msg) {
        validMessages++;

        time_t now = time(nullptr);
        struct tm* timeinfo = localtime(&now);
        char timeStr[32];
        strftime(timeStr, sizeof(timeStr), "%H:%M:%S", timeinfo);

        uint8_t msgClass = msg.getClass();
        uint8_t msgId = msg.getId();
        size_t length = msg.getLength();

        bool isRawx = (msgClass == UbxConstants::UBX_CLASS_RXM && msgId == UbxConstants::UBX_RXM_RAWX);
        bool isSfrbx = (msgClass == UbxConstants::UBX_CLASS_RXM && msgId == UbxConstants::UBX_RXM_SFRBX);
        bool isAck   = (msgClass == UbxConstants::UBX_CLASS_ACK);
        bool isMonVer = (msgClass == UbxConstants::UBX_CLASS_MON && msgId == UbxConstants::UBX_MON_VER);
        bool isCfgRate = (msgClass == UbxConstants::UBX_CLASS_CFG && msgId == 0x08);
        bool isCfgMsg = (msgClass == UbxConstants::UBX_CLASS_CFG && msgId == 0x01);

        if (showAll || isRawx || isSfrbx || isAck || isMonVer || isCfgRate || isCfgMsg) {
            printf("[%s] %sClass=0x%02X, ID=0x%02X (%s), Len=%zu%s\n",
                   timeStr, COLOR_BLUE, msgClass, msgId,
                   getMessageTypeName(msgClass, msgId), length, COLOR_RESET);
        }

        if (isRawx) {
            rawxCount++;
            
            // Parse with UbxParser
            int result = 0;
            if (parser) {
                result = parser->handleRawx(msg.getPayload(), msg.getLength());
            }
            
            // Display summary
            if (parser && result > 0) {
                displayRawxSummary(msg, parser->getObs());
            } else {
                displayRawxSummary(msg, obs_t());
            }
            
            logMessage("RXM-RAWX", msg, timeStr);
            
        } else if (isSfrbx) {
            sfrbxCount++;
            
            // Parse with UbxParser
            int result = 0;
            if (parser) {
                result = parser->handleSfrbx(msg.getPayload(), msg.getLength());
            }
            
            // Display summary
            displaySfrbxSummary(msg, result);
            
            // Display navigation status every 30 seconds
            if (parser && (now - lastNavDisplay >= 30)) {
                displayNavStatus(parser->getNav());
                lastNavDisplay = now;
            }
            
            logMessage("RXM-SFRBX", msg, timeStr);
            
        } else if (isAck) {
            displayAckNak(msg);
        } else if (isMonVer) {
            displayMonVer(msg);
        } else if (isCfgRate) {
            displayCfgRate(msg);
        } else if (isCfgMsg) {
            displayCfgMsg(msg);
        } else {
            otherCount++;
        }

        if (isRawx || isSfrbx || isAck || isMonVer || isCfgRate || isCfgMsg) {
            printf("\n");
        }
    }

    void displayStats(unsigned long elapsed) {
        printf("%s--- Statistics: RAWX=%lu, SFRBX=%lu, Other=%lu, Time=%lus ---%s\n",
               COLOR_MAGENTA, rawxCount, sfrbxCount, otherCount, elapsed, COLOR_RESET);
        
        if (parser) {
            displayNavStatus(parser->getNav());
        }
    }

    void displayFinalStats(unsigned long elapsed) {
        printf("\n%s=== Final Statistics ===%s\n", COLOR_CYAN, COLOR_RESET);
        printf("Runtime: %lu seconds\n", elapsed);
        printf("Total messages: %lu\n", validMessages);
        printf("  RAWX: %lu\n", rawxCount);
        printf("  SFRBX: %lu\n", sfrbxCount);
        printf("  Other: %lu\n", otherCount);
        
        if (parser) {
            printf("\n");
            displayNavStatus(parser->getNav());
        }
        
        if (rawxCount == 0 && sfrbxCount == 0) {
            printf("\n%s⚠ No RAWX/SFRBX received!%s\n", COLOR_YELLOW, COLOR_RESET);
            printf("Check the CFG-MSG poll responses above.\n");
            printf("If UART1 rate shows 0, the receiver isn't configured correctly.\n");
        }
    }

private:
    void logMessage(const char* label, const UbxMessage& msg, const char* timeStr) {
        if (!logFile) return;
        fprintf(logFile, "[%s] %s (Len=%zu)\n", timeStr, label, msg.getLength());
        for (size_t i = 0; i < msg.getLength(); i++) {
            fprintf(logFile, "%02X ", msg.getPayloadField<uint8_t>(i));
            if ((i + 1) % 16 == 0) fprintf(logFile, "\n");
        }
        fprintf(logFile, "\n\n");
        fflush(logFile);
    }
};

int main(int argc, char* argv[])
{
    signal(SIGINT, signalHandler);
    
    const char* portName = nullptr;
    int baudRate = 38400;
    const char* logFileName = nullptr;
    FILE* logFile = nullptr;
    bool showAll = false;

    // Create UbxParser instance
    UbxParser parser;
    parser.enableLogging("ubx_parser.log");
    
    // Parse command line arguments
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "-p") == 0 && i + 1 < argc) {
            portName = argv[++i];
        } else if (strcmp(argv[i], "-b") == 0 && i + 1 < argc) {
            baudRate = atoi(argv[++i]);
        } else if (strcmp(argv[i], "-l") == 0 && i + 1 < argc) {
            logFileName = argv[++i];
        } else if (strcmp(argv[i], "-a") == 0) {
            showAll = true;
        } else if (strcmp(argv[i], "-h") == 0) {
            printf("Usage: %s [options]\n", argv[0]);
            printf("Options:\n");
            printf("  -p <port>    Serial port\n");
            printf("  -b <baud>    Baud rate (default: 38400)\n");
            printf("  -l <file>    Log file\n");
            printf("  -a           Show ALL messages\n");
            printf("  -h           Help\n");
            return 0;
        }
    }
    
    if (!portName) {
#ifdef CE_WINDOWS
        portName = "\\\\.\\COM10";
#else
        portName = "/dev/ttyUSB0";
#endif
    }
    
    if (logFileName) {
        logFile = fopen(logFileName, "w");
        if (logFile) {
            printf("Logging to: %s\n", logFileName);
        }
    }
    
    ceSerial com(portName, baudRate, 8, 'N', 1);
    
    printf("%s=== UBX RAWX/SFRBX Diagnostic with Parser ===%s\n", COLOR_GREEN, COLOR_RESET);
    printf("Port: %s @ %d baud\n", com.GetPort().c_str(), baudRate);
    if (showAll) printf("%s[Showing ALL messages]%s\n", COLOR_YELLOW, COLOR_RESET);
    
    if (com.Open() != 0) {
        printf("%sError: Cannot open port!%s\n", COLOR_RED, COLOR_RESET);
        return 1;
    }
    
    printf("%s✓ Port opened%s\n\n", COLOR_GREEN, COLOR_RESET);
    
    UbxProtocol protocol;
    
    // Step 1: Get receiver version
    printf("%s=== Step 1: Request MON-VER ===%s\n", COLOR_YELLOW, COLOR_RESET);
    uint8_t pollVer[8];
    size_t verLen = protocol.generateMessage(UbxConstants::UBX_CLASS_MON, 
                                             UbxConstants::UBX_MON_VER,
                                             nullptr, 0, pollVer, sizeof(pollVer));
    if (verLen > 0) {
        com.Write((char*)pollVer, verLen);
        printf("=> Sent MON-VER poll\n");
    }
    ceSerial::Delay(1000);
    
    // Step 2: Poll current measurement rate
    printf("\n%s=== Step 2: Poll CFG-RATE ===%s\n", COLOR_YELLOW, COLOR_RESET);
    uint8_t pollRate[8];
    size_t rateLen = protocol.generateMessage(UbxConstants::UBX_CLASS_CFG, 
                                              UbxConstants::UBX_CFG_RATE,
                                              nullptr, 0, pollRate, sizeof(pollRate));
    if (rateLen > 0) {
        com.Write((char*)pollRate, rateLen);
        printf("=> Sent CFG-RATE poll\n");
    }
    ceSerial::Delay(500);
    
    // Step 3: Set measurement rate to 1Hz (1000ms)
    printf("\n%s=== Step 3: Configure CFG-RATE ===%s\n", COLOR_YELLOW, COLOR_RESET);
    uint8_t cfgRateData[6];
    cfgRateData[0] = 0xE8; cfgRateData[1] = 0x03; // 1000 ms
    cfgRateData[2] = 0x01; cfgRateData[3] = 0x00; // navRate = 1
    cfgRateData[4] = 0x01; cfgRateData[5] = 0x00; // timeRef = 1 (GPS time)
    
    uint8_t txRate[32];
    size_t setCfgRateLen = protocol.generateMessage(UbxConstants::UBX_CLASS_CFG,
                                                    UbxConstants::UBX_CFG_RATE,
                                                    cfgRateData, 6, txRate, sizeof(txRate));
    if (setCfgRateLen > 0) {
        com.Write((char*)txRate, setCfgRateLen);
        printf("=> Set measurement rate to 1000ms (1Hz)\n");
    }
    ceSerial::Delay(500);
    
    // Step 4: Configure message rates
    printf("\n%s=== Step 4: Enable RXM-RAWX and RXM-SFRBX ===%s\n", COLOR_YELLOW, COLOR_RESET);
    
    // Enable RXM-RAWX on UART1
    printf("=> Enabling RXM-RAWX...\n");
    uint8_t cfgRawx[8];
    cfgRawx[0] = UbxConstants::UBX_CLASS_RXM;
    cfgRawx[1] = UbxConstants::UBX_RXM_RAWX;
    cfgRawx[2] = 0; // DDC
    cfgRawx[3] = 1; // UART1 rate = 1
    cfgRawx[4] = 0; // UART2
    cfgRawx[5] = 0; // USB
    cfgRawx[6] = 0; // SPI
    cfgRawx[7] = 0; // Reserved
    
    uint8_t txRawx[32];
    size_t rawxLen = protocol.generateMessage(UbxConstants::UBX_CLASS_CFG,
                                              UbxConstants::UBX_CFG_MSG,
                                              cfgRawx, 8, txRawx, sizeof(txRawx));
    if (rawxLen > 0) {
        com.Write((char*)txRawx, rawxLen);
    }
    ceSerial::Delay(500);
    
    // Enable RXM-SFRBX on UART1
    printf("=> Enabling RXM-SFRBX...\n");
    uint8_t cfgSfrbx[8];
    cfgSfrbx[0] = UbxConstants::UBX_CLASS_RXM;
    cfgSfrbx[1] = UbxConstants::UBX_RXM_SFRBX;
    cfgSfrbx[2] = 0; // DDC
    cfgSfrbx[3] = 1; // UART1 rate = 1
    cfgSfrbx[4] = 0; // UART2
    cfgSfrbx[5] = 0; // USB
    cfgSfrbx[6] = 0; // SPI
    cfgSfrbx[7] = 0; // Reserved
    
    uint8_t txSfrbx[32];
    size_t sfrbxLen = protocol.generateMessage(UbxConstants::UBX_CLASS_CFG,
                                               UbxConstants::UBX_CFG_MSG,
                                               cfgSfrbx, 8, txSfrbx, sizeof(txSfrbx));
    if (sfrbxLen > 0) {
        com.Write((char*)txSfrbx, sfrbxLen);
    }
    
    printf("\n%sWaiting for ACKs...%s\n", COLOR_YELLOW, COLOR_RESET);
    ceSerial::Delay(1000);
    
    printf("\n%s=== Monitoring Messages ===%s\n", COLOR_YELLOW, COLOR_RESET);
    printf("Press Ctrl+C to exit\n\n");
    
    time_t startTime = time(nullptr);
    uint8_t buffer[4096];

    // Initialize handler with parser
    MessageHandler::instance().initialize(&parser, logFile, showAll);
    
    while (running) {
        int bytesAvailable = com.Available();
        
        if (bytesAvailable > 0) {
            int bytesToRead = (bytesAvailable > sizeof(buffer)) ? sizeof(buffer) : bytesAvailable;
            int bytesRead = com.ReadBuffer((char*)buffer, bytesToRead);
            if (bytesRead > 0) {
                protocol.processData(buffer, bytesRead, 
                    [](const UbxMessage& msg) { 
                        MessageHandler::instance().handle(msg); 
                    });
            }
        }
        
        ceSerial::Delay(10);
        
        // Stats every 30 seconds
        static time_t lastStats = 0;
        time_t now = time(nullptr);
        if (now - lastStats >= 30) {
            lastStats = now;
            unsigned long elapsed = now - startTime;
            MessageHandler::instance().displayStats(elapsed);
        }
    }
    
    // Final stats
    time_t endTime = time(nullptr);
    unsigned long elapsed = endTime - startTime;
    MessageHandler::instance().displayFinalStats(elapsed);
    
    com.Close();
    if (logFile) fclose(logFile);
    
    printf("\n%sDone!%s\n", COLOR_GREEN, COLOR_RESET);
    return 0;
}