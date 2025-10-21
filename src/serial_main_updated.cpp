
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
