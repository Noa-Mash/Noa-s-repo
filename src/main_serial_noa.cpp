/*
 
 * 
 * This program only displays RXM-RAWX and RXM-SFRBX messages
 * and shows confirmation of sent configuration commands.
 */

#include <stdio.h>
#include <signal.h>
#include <stdint.h>
#include <string.h>
#include <time.h>
#include <stdlib.h>
#include "ceserial.h"
#include "serial.hpp"

using namespace std;

volatile bool running = true;

// Color codes for terminal output
#define COLOR_RESET   "\033[0m"
#define COLOR_RED     "\033[31m"
#define COLOR_GREEN   "\033[32m"
#define COLOR_YELLOW  "\033[33m"
#define COLOR_BLUE    "\033[34m"
#define COLOR_MAGENTA "\033[35m"
#define COLOR_CYAN    "\033[36m"

//-----------------------------------------------------------
void signalHandler(int signum) {
    printf("\n%sReceived Ctrl+C, exiting...%s\n", COLOR_YELLOW, COLOR_RESET);
    running = false;
}

//-----------------------------------------------------------
// Helper function to print hex dump of data
void printHexDump(const uint8_t* data, size_t length, const char* prefix = "") {
    printf("%s", prefix);
    for (size_t i = 0; i < length; i++) {
        printf("%02X ", data[i]);
        if ((i + 1) % 16 == 0) {
            printf("\n%s", prefix);
        }
    }
    if (length % 16 != 0) {
        printf("\n");
    }
}

//-----------------------------------------------------------
// Message type to string converter
const char* getMessageTypeName(uint8_t msgClass, uint8_t msgId) {
    if (msgClass == UbxConstants::UBX_CLASS_RXM) {
        switch (msgId) {
            case UbxConstants::UBX_RXM_RAWX: return "RXM-RAWX";
            case UbxConstants::UBX_RXM_SFRBX: return "RXM-SFRBX";
            default: return "RXM-UNKNOWN";
        }
    } else if (msgClass == UbxConstants::UBX_CLASS_ACK) {
        switch (msgId) {
            case UbxConstants::UBX_ACK_ACK: return "ACK-ACK";
            case UbxConstants::UBX_ACK_NAK: return "ACK-NAK";
            default: return "ACK-UNKNOWN";
        }
    }
    return "UNKNOWN";
}

//-----------------------------------------------------------
// Display RAWX message data
void displayRawxData(const UbxMessage& msg) {
    if (msg.getLength() < 16) {
        printf("%sInvalid RAWX payload length: %zu%s\n", 
               COLOR_RED, msg.getLength(), COLOR_RESET);
        return;
    }
    
    // Parse RAWX header
    double rcvTow = msg.getPayloadField<double>(0);      // Receiver time of week (s)
    uint16_t week = msg.getPayloadField<uint16_t>(8);    // GPS week
    int8_t leapS = msg.getPayloadField<int8_t>(10);      // Leap seconds
    uint8_t numMeas = msg.getPayloadField<uint8_t>(11);  // Number of measurements
    uint8_t recStat = msg.getPayloadField<uint8_t>(12);  // Receiver tracking status
    
    printf("%s=== RXM-RAWX Data ===%s\n", COLOR_CYAN, COLOR_RESET);
    printf("  GPS Week: %u, TOW: %.3f s, Leap Seconds: %d\n", week, rcvTow, leapS);
    printf("  Number of Measurements: %u, Receiver Status: 0x%02X\n", numMeas, recStat);
    printf("  Payload size: %zu bytes (header: 16 bytes, measurements: %zu bytes)\n", 
           msg.getLength(), msg.getLength() - 16);
    
    // Show first few measurements if available
    size_t measBlockSize = 32; // Each measurement is 32 bytes
    size_t availMeas = (msg.getLength() - 16) / measBlockSize;
    if (availMeas > 0 && availMeas <= numMeas) {
        printf("  First measurement details:\n");
        size_t offset = 16; // Start after header
        
        double prMes = msg.getPayloadField<double>(offset + 0);    // Pseudorange (m)
        double cpMes = msg.getPayloadField<double>(offset + 8);    // Carrier phase (cycles)
        float doMes = msg.getPayloadField<float>(offset + 16);     // Doppler (Hz)
        uint8_t gnssId = msg.getPayloadField<uint8_t>(offset + 20);
        uint8_t svId = msg.getPayloadField<uint8_t>(offset + 21);
        uint8_t freqId = msg.getPayloadField<uint8_t>(offset + 23);
        uint8_t locktime = msg.getPayloadField<uint16_t>(offset + 24);
        uint8_t cno = msg.getPayloadField<uint8_t>(offset + 26);   // C/N0 (dBHz)
        
        const char* gnssName = "Unknown";
        if (gnssId == 0) gnssName = "GPS";
        else if (gnssId == 1) gnssName = "SBAS";
        else if (gnssId == 2) gnssName = "Galileo";
        else if (gnssId == 3) gnssName = "BeiDou";
        else if (gnssId == 5) gnssName = "QZSS";
        else if (gnssId == 6) gnssName = "GLONASS";
        
        printf("    GNSS: %s (ID=%u), SV: %u, Freq: %u\n", gnssName, gnssId, svId, freqId);
        printf("    Pseudorange: %.3f m, Carrier Phase: %.3f cycles\n", prMes, cpMes);
        printf("    Doppler: %.3f Hz, C/N0: %u dBHz, Locktime: %u\n", doMes, cno, locktime);
    }
}

//-----------------------------------------------------------
// Display SFRBX message data
void displaySfrbxData(const UbxMessage& msg) {
    if (msg.getLength() < 8) {
        printf("%sInvalid SFRBX payload length: %zu%s\n", 
               COLOR_RED, msg.getLength(), COLOR_RESET);
        return;
    }
    
    // Parse SFRBX header
    uint8_t gnssId = msg.getPayloadField<uint8_t>(0);
    uint8_t svId = msg.getPayloadField<uint8_t>(1);
    uint8_t freqId = msg.getPayloadField<uint8_t>(3);
    uint8_t numWords = msg.getPayloadField<uint8_t>(4);
    uint8_t chn = msg.getPayloadField<uint8_t>(5);
    uint8_t version = msg.getPayloadField<uint8_t>(6);
    
    const char* gnssName = "Unknown";
    if (gnssId == 0) gnssName = "GPS";
    else if (gnssId == 1) gnssName = "SBAS";
    else if (gnssId == 2) gnssName = "Galileo";
    else if (gnssId == 3) gnssName = "BeiDou";
    else if (gnssId == 5) gnssName = "QZSS";
    else if (gnssId == 6) gnssName = "GLONASS";
    
    printf("%s=== RXM-SFRBX Data ===%s\n", COLOR_CYAN, COLOR_RESET);
    printf("  GNSS: %s (ID=%u), SV: %u, Channel: %u\n", gnssName, gnssId, svId, chn);
    printf("  Frequency ID: %u, Number of Words: %u, Version: %u\n", freqId, numWords, version);
    printf("  Total payload size: %zu bytes (header: 8 bytes, data: %zu bytes)\n", 
           msg.getLength(), msg.getLength() - 8);
    
    // Show first few data words
    size_t dataSize = msg.getLength() - 8;
    if (dataSize > 0) {
        printf("  Data words (first 32 bytes): ");
        size_t displayBytes = (dataSize < 32) ? dataSize : 32;
        for (size_t i = 0; i < displayBytes; i++) {
            printf("%02X ", msg.getPayloadField<uint8_t>(8 + i));
            if ((i + 1) % 16 == 0) printf("\n                                ");
        }
        if (dataSize > 32) {
            printf("... (%zu more bytes)", dataSize - 32);
        }
        printf("\n");
    }
}

//-----------------------------------------------------------
// Display ACK/NAK data
void displayAckNak(const UbxMessage& msg) {
    if (msg.getLength() >= 2) {
        uint8_t ackClass = msg.getPayloadField<uint8_t>(0);
        uint8_t ackId = msg.getPayloadField<uint8_t>(1);
        
        const char* msgName = "UNKNOWN";
        if (ackClass == UbxConstants::UBX_CLASS_CFG && ackId == UbxConstants::UBX_CFG_MSG) {
            msgName = "CFG-MSG";
        }
        
        if (msg.getId() == UbxConstants::UBX_ACK_ACK) {
            printf("%s✓ ACK received: Configuration accepted for Class=0x%02X, ID=0x%02X (%s)%s\n",
                   COLOR_GREEN, ackClass, ackId, msgName, COLOR_RESET);
        } else {
            printf("%s✗ NAK received: Configuration rejected for Class=0x%02X, ID=0x%02X (%s)%s\n",
                   COLOR_RED, ackClass, ackId, msgName, COLOR_RESET);
        }
    }
}

//-----------------------------------------------------------
// Message handler callback
class MessageHandler {
    unsigned long& validMessages;
    unsigned long& rawxCount;
    unsigned long& sfrbxCount;
    unsigned long& ackCount;
    unsigned long& nakCount;
    FILE* logFile;
    
public:
    MessageHandler(unsigned long& msgCount, unsigned long& rawx, 
                   unsigned long& sfrbx, unsigned long& ack, unsigned long& nak,
                   FILE* log = nullptr) 
        : validMessages(msgCount), rawxCount(rawx), sfrbxCount(sfrbx),
          ackCount(ack), nakCount(nak), logFile(log) {}
    
    void operator()(const UbxMessage& msg) {
        validMessages++;
        
        // Get timestamp
        time_t now = time(nullptr);
        struct tm* timeinfo = localtime(&now);
        char timeStr[32];
        strftime(timeStr, sizeof(timeStr), "%H:%M:%S", timeinfo);
        
        uint8_t msgClass = msg.getClass();
        uint8_t msgId = msg.getId();
        size_t length = msg.getLength();
        
        // Only process and display RAWX, SFRBX, and ACK/NAK messages
        if (msgClass == UbxConstants::UBX_CLASS_RXM) {
            printf("[%s] %sUBX Message: Class=0x%02X, ID=0x%02X (%s), Length=%zu%s\n",
                   timeStr, COLOR_BLUE, msgClass, msgId, 
                   getMessageTypeName(msgClass, msgId), length, COLOR_RESET);
            
            if (msgId == UbxConstants::UBX_RXM_RAWX) {
                rawxCount++;
                displayRawxData(msg);
                
                // Log full hex dump to file
                if (logFile) {
                    fprintf(logFile, "[%s] RXM-RAWX (Length=%zu)\n", timeStr, length);
                    fprintf(logFile, "Header (0xB5 0x62): B5 62 %02X %02X %02X %02X\n",
                            msgClass, msgId, (uint8_t)(length & 0xFF), (uint8_t)(length >> 8));
                    fprintf(logFile, "Payload:\n");
                    for (size_t i = 0; i < length; i++) {
                        fprintf(logFile, "%02X ", msg.getPayloadField<uint8_t>(i));
                        if ((i + 1) % 16 == 0) fprintf(logFile, "\n");
                    }
                    if (length % 16 != 0) fprintf(logFile, "\n");
                    fprintf(logFile, "\n");
                    fflush(logFile);
                }
            } else if (msgId == UbxConstants::UBX_RXM_SFRBX) {
                sfrbxCount++;
                displaySfrbxData(msg);
                
                // Log full hex dump to file
                if (logFile) {
                    fprintf(logFile, "[%s] RXM-SFRBX (Length=%zu)\n", timeStr, length);
                    fprintf(logFile, "Header (0xB5 0x62): B5 62 %02X %02X %02X %02X\n",
                            msgClass, msgId, (uint8_t)(length & 0xFF), (uint8_t)(length >> 8));
                    fprintf(logFile, "Payload:\n");
                    for (size_t i = 0; i < length; i++) {
                        fprintf(logFile, "%02X ", msg.getPayloadField<uint8_t>(i));
                        if ((i + 1) % 16 == 0) fprintf(logFile, "\n");
                    }
                    if (length % 16 != 0) fprintf(logFile, "\n");
                    fprintf(logFile, "\n");
                    fflush(logFile);
                }
            }
            printf("\n");
        } else if (msgClass == UbxConstants::UBX_CLASS_ACK) {
            printf("[%s] ", timeStr);
            if (msgId == UbxConstants::UBX_ACK_ACK) {
                ackCount++;
            } else {
                nakCount++;
            }
            displayAckNak(msg);
            printf("\n");
        }
        // Silently ignore all other message types
    }
};

//-----------------------------------------------------------
int main(int argc, char* argv[])
{
    // Set up Ctrl+C handler
    signal(SIGINT, signalHandler);
    
    // Default configuration
    const char* portName = nullptr;
    int baudRate = 38400;
    const char* logFileName = nullptr;
    FILE* logFile = nullptr;
    
    // Parse command line arguments
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "-p") == 0 && i + 1 < argc) {
            portName = argv[++i];
        } else if (strcmp(argv[i], "-b") == 0 && i + 1 < argc) {
            baudRate = atoi(argv[++i]);
        } else if (strcmp(argv[i], "-l") == 0 && i + 1 < argc) {
            logFileName = argv[++i];
        } else if (strcmp(argv[i], "-h") == 0) {
            printf("Usage: %s [options]\n", argv[0]);
            printf("Options:\n");
            printf("  -p <port>    Serial port (e.g., /dev/ttyUSB0 or COM10)\n");
            printf("  -b <baud>    Baud rate (default: 38400)\n");
            printf("  -l <file>    Log file for hex dumps (optional)\n");
            printf("  -h           Show this help\n");
            return 0;
        }
    }
    
    // Open log file if specified
    if (logFileName) {
        logFile = fopen(logFileName, "w");
        if (!logFile) {
            printf("%sWarning: Could not open log file %s%s\n", 
                   COLOR_YELLOW, logFileName, COLOR_RESET);
        } else {
            printf("Logging hex dumps to: %s\n", logFileName);
            // Write header to log file
            time_t now = time(nullptr);
            fprintf(logFile, "# UBX RAWX/SFRBX Hex Dump Log\n");
            fprintf(logFile, "# Started: %s", ctime(&now));
            fprintf(logFile, "# Port: %s, Baud: %d\n\n", portName ? portName : "default", baudRate);
            fflush(logFile);
        }
    }
    
    // Set default port if not specified
    if (!portName) {
#ifdef CE_WINDOWS
        portName = "\\\\.\\COM14";
#else
        portName = "/dev/ttyUSB0";
#endif
    }
    
    // Create serial port
    ceSerial com(portName, baudRate, 8, 'N', 1);
    
    printf("%s=== UBX RAWX/SFRBX Monitor ===%s\n", COLOR_GREEN, COLOR_RESET);
    printf("Opening port %s at %d baud...\n", com.GetPort().c_str(), baudRate);
    
    if (com.Open() != 0) {
        printf("%sError: Failed to open serial port!%s\n", COLOR_RED, COLOR_RESET);
        return 1;
    }
    
    printf("%s✓ Port opened successfully!%s\n", COLOR_GREEN, COLOR_RESET);
    
    // Create UBX protocol handler
    UbxProtocol protocol;
    
    // Send configuration commands
    printf("\n%s=== Configuring GNSS Receiver ===%s\n", COLOR_YELLOW, COLOR_RESET);
    
    // Configure RXM-RAWX output at 1Hz (raw measurements)
    printf("→ Sending CFG-MSG for RXM-RAWX (raw measurements)...\n");
    uint8_t cfgRawx[8];
    cfgRawx[0] = UbxConstants::UBX_CLASS_RXM;
    cfgRawx[1] = UbxConstants::UBX_RXM_RAWX;
    cfgRawx[2] = 1; // Rate on current port
    memset(&cfgRawx[3], 0, 5); // Other ports disabled

    uint8_t txRawx[32];
    size_t rawxLen = protocol.generateMessage(
        UbxConstants::UBX_CLASS_CFG, 
        UbxConstants::UBX_CFG_MSG,
        cfgRawx, 8, txRawx, sizeof(txRawx));

    if (rawxLen > 0) {
        com.Write((char*)txRawx, rawxLen);
        printf("  Sent %zu bytes: ", rawxLen);
        printHexDump(txRawx, rawxLen, "  ");
    }

    ceSerial::Delay(200); // Wait for ACK

    // Configure RXM-SFRBX output at 1Hz (satellite broadcast data)
    printf("\n→ Sending CFG-MSG for RXM-SFRBX (subframe data)...\n");
    uint8_t cfgSfrbx[8];
    cfgSfrbx[0] = UbxConstants::UBX_CLASS_RXM;
    cfgSfrbx[1] = UbxConstants::UBX_RXM_SFRBX;
    cfgSfrbx[2] = 1;
    memset(&cfgSfrbx[3], 0, 5);

    uint8_t txSfrbx[32];
    size_t sfrbxLen = protocol.generateMessage(
        UbxConstants::UBX_CLASS_CFG,
        UbxConstants::UBX_CFG_MSG,
        cfgSfrbx, 8, txSfrbx, sizeof(txSfrbx));

    if (sfrbxLen > 0) {
        com.Write((char*)txSfrbx, sfrbxLen);
        printf("  Sent %zu bytes: ", sfrbxLen);
        printHexDump(txSfrbx, sfrbxLen, "  ");
    }

    printf("\n%sWaiting for acknowledgments...%s\n", COLOR_YELLOW, COLOR_RESET);
    ceSerial::Delay(500); // Wait for ACKs

    // Statistics
    unsigned long totalMessages = 0;
    unsigned long validMessages = 0;
    unsigned long rawxCount = 0;
    unsigned long sfrbxCount = 0;
    unsigned long ackCount = 0;
    unsigned long nakCount = 0;
    time_t startTime = time(nullptr);
    
    printf("\n%s=== Reading RAWX and SFRBX messages... Press Ctrl+C to exit ===%s\n\n", 
           COLOR_YELLOW, COLOR_RESET);
    
    // Create message handler
    MessageHandler handler(validMessages, rawxCount, sfrbxCount, ackCount, nakCount, logFile);
    
    // Main reading loop
    uint8_t buffer[4096]; // Larger buffer for RAWX messages
    while (running) {
        int bytesAvailable = com.Available();
        
        if (bytesAvailable > 0) {
            int bytesToRead = (bytesAvailable > sizeof(buffer)) ? sizeof(buffer) : bytesAvailable;
            int bytesRead = com.ReadBuffer((char*)buffer, bytesToRead);        
            if (bytesRead > 0) {
                // Process data through the protocol handler
                protocol.processData(buffer, bytesRead, handler);
                totalMessages++;
            }
        }
        
        ceSerial::Delay(10); // Small delay to prevent busy waiting
        
        // Print statistics every 30 seconds
        static time_t lastStats = 0;
        time_t now = time(nullptr);
        if (now - lastStats >= 30) {
            lastStats = now;
            unsigned long elapsed = now - startTime;
            printf("%s--- Stats: RAWX=%lu, SFRBX=%lu, ACK=%lu, NAK=%lu, Runtime=%lus ---%s\n",
                   COLOR_MAGENTA, rawxCount, sfrbxCount, ackCount, nakCount,
                   elapsed, COLOR_RESET);
        }
    }
    
    // Print final statistics
    time_t endTime = time(nullptr);
    unsigned long elapsed = endTime - startTime;
    printf("\n%s=== Final Statistics ===%s\n", COLOR_CYAN, COLOR_RESET);
    printf("Total runtime: %lu seconds\n", elapsed);
    printf("Total UBX messages received: %lu\n", validMessages);
    printf("  RXM-RAWX messages: %lu\n", rawxCount);
    printf("  RXM-SFRBX messages: %lu\n", sfrbxCount);
    printf("  ACK messages: %lu\n", ackCount);
    printf("  NAK messages: %lu\n", nakCount);
    if (elapsed > 0) {
        printf("Average RAWX rate: %.2f messages/second\n", (float)rawxCount / elapsed);
        printf("Average SFRBX rate: %.2f messages/second\n", (float)sfrbxCount / elapsed);
    }
    
    printf("\nClosing port %s...\n", com.GetPort().c_str());
    com.Close();
    
    // Close log file if open
    if (logFile) {
        time_t now = time(nullptr);
        fprintf(logFile, "\n# Session ended: %s", ctime(&now));
        fclose(logFile);
        printf("Log file closed: %s\n", logFileName);
    }
    
    printf("%s✓ Done!%s\n", COLOR_GREEN, COLOR_RESET);
    
    return 0;
}