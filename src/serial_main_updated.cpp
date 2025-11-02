/*
 * test_parser.cpp - MINIMAL test for UBX parser
 * Just checks if RAWX and SFRBX parsing works
 */

#include <stdio.h>
#include <signal.h>
#include "ceserial.h"
#include "MiniUBX.hpp"
#include "updated_UBX_parser.hpp"

using namespace std;
using namespace UbxConstants;

volatile bool running = true;

void signalHandler(int) {
    printf("\nExiting...\n");
    running = false;
}

int main(int argc, char* argv[])
{
    signal(SIGINT, signalHandler);
    
    // Parse port
    const char* port = (argc > 1) ? argv[1] : "/dev/ttyUSB0";
    int baud = (argc > 2) ? atoi(argv[2]) : 38400;
    
    // Create parser
    UbxParser parser;
    
    // Open serial
    ceSerial com(port, baud, 8, 'N', 1);
    printf("Testing parser on %s @ %d baud\n", port, baud);
    
    if (com.Open() != 0) {
        printf("ERROR: Cannot open port!\n");
        return 1;
    }
    printf("Port opened\n");
    
    UbxProtocol protocol;
    
    // Configure receiver: Enable RAWX and SFRBX
    printf("Configuring...\n");
    
    // Set 1Hz rate
    uint8_t rateData[6] = {0xE8, 0x03, 0x01, 0x00, 0x01, 0x00};
    uint8_t txBuf[32];
    size_t len = protocol.generateMessage(UBX_CLASS_CFG, UBX_CFG_RATE, 
                                         rateData, 6, txBuf, sizeof(txBuf));
    com.Write((char*)txBuf, len);
    ceSerial::Delay(500);
    
    // Enable RAWX
    uint8_t rawxCfg[8] = {UBX_CLASS_RXM, UBX_RXM_RAWX, 0, 1, 0, 0, 0, 0};
    len = protocol.generateMessage(UBX_CLASS_CFG, UBX_CFG_MSG, 
                                   rawxCfg, 8, txBuf, sizeof(txBuf));
    com.Write((char*)txBuf, len);
    ceSerial::Delay(500);
    
    // Enable SFRBX
    uint8_t sfrbxCfg[8] = {UBX_CLASS_RXM, UBX_RXM_SFRBX, 0, 1, 0, 0, 0, 0};
    len = protocol.generateMessage(UBX_CLASS_CFG, UBX_CFG_MSG, 
                                   sfrbxCfg, 8, txBuf, sizeof(txBuf));
    com.Write((char*)txBuf, len);
    
    printf("Waiting for messages... (Ctrl+C to exit)\n\n");
    ceSerial::Delay(1000);
    
    // Test loop
    uint8_t buffer[4096];
    unsigned long rawxCount = 0, sfrbxCount = 0, ephCount = 0;
    
    while (running) {
        int available = com.Available();
        
        if (available > 0) {
            int toRead = (available > sizeof(buffer)) ? sizeof(buffer) : available;
            int bytesRead = com.ReadBuffer((char*)buffer, toRead);
            
            if (bytesRead > 0) {
                protocol.processData(buffer, bytesRead, 
                    [&](const UbxMessage& msg) {
                        // Handle RAWX
                        if (msg.getClass() == UBX_CLASS_RXM && 
                            msg.getId() == UBX_RXM_RAWX) {
                            rawxCount++;
                            int result = parser.handleRawx(msg.getPayload(), msg.getLength());
                            if (result && rawxCount % 5 == 0) {
                                const obs_t& obs = parser.getObs();
                                printf("RAWX #%lu: %d observations\n", rawxCount, obs.n);
                            }
                        }
                        // Handle SFRBX
                        else if (msg.getClass() == UBX_CLASS_RXM && 
                                 msg.getId() == UBX_RXM_SFRBX) {
                            sfrbxCount++;
                            int result = parser.handleSfrbx(msg.getPayload(), msg.getLength());
                            if (result == 2) {
                                ephCount++;
                                printf("SFRBX #%lu: Ephemeris decoded! (Total: %lu)\n", 
                                       sfrbxCount, ephCount);
                            }
                        }
                    });
            }
        }
        
        ceSerial::Delay(10);
    }
    
    // Summary
    printf("\n=== Test Results ===\n");
    printf("RAWX messages:  %lu\n", rawxCount);
    printf("SFRBX messages: %lu\n", sfrbxCount);
    printf("Ephemerides:    %lu\n", ephCount);
    printf("\n");
    
    if (rawxCount > 0 && ephCount > 0) {
        printf("✓ Parser working!\n");
    } else {
        printf("✗ Check receiver configuration\n");
    }
    
    com.Close();
    return 0;
}