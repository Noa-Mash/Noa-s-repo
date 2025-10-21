#pragma once
#include "rtklib.h"
#include <cstdint>
#include <string>

/**
 * UBX Parser: extracts basic RAWX and SFRBX info and populates RTKLIB
 * data structures (obs_t, nav_t) for future processing.
 * No dependency on solvers or positioning.
 */
class UbxParser {
public:
    UbxParser();
    ~UbxParser();

    // Enable or disable file logging of decoded messages
    void enableLogging(const std::string& filename);
    void disableLogging();

    // Feed UBX message payloads
    int handleRawx(const uint8_t* payload, size_t length);
    int handleSfrbx(const uint8_t* payload, size_t length);

    // Access parsed data
    const obs_t& getObs() const { return obs; }
    const nav_t& getNav() const { return nav; }

private:
    obs_t obs;
    nav_t nav;
    FILE* logFile;
    bool loggingEnabled;

    void logObs(const obsd_t* data, int n);
    void logNav(const nav_t* nav);
};
