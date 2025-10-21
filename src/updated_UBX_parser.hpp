#pragma once
#include "rtklib.h"
#include <cstdint>
#include <string>

// External RTKLIB functions (exported from rcvraw.c)
extern "C" {
    extern int decode_frame(const unsigned char *buff, eph_t *eph, alm_t *alm,
                           double *ion, double *utc, int *leaps);
    extern int decode_gal_inav(const unsigned char *buff, eph_t *eph);
    extern int decode_bds_d1(const unsigned char *buff, eph_t *eph);
    extern int decode_bds_d2(const unsigned char *buff, eph_t *eph);
    extern int decode_glostr(const unsigned char *buff, geph_t *geph);
    extern int test_glostr(const unsigned char *buff);
}

/**
 * UBX Parser: Handles RXM-RAWX and RXM-SFRBX
 * Uses RTKLIB for navigation message decoding
 */
class UbxParser {
public:
    UbxParser();
    ~UbxParser();

    void enableLogging(const std::string& filename);
    void disableLogging();

    int handleRawx(const uint8_t* payload, size_t length);
    int handleSfrbx(const uint8_t* payload, size_t length);

    const obs_t& getObs() const { return obs; }
    const nav_t& getNav() const { return nav; }
    
    void clearObs();

private:
    obs_t obs;
    nav_t nav;
    FILE* logFile;
    bool loggingEnabled;
    
    gtime_t current_time;
    double lockt[MAXSAT][NFREQ+NEXOBS];
    unsigned char halfc[MAXSAT][NFREQ+NEXOBS];
    unsigned char subfrm[MAXSAT][380];
    
    // Decoding functions
    int decodeSfrbx(int sys, int sat, const uint8_t* data, size_t length, int freqId);
    int decodeGpsFrame(int sat);
    int decodeGalileoInav(int sat, const uint8_t* data, size_t length);
    int decodeBeidouD1(int sat);
    int decodeBeidouD2(int sat);
    int decodeGlonassStr(int sat, const uint8_t* data, size_t length, int freqId);
    
    // Helper functions (static in rcvraw.c, so we need our own)
    static unsigned int getbitu2(const unsigned char *buff, int p1, int l1, int p2, int l2);
    static int getbits2(const unsigned char *buff, int p1, int l1, int p2, int l2);
    static unsigned int getbitu3(const unsigned char *buff, int p1, int l1, int p2, int l2, int p3, int l3);
    static int getbits3(const unsigned char *buff, int p1, int l1, int p2, int l2, int p3, int l3);
    static unsigned int merge_two_u(unsigned int a, unsigned int b, int n);
    static int merge_two_s(int a, unsigned int b, int n);
    static double getbitg(const unsigned char *buff, int pos, int len);
    
    // Utility
    static int ubxSys(int gnssId);
};