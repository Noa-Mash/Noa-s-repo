#include "updated_UBX_parser.hpp"
#include <cstdio>
#include <cstring>

// ============================================================================
// HELPER FUNCTIONS
// These are STATIC in rcvraw.c, so we must define our own versions
// They use getbitu/getbits from rtkcmn.c (which ARE exported)
// ============================================================================

unsigned int UbxParser::getbitu2(const unsigned char *buff, int p1, int l1, int p2, int l2) {
    return (getbitu(buff,p1,l1)<<l2)+getbitu(buff,p2,l2);
}

int UbxParser::getbits2(const unsigned char *buff, int p1, int l1, int p2, int l2) {
    if (getbitu(buff,p1,1))
        return (int)((getbits(buff,p1,l1)<<l2)+getbitu(buff,p2,l2));
    else
        return (int)getbitu2(buff,p1,l1,p2,l2);
}

unsigned int UbxParser::getbitu3(const unsigned char *buff, int p1, int l1, int p2, int l2, int p3, int l3) {
    return (getbitu(buff,p1,l1)<<(l2+l3))+(getbitu(buff,p2,l2)<<l3)+getbitu(buff,p3,l3);
}

int UbxParser::getbits3(const unsigned char *buff, int p1, int l1, int p2, int l2, int p3, int l3) {
    if (getbitu(buff,p1,1))
        return (int)((getbits(buff,p1,l1)<<(l2+l3))+(getbitu(buff,p2,l2)<<l3)+getbitu(buff,p3,l3));
    else
        return (int)getbitu3(buff,p1,l1,p2,l2,p3,l3);
}

unsigned int UbxParser::merge_two_u(unsigned int a, unsigned int b, int n) {
    return (a<<n)+b;
}

int UbxParser::merge_two_s(int a, unsigned int b, int n) {
    return (int)((a<<n)+b);
}

double UbxParser::getbitg(const unsigned char *buff, int pos, int len) {
    double value=getbitu(buff,pos+1,len-1);
    return getbitu(buff,pos,1)?-value:value;
}

int UbxParser::ubxSys(int gnssId) {
    switch (gnssId) {
        case 0: return SYS_GPS;
        case 1: return SYS_SBS;
        case 2: return SYS_GAL;
        case 3: return SYS_CMP;
        case 5: return SYS_QZS;
        case 6: return SYS_GLO;
    }
    return 0;
}

// ============================================================================
// CONSTRUCTOR / DESTRUCTOR
// ============================================================================

UbxParser::UbxParser() : logFile(nullptr), loggingEnabled(false)
{
    memset(&obs, 0, sizeof(obs));
    memset(&nav, 0, sizeof(nav));
    memset(lockt, 0, sizeof(lockt));
    memset(halfc, 0, sizeof(halfc));
    memset(subfrm, 0, sizeof(subfrm));
    
    current_time.time = 0;
    current_time.sec = 0.0;

    // Allocate obs_t
    obs.nmax = MAXOBS; 
    obs.n = 0;
    obs.data = (obsd_t*)malloc(sizeof(obsd_t) * obs.nmax);
    if (obs.data) memset(obs.data, 0, sizeof(obsd_t) * obs.nmax);

    // Allocate nav_t
    nav.nmax = MAXSAT; nav.n = 0;
    nav.ngmax = NSATGLO; nav.ng = 0;
    nav.nsmax = NSATSBS*2; nav.ns = 0;
    nav.namax = MAXSAT; nav.na = 0;
    
    nav.eph = (eph_t*)malloc(sizeof(eph_t) * nav.nmax);
    nav.geph = (geph_t*)malloc(sizeof(geph_t) * nav.ngmax);
    nav.seph = (seph_t*)malloc(sizeof(seph_t) * nav.nsmax);
    nav.alm = (alm_t*)malloc(sizeof(alm_t) * nav.namax);
    
    if (nav.eph) {
        for (int i = 0; i < nav.nmax; i++) {
            memset(&nav.eph[i], 0, sizeof(eph_t));
            nav.eph[i].sat = 0;
            nav.eph[i].iode = nav.eph[i].iodc = -1;
        }
    }
    if (nav.geph) {
        for (int i = 0; i < nav.ngmax; i++) {
            memset(&nav.geph[i], 0, sizeof(geph_t));
            nav.geph[i].sat = 0;
            nav.geph[i].iode = -1;
        }
    }
    if (nav.seph) {
        memset(nav.seph, 0, sizeof(seph_t) * nav.nsmax);
    }
    if (nav.alm) {
        for (int i = 0; i < nav.namax; i++) {
            memset(&nav.alm[i], 0, sizeof(alm_t));
            nav.alm[i].sat = 0;
            nav.alm[i].svh = -1;
        }
    }
}

UbxParser::~UbxParser() {
    if (logFile) fclose(logFile);
    if (obs.data) free(obs.data);
    if (nav.eph) free(nav.eph);
    if (nav.geph) free(nav.geph);
    if (nav.seph) free(nav.seph);
    if (nav.alm) free(nav.alm);
}

void UbxParser::enableLogging(const std::string& filename) {
    logFile = fopen(filename.c_str(), "w");
    if (logFile) {
        loggingEnabled = true;
        fprintf(logFile, "# UBX Parser Log\n");
        fflush(logFile);
    }
}

void UbxParser::disableLogging() {
    if (logFile) fclose(logFile);
    logFile = nullptr;
    loggingEnabled = false;
}

void UbxParser::clearObs() {
    obs.n = 0;
}

// ============================================================================
// GPS/QZSS FRAME DECODER
// Uses RTKLIB's decode_frame() from rcvraw.c
// ============================================================================

int UbxParser::decodeGpsFrame(int sat) {
    eph_t eph = {0};
    
    // Use RTKLIB decoder
    if (decode_frame(subfrm[sat-1],      &eph, NULL, NULL, NULL, NULL) != 1 ||
        decode_frame(subfrm[sat-1] + 30, &eph, NULL, NULL, NULL, NULL) != 2 ||
        decode_frame(subfrm[sat-1] + 60, &eph, NULL, NULL, NULL, NULL) != 3) {
        return 0;
    }
    
    eph.sat = sat;
    nav.eph[sat-1] = eph;
    
    if (loggingEnabled && logFile) {
        fprintf(logFile, "  GPS/QZS Eph: sat=%d iode=%d iodc=%d\n", sat, eph.iode, eph.iodc);
        fflush(logFile);
    }
    
    return 2;
}

// ============================================================================
// GALILEO I/NAV DECODER
// Uses RTKLIB's decode_gal_inav() from rcvraw.c
// Uses RTKLIB's rtk_crc24q() from rtkcmn.c
// Uses RTKLIB's getbitu/setbitu from rtkcmn.c
// ============================================================================

int UbxParser::decodeGalileoInav(int sat, const uint8_t* data, size_t length) {
    if (length < 32) return 0;
    
    unsigned char buff[32], crc_buff[26] = {0};
    memcpy(buff, data, 32);
    
    int part1 = getbitu(buff, 0, 1);
    int page1 = getbitu(buff, 1, 1);
    int part2 = getbitu(buff + 16, 0, 1);
    int page2 = getbitu(buff + 16, 1, 1);
    
    if (page1 == 1 || page2 == 1) return 0;
    if (part1 != 0 || part2 != 1) return 0;
    
    // CRC check using RTKLIB function
    for (int i = 0, j = 4; i < 15; i++, j += 8) {
        setbitu(crc_buff, j, 8, getbitu(buff, i*8, 8));
    }
    for (int i = 0, j = 118; i < 11; i++, j += 8) {
        setbitu(crc_buff, j, 8, getbitu(buff + 16, i*8, 8));
    }
    if (rtk_crc24q(crc_buff, 25) != getbitu(buff + 16, 82, 24)) return 0;
    
    int type = getbitu(buff, 2, 6);
    if (type > 6) return 0;
    
    if (type == 2) subfrm[sat-1][112] = 0;
    
    // Save page data
    int k = type * 16;
    for (int i = 0, j = 2; i < 14; i++, j += 8) {
        subfrm[sat-1][k++] = getbitu(buff, j, 8);
    }
    for (int i = 0, j = 2; i < 2; i++, j += 8) {
        subfrm[sat-1][k++] = getbitu(buff + 16, j, 8);
    }
    
    subfrm[sat-1][112] |= (1 << type);
    
    if (subfrm[sat-1][112] != 0x7F) return 0;
    
    // Decode using RTKLIB function
    eph_t eph = {0};
    if (!decode_gal_inav(subfrm[sat-1], &eph) || eph.sat != sat) return 0;
    
    nav.eph[sat-1] = eph;
    
    if (loggingEnabled && logFile) {
        fprintf(logFile, "  Galileo Eph: sat=%d iode=%d\n", sat, eph.iode);
        fflush(logFile);
    }
    
    return 2;
}

// ============================================================================
// BEIDOU D1 DECODER
// Uses RTKLIB's decode_bds_d1() from rcvraw.c
// Uses RTKLIB's setbitu from rtkcmn.c
// ============================================================================

int UbxParser::decodeBeidouD1(int sat) {
    eph_t eph = {0};
    
    // Use RTKLIB decoder
    if (!decode_bds_d1(subfrm[sat-1], &eph)) return 0;
    
    eph.sat = sat;
    nav.eph[sat-1] = eph;
    
    if (loggingEnabled && logFile) {
        fprintf(logFile, "  BeiDou D1 Eph: sat=%d iode=%d\n", sat, eph.iode);
        fflush(logFile);
    }
    
    return 2;
}

// ============================================================================
// BEIDOU D2 DECODER
// Uses RTKLIB's decode_bds_d2() from rcvraw.c
// ============================================================================

int UbxParser::decodeBeidouD2(int sat) {
    eph_t eph = {0};
    
    // Use RTKLIB decoder
    if (!decode_bds_d2(subfrm[sat-1], &eph)) return 0;
    
    eph.sat = sat;
    nav.eph[sat-1] = eph;
    
    if (loggingEnabled && logFile) {
        fprintf(logFile, "  BeiDou D2 Eph: sat=%d iode=%d\n", sat, eph.iode);
        fflush(logFile);
    }
    
    return 2;
}

// ============================================================================
// GLONASS STRING DECODER
// Uses RTKLIB's test_glostr() and decode_glostr() from rcvraw.c
// Uses RTKLIB's getbitu, satsys from rtkcmn.c
// ============================================================================

int UbxParser::decodeGlonassStr(int sat, const uint8_t* data, size_t length, int freqId) {
    if (length < 16) return 0;
    
    unsigned char buff[64];
    int k = 0;
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            buff[k++] = data[i*4 + (3-j)];
        }
    }
    
    // Use RTKLIB hamming test
    if (!test_glostr(buff)) return 0;
    
    int m = getbitu(buff, 1, 4);
    if (m < 1 || m > 15) return 0;
    
    // Check frame ID
    unsigned char *fid = subfrm[sat-1] + 150;
    if (fid[0] != buff[12] || fid[1] != buff[13]) {
        for (int i = 0; i < 4; i++) {
            memset(subfrm[sat-1] + i*10, 0, 10);
        }
        memcpy(fid, buff + 12, 2);
    }
    
    memcpy(subfrm[sat-1] + (m-1)*10, buff, 10);
    
    if (m != 4) return 0;
    
    geph_t geph = {0};
    geph.tof = current_time;
    
    // Use RTKLIB decoder
    if (!decode_glostr(subfrm[sat-1], &geph) || geph.sat != sat) return 0;
    
    geph.frq = freqId - 7;
    
    int prn;
    satsys(sat, &prn);
    if (prn < 1 || prn > NSATGLO) return 0;
    
    nav.geph[prn-1] = geph;
    
    if (loggingEnabled && logFile) {
        fprintf(logFile, "  GLONASS Eph: sat=%d frq=%d iode=%d\n", sat, geph.frq, geph.iode);
        fflush(logFile);
    }
    
    return 2;
}

// ============================================================================
// RXM-RAWX HANDLER
// Uses RTKLIB's gpst2time, satno, satsys from rtkcmn.c
// ============================================================================

int UbxParser::handleRawx(const uint8_t* payload, size_t length) {
    if (length < 16) return 0;

    double rcvTow;
    uint16_t week;
    uint8_t numMeas;
    
    memcpy(&rcvTow, payload, 8);
    memcpy(&week, payload + 8, 2);
    memcpy(&numMeas, payload + 11, 1);
    
    if (week == 0) return 0;
    if (length < 16 + numMeas * 32) return 0;
    
    // Use RTKLIB time function
    current_time = gpst2time(week, rcvTow);
    obs.n = 0;
    
    if (loggingEnabled && logFile) {
        fprintf(logFile, "RAWX: Week=%u TOW=%.3f NumMeas=%u\n", week, rcvTow, numMeas);
    }
    
    int n = 0;
for (int i = 0; i < numMeas && n < MAXOBS; i++) {
    const uint8_t* rec = payload + 16 + i * 32;
    
    double prMes, cpMes;
    float doMes;
    uint8_t gnssId, svId, cno, cpStdev, trkStat;
    uint16_t locktime;
    
    memcpy(&prMes, rec + 0, 8);
    memcpy(&cpMes, rec + 8, 8);
    memcpy(&doMes, rec + 16, 4);
    gnssId = rec[20];
    svId = rec[21];
    locktime = rec[24] | (rec[25] << 8);
    cno = rec[26];
    cpStdev = rec[28];
    trkStat = rec[30];
    
    // DEBUG 1: Check raw data
    if (i < 3) {  // Only print first 3 to avoid spam
        printf("DEBUG [%d]: gnssId=%u svId=%u PR=%.3f trkStat=0x%02X\n", 
               i, gnssId, svId, prMes, trkStat);
    }
    
    int sys = ubxSys(gnssId);
    
    // DEBUG 2: Check system conversion
    if (i < 3) {
        printf("DEBUG: sys=0x%02X (GPS=0x01, GAL=0x08, BDS=0x20)\n", sys);
    }
    
    if (!sys) continue;
    
    int prn = svId;
    if (sys == SYS_QZS) prn += 192;
    
    // Use RTKLIB function
    int sat = satno(sys, prn);
    
    // DEBUG 3: Check satellite number
    if (i < 3) {
        printf("DEBUG: prn=%d -> sat=%d\n", prn, sat);
    }
    
    if (!sat) continue;
    
    int cpstd = cpStdev & 15;
    bool cpValid = (trkStat & 0x02) && (cpMes != -0.5) && (cpstd <= 5);
    bool prValid = (trkStat & 0x01);
    
    obsd_t* obs_data = &obs.data[n];
    
    // DEBUG 4: Before setting data
    if (i < 3) {
        printf("DEBUG: About to set obs.data[%d]\n", n);
    }
    
    memset(obs_data, 0, sizeof(obsd_t));
    
    obs_data->time = current_time;
    obs_data->sat = sat;
    obs_data->P[0] = prValid ? prMes : 0.0;
    obs_data->L[0] = cpValid ? cpMes : 0.0;
    obs_data->D[0] = doMes;
    obs_data->SNR[0] = cno * 4;
    
    // DEBUG 5: After setting data
    if (i < 3) {
        printf("DEBUG: Set obs.data[%d].sat=%d P[0]=%.3f\n", 
               n, obs_data->sat, obs_data->P[0]);
    }
    
    int slip = (locktime == 0 || locktime < lockt[sat-1][0]) ? 1 : 0;
    lockt[sat-1][0] = locktime;
    
    bool halfv = (trkStat & 0x04) ? true : false;
    bool halfc_curr = (trkStat & 0x08) ? true : false;
    
    obs_data->LLI[0] = 0;
    if (cpValid) {
        if (slip) obs_data->LLI[0] |= LLI_SLIP;
        if (!halfv) obs_data->LLI[0] |= LLI_HALFC;
        if (halfc_curr != halfc[sat-1][0]) obs_data->LLI[0] |= LLI_SLIP;
        halfc[sat-1][0] = halfc_curr;
    }
    
    if (sys == SYS_CMP) obs_data->code[0] = CODE_L1I;
    else if (sys == SYS_GAL) obs_data->code[0] = CODE_L1X;
    else obs_data->code[0] = CODE_L1C;
    
    n++;
}

// DEBUG 6: After loop - check final result
obs.n = n;
printf("\nDEBUG: Loop finished. Total valid obs: %d\n", n);
printf("DEBUG: Verifying first 3 stored observations:\n");
for (int i = 0; i < obs.n && i < 3; i++) {
    int prn_check;
    int sys_check = satsys(obs.data[i].sat, &prn_check);
    printf("  obs.data[%d]: sat=%d sys=0x%02X prn=%d P[0]=%.3f\n", 
           i, obs.data[i].sat, sys_check, prn_check, obs.data[i].P[0]);
}
printf("\n");
    
    obs.n = n;
    
    if (loggingEnabled && logFile) {
        fprintf(logFile, "RAWX: Parsed %d observations\n\n", n);
        fflush(logFile);
    }
    
    return n > 0 ? 1 : 0;
}

// ============================================================================
// RXM-SFRBX HANDLER
// Uses RTKLIB's setbitu, satno, satsys from rtkcmn.c
// ============================================================================

int UbxParser::decodeSfrbx(int sys, int sat, const uint8_t* data, size_t length, int freqId) {
    if (sys == SYS_GPS || sys == SYS_QZS) {
        if (length < 40) return 0;
        
        unsigned int words[10];
        for (int i = 0; i < 10; i++) {
            uint32_t word;
            memcpy(&word, data + i*4, 4);
            words[i] = (word >> 6) & 0xFFFFFF;
        }
        
        int id = (words[1] >> 2) & 7;
        if (id < 1 || id > 5) return 0;
        
        // Use RTKLIB's setbitu
        for (int i = 0; i < 10; i++) {
            setbitu(subfrm[sat-1] + (id-1)*30, i*24, 24, words[i]);
        }
        
        if (id == 3) return decodeGpsFrame(sat);
        
        if (id == 4 || id == 5) {
            // Use RTKLIB decoder
            decode_frame(subfrm[sat-1] + (id-1)*30, NULL, nav.alm,
                        sys == SYS_GPS ? nav.ion_gps : nav.ion_qzs,
                        sys == SYS_GPS ? nav.utc_gps : nav.utc_qzs,
                        &nav.leaps);
            return 9;
        }
        
    } else if (sys == SYS_GAL) {
        return decodeGalileoInav(sat, data, length);
        
    } else if (sys == SYS_CMP) {
        if (length < 40) return 0;
        
        unsigned int words[10];
        for (int i = 0; i < 10; i++) {
            memcpy(&words[i], data + i*4, 4);
            words[i] &= 0x3FFFFFFF;
        }
        
        int prn;
        satsys(sat, &prn);  // Use RTKLIB function
        int id = (words[0] >> 12) & 0x07;
        
        if (prn > 5) { // IGSO/MEO - D1
            if (id < 1 || id > 5) return 0;
            
            for (int i = 0; i < 10; i++) {
                setbitu(subfrm[sat-1] + (id-1)*38, i*30, 30, words[i]);
            }
            
            if (id == 3) return decodeBeidouD1(sat);
            
        } else { // GEO - D2
            if (id != 1) return 0;
            
            int pgn = (words[1] >> 14) & 0x0F;
            if (pgn < 1 || pgn > 10) return 0;
            
            for (int i = 0; i < 10; i++) {
                setbitu(subfrm[sat-1] + (pgn-1)*38, i*30, 30, words[i]);
            }
            
            if (pgn == 10) return decodeBeidouD2(sat);
        }
        
    } else if (sys == SYS_GLO) {
        return decodeGlonassStr(sat, data, length, freqId);
    }
    
    return 0;
}

int UbxParser::handleSfrbx(const uint8_t* payload, size_t length) {
    if (length < 8) return 0;

    uint8_t gnssId = payload[0];
    uint8_t svId = payload[1];
    uint8_t freqId = payload[3];
    
    int sys = ubxSys(gnssId);
    if (!sys) return 0;
    
    int prn = svId;
    if (sys == SYS_QZS) prn += 192;
    
    // Use RTKLIB function
    int sat = satno(sys, prn);
    if (!sat) return 0;
    
    if (loggingEnabled && logFile) {
        fprintf(logFile, "SFRBX: GNSS=%u SV=%u Sat=%d FreqId=%u\n", gnssId, svId, sat, freqId);
    }
    
    int result = decodeSfrbx(sys, sat, payload + 8, length - 8, freqId);
    
    if (loggingEnabled && logFile) {
        if (result == 2) fprintf(logFile, "  Successfully decoded ephemeris\n");
        else if (result == 9) fprintf(logFile, "  Successfully decoded almanac/iono\n");
        fprintf(logFile, "\n");
        fflush(logFile);
    }
    
    return result;
}

// ============================================================================
// MESSAGE HANDLER (NEW - moved from main)
// ============================================================================

void UbxParser::handleUbxMessage(const UbxMessage& msg) {
    uint8_t msgClass = msg.getClass();
    uint8_t msgId = msg.getId();
    
    // Handle RXM-RAWX
    if (msgClass == UbxConstants::UBX_CLASS_RXM && msgId == UbxConstants::UBX_RXM_RAWX) {
        stats.rawxCount++;
        int result = handleRawx(msg.getPayload(), msg.getLength());
        if (result && rawxCallback) {
            rawxCallback(obs);
        }
    }
    // Handle RXM-SFRBX
    else if (msgClass == UbxConstants::UBX_CLASS_RXM && msgId == UbxConstants::UBX_RXM_SFRBX) {
        stats.sfrbxCount++;
        int result = handleSfrbx(msg.getPayload(), msg.getLength());
        if (result == 2) {
            stats.ephCount++;
            if (ephCallback) {
                ephCallback(result);
            }
        }
    }
    // Handle ACK/NAK
    else if (msgClass == UbxConstants::UBX_CLASS_ACK) {
        if (msgId == UbxConstants::UBX_ACK_ACK) {
            stats.ackCount++;
        } else if (msgId == UbxConstants::UBX_ACK_NAK) {
            stats.nakCount++;
        }
    }
    // Count others
    else {
        stats.otherCount++;
    }
}