#include "ubx_parser.hpp"
#include <cstdio>
#include <cstring>

UbxParser::UbxParser() : logFile(nullptr), loggingEnabled(false) 
{
    memset(&obs, 0, sizeof(obs));
    memset(&nav, 0, sizeof(nav));

    // allocate obs_t
    obs.nmax = MAXOBS; 
    obs.n = 0;
    obs.data = (obsd_t*)malloc(sizeof(obsd_t) * obs.nmax);
    if (obs.data) memset(obs.data, 0, sizeof(obsd_t) * obs.nmax); 
    // TODO:: what happens if malloc fails?

    // allocate nav_t
    nav.nmax = 128; nav.n = 0;
    nav.ngmax = 128; nav.ng = 0;
    nav.nsmax = 128; nav.ns = 0;
    nav.nemax = 128; nav.ne = 0;
    nav.ncmax = 128; nav.nc = 0;
    nav.namax = 128; nav.na = 0;
    nav.ntmax = 128; nav.nt = 0;
    nav.nnmax = 128; nav.nn = 0;
    nav.eph = (eph_t*)malloc(sizeof(eph_t) * nav.nmax);
    nav.geph = (geph_t*)malloc(sizeof(geph_t) * nav.ngmax);
    nav.seph = (seph_t*)malloc(sizeof(seph_t) * nav.nsmax);
    nav.peph = (peph_t*)malloc(sizeof(peph_t) * nav.nemax);
    nav.pclk = (pclk_t*)malloc(sizeof(pclk_t) * nav.ncmax);
    nav.alm = (alm_t*)malloc(sizeof(alm_t) * nav.namax);
    nav.tec = (tec_t*)malloc(sizeof(tec_t) * nav.ntmax);
    nav.stec = (stec_t*)malloc(sizeof(stec_t) * nav.nnmax);
    if (nav.eph) memset(nav.eph, 0, sizeof(eph_t) * nav.nmax);
    if (nav.geph) memset(nav.geph, 0, sizeof(geph_t) * nav.ngmax);
    if (nav.seph) memset(nav.seph, 0, sizeof(seph_t) * nav.nsmax);
    if (nav.peph) memset(nav.peph, 0, sizeof(peph_t) * nav.nemax);
    if (nav.pclk) memset(nav.pclk, 0, sizeof(pclk_t) * nav.ncmax);
    if (nav.alm) memset(nav.alm, 0, sizeof(alm_t) * nav.namax);
    if (nav.tec) memset(nav.tec, 0, sizeof(tec_t) * nav.ntmax);
    if (nav.stec) memset(nav.stec, 0, sizeof(stec_t) * nav.nnmax);

    // TODO:: what happens if malloc fails?
}

UbxParser::~UbxParser() {
    if (logFile) fclose(logFile);
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

/**
 * Handle RXM-RAWX message: extract time, week, and number of observations.
 */
#pragma pack(1)
typedef struct
{
    double prMes;
    double cpMes;
    float doMes;
    uint8_t gnssId;
    uint8_t svId;
    uint8_t reserved2;
    uint8_t freqId;
    uint16_t locktime;
    uint8_t cno; 
    uint8_t prStdev;     // resolution = 0.01 m
    uint8_t cpStdev;     // 0.004 cycles
    uint8_t doStdev;     // 0.002 Hz
    uint8_t trkStat;
    uint8_t reserved3;
} RAWX_record_t;
#pragma pack()

void print_raw_x_data(RAWX_record_t* data)
{
    printf("  RAWX Record:\n");
    printf("    PR Mes: %.3f m\n", data->prMes);
    printf("    CP Mes: %.3f cycles\n", data->cpMes);
    printf("    DO Mes: %.3f Hz\n", data->doMes);
    printf("    GNSS ID: %u\n", data->gnssId);
    printf("    SV ID: %u\n", data->svId);
    printf("    Freq ID: %u\n", data->freqId);
    printf("    Lock Time: %u ms\n", data->locktime);
    printf("    C/N0: %u dB-Hz\n", data->cno);
    printf("    PR Std Dev: %.2f m\n", data->prStdev * 0.01);
    printf("    CP Std Dev: %.3f cycles\n", data->cpStdev * 0.004);
    printf("    DO Std Dev: %.3f Hz\n", data->doStdev * 0.002);
    printf("    Track Status: 0x%02X\n", data->trkStat);
}

int UbxParser::handleRawx(const uint8_t* payload, size_t length) {
    //
    //copy this data to rawx_t buffer according to ,,,,
    //run ublox.c decoder to rawx
    //
    if (length < 16) return 0;

    double rcvTow;
    uint16_t week;
    int8_t leaps;
    uint8_t numMeas;
    uint8_t rcvStat;
    RAWX_record_t* rawx_rec = nullptr;

    printf("Handling RXM-RAWX message of length %zu\n", length);
    memcpy(&rcvTow, payload, 8);
    memcpy(&week, payload + 8, 2);
    memcpy(&leaps, payload + 10, 1);
    memcpy(&numMeas, payload + 11, 1);
    memcpy(&rcvStat, payload + 12, 1);

    printf("  TOW: %.3f, Week: %u, NumMeas: %u\n", rcvTow, week, numMeas);

    gtime_t t = gpst2time(week, rcvTow);
    obs.n = numMeas;
    if (numMeas > MAXOBS) obs.n = MAXOBS;

    for (int i = 0; i < obs.n; i++) 
    {
        rawx_rec = (RAWX_record_t*)(payload + 16 + i * sizeof(RAWX_record_t));
        print_raw_x_data(rawx_rec);

        obsd_t* cur_ptr = &obs.data[i];
        // memset(cur_ptr, 0, sizeof(obsd_t));
        // cur_ptr->time = t;
        // cur_ptr->sat = i + 1;
    }

    // if (loggingEnabled && logFile) 
    // {
    //     fprintf(logFile, "> RAWX week=%u tow=%.3f n=%d\n", week, rcvTow, obs.n);
    //     fflush(logFile);
    // }

    return 1;
}

/**
 * Handle RXM-SFRBX message: extract GNSS ID, SV ID, and raw frame bytes.
 */
int UbxParser::handleSfrbx(const uint8_t* payload, size_t length) {
    if (length < 8) return 0;

    uint8_t gnssId = payload[0];
    uint8_t svId = payload[1];
    uint8_t numWords = payload[2];
    uint8_t version = payload[3];

    if (loggingEnabled && logFile) {
        fprintf(logFile, "> SFRBX gnss=%u sv=%u words=%u ver=%u len=%zu\n",
                gnssId, svId, numWords, version, length);
        fflush(logFile);
    }

    // You could later integrate this with eph_t decoding logic.
    return 1;
}

void UbxParser::logObs(const obsd_t* data, int n) {
    if (!loggingEnabled || !logFile) return;
    for (int i = 0; i < n; i++) {
        const obsd_t& o = data[i];
        fprintf(logFile, "%4d %.3f\n", o.sat, time2gpst(o.time, nullptr));
    }
    fflush(logFile);
}

void UbxParser::logNav(const nav_t* nav) {
    if (!loggingEnabled || !logFile) return;
    fprintf(logFile, "# NAV entries: %d\n", nav->n);
    fflush(logFile);
}
