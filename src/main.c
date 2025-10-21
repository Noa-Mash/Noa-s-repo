#include "rtklib.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <signal.h>
#define STR_AUTO 16  // TODO: Temporary fix. Replace with explicit STR_* (e.g., STR_FILE, STR_SERIAL) for clarity.

typedef enum { MODE_UNDEF=0, MODE_SPP=1, MODE_RTK=2 } run_mode_t;

typedef struct {
    run_mode_t mode;
    int is_static;                 /* 0: kinematic (default), 1: static */
    int navsys_mask;               /* SYS_* bitmask */
    char rover_uri[1024];
    char base_uri[1024];
    char out_pos[1024]; /* look into removing if using nmea */
    char out_nmea[1024];
    char out_trace[1024];
} app_cfg_t;

static volatile int g_stop = 0; /* needs to be integrated into general program stop sign*/
static void on_sigint(int sig) { (void)sig; g_stop = 1; }

static int starts_with(const char *s, const char *pfx) {
    return s && pfx && strncmp(s, pfx, strlen(pfx)) == 0;
}

/* Trim helpers */
static char *ltrim(char *s){ while(*s && isspace((unsigned char)*s)) s++; return s; }
static void rtrim(char *s){ size_t n=strlen(s); while(n&&isspace((unsigned char)s[n-1])) s[--n]=0; }

/* Parse comma-separated GNSS list like "GPS,GAL,GLO,BDS,QZS" into SYS_* mask */
static int parse_navsys(const char *csv) {
    int mask = 0;
    char buf[128]; strncpy(buf, csv?csv:"", sizeof(buf)-1); buf[sizeof(buf)-1]=0;
    for (char *tok = strtok(buf, ","); tok; tok=strtok(NULL, ",")) {
        if (!strcmp(tok,"GPS")) mask |= SYS_GPS;
        else if (!strcmp(tok,"GAL")) mask |= SYS_GAL;
        else if (!strcmp(tok,"GLO")) mask |= SYS_GLO;
        else if (!strcmp(tok,"BDS") || !strcmp(tok,"CMP")) mask |= SYS_CMP;
        else if (!strcmp(tok,"QZS")) mask |= SYS_QZS;
    }
    return mask ? mask : (SYS_GPS|SYS_GLO|SYS_GAL|SYS_QZS|SYS_CMP);
}

/* Minimal INI reader: supports [section] and key = value (no escapes) */
static int load_config(const char *path, app_cfg_t *cfg) {
    FILE *f = fopen(path, "r");
    if (!f) return 0;

    char line[2048], section[64]="";
    while (fgets(line, sizeof(line), f)) {
        char *p = ltrim(line); rtrim(p);
        if (!*p || *p=='#' || *p==';') continue;
        if (*p=='[') {
            char *q = strchr(p, ']');
            if (q) { *q=0; strncpy(section, p+1, sizeof(section)-1); section[sizeof(section)-1]=0; }
            continue;
        }
        char *eq = strchr(p, '=');
        if (!eq) continue;
        *eq = 0;
        char *key = ltrim(p); rtrim(key);
        char *val = ltrim(eq+1); rtrim(val);

        if (!strcmp(section,"mode")) {
            if (!strcmp(key,"type")) {
                if (!strcmp(val,"spp")) cfg->mode = MODE_SPP;
                else if (!strcmp(val,"rtk")) cfg->mode = MODE_RTK;
            } else if (!strcmp(key,"kinematic")) {
                cfg->is_static = (!strcmp(val,"true")||!strcmp(val,"1")) ? 0 : cfg->is_static;
            } else if (!strcmp(key,"static")) {
                cfg->is_static = (!strcmp(val,"true")||!strcmp(val,"1")) ? 1 : cfg->is_static;
            }
        }
        else if (!strcmp(section,"rover") && !strcmp(key,"uri")) {
            strncpy(cfg->rover_uri, val, sizeof(cfg->rover_uri)-1);
        }
        else if (!strcmp(section,"base") && !strcmp(key,"uri")) {
            strncpy(cfg->base_uri, val, sizeof(cfg->base_uri)-1);
        }
        else if (!strcmp(section,"output")) {
            if (!strcmp(key,"pos"))   strncpy(cfg->out_pos,   val, sizeof(cfg->out_pos)-1);
            else if (!strcmp(key,"nmea"))  strncpy(cfg->out_nmea,  val, sizeof(cfg->out_nmea)-1);
            else if (!strcmp(key,"trace")) strncpy(cfg->out_trace, val, sizeof(cfg->out_trace)-1);
        }
        else if (!strcmp(section,"systems") && !strcmp(key,"enabled")) {
            cfg->navsys_mask = parse_navsys(val);
        }
    }
    fclose(f);
    return 1;
}

static void print_config(const app_cfg_t *c) {
    printf("Config:\n");
    printf("  mode       : %s\n", c->mode==MODE_RTK?"RTK":(c->mode==MODE_SPP?"SPP":"UNDEF"));
    printf("  kinematics : %s\n", c->is_static? "STATIC":"KINEMATIC");
    printf("  rover_uri  : %s\n", c->rover_uri[0]?c->rover_uri:"<none>");
    printf("  base_uri   : %s\n", c->base_uri[0]?c->base_uri:"<none>");
    printf("  out.pos    : %s\n", c->out_pos);
    if (c->out_nmea[0])  printf("  out.nmea   : %s\n", c->out_nmea);
    if (c->out_trace[0]) printf("  out.trace  : %s\n", c->out_trace);
}

int main(int argc, char **argv) {
    app_cfg_t C;
    memset(&C, 0, sizeof(C));
    C.mode        = MODE_RTK;
    C.is_static   = 0;
    C.navsys_mask = SYS_GPS|SYS_GLO|SYS_GAL|SYS_QZS|SYS_CMP;
    strncpy(C.out_pos,   "live.pos",   sizeof(C.out_pos)-1);
    C.out_nmea[0]  = 0;
    C.out_trace[0] = 0;

    const char *cfg_path = (argc > 1) ? argv[1] : "config.ini";
    if (!load_config(cfg_path, &C)) {
        printf("Note: config file '%s' not found, using built-in defaults.\n", cfg_path);
    }

    if (C.mode==MODE_UNDEF) {
        fprintf(stderr, "Error: mode is undefined. Set [mode] type = spp|rtk in config.ini\n");
        return 2;
    }
    if (!C.rover_uri[0]) {
        fprintf(stderr, "Error: rover URI is empty. Set [rover] uri = ... in config.ini\n");
        return 2;
    }
    if (C.mode==MODE_RTK && !C.base_uri[0]) {
        fprintf(stderr, "Error: RTK mode requires [base] uri in config.ini\n");
        return 2;
    }

    print_config(&C);

    // Properly initialize structures with defaults
    prcopt_t prcopt = prcopt_default;
    solopt_t solopt = solopt_default;
    filopt_t filopt = {0};

    // Configure processing options
    if (C.mode == MODE_SPP) {
        prcopt.mode   = PMODE_SINGLE;
        prcopt.modear  = ARMODE_OFF;
    } else {
        prcopt.mode   = C.is_static ? PMODE_STATIC : PMODE_KINEMA;
        prcopt.modear  = ARMODE_FIXHOLD;
    }
    prcopt.navsys  = C.navsys_mask;
    prcopt.ionoopt = IONOOPT_BRDC;
    prcopt.tropopt = TROPOPT_SAAS;

    // Configure solution options - FIXED: Use proper RTKLIB constants
    solopt.posf    = SOLF_LLH;
    solopt.times   = 1;
    solopt.timef   = 1;
    solopt.outhead = 1;
    solopt.datum   = DATUM_WGS84;  // Use proper RTKLIB constant
    solopt.height  = HGT_ELLIPS;   // Use proper RTKLIB constant

    if (C.out_trace[0]) {
        rtkopenstat(C.out_trace, 3);
    }

    static const char EMPTY[] = "";
    int   str_types[8] = {STR_NONE};
    int   str_fmts [8] = {0};
    char *str_paths[8];
    for (int i = 0; i < 8; i++) str_paths[i] = (char *)EMPTY;

    str_types[0] = STR_AUTO;
    str_paths[0] = C.rover_uri;
    if (starts_with(C.rover_uri, "serial://") || starts_with(C.rover_uri, "file://")) {
        str_fmts[0] = STRFMT_UBX;
    } else if (starts_with(C.rover_uri, "ntrip://") || starts_with(C.rover_uri, "tcpcli://")) {
        str_fmts[0] = STRFMT_RTCM3;
    } else {
        str_fmts[0] = STRFMT_UBX;
    }

    if (C.mode == MODE_RTK) {
        str_types[1] = STR_AUTO;
        str_paths[1] = C.base_uri;
        str_fmts [1] = STRFMT_RTCM3;
    }

    str_types[3] = STR_FILE;  str_paths[3] = C.out_pos;

    if (C.out_nmea[0]) {
        str_types[4] = STR_FILE; str_paths[4] = C.out_nmea;
        solopt.nmeaintv[0] = 1.0;
        solopt.nmeaintv[1] = 1.0;
    }

    rtksvr_t svr; char errmsg[1024] = {0};
    memset(&svr, 0, sizeof(svr));
    rtksvrinit(&svr);

    const int cycle_ms = 10;
    const int buffsize = 32768;
    const int navsel   = 0;
    char *rcvopt[3]    = { "", "", "" };
    const int nmea_cycle = 0, nmea_req = 0;
    char *nmea_pos[3]  = { "0.0", "0.0", "0.0" };

    char *cmds[3] = { "", "", "" };
    stream_t *moni = NULL;
    const double nmeapos[3] = {0};

    int ok = rtksvrstart(&svr, cycle_ms, buffsize,
                     str_types, str_paths, str_fmts, navsel,
                     cmds, rcvopt, nmea_cycle, nmea_req,
                     nmeapos, &prcopt, &solopt, moni);

    if (!ok) {
        fprintf(stderr, "rtksvrstart failed: %s\n", errmsg);
        if (C.out_trace[0]) rtkclosestat();
        return 3;
    }

    printf("RTKLIB server started. Press Ctrl+C to stop.\n");
    signal(SIGINT, on_sigint);

    while (!g_stop) {
#if defined(_WIN32)
        Sleep(10);
#else
        struct timespec ts = {0, 10*1000*1000};
        nanosleep(&ts, NULL);
#endif
    }

    printf("\nStopping server...\n");
    rtksvrstop(&svr, cmds);
    if (C.out_trace[0]) rtkclosestat();
    printf("Bye.\n");
    return 0;
}