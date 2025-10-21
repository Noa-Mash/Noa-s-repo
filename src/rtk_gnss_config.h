#ifndef __RTK_GNSS_CONFIG_H
#define __RTK_GNSS_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stddef.h>  /* For size_t */

/* Configuration limits and defaults */
#define RTK_CONFIG_MAX_PATH_LEN    1024
#define RTK_CONFIG_MAX_URI_LEN     1024
#define RTK_CONFIG_DEFAULT_OUTFILE "live.pos"

/* Navigation system bitmask definitions remove once a final header is included */
#define SYS_NONE    0x00                /* navigation system: none */
#define SYS_GPS     0x01                /* navigation system: GPS */
#define SYS_SBS     0x02                /* navigation system: SBAS */
#define SYS_GLO     0x04                /* navigation system: GLONASS */
#define SYS_GAL     0x08                /* navigation system: Galileo */
#define SYS_QZS     0x10                /* navigation system: QZSS */
#define SYS_CMP     0x20                /* navigation system: BeiDou */
#define SYS_IRN     0x40                /* navigation system: IRNSS */
#define SYS_LEO     0x80                /* navigation system: LEO */
#define SYS_ALL     0xFF                /* navigation system: all */

typedef enum 
{ 
    MODE_UNDEF=0, 
    MODE_SPP=1, 
    MODE_RTK=2 
} run_mode_t;

typedef struct {
    run_mode_t mode;
    int is_static;                 /* 0: kinematic (default), 1: static */
    int navsys_mask;               /* SYS_* bitmask */
    char rover_uri[RTK_CONFIG_MAX_URI_LEN];            /* Rover data source */
    char base_uri[RTK_CONFIG_MAX_URI_LEN];             /* Base station data source */
    char out_pos[RTK_CONFIG_MAX_PATH_LEN];             /* Position output file */
    char out_nmea[RTK_CONFIG_MAX_PATH_LEN];            /* NMEA output file */
    char out_trace[RTK_CONFIG_MAX_PATH_LEN];
} rtk_gnss_config_t;


int rtk_gnss_config_init(rtk_gnss_config_t* config, const char* config_file_path);

void rtk_gnss_config_deinit(rtk_gnss_config_t* config);

void rtk_gnss_config_print(const rtk_gnss_config_t* config);


#ifdef __cplusplus
}
#endif

#endif  // __RTK_GNSS_CONFIG_H
