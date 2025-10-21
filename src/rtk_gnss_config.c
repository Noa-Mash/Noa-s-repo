#include "rtk_gnss_config.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include "ini.h"

#define MATCH(s, n) (strcmp(section, s) == 0 && strcmp(name, n) == 0)

/* Parse comma-separated GNSS list like "GPS,GAL,GLO,BDS,QZS" into bitmask */
static int parse_navsys(const char *csv) {
    int mask = 0;
    
    if (!csv || strlen(csv) == 0) {
        return 0; // Return 0 for empty, caller will use default
    }
    
    // Copy string because strtok modifies it
    char buf[128]; 
    strncpy(buf, csv, sizeof(buf) - 1); 
    buf[sizeof(buf) - 1] = '\0';
    
    // Parse each token separated by commas
    char *tok = strtok(buf, ",");
    while (tok != NULL) {
        // Trim whitespace from token (in case of "GPS, GAL, GLO")
        while (*tok == ' ' || *tok == '\t') tok++; // Skip leading
        char *end = tok + strlen(tok) - 1;
        while (end > tok && (*end == ' ' || *end == '\t')) *end-- = '\0'; // Remove trailing
        
        // Match against known systems using RTKLIB constants
        if (strcmp(tok, "GPS") == 0) {
            mask |= SYS_GPS;     // 0x01
        } else if (strcmp(tok, "GAL") == 0 || strcmp(tok, "GALILEO") == 0) {
            mask |= SYS_GAL;     // 0x08
        } else if (strcmp(tok, "GLO") == 0 || strcmp(tok, "GLONASS") == 0) {
            mask |= SYS_GLO;     // 0x04
        } else if (strcmp(tok, "BDS") == 0 || strcmp(tok, "CMP") == 0 || strcmp(tok, "BEIDOU") == 0) {
            mask |= SYS_CMP;     // 0x20
        } else if (strcmp(tok, "QZS") == 0 || strcmp(tok, "QZSS") == 0) {
            mask |= SYS_QZS;     // 0x10
        } else if (strcmp(tok, "SBS") == 0 || strcmp(tok, "SBAS") == 0) {
            mask |= SYS_SBS;     // 0x02
        } else if (strcmp(tok, "IRN") == 0 || strcmp(tok, "IRNSS") == 0) {
            mask |= SYS_IRN;     // 0x40
        } else if (strcmp(tok, "LEO") == 0) {
            mask |= SYS_LEO;     // 0x80
        } else {
            printf("Warning: Unknown GNSS system '%s'\n", tok);
        }
        
        tok = strtok(NULL, ",");
    }
    
    // Return parsed mask, or default if no valid systems found
    return mask ? mask : (SYS_GPS|SYS_GLO|SYS_GAL|SYS_QZS|SYS_CMP);
}

/* INI handler callback - called by ini_parse for each key=value pair */
static int ini_handler_entry(void* user, const char* section, const char* name, const char* value) {
    rtk_gnss_config_t* config = (rtk_gnss_config_t*)user; /* casting */
    
    // Validate inputs (paranoid but safe)
    if (!config || !section || !name || !value) {
        return 0; // Error - stop parsing
    }
    
    // [mode] section
    if (MATCH("mode", "type")) {
        if (strcmp(value, "spp") == 0) {
            config->mode = MODE_SPP;
        } else if (strcmp(value, "rtk") == 0) {
            config->mode = MODE_RTK;
        } else {
            printf("Error: Invalid mode type '%s'. Must be 'spp' or 'rtk'\n", value);
            return 0; // Stop parsing on error
        }
    }
    else if (MATCH("mode", "kinematic")) {
        if (strcmp(value, "true") == 0 || strcmp(value, "1") == 0) {
            config->is_static = 0; // kinematic = not static
        } else if (strcmp(value, "false") == 0 || strcmp(value, "0") == 0) {
            config->is_static = 1; // not kinematic = static
        } else {
            printf("Error: Invalid kinematic value '%s'. Must be 'true' or 'false'\n", value);
            return 0;
        }
    }
    else if (MATCH("mode", "static")) {
        if (strcmp(value, "true") == 0 || strcmp(value, "1") == 0) {
            config->is_static = 1; // static = stationary
        } else if (strcmp(value, "false") == 0 || strcmp(value, "0") == 0) {
            config->is_static = 0; // not static = kinematic
        } else {
            printf("Error: Invalid static value '%s'. Must be 'true' or 'false'\n", value);
            return 0;
        }
    }
    
    // [rover] section
    else if (MATCH("rover", "uri")) {
        if (strlen(value) >= sizeof(config->rover_uri)) {
            printf("Error: Rover URI too long (max %zu chars)\n", sizeof(config->rover_uri) - 1);
            return 0;
        }
        strncpy(config->rover_uri, value, sizeof(config->rover_uri) - 1);
        config->rover_uri[sizeof(config->rover_uri) - 1] = '\0';
    }
    
    // [base] section
    else if (MATCH("base", "uri")) {
        if (strlen(value) >= sizeof(config->base_uri)) {
            printf("Error: Base URI too long (max %zu chars)\n", sizeof(config->base_uri) - 1);
            return 0;
        }
        strncpy(config->base_uri, value, sizeof(config->base_uri) - 1);
        config->base_uri[sizeof(config->base_uri) - 1] = '\0';
    }
    
    // [output] section
    else if (MATCH("output", "pos")) {
        if (strlen(value) >= sizeof(config->out_pos)) {
            printf("Error: Position output path too long (max %zu chars)\n", sizeof(config->out_pos) - 1);
            return 0;
        }
        strncpy(config->out_pos, value, sizeof(config->out_pos) - 1);
        config->out_pos[sizeof(config->out_pos) - 1] = '\0';
    }
    else if (MATCH("output", "nmea")) {
        if (strlen(value) >= sizeof(config->out_nmea)) {
            printf("Error: NMEA output path too long (max %zu chars)\n", sizeof(config->out_nmea) - 1);
            return 0;
        }
        strncpy(config->out_nmea, value, sizeof(config->out_nmea) - 1);
        config->out_nmea[sizeof(config->out_nmea) - 1] = '\0';
    }
    else if (MATCH("output", "trace")) {
        if (strlen(value) >= sizeof(config->out_trace)) {
            printf("Error: Trace output path too long (max %zu chars)\n", sizeof(config->out_trace) - 1);
            return 0;
        }
        strncpy(config->out_trace, value, sizeof(config->out_trace) - 1);
        config->out_trace[sizeof(config->out_trace) - 1] = '\0';
    }
    
    // [systems] section
    else if (MATCH("systems", "enabled")) {
        int mask = parse_navsys(value);
        if (mask > 0) {
            config->navsys_mask = mask;
        } else {
            printf("Warning: No valid GNSS systems in '%s', keeping defaults\n", value);
            // Keep existing default, don't overwrite
        }
    }
    
    // Unknown section/key - just ignore with info message
    else {
        printf("Info: Ignoring unknown config option [%s] %s = %s\n", section, name, value);
    }
    
    return 1; // Continue parsing
}

/* Initialize configuration with defaults and load from INI file */
int rtk_gnss_config_init(rtk_gnss_config_t* config, const char* config_file_path) {
    // Input validation
    if (!config) {
        printf("Error: config pointer is NULL\n");
        return -1;
    }
    
    if (!config_file_path) {
        printf("Error: config_file_path is NULL\n");
        return -1;
    }
    
    // Initialize structure with safe defaults
    memset(config, 0, sizeof(rtk_gnss_config_t));
    
    // Set reasonable defaults
    config->mode = MODE_UNDEF;          // Must be explicitly set in config
    config->is_static = 0;              // Default to kinematic (moving)
   config->navsys_mask = (SYS_GPS|SYS_GAL|SYS_GLO|SYS_CMP|SYS_QZS); // All systems enabled by default
    
    // Default output files
    strncpy(config->out_pos, "live.pos", sizeof(config->out_pos) - 1);
    config->out_pos[sizeof(config->out_pos) - 1] = '\0';
    
    // Other outputs start empty (disabled unless specified)
    config->out_nmea[0] = '\0';
    config->out_trace[0] = '\0';
    
    // URIs start empty (must be configured)
    config->rover_uri[0] = '\0';
    config->base_uri[0] = '\0';
    
    // Parse the INI file using the ini library
    int parse_result = ini_parse(config_file_path, ini_handler_entry, config);
    
    if (parse_result < 0) {
        printf("Error: Cannot open config file '%s'\n", config_file_path);
        return -1;
    } else if (parse_result > 0) {
        printf("Error: Parse error at line %d in '%s'\n", parse_result, config_file_path);
        return -1;
    }
    
    printf("Configuration loaded successfully from '%s'\n", config_file_path);
    return 0;
}

/* Clean up configuration resources */
void rtk_gnss_config_deinit(rtk_gnss_config_t* config) {
    if (config) {
        // For future use - close files, free memory, etc.
        // For now, just clear the structure
        memset(config, 0, sizeof(rtk_gnss_config_t));
    }
}

/* Print configuration in human-readable format */
void rtk_gnss_config_print(const rtk_gnss_config_t* config) {
    if (!config) {
        printf("Error: config pointer is NULL\n");
        return;
    }
    
    printf("\n=== RTK GNSS Configuration ===\n");
    
    // Processing mode
    printf("Mode          : ");
    switch (config->mode) {
        case MODE_SPP: printf("SPP (Single Point Positioning)\n"); break;
        case MODE_RTK: printf("RTK (Real-Time Kinematic)\n"); break;
        default: printf("UNDEFINED\n"); break;
    }
    
    // Movement type
    printf("Movement      : %s\n", config->is_static ? "STATIC" : "KINEMATIC");
    
    // GNSS systems using RTKLIB constants
    printf("GNSS Systems  : ");
    if (config->navsys_mask == SYS_NONE) {
        printf("NONE");
    } else {
        int first = 1;
        if (config->navsys_mask & SYS_GPS) { printf("%sGPS", first ? "" : ","); first = 0; }
        if (config->navsys_mask & SYS_GLO) { printf("%sGLO", first ? "" : ","); first = 0; }
        if (config->navsys_mask & SYS_GAL) { printf("%sGAL", first ? "" : ","); first = 0; }
        if (config->navsys_mask & SYS_CMP) { printf("%sBDS", first ? "" : ","); first = 0; }
        if (config->navsys_mask & SYS_QZS) { printf("%sQZS", first ? "" : ","); first = 0; }
        if (config->navsys_mask & SYS_SBS) { printf("%sSBS", first ? "" : ","); first = 0; }
        if (config->navsys_mask & SYS_IRN) { printf("%sIRN", first ? "" : ","); first = 0; }
        if (config->navsys_mask & SYS_LEO) { printf("%sLEO", first ? "" : ","); first = 0; }
    }
    printf(" (mask: 0x%02X)\n", config->navsys_mask);
    
    // Data sources
    printf("Rover Source  : %s\n", config->rover_uri[0] ? config->rover_uri : "<not configured>");
    printf("Base Source   : %s\n", config->base_uri[0] ? config->base_uri : "<not configured>");
    
    // Output files
    printf("Position Out  : %s\n", config->out_pos);
    if (config->out_nmea[0]) {
        printf("NMEA Out      : %s\n", config->out_nmea);
    }
    if (config->out_trace[0]) {
        printf("Trace Out     : %s\n", config->out_trace);
    }
    
    printf("==============================\n\n");
}