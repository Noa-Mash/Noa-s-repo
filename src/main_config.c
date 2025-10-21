#include <stdio.h>
#include "rtk_gnss_config.h"

int main()
{
    printf("RTK GNSS Configuration Test Program\n");
    printf("===================================\n\n");
    
    // Check init
    printf("Testing rtk_gnss_config_init()...\n");
    rtk_gnss_config_t config;
    
    int result = rtk_gnss_config_init(&config, "config.ini");
    if (result == 0) {
        printf("✓ Configuration initialized successfully\n\n");
    } else {
        printf("✗ Configuration initialization failed\n");
        printf("Make sure 'config.ini' exists in the current directory\n");
        return 1;
    }
    
    // Check print
    printf("Testing rtk_gnss_config_print()...\n");
    rtk_gnss_config_print(&config);
    
    // Basic validation
    printf("Basic validation checks:\n");
    printf("------------------------\n");
    
    if (config.mode == MODE_UNDEF) {
        printf("⚠ Warning: Mode is undefined\n");
    } else {
        printf("✓ Mode is configured: %s\n", 
               config.mode == MODE_RTK ? "RTK" : "SPP");
    }
    
    if (config.rover_uri[0] == '\0') {
        printf("⚠ Warning: Rover URI not configured\n");
    } else {
        printf("✓ Rover URI configured\n");
    }
    
    if (config.mode == MODE_RTK && config.base_uri[0] == '\0') {
        printf("⚠ Warning: RTK mode requires base URI\n");
    } else if (config.mode == MODE_RTK) {
        printf("✓ Base URI configured for RTK mode\n");
    }
    
    printf("\n");
    
    // Check deinit
    printf("Testing rtk_gnss_config_deinit()...\n");
    rtk_gnss_config_deinit(&config);
    printf("✓ Configuration cleaned up\n\n");
    
    printf("All tests completed successfully!\n");
    return 0;
}