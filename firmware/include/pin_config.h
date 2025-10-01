/**
 * @file pin_config.h
 * @author dom gasperini
 * @brief contains the pin definitions for mini gps
 * @version 6
 * @date 2025-09-30
 */

/*
===========================================================
                    includes
===========================================================
*/

#include <esp_pm.h>

/*
===========================================================
                    power configuration
===========================================================
*/

/**
 * @brief define the cpu clock speed profile and light sleep capabilites
 */
esp_pm_config_esp32s3_t power_configuration{
    .max_freq_mhz = 240,
    .min_freq_mhz = 240,
    .light_sleep_enable = false,
};

/*
===========================================================
                    pin definitions
===========================================================
*/

// io
#define SELECT_BUTTON 0 // top | pulled up
#define OPTION_BUTTON 1 // middle | pulled down
#define RETURN_BUTTON 2 // bottom | pulled down

// gps
#define GPS_WAKE_PIN A4 // enable pin specific to the gps module hardware | HIGH = on | LOW = off
