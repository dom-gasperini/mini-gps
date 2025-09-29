/**
 * @file pin_config.h
 * @author dom gasperini
 * @brief mini-gps
 * @version 5.1
 * @date 2025-09-26
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

esp_pm_config_esp32s3_t power_configuration{
    .max_freq_mhz = 240,
    .min_freq_mhz = 240,
    .light_sleep_enable = true,
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
#define GPS_ENABLE_PIN A4 // the enable pin specific to the gps module hardware | HIGH = on | LOW = off

// display