/**
 * @file pin_config.h
 * @author dom gasperini
 * @brief mini-gps
 * @version 5.0
 * @date 2025-08-18
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
    .light_sleep_enable = false,
};

/*
===========================================================
                    pin definitions
===========================================================
*/

// io
#define SELECT_BUTTON 0 // top | pulled up
#define NEXT_BUTTON 1   // middle | pulled down
#define RETURN_BUTTON 2 // bottom | pulled down

// gps
#define GPS_ENABLE_PIN 20 // the enable pin specific to the gps module hardware | HIGH = on | LOW = off

// display