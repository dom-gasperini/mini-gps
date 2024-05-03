/**
 * @file pinConfig.h
 * @author dom gasperini
 * @brief mini-gps
 * @version 0.1
 * @date 2024-05-02
 */

/*
===============================================================================================
                                    Includes
===============================================================================================
*/

#include <esp_pm.h>

/*
===========================================================
                    Power Configuration
===========================================================
*/

esp_pm_config_esp32_t power_configuration{
    .max_freq_mhz = 240,
    .min_freq_mhz = 240,
    .light_sleep_enable = false,
};

/*
===========================================================
                    Pin Definitions
===========================================================
*/

// display
#define TFT_SCK 18
#define TFT_MOSI 23
#define TFT_MISO 19
#define TFT_CS 22
#define TFT_DC 21
#define TFT_RESET 17

// gps
#define GPS_TX 16
#define GPS_RX 17
