/**
 * @file pin_config.h
 * @author dom gasperini
 * @brief mini-gps
 * @version 1.0
 * @date 2024-05-06
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

esp_pm_config_esp32_t power_configuration{
    .max_freq_mhz = 80,
    .min_freq_mhz = 80,
    .light_sleep_enable = false,
};

/*
===========================================================
                    pin definitions
===========================================================
*/

// inputs
#define SLEEP_BUTTON_PIN 18

// display
// #define TFT_SCK 27
// #define TFT_MOSI 32
// #define TFT_MISO 14
// #define TFT_CS 33
// #define TFT_DC 25
// #define TFT_RESET 26

// gps
#define GPS_RX 13
#define GPS_TX 12
