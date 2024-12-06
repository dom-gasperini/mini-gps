/**
 * @file pin_config.h
 * @author dom gasperini
 * @brief mini-gps
 * @version 2.0
 * @date 2024-06-08
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
    .max_freq_mhz = 240,
    .min_freq_mhz = 240,
    .light_sleep_enable = false,
};

/*
===========================================================
                    pin definitions
===========================================================
*/

// inputs

// ic2
#define I2C_SCL_PIN 19
#define I2C_SDA_PIN 18

// display
// #define TFT_SCK 27
// #define TFT_MOSI 32
// #define TFT_MISO 14
// #define TFT_CS 33
// #define TFT_DC 25
// #define TFT_RESET 26
