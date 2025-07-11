/**
 * @file pin_config.h
 * @author dom gasperini
 * @brief mini-gps
 * @version 4.0
 * @date 2024-07-11
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
#define POWER_BUTTON_PIN 3
#define LOW_POWER_MODE_PIN 2

// ic2
#define I2C_SCL_PIN 22
#define I2C_SDA_PIN 21
#define I2C_POWER_TOGGLE 23

// display
#define DISPLAY_POWER_TOGGLE 7
#define SPI_SCLK_PIN 10
#define SPI_MOSI_PIN 11
#define SPI_MISO_PIN 12
#define SPI_CS_PIN 9
#define SPI_DC_PIN 8
#define SPI_RESET_PIN 6
