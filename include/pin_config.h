/**
 * @file pinConfig.h
 * @author dom gasperini
 * @brief this file holds the pin layout for the board I/O
 * @version 1.0
 * @date 2024-01-11
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

// Screen
#define DISPLAY_YP_PIN 1 // must be analog
#define DISPLAY_XM_PIN 2 // must be analog
#define DISPLAY_YM_PIN 3 // digital
#define DISPLAY_XP_PIN 4 // digital

#define DISPLAY_TS_MINX 150
#define DISPLAY_TS_MAXX 120
#define DISPLAY_TS_MINY 920
#define DISPLAY_TS_MAXY 940

#define DISPLAY_MIN_PRESSURE 0
#define DISPLAY_MAX_PRESSURE 1000

#define DISPLAY_CS_PIN 19
#define DISPLAY_DC_PIN 9

// SPI Pins
// MOSI: 23
// MISO: 19
// SCK: 18
// SS: 5

// LoRa
#define LORA_CS_PIN 10
#define LORA_RESET_PIN 9
#define LORA_IRQ_PIN 8

// Inputs

// Outputs
