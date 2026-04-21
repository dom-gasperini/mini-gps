/**
 * @file data_types.h
 * @brief
 * @date 2026-04-17
 */

/*
========================================================
                    includes
========================================================
*/

#include <Arduino.h>

/*
========================================================
                    data types
========================================================
*/

/**
 * @brief setup managment
 */
typedef struct
{
    bool ioActive;
    bool displayActive;
    bool gpsActive;
} InitDeviceType;