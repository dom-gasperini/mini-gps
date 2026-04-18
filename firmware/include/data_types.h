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

/**
 * @brief debug data
 */
typedef struct
{
    // debug toggle
    bool debugEnabled;
    bool IO_debugEnabled;
    bool gps_debugEnabled;
    bool display_debugEnabled;
    bool scheduler_debugEnable;

    // display debugging
    String debugText;

    // scheduler data
    unsigned long executiveTaskCount;
    unsigned long ioTaskCount;
    unsigned long gpsTaskCount;
    unsigned long displayTaskCount;

    int displayRefreshRate;

    unsigned long executiveTaskPreviousCount;
    unsigned long ioTaskPreviousCount;
    unsigned long gpsTaskPreviousCount;
    unsigned long displayTaskPreviousCount;
} DebuggerType;