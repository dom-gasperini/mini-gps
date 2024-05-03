/**
 * @file data_types.h
 * @author dom gasperini
 * @brief mini-gps
 * @version 0.1
 * @date 2024-05-02
 */

/*
========================================================
                    includes
========================================================
*/

/*
========================================================
                    data types
========================================================
*/

/**
 * @brief mini-gps data frame
 */
typedef struct Data
{
    double latitude;
    double longitude;
    double altitude;
} Data;

/**
 * @brief debugger structure
 *
 */
typedef struct Debugger
{
    // debug toggle
    bool debugEnabled;
    bool IO_debugEnabled;
    bool gps_debugEnabled;
    bool display_debugEnabled;
    bool scheduler_debugEnable;

    // scheduler data
    unsigned long ioTaskCount;
    unsigned long gpsTaskCount;
    unsigned long displayTaskCount;

    unsigned long ioTaskPreviousCount;
    unsigned long gpsTaskPreviousCount;
    unsigned long displayTaskPreviousCount;
} Debugger;

// debug functions
void PrintDebug();
void PrintGPSDebug();
void PrintIODebug();
void PrintSchedulerDebug();