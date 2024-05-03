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
 * @brief all data about satellites
 */
typedef struct SatelliteData
{
    bool active;
    int elevation;
    int azimuth;
    int snr;
} SatelliteData;

/**
 * @brief mini-gps data frame
 */
typedef struct Data
{
    double latitude;
    double longitude;
    double altitude;

    double speed;

    int year;
    int month;
    int day;

    int hour;
    int minute;
    int second;

    float avgSignalStrength;

    int numSats;
    SatelliteData satellites[];
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