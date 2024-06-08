/**
 * @file data_types.h
 * @author dom gasperini
 * @brief mini-gps
 * @version 2.0
 * @date 2024-06-08
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
    bool connected;
    bool wasConnected;
    bool validTime;
    bool wasValidTime;
    uint8_t fixQuality;
    float dtLastFix;
    float dtSinceDate;
    float dtSinceTime;

    float latitude;
    float longitude;
    float altitude;

    float speed;
    float angle;

    int year;
    int month;
    int day;

    int hour;
    int minute;
    int second;
    long timeout;

    int numSats;
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
    bool i2c_debugEnabled;
    bool display_debugEnabled;
    bool scheduler_debugEnable;

    // display debugging
    String debugText;

    // scheduler data
    unsigned long ioWriteTaskCount;
    unsigned long ioReadTaskCount;
    unsigned long i2cTaskCount;
    unsigned long displayTaskCount;

    int displayRefreshRate;

    unsigned long ioReadTaskPreviousCount;
    unsigned long ioWriteTaskPreviousCount;
    unsigned long i2cTaskPreviousCount;
    unsigned long displayTaskPreviousCount;
} Debugger;

// debug functions
void PrintDebug();
void PrintI2CDebug();
void PrintIODebug();
void PrintDisplayDebug();
void PrintSchedulerDebug();