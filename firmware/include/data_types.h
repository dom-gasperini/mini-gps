/**
 * @file data_types.h
 * @author dom gasperini
 * @brief mini-gps
 * @version 4.0
 * @date 2024-07-11
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
typedef struct
{
    bool active;
    int elevation;
    int azimuth;
    int snr;
} SatelliteData;

/**
 * @brief mini-gps data frame
 */
typedef struct
{
    bool connected;
    bool wasConnected;
    bool validDate;
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
} GpsDataType;

/**
 * @brief debugger structure
 */
typedef struct
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
    unsigned long ioTaskCount;
    unsigned long i2cTaskCount;
    unsigned long displayTaskCount;

    int displayRefreshRate;

    unsigned long ioTaskPreviousCount;
    unsigned long i2cTaskPreviousCount;
    unsigned long displayTaskPreviousCount;
} Debugger;

// debug functions
void PrintDebug();
void PrintI2CDebug();
void PrintIODebug();
void PrintDisplayDebug();
void PrintSchedulerDebug();