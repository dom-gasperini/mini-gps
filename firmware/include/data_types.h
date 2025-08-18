/**
 * @file data_types.h
 * @author dom gasperini
 * @brief mini-gps
 * @version 5.0
 * @date 2025-08-18
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
 * @brief setup managment struct
 */
typedef struct
{
    bool ioActive;
    bool displayActive;
    bool gpsActive;
} InitDeviceType;

typedef struct
{
    float waypointLatitude;
    float waypointLongitude;
} WaypointCoordinatesType;

/**
 *
 */
typedef enum
{
    GPS_MODE = 0,
    WAYPOINT_MODE,
    BATTERY_MODE,
    ERROR_MODE, // error mode is a mode cycle select bookend, add new cycle modes before here
    FLASHLIGHT_MODE,
} DisplayModeType;

typedef struct
{
    bool lowBatteryModeEnable;
    bool sleepModeEnable;

    float batteryPercent;
    float batteryVoltage;
    float batteryChargeRate;

    uint8_t chipId;
    uint8_t alertStatus;
} PowerDataType;

typedef struct
{
    DisplayModeType displayMode;
    DisplayModeType previousDisplayMode;
    bool specialSelected;
    int displayRefreshCounter;
} DisplayDataType;

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

    WaypointCoordinatesType waypoint;

    float speed;
    float angle;

    int year;
    int month;
    int day;

    int hour;
    int minute;
    int second;
    long timeout;

    uint8_t numSats;
} GpsDataType;

/**
 * @brief all data about satellites
 */
typedef struct
{
    bool active;
    int elevation;
    int azimuth;
    int snr;
} SatelliteDataType;

typedef struct
{
    PowerDataType power;
    DisplayDataType display;
    GpsDataType gps;
} SystemDataType;

/**
 * @brief debugger structure
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
    unsigned long ioTaskCount;
    unsigned long gpsTaskCount;
    unsigned long displayTaskCount;

    int displayRefreshRate;

    unsigned long ioTaskPreviousCount;
    unsigned long gpsTaskPreviousCount;
    unsigned long displayTaskPreviousCount;
} DebuggerType;

// debug functions
void PrintDebug();
void PrintGpsDebug();
void PrintIODebug();
void PrintDisplayDebug();
void PrintSchedulerDebug();