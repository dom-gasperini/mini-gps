/**
 * @file data_types.h
 * @author dom gasperini
 * @brief defines all of the custom data types used throughout mini gps firmware
 * @version 6
 * @date 2025-09-30
 */

/*
========================================================
                    includes
========================================================
*/

#include <Arduino.h>
#include <vector>

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

/**
 * @brief all of the data associated with a waypoint
 */
typedef struct
{
    float latitude;
    float longitude;
    String name;
} WaypointCoordinatesType;

/**
 * @brief general waypoint data
 */
typedef struct
{
    std::vector<WaypointCoordinatesType> waypoints;
    int selectedWaypoint;
} WaypointDataType;

/**
 * @brief the various full screen display modes
 */
typedef enum
{
    GPS_MODE = 0,
    WAYPOINT_MODE,
    SYSTEM_MODE,
    ERROR_MODE, // error mode is a mode cycle select bookend, add new cycle modes before here
    SLEEP_PROMPT_MODE,
    FLASHLIGHT_MODE,
} DisplayModeType;

/**
 * @brief input flags for the user buttons
 */
typedef struct
{
    bool selectShortPress;
    bool selectLongPress;

    bool optionShortPress;
    bool optionLongPress;

    bool returnShortPress;
    bool returnLongPress;

    bool specialShortPress;
    bool specialLongPress;
} InputFlagsType;

/**
 * @brief all data associated with the power state of the device
 */
typedef struct
{
    bool lowBatteryModeEnable;
    bool sleepModeEnable;

    float batteryPercent;
    float batteryVoltage;
    float batteryChargeRate;
    uint8_t alertStatus;
} PowerDataType;

/**
 * @brief all data associated with the state of the display
 */
typedef struct
{
    DisplayModeType displayMode;
    DisplayModeType previousDisplayMode;
} DisplayDataType;

/**
 * @brief all gps data
 */
typedef struct
{
    bool validDate;
    uint8_t fixQuality;
    float dtLastFix;
    float dtSinceDate;
    float dtSinceTime;

    float latitude;
    float longitude;
    float altitude;

    float speed;
    float heading;

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
 * @brief all data associated with the mini gps system state
 */
typedef struct
{
    InputFlagsType inputFlags;
    PowerDataType power;
    DisplayDataType display;
    WaypointDataType waypointData;
    bool enableFlashlight;
} SystemDataType;

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
    unsigned long ioTaskCount;
    unsigned long gpsTaskCount;
    unsigned long displayTaskCount;

    int displayRefreshRate;

    unsigned long ioTaskPreviousCount;
    unsigned long gpsTaskPreviousCount;
    unsigned long displayTaskPreviousCount;
} DebuggerType;