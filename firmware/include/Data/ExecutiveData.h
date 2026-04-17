#ifndef EXECUTIVE_DATA_H
#define EXECUTIVE_DATA_H

#include <Arduino.h>
#include <vector>
#include <mutex>

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
 *
 */
class ExecutiveData
{
public:
    ExecutiveData();

    // ----- Power -----
    void setLowBatteryModeEnable(bool value);
    bool getLowBatteryModeEnable();

    void setSleepModeEnable(bool value);
    bool getSleepModeEnable();

    void setBatteryPercent(float value);
    float getBatteryPercent();

    void setBatteryVoltage(float value);
    float getBatteryVoltage();

    void setBatteryChargeRate(float value);
    float getBatteryChargeRate();

    void setAlertStatus(bool value);
    bool getAlertStatus();

    // ----- Display -----
    void setDisplayMode(DisplayModeType mode);
    DisplayModeType getDisplayMode();

    void setPreviousDisplayMode(DisplayModeType mode);
    DisplayModeType getPreviousDisplayMode();

    void setDisplayDebugEnabled(bool value);
    bool getDisplayDebugEnabled();

    // ----- Waypoint Data -----
    void setWaypoints(const std::vector<WaypointCoordinatesType> &waypoints);
    std::vector<WaypointCoordinatesType> getWaypoints();

    void setSelectedWaypoint(int index);
    int getSelectedWaypoint();

    // ----- Flashlight -----
    void setFlashlightEnabled(bool enabled);
    bool getFlashlightEnabled();

private:
    // ----- Power -----
    bool lowBatteryModeEnable_;
    bool sleepModeEnable_;
    float batteryPercent_;
    float batteryVoltage_;
    float batteryChargeRate_;
    bool alertStatus_;

    // ----- Display -----
    DisplayModeType displayMode_;
    DisplayModeType previousDisplayMode_;
    bool displayDebugEnable_;

    // ----- Waypoint Data -----
    std::vector<WaypointCoordinatesType> waypoints_;
    int selectedWaypoint_;

    // ----- Flashlight -----
    bool enableFlashlight_;

    std::mutex mutex_;
};

#endif // EXECUTIVE_DATA_H