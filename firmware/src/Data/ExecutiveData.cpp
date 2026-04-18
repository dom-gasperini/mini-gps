#include "Data/ExecutiveData.h"

// Constructor
ExecutiveData::ExecutiveData()
    : lowBatteryModeEnable_(false),
      sleepModeEnable_(false),
      batteryPercent_(0.0f),
      batteryVoltage_(0.0f),
      batteryChargeRate_(0.0f),
      alertStatus_(false),
      displayMode_(GPS_MODE),
      previousDisplayMode_(GPS_MODE),
      displayDebugEnable_(false),
      waypoints_(),
      selectedWaypoint_(0),
      enableFlashlight_(false)
{
}

// ----- Power -----
void ExecutiveData::setLowBatteryModeEnable(bool value)
{
    std::lock_guard<std::mutex> lock(mutex_);
    lowBatteryModeEnable_ = value;
}

bool ExecutiveData::getLowBatteryModeEnable()
{
    std::lock_guard<std::mutex> lock(mutex_);
    return lowBatteryModeEnable_;
}

void ExecutiveData::setSleepModeEnable(bool value)
{
    std::lock_guard<std::mutex> lock(mutex_);
    sleepModeEnable_ = value;
}

bool ExecutiveData::getSleepModeEnable()
{
    std::lock_guard<std::mutex> lock(mutex_);
    return sleepModeEnable_;
}

void ExecutiveData::setBatteryPercent(float value)
{
    std::lock_guard<std::mutex> lock(mutex_);
    batteryPercent_ = value;
}

float ExecutiveData::getBatteryPercent()
{
    std::lock_guard<std::mutex> lock(mutex_);
    return batteryPercent_;
}

void ExecutiveData::setBatteryVoltage(float value)
{
    std::lock_guard<std::mutex> lock(mutex_);
    batteryVoltage_ = value;
}

float ExecutiveData::getBatteryVoltage()
{
    std::lock_guard<std::mutex> lock(mutex_);
    return batteryVoltage_;
}

void ExecutiveData::setBatteryChargeRate(float value)
{
    std::lock_guard<std::mutex> lock(mutex_);
    batteryChargeRate_ = value;
}

float ExecutiveData::getBatteryChargeRate()
{
    std::lock_guard<std::mutex> lock(mutex_);
    return batteryChargeRate_;
}

void ExecutiveData::setAlertStatus(bool value)
{
    std::lock_guard<std::mutex> lock(mutex_);
    alertStatus_ = value;
}

bool ExecutiveData::getAlertStatus()
{
    std::lock_guard<std::mutex> lock(mutex_);
    return alertStatus_;
}

// ----- Display -----
void ExecutiveData::setDisplayMode(DisplayModeType mode)
{
    std::lock_guard<std::mutex> lock(mutex_);
    displayMode_ = mode;
}

DisplayModeType ExecutiveData::getDisplayMode()
{
    std::lock_guard<std::mutex> lock(mutex_);
    return displayMode_;
}

void ExecutiveData::setPreviousDisplayMode(DisplayModeType mode)
{
    std::lock_guard<std::mutex> lock(mutex_);
    previousDisplayMode_ = mode;
}

DisplayModeType ExecutiveData::getPreviousDisplayMode()
{
    std::lock_guard<std::mutex> lock(mutex_);
    return previousDisplayMode_;
}

void ExecutiveData::setDisplayDebugEnabled(bool value)
{
    std::lock_guard<std::mutex> lock(mutex_);
    displayDebugEnable_ = value;
}

bool ExecutiveData::getDisplayDebugEnabled()
{
    std::lock_guard<std::mutex> lock(mutex_);
    return displayDebugEnable_;
}

// ----- Waypoint Data -----
void ExecutiveData::setWaypoints(const std::vector<WaypointCoordinatesType> &waypoints)
{
    std::lock_guard<std::mutex> lock(mutex_);
    waypoints_ = waypoints;
}

std::vector<WaypointCoordinatesType> ExecutiveData::getWaypoints()
{
    std::lock_guard<std::mutex> lock(mutex_);
    return waypoints_;
}

void ExecutiveData::setSelectedWaypoint(int index)
{
    std::lock_guard<std::mutex> lock(mutex_);
    selectedWaypoint_ = index;
}

int ExecutiveData::getSelectedWaypoint()
{
    std::lock_guard<std::mutex> lock(mutex_);
    return selectedWaypoint_;
}

// ----- Flashlight -----
void ExecutiveData::setFlashlightEnabled(bool enabled)
{
    std::lock_guard<std::mutex> lock(mutex_);
    enableFlashlight_ = enabled;
}

bool ExecutiveData::getFlashlightEnabled()
{
    std::lock_guard<std::mutex> lock(mutex_);
    return enableFlashlight_;
}