/**
 * @file GpsData.cpp
 * @brief
 * @date 2026-04-17
 */

/*
===============================================================================================
                                    includes
===============================================================================================
*/

#include "Data/GpsData.h"

/*
===============================================================================================
                                    functions
===============================================================================================
*/

/**
 *
 */
GpsData::GpsData()
    : validDate(false),
      fixQuality(0),
      dtLastFix(0.0f),
      dtSinceDate(0.0f),
      dtSinceTime(0.0f),
      latitude(0.0f),
      longitude(0.0f),
      altitude(0.0f),
      speed(0.0f),
      heading(0.0f),
      year(0),
      month(0),
      day(0),
      hour(0),
      minute(0),
      second(0),
      timeout(0),
      numSats(0),
      refreshRate(0.0f)
{
}

// ----- Getters -----
bool GpsData::getValidDate() const
{
    std::lock_guard<std::mutex> lock(mutex);
    return validDate;
}

int GpsData::getFixQuality() const
{
    std::lock_guard<std::mutex> lock(mutex);
    return fixQuality;
}

float GpsData::getDtLastFix() const
{
    std::lock_guard<std::mutex> lock(mutex);
    return dtLastFix;
}

float GpsData::getDtSinceDate() const
{
    std::lock_guard<std::mutex> lock(mutex);
    return dtSinceDate;
}

float GpsData::getDtSinceTime() const
{
    std::lock_guard<std::mutex> lock(mutex);
    return dtSinceTime;
}

float GpsData::getLatitude() const
{
    std::lock_guard<std::mutex> lock(mutex);
    return latitude;
}

float GpsData::getLongitude() const
{
    std::lock_guard<std::mutex> lock(mutex);
    return longitude;
}

float GpsData::getAltitude() const
{
    std::lock_guard<std::mutex> lock(mutex);
    return altitude;
}

float GpsData::getSpeed() const
{
    std::lock_guard<std::mutex> lock(mutex);
    return speed;
}

float GpsData::getHeading() const
{
    std::lock_guard<std::mutex> lock(mutex);
    return heading;
}

int GpsData::getYear() const
{
    std::lock_guard<std::mutex> lock(mutex);
    return year;
}

int GpsData::getMonth() const
{
    std::lock_guard<std::mutex> lock(mutex);
    return month;
}

int GpsData::getDay() const
{
    std::lock_guard<std::mutex> lock(mutex);
    return day;
}

int GpsData::getHour() const
{
    std::lock_guard<std::mutex> lock(mutex);
    return hour;
}

int GpsData::getMinute() const
{
    std::lock_guard<std::mutex> lock(mutex);
    return minute;
}

int GpsData::getSecond() const
{
    std::lock_guard<std::mutex> lock(mutex);
    return second;
}

int GpsData::getTimeout() const
{
    std::lock_guard<std::mutex> lock(mutex);
    return timeout;
}

int GpsData::getNumSats() const
{
    std::lock_guard<std::mutex> lock(mutex);
    return numSats;
}

float GpsData::getRefreshRate() const
{
    std::lock_guard<std::mutex> lock(mutex);
    return refreshRate;
}

// ----- Setters -----
void GpsData::setValidDate(bool value)
{
    std::lock_guard<std::mutex> lock(mutex);
    validDate = value;
}

void GpsData::setFixQuality(int value)
{
    std::lock_guard<std::mutex> lock(mutex);
    fixQuality = value;
}

void GpsData::setDtLastFix(float value)
{
    std::lock_guard<std::mutex> lock(mutex);
    dtLastFix = value;
}

void GpsData::setDtSinceDate(float value)
{
    std::lock_guard<std::mutex> lock(mutex);
    dtSinceDate = value;
}

void GpsData::setDtSinceTime(float value)
{
    std::lock_guard<std::mutex> lock(mutex);
    dtSinceTime = value;
}

void GpsData::setLatitude(float value)
{
    std::lock_guard<std::mutex> lock(mutex);
    latitude = value;
}

void GpsData::setLongitude(float value)
{
    std::lock_guard<std::mutex> lock(mutex);
    longitude = value;
}

void GpsData::setAltitude(float value)
{
    std::lock_guard<std::mutex> lock(mutex);
    altitude = value;
}

void GpsData::setSpeed(float value)
{
    std::lock_guard<std::mutex> lock(mutex);
    speed = value;
}

void GpsData::setHeading(float value)
{
    std::lock_guard<std::mutex> lock(mutex);
    heading = value;
}

void GpsData::setYear(int value)
{
    std::lock_guard<std::mutex> lock(mutex);
    year = value;
}

void GpsData::setMonth(int value)
{
    std::lock_guard<std::mutex> lock(mutex);
    month = value;
}

void GpsData::setDay(int value)
{
    std::lock_guard<std::mutex> lock(mutex);
    day = value;
}

void GpsData::setHour(int value)
{
    std::lock_guard<std::mutex> lock(mutex);
    hour = value;
}

void GpsData::setMinute(int value)
{
    std::lock_guard<std::mutex> lock(mutex);
    minute = value;
}

void GpsData::setSecond(int value)
{
    std::lock_guard<std::mutex> lock(mutex);
    second = value;
}

void GpsData::setTimeout(int value)
{
    std::lock_guard<std::mutex> lock(mutex);
    timeout = value;
}

void GpsData::setNumSats(int value)
{
    std::lock_guard<std::mutex> lock(mutex);
    numSats = value;
}

void GpsData::setRefreshRate(float value)
{
    std::lock_guard<std::mutex> lock(mutex);
    refreshRate = value;
}