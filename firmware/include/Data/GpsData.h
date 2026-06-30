/**
 * @file GpsData.h
 * @brief
 * @date 2026-04-17
 */

#ifndef GPSDATA_H
#define GPSDATA_H

/*
===============================================================================================
                                    includes
===============================================================================================
*/

#include <mutex>

/*
===============================================================================================
                                    definitions
===============================================================================================
*/

#define MIN_SPEED 0.0 // minimum number of knots before displaying speed to due resolution limitations
#define MIN_SATS 3    // min number of sats to have a fix

/*
===============================================================================================
                                    class
===============================================================================================
*/

/**
 *
 */
class GpsData
{
private:
    mutable std::mutex mutex;

    bool validDate;
    int fixQuality;
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
    int timeout;
    int numSats;
    float refreshRate;

public:
    GpsData();

    // Getters
    bool getValidDate() const;
    int getFixQuality() const;
    float getDtLastFix() const;
    float getDtSinceDate() const;
    float getDtSinceTime() const;
    float getLatitude() const;
    float getLongitude() const;
    float getAltitude() const;
    float getSpeed() const;
    float getHeading() const;
    int getYear() const;
    int getMonth() const;
    int getDay() const;
    int getHour() const;
    int getMinute() const;
    int getSecond() const;
    int getTimeout() const;
    int getNumSats() const;
    float getRefreshRate() const;

    // Setters
    void setValidDate(bool value);
    void setFixQuality(int value);
    void setDtLastFix(float value);
    void setDtSinceDate(float value);
    void setDtSinceTime(float value);
    void setLatitude(float value);
    void setLongitude(float value);
    void setAltitude(float value);
    void setSpeed(float value);
    void setHeading(float value);
    void setYear(int value);
    void setMonth(int value);
    void setDay(int value);
    void setHour(int value);
    void setMinute(int value);
    void setSecond(int value);
    void setTimeout(int value);
    void setNumSats(int value);
    void setRefreshRate(float value);
};

#endif // GPSDATA_H