/*
===============================================================================================
                                    includes
===============================================================================================
*/

#include <Arduino.h>
#include <Adafruit_GPS.h> // gps parsing library
#include <Data/GpsData.h>

/*
===============================================================================================
                                    definitions
===============================================================================================
*/

#define DELTA_FIX_HARDSTOP 100000                // seconds
#define METERS_TO_FEET 3.28084                   // mulitplier for converting meteres to feet
#define KNOTS_TO_MPH 1.1507795                   // mulitplier for converting knots to mph
#define INIT_OPERATING_YEAR 2025                 // first operational year
#define EOL_YEAR 2079                            // end of life operating year
#define GPS_REFRESH_RATE_CALCULATE_INTERVAL 1000 // in millisecond

/*
===============================================================================================
                                  global variables
===============================================================================================
*/
unsigned long g_gpsCounter = 0;
unsigned long g_lastGpsTime = 0;

/**
 *
 */
void GpsManager(Adafruit_GPS gpsModule, GpsData *gpsData)
{
    // read gps data
    while (gpsModule.read() != 0)
    {
        // process gps data
        if (gpsModule.newNMEAreceived())
        {
            if (gpsModule.parse(gpsModule.lastNMEA())) // this sets the newNMEAreceived() flag to false
            {
                // connection
                gpsData->setNumSats(gpsModule.satellites);
                gpsData->setFixQuality(gpsModule.fixquality);
                if (gpsModule.secondsSinceFix() < DELTA_FIX_HARDSTOP)
                {
                    gpsData->setDtLastFix(gpsModule.secondsSinceFix());
                }
                else
                {
                    gpsData->setDtLastFix(-1);
                }

                gpsData->setDtSinceTime(gpsModule.secondsSinceTime());
                gpsData->setDtSinceDate(gpsModule.secondsSinceDate());

                //  location
                gpsData->setLatitude(gpsModule.latitudeDegrees);
                gpsData->setLongitude(gpsModule.longitudeDegrees);
                gpsData->setAltitude(gpsModule.altitude * METERS_TO_FEET);

                // vector
                gpsData->setSpeed(gpsModule.speed); // speed is given in knots
                gpsData->setHeading(gpsModule.angle);

                // apply velocity deadband
                if (gpsData->getSpeed() >= MIN_SPEED)
                {
                    gpsData->setSpeed(gpsData->getSpeed() * KNOTS_TO_MPH); // convert to mph
                }
                else
                {
                    gpsData->setSpeed(0.0f);
                    gpsData->setHeading(0.0f);
                }

                // data
                gpsData->setYear(gpsModule.year + 2000);
                gpsData->setMonth(gpsModule.month);
                gpsData->setDay(gpsModule.day);

                // determine if date time information is valid
                gpsData->setValidDate((gpsData->getYear() >= INIT_OPERATING_YEAR && gpsData->getYear() < EOL_YEAR));

                // time
                gpsData->setHour(gpsModule.hour);
                gpsData->setMinute(gpsModule.minute);
                gpsData->setSecond(gpsModule.seconds);

                // calculate refresh rate
                g_gpsCounter++;
                unsigned long now = millis();
                if (now - g_lastGpsTime >= GPS_REFRESH_RATE_CALCULATE_INTERVAL)
                {
                    float refreshRate = g_gpsCounter * 1000.0 / (now - g_lastGpsTime);
                    g_gpsCounter = 0;
                    g_lastGpsTime = now;

                    gpsData->setRefreshRate(refreshRate);
                }
            }
        }
    }
}