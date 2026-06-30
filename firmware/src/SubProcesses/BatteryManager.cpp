/**
 * @file BatteryManager.cpp
 * @brief
 * @date 2026-04-17
 */

/*
===============================================================================================
                                   includes
===============================================================================================
*/

#include <Arduino.h>
#include <Adafruit_MAX1704X.h> // battery managment chip library

#include <Data/ExecutiveData.h>

/*
===============================================================================================
                                    definitions
===============================================================================================
*/

#define BATTERY_POLL_INTERVAL 2500 // in milliseconds
#define LOW_BATTERY_THRESHOLD 10   // in %

/*
===============================================================================================
                                  global variables
===============================================================================================
*/

unsigned long g_lastBatteryCheckTime = 0;

/*
===============================================================================================
                                    functions
===============================================================================================
*/

/**
 * @brief collect information related to the battery
 */
void BatteryManager(Adafruit_MAX17048 *batteryModule, ExecutiveData *executiveData)
{
    bool processBatteryData = false;

    // do time keeping
    unsigned long now = millis();
    if (now - g_lastBatteryCheckTime >= BATTERY_POLL_INTERVAL)
    {
        processBatteryData = true;
        g_lastBatteryCheckTime = now;
    }

    // collect battery data
    if (processBatteryData)
    {
        // poll battery chip
        executiveData->setBatteryPercent(batteryModule->cellPercent());
        executiveData->setBatteryVoltage(batteryModule->cellVoltage());
        executiveData->setBatteryChargeRate(batteryModule->chargeRate());

        // low battery logic
        if (executiveData->getBatteryPercent() <= LOW_BATTERY_THRESHOLD)
        {
            executiveData->setLowBatteryModeEnable(true);
        }
    }
}