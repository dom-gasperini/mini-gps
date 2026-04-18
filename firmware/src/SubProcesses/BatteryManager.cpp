#include <Arduino.h>
#include <Adafruit_MAX1704X.h> // battery managment chip library
#include <Data/ExecutiveData.h>

#define BATTERY_POLL_INTERVAL 2000 // in milliseconds
#define LOW_BATTERY_THRESHOLD 10   // in %

unsigned long g_lastBatteryCheckTime = 0;

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

        // --- low battery logic --- //
        if (executiveData->getBatteryPercent() <= LOW_BATTERY_THRESHOLD)
        {
            executiveData->setLowBatteryModeEnable(true);
        }
        // --- low battery logic --- //

        processBatteryData = false;
    }
}