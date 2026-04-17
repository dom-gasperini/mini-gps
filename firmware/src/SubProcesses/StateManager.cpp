/*
===============================================================================================
                                    includes
===============================================================================================
*/

#include <Arduino.h>

#include <Data/ExecutiveData.h>
#include <Data/IoData.h>

/**
 *
 */
void StateManager(ExecutiveData *executiveData, IoData *ioData)
{
    switch (executiveData->getDisplayMode())
    {
    case GPS_MODE:
        if (ioData->getOptionShortPress())
        {
            executiveData->setDisplayDebugEnabled(!executiveData->getDisplayDebugEnabled());
            ioData->setOptionShortPress(false);
        }

        if (ioData->getReturnShortPress())
        {
            executiveData->setDisplayMode(WAYPOINT_MODE);
            // g_previousSelectedWaypoint = -1;  // force computation of vector
            executiveData->setDisplayDebugEnabled(false);
        }

        if (ioData->getReturnLongPress())
        {
            executiveData->setDisplayMode(GPS_MODE);
        }
        break;

    case WAYPOINT_MODE:
        if (ioData->getSelectShortPress())
        {
            executiveData->setSelectedWaypoint(executiveData->getSelectedWaypoint() + 1);

            if (executiveData->getSelectedWaypoint() >= (executiveData->getWaypoints().size()))
            {
                executiveData->setSelectedWaypoint(0);
            }
        }

        if (ioData->getReturnShortPress())
        {
            executiveData->setDisplayMode(SYSTEM_MODE);
        }

        if (ioData->getReturnLongPress())
        {
            executiveData->setDisplayMode(GPS_MODE);
        }
        break;

    case SYSTEM_MODE:
        if (ioData->getSelectShortPress())
        {
            executiveData->setDisplayMode(SLEEP_PROMPT_MODE);
            // g_drewMoonIcon = false;
        }

        if (ioData->getOptionShortPress())
        {
            executiveData->setDisplayMode(FLASHLIGHT_MODE);
        }

        if (ioData->getReturnShortPress())
        {
            executiveData->setDisplayMode(GPS_MODE);
            executiveData->setSleepModeEnable(false);
        }

        if (ioData->getReturnLongPress())
        {
            executiveData->setDisplayMode(GPS_MODE);
            executiveData->setSleepModeEnable(false);
        }
        break;

    case SLEEP_PROMPT_MODE:
        if (ioData->getReturnShortPress())
        {
            executiveData->setDisplayMode(SYSTEM_MODE);
            executiveData->setSleepModeEnable(false);
        }

        if (ioData->getOptionShortPress())
        {
            executiveData->setSleepModeEnable(true);
        }
        break;

    case FLASHLIGHT_MODE:
        if (ioData->getOptionShortPress())
        {
            executiveData->setFlashlightEnabled(!executiveData->getFlashlightEnabled());
        }

        if (ioData->getReturnShortPress())
        {
            executiveData->setDisplayMode(SYSTEM_MODE);
        }
        break;

    default:
        executiveData->setDisplayMode(ERROR_MODE);

        if (ioData->getReturnShortPress())
        {
            executiveData->setDisplayMode(GPS_MODE);
        }
        break;
    }
}