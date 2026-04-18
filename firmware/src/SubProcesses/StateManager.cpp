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
            executiveData->setDisplayDebugEnabled(false);
            ioData->setReturnShortPress(false);
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
            ioData->setSelectShortPress(false);
        }

        if (ioData->getReturnShortPress())
        {
            executiveData->setDisplayMode(SYSTEM_MODE);
            ioData->setReturnShortPress(false);
        }
        break;

    case SYSTEM_MODE:
        if (ioData->getSelectShortPress())
        {
            executiveData->setDisplayMode(SLEEP_PROMPT_MODE);
            ioData->setSelectShortPress(false);
        }

        if (ioData->getOptionShortPress())
        {
            executiveData->setDisplayMode(FLASHLIGHT_MODE);
            ioData->setOptionShortPress(false);
        }

        if (ioData->getReturnShortPress())
        {
            executiveData->setDisplayMode(GPS_MODE);
            executiveData->setSleepModeEnable(false);
            ioData->setReturnShortPress(false);
        }
        break;

    case SLEEP_PROMPT_MODE:
        if (ioData->getReturnShortPress())
        {
            executiveData->setDisplayMode(SYSTEM_MODE);
            executiveData->setSleepModeEnable(false);
            ioData->setReturnShortPress(false);
        }

        if (ioData->getOptionShortPress())
        {
            executiveData->setSleepModeEnable(true);
            ioData->setOptionShortPress(false);
        }
        break;

    case FLASHLIGHT_MODE:
        if (ioData->getOptionShortPress())
        {
            executiveData->setFlashlightEnabled(!executiveData->getFlashlightEnabled());
            ioData->setOptionShortPress(false);
        }

        if (ioData->getReturnShortPress())
        {
            executiveData->setDisplayMode(SYSTEM_MODE);
            ioData->setReturnShortPress(false);
        }
        break;

    default:
        executiveData->setDisplayMode(ERROR_MODE);

        if (ioData->getReturnShortPress())
        {
            executiveData->setDisplayMode(GPS_MODE);
            ioData->setReturnShortPress(false);
        }
        break;
    }
}