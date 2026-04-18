/**
 * @file data_types.h
 * @brief
 * @date 2026-04-17
 */

/*
===============================================================================================
                                   includes
===============================================================================================
*/

#include <Arduino.h>

#include <Data/ExecutiveData.h>
#include <Data/IoData.h>

/*
===============================================================================================
                                    functions
===============================================================================================
*/

/**
 *
 */
void StateManager(ExecutiveData *executiveData, IoData *ioData)
{
    // control state based on io input
    switch (executiveData->getDisplayMode())
    {
    case GPS_MODE:
        // select
        ioData->setSelectShortPress(false);

        // option
        if (ioData->getOptionShortPress())
        {
            executiveData->setDisplayDebugEnabled(!executiveData->getDisplayDebugEnabled());
            ioData->setOptionShortPress(false);
        }

        // return
        if (ioData->getReturnShortPress())
        {
            executiveData->setDisplayMode(WAYPOINT_MODE);
            executiveData->setDisplayDebugEnabled(false);
            ioData->setReturnShortPress(false);
        }
        break;

    case WAYPOINT_MODE:
        // select
        if (ioData->getSelectShortPress())
        {
            executiveData->setSelectedWaypoint(executiveData->getSelectedWaypoint() + 1);

            if (executiveData->getSelectedWaypoint() >= (executiveData->getWaypoints().size()))
            {
                executiveData->setSelectedWaypoint(0);
            }
            ioData->setSelectShortPress(false);
        }

        // option
        ioData->setOptionShortPress(false);

        // return
        if (ioData->getReturnShortPress())
        {
            executiveData->setDisplayMode(SYSTEM_MODE);
            ioData->setReturnShortPress(false);
        }
        break;

    case SYSTEM_MODE:
        // select
        if (ioData->getSelectShortPress())
        {
            executiveData->setDisplayMode(SLEEP_PROMPT_MODE);
            ioData->setSelectShortPress(false);
        }

        // option
        if (ioData->getOptionShortPress())
        {
            executiveData->setDisplayMode(FLASHLIGHT_MODE);
            ioData->setOptionShortPress(false);
        }

        // return
        if (ioData->getReturnShortPress())
        {
            executiveData->setDisplayMode(GPS_MODE);
            ioData->setReturnShortPress(false);
        }
        break;

    case ERROR_MODE:
        // select
        ioData->setSelectShortPress(false);

        // option
        ioData->setOptionShortPress(false);

        // return
        if (ioData->getReturnShortPress())
        {
            executiveData->setDisplayMode(GPS_MODE);
            ioData->setReturnShortPress(false);
        }
        break;

    case SLEEP_PROMPT_MODE:
        // select
        ioData->setSelectShortPress(false);

        // option
        if (ioData->getOptionShortPress())
        {
            executiveData->setSleepModeEnable(true);
            ioData->setOptionShortPress(false);
        }

        // return
        if (ioData->getReturnShortPress())
        {
            executiveData->setDisplayMode(SYSTEM_MODE);
            ioData->setReturnShortPress(false);
        }

        break;

    case FLASHLIGHT_MODE:
        // select
        ioData->setSelectShortPress(false);

        // option
        if (ioData->getOptionShortPress())
        {
            executiveData->setFlashlightEnabled(!executiveData->getFlashlightEnabled());
            ioData->setOptionShortPress(false);
        }

        // return
        if (ioData->getReturnShortPress())
        {
            executiveData->setDisplayMode(SYSTEM_MODE);
            ioData->setReturnShortPress(false);
        }
        break;

    default:
        executiveData->setDisplayMode(ERROR_MODE);
        break;
    }
}