/*
===============================================================================================
                                    includes
===============================================================================================
*/

#include <Arduino.h>
#include <pin_config.h>
#include <Data/IoData.h>

/*
===============================================================================================
                                    definitions
===============================================================================================
*/

#define SHORT_PRESS_DURATION 100 // in milliseconds
#define LONG_PRESS_DURATION 200  // in milliseconds

/*
===============================================================================================
                                  global variables
===============================================================================================
*/

bool g_selectButtonPreviousState = LOW;
unsigned long g_selectButtonCounter = 0;
bool g_selectButtonToggle = false;

bool g_optionButtonPreviousState = LOW;
unsigned long g_optionButtonCounter = 0;
bool g_optionButtonToggle = false;

bool g_returnButtonPreviousState = LOW;
unsigned long g_returnButtonCounter = 0;

/**
 *
 */
void ButtonManager(IoData *ioData)
{
    // --- select button --- //
    bool selectButtonState = digitalRead(SELECT_BUTTON);

    if (selectButtonState != g_selectButtonPreviousState)
    {
        g_selectButtonCounter = millis();
        g_selectButtonPreviousState = selectButtonState;

        // check specifically for LOW -> HIGH
        if (g_selectButtonPreviousState == HIGH)
        {
            if (millis() - g_selectButtonCounter > SHORT_PRESS_DURATION)
            {
                ioData->setSelectLongPress(true);
            }
            else
            {
                ioData->setSelectShortPress(true);
            }
        }
    }
    g_selectButtonPreviousState = selectButtonState;
    // --- select button --- //

    // --- option button --- //
    bool optionButtonState = digitalRead(OPTION_BUTTON);

    if (optionButtonState != g_optionButtonPreviousState)
    {
        g_optionButtonCounter = millis();
        g_optionButtonPreviousState = optionButtonState;

        // check specifically for HIGH -> LOW
        if (g_optionButtonPreviousState == LOW)
        {
            if (millis() - g_optionButtonCounter > SHORT_PRESS_DURATION)
            {
                ioData->setOptionLongPress(true);
            }
            else
            {
                ioData->setOptionShortPress(true);
            }
        }
    }
    g_optionButtonPreviousState = optionButtonState;
    // --- option button --- //

    // --- return button --- //
    bool returnButtonState = digitalRead(RETURN_BUTTON);

    if (returnButtonState != g_returnButtonPreviousState)
    {
        g_returnButtonCounter = millis();
        g_returnButtonPreviousState = returnButtonState;

        // check specifically for HIGH -> LOW
        if (g_returnButtonPreviousState == LOW)
        {
            if (millis() - g_returnButtonCounter > SHORT_PRESS_DURATION)
            {
                ioData->setReturnLongPress(true);
            }
            else
            {
                ioData->setReturnShortPress(true);
            }
        }
    }
    g_returnButtonPreviousState = returnButtonState;
    // --- return button --- //
}