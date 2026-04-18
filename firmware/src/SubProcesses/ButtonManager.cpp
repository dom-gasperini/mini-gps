/**
 * @file ButtonManager.cpp
 * @brief
 * @date 2026-04-17
 */

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

#define DEBOUNCE_DURATION 8
#define LONG_PRESS_DURATION 1500 // in milliseconds

/*
===============================================================================================
                                  global variables
===============================================================================================
*/

bool g_selectButtonPreviousState = HIGH;
bool g_selectButtonStableState = LOW;
bool g_selectButtonLongPressFired = false;
unsigned long g_selectButtonDebounceTime = 0;
unsigned long g_selectButtonPressStartTime = 0;

bool g_optionButtonPreviousState = LOW;
bool g_optionButtonStableState = HIGH;
bool g_optionButtonLongPressFired = false;
unsigned long g_optionButtonDebounceTime = 0;
unsigned long g_optionButtonPressStartTime = 0;

bool g_returnButtonPreviousState = LOW;
bool g_returnButtonStableState = HIGH;
bool g_returnButtonLongPressFired = false;
unsigned long g_returnButtonDebounceTime = 0;
unsigned long g_returnButtonPressStartTime = 0;

/*
===============================================================================================
                                    functions
===============================================================================================
*/

/**
 *
 */
void ButtonManager(IoData *ioData)
{
    unsigned long now = millis();

    // =========================
    // SELECT BUTTON
    // =========================
    {
        bool raw = digitalRead(SELECT_BUTTON);

        if (raw != g_selectButtonPreviousState)
        {
            g_selectButtonDebounceTime = now;
            g_selectButtonPreviousState = raw;
        }

        if ((now - g_selectButtonDebounceTime) > DEBOUNCE_DURATION)
        {
            if (raw != g_selectButtonStableState)
            {
                bool prev = g_selectButtonStableState;
                g_selectButtonStableState = raw;

                // EDGE: pressed (HIGH assumed active)
                if (prev == LOW && g_selectButtonStableState == HIGH)
                {
                    g_selectButtonPressStartTime = now;
                    g_selectButtonLongPressFired = false;
                }

                // EDGE: released
                if (prev == HIGH && g_selectButtonStableState == LOW)
                {
                    if (!g_selectButtonLongPressFired)
                    {
                        ioData->setSelectShortPress(true);
                    }
                }
            }
        }

        // long press (state-based but NOT edge-based)
        if (g_selectButtonStableState == LOW && !g_selectButtonLongPressFired)
        {
            if ((now - g_selectButtonPressStartTime) >= LONG_PRESS_DURATION)
            {
                g_selectButtonLongPressFired = true;
                ioData->setSelectLongPress(true);
            }
        }
    }

    // =========================
    // OPTION BUTTON
    // =========================
    {
        bool raw = digitalRead(OPTION_BUTTON);

        if (raw != g_optionButtonPreviousState)
        {
            g_optionButtonDebounceTime = now;
            g_optionButtonPreviousState = raw;
        }

        if ((now - g_optionButtonDebounceTime) > DEBOUNCE_DURATION)
        {
            if (raw != g_optionButtonStableState)
            {
                bool prev = g_optionButtonStableState;
                g_optionButtonStableState = raw;

                if (prev == HIGH && g_optionButtonStableState == LOW)
                {
                    g_optionButtonPressStartTime = now;
                    g_optionButtonLongPressFired = false;
                }

                if (prev == LOW && g_optionButtonStableState == HIGH)
                {
                    if (!g_optionButtonLongPressFired)
                    {
                        ioData->setOptionShortPress(true);
                    }
                }
            }
        }

        if (g_optionButtonStableState == HIGH && !g_optionButtonLongPressFired)
        {
            if ((now - g_optionButtonPressStartTime) >= LONG_PRESS_DURATION)
            {
                g_optionButtonLongPressFired = true;
                ioData->setOptionLongPress(true);
            }
        }
    }

    // =========================
    // RETURN BUTTON
    // =========================
    {
        bool raw = digitalRead(RETURN_BUTTON);

        if (raw != g_returnButtonPreviousState)
        {
            g_returnButtonDebounceTime = now;
            g_returnButtonPreviousState = raw;
        }

        if ((now - g_returnButtonDebounceTime) > DEBOUNCE_DURATION)
        {
            if (raw != g_returnButtonStableState)
            {
                bool prev = g_returnButtonStableState;
                g_returnButtonStableState = raw;

                if (prev == HIGH && g_returnButtonStableState == LOW)
                {
                    g_returnButtonPressStartTime = now;
                    g_returnButtonLongPressFired = false;
                }

                if (prev == LOW && g_returnButtonStableState == HIGH)
                {
                    if (!g_returnButtonLongPressFired)
                    {
                        ioData->setReturnShortPress(true);
                    }
                }
            }
        }

        if (g_returnButtonStableState == HIGH && !g_returnButtonLongPressFired)
        {
            if ((now - g_returnButtonPressStartTime) >= LONG_PRESS_DURATION)
            {
                g_returnButtonLongPressFired = true;
                ioData->setReturnLongPress(true);
            }
        }
    }
}
