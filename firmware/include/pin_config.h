/**
 * @file pin_config.h
 * @author dom gasperini
 * @brief contains the pin definitions for mini gps
 * @version 6
 * @date 2026-04-17
 */

/*
===========================================================
                    pin definitions
===========================================================
*/

// buttons
#define SELECT_BUTTON 0 // top    | pulled up
#define OPTION_BUTTON 1 // middle | pulled down
#define RETURN_BUTTON 2 // bottom | pulled down

// gps
#define GPS_WAKE_PIN A4 // enable pin specific to the gps module hardware | HIGH = on | LOW = off
