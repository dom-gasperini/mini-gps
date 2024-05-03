/**
 * @file dataTypes.h
 * @author dom gasperini
 * @brief mini-gps
 * @version 0.1
 * @date 2024-05-02
 */

/*
===============================================================================================
                                    Includes
===============================================================================================
*/

#include <vector>

/*
===============================================================================================
                                    Data Types
===============================================================================================
*/

// /**
//  * for managing the mode the display is in
//  */
// typedef enum DisplayMode
// {
//     HOME = 0,
//     RECIEVE = 1,
//     TRANSMIT = 2,
//     SETTINGS = 3,
//     MISC = 4,
// } DisplayMode;

/**
 * @brief mini-gps data frame
 */
typedef struct Data
{
    double latitude;
    double longitude;
    double altitude;
} Data;

/**
 * @brief debugger structure
 *
 */
typedef struct Debugger
{
    // debug toggle
    bool debugEnabled;
    bool IO_debugEnabled;
    bool display_debugEnabled;
    bool scheduler_debugEnable;

    // OS data
    // DisplayMode displayMode;

    // scheduler data
    unsigned long ioTaskCount;
    unsigned long displayTaskCount;

    unsigned long ioTaskPreviousCount;
    unsigned long displayTaskPreviousCount;
} Debugger;

// debug functions
void PrintDebug();
void PrintIODebug();
void PrintSchedulerDebug();