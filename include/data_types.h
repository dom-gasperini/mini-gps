/**
 * @file dataTypes.h
 * @author dom gasperini
 * @brief all of the unique data types used to manage the state of the mini-gps device
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
//  * the standard pulsar message frame
//  */
// typedef struct MessageDataFrame
// {
//     String id;
//     String message;
// } MessageDataFrame;

// /**
//  * for managin the mode the display is in
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
 * @brief Debugger Structure
 *
 */
typedef struct Debugger
{
    // debug toggle
    bool debugEnabled;
    // bool network_debugEnabled;
    bool IO_debugEnabled;
    bool display_debugEnabled;
    bool scheduler_debugEnable;

    // OS data
    // DisplayMode displayMode;

    // // networking
    // MessageDataFrame outgoingMessage;

    // scheduler data
    unsigned long ioTaskCount;
    // unsigned long networkTaskCount;
    unsigned long displayTaskCount;

    unsigned long ioTaskPreviousCount;
    // unsigned long networkTaskPreviousCount;
    unsigned long displayTaskPreviousCount;
} Debugger;

// debug functions
void PrintDebug();
void PrintIODebug();
void PrintNetworkDebug();
void PrintSchedulerDebug();