/**
 * @file main.cpp
 * @author dom gasperini
 * @brief mini-gps
 * @version 0.1
 * @date 2024-05-02
 *
 * @ref https://espressif-docs.readthedocs-hosted.com/projects/arduino-esp32/en/latest/libraries.html#apis      (api and hal docs)
 * @ref https://docs.espressif.com/projects/esp-idf/en/latest/esp32/_images/esp32-devkitC-v4-pinout.png         (pinout & overview)
 * @ref https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/system/freertos_idf.html      (FreeRTOS for ESP32 docs)
 */

/*
===============================================================================================
                                    includes
===============================================================================================
*/

// core
#include <Arduino.h>
#include "rtc.h"
#include "rtc_clk_common.h"
#include <vector>
#include <Wire.h>
#include <SPI.h>

// functionality
#include "HardwareSerial.h"
#include "TinyGPSPlus.h"
#include <TFT_eSPI.h>

// custom headers
#include <data_types.h>
#include <pin_config.h>

/*
===============================================================================================
                                    definitions
===============================================================================================
*/

// display
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

// gps
#define GPS_BAUD 9600

// tasks & timers
#define TASK_STACK_SIZE 4096      // in bytes
#define IO_REFRESH_RATE 1000      // in RTOS ticks (1 tick = ~1 millisecond)
#define DISPLAY_REFRESH_RATE 1000 // in RTOS ticks (1 tick = ~1 millisecond)
#define DEBUG_REFRESH_RATE 1000   // in RTOS ticks (1 tick = ~1 millisecond)

#define ENABLE_DEBUG true // master debug message control

/*
===============================================================================================
                                  global variables
===============================================================================================
*/

/**
 * @brief debugger structure used for organizing debug information
 */
Debugger debugger = {
    // debug toggle
    .debugEnabled = ENABLE_DEBUG,
    // .network_debugEnabled = false,
    .IO_debugEnabled = true,
    .display_debugEnabled = false,
    .scheduler_debugEnable = false,

    // .displayMode = HOME,

    // .outgoingMessage = {},

    // scheduler data
    .ioTaskCount = 0,
    .gpsTaskCount = 0,
    .displayTaskCount = 0,

    .ioTaskPreviousCount = 0,
    .gpsTaskPreviousCount = 0,
    .displayTaskPreviousCount = 0,
};

Data data = {
    .latitude = 0.0f,
    .longitude = 0.0f,
    .altitude = 0.0f,
};

// mutex
SemaphoreHandle_t xMutex = NULL;

// gps
TinyGPSPlus gps;
HardwareSerial serialGPS(1);

// display
TFT_eSPI tft = TFT_eSPI();

// hardware timer
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

// rtos task handles
TaskHandle_t xHandleGPS = NULL;
TaskHandle_t xHandleIO = NULL;
TaskHandle_t xHandleDisplay = NULL;
TaskHandle_t xHandleDebug = NULL;

/*
===============================================================================================
                                  function declarations
===============================================================================================
*/

// tasks
void GPSTask(void *pvParameters);
void IOTask(void *pvParameters);
void DisplayTask(void *pvParameters);
void DebugTask(void *pvParameters);

// helpers
String TaskStateToString(eTaskState state);

/*
================================================================================
                                  setup
================================================================================
*/

void setup()
{
  // set power configuration
  esp_pm_configure(&power_configuration);

  if (debugger.debugEnabled)
  {
    vTaskDelay(3000); // delay startup by 3 seconds
  }

  struct Setup
  {
    bool ioActive = false;
    bool displayActive = false;
    bool loraActive = false;
  } setup;

  // --------------------------- initialize serial  --------------------------- //
  Serial.begin(9600);
  Serial.printf("\n\n|--- STARTING SETUP ---|\n\n");
  // -------------------------------------------------------------------------- //

  // -------------------------- initialize gpio ------------------------------ //
  // analogReadResolution(12);

  // inputs

  // outputs

  // interrupts

  setup.ioActive = true;
  Serial.printf("GPIO INIT [ SUCCESS ]\n");
  // -------------------------------------------------------------------------- //

  // -------------------------- initialize gps -------------------------------- //

  serialGPS.begin(GPS_BAUD, SERIAL_8N1, GPS_RX, GPS_TX);

  // -------------------------------------------------------------------------- //
  // -------------------------- initialize display --------------------------- //
  tft.begin();
  tft.fillScreen(TFT_BLACK);

  setup.displayActive = true;
  // -------------------------------------------------------------------------- //

  // --------------------- scheduler & task status ---------------------------- //
  // init mutex
  xMutex = xSemaphoreCreateMutex();

  // task setup status
  Serial.printf("\nTask Setup Status:\n");
  Serial.printf("I/O TASK SETUP: %s\n", setup.ioActive ? "COMPLETE" : "FAILED");
  Serial.printf("DISPLAY TASK SETUP: %s\n", setup.displayActive ? "COMPLETE" : "FAILED");

  // start tasks
  if (xMutex != NULL)
  {

    if (setup.ioActive)
    {
      xTaskCreate(GPSTask, "IO-Task", TASK_STACK_SIZE, NULL, tskIDLE_PRIORITY, &xHandleIO);
    }

    if (setup.displayActive)
    {
      xTaskCreate(DisplayTask, "Display-Task", TASK_STACK_SIZE, NULL, tskIDLE_PRIORITY, &xHandleDisplay);
    }

    if (debugger.debugEnabled == true)
    {
      xTaskCreate(DebugTask, "Debugger", TASK_STACK_SIZE, NULL, tskIDLE_PRIORITY, &xHandleDebug);
    }
  }
  else
  {
    Serial.printf("FAILED TO INIT MUTEX!\nHALTING OPERATIONS!");
    while (1)
    {
    };
  }
  // task status
  Serial.printf("\nTask Status:\n");
  if (xHandleIO != NULL)
    Serial.printf("I/O TASK STATUS: %s\n", TaskStateToString(eTaskGetState(xHandleIO)));
  else
    Serial.printf("I/O TASK STATUS: DISABLED!\n");

  if (xHandleDisplay != NULL)
    Serial.printf("DISPLAY TASK STATUS: %s\n", TaskStateToString(eTaskGetState(xHandleDisplay)));
  else
    Serial.printf("DISPLAY TASK STATUS: DISABLED!\n");

  // scheduler status
  if (xTaskGetSchedulerState() == 2)
  {
    Serial.printf("\nScheduler Status: RUNNING\n");

    // clock frequency
    rtc_cpu_freq_config_t clock_config;
    rtc_clk_cpu_freq_get_config(&clock_config);
    Serial.printf("CPU Frequency: %dMHz\n", clock_config.freq_mhz);
  }
  else
  {
    Serial.printf("\nScheduler STATUS: FAILED\nHALTING OPERATIONS");
    while (1)
    {
    };
  }
  Serial.printf("\n\n|--- END SETUP ---|\n\n");
  // --------------------------------------------------------------------------- //
}

/*
===============================================================================================
                                freertos task functions
===============================================================================================
*/

/**
 * @brief manages the display
 * @param pvParameters parameters passed to task
 */
void DisplayTask(void *pvParameters)
{
  for (;;)
  {
    // check for mutex availability
    if (xSemaphoreTake(xMutex, (TickType_t)10) == pdTRUE)
    {
      // display things

      // release mutex!
      xSemaphoreGive(xMutex);
    }

    // debugging
    if (debugger.debugEnabled)
    {
      debugger.displayTaskCount++;
    }

    // limit task refresh rate
    vTaskDelay(DISPLAY_REFRESH_RATE);
  }
}

/**
 * @brief I/O
 * @param pvParameters parameters passed to task
 */
void GPSTask(void *pvParameters)
{
  for (;;)
  {
    // check for mutex availability
    if (xSemaphoreTake(xMutex, (TickType_t)10) == pdTRUE)
    {
      gps.encode(serialGPS.read());

      data.latitude = gps.location.lat();
      data.longitude = gps.location.lng();
      data.altitude = gps.altitude.meters();

      // release mutex!
      xSemaphoreGive(xMutex);
    }

    // debugging
    if (debugger.debugEnabled)
    {
      debugger.gpsTaskCount++;
    }

    // limit task refresh rate
    vTaskDelay(IO_REFRESH_RATE);
  }
}

/**
 * @brief manages toggle-able debug settings & scheduler debugging
 */
void DebugTask(void *pvParameters)
{
  for (;;)
  {
    // I/O
    if (debugger.IO_debugEnabled)
    {
      PrintIODebug();
    }

    // Scheduler
    if (debugger.scheduler_debugEnable)
    {
      PrintSchedulerDebug();
    }

    // limit refresh rate
    vTaskDelay(DEBUG_REFRESH_RATE);
  }
}

/*
===============================================================================================
                                    helper functions
================================================================================================
*/

/**
 *
 */
String TaskStateToString(eTaskState state)
{
  // init
  String stateStr;

  // get state
  switch (state)
  {
  case eReady:
    stateStr = "RUNNING";
    break;

  case eBlocked:
    stateStr = "BLOCKED";
    break;

  case eSuspended:
    stateStr = "SUSPENDED";
    break;

  case eDeleted:
    stateStr = "DELETED";
    break;

  default:
    stateStr = "ERROR";
    break;
  }

  return stateStr;
}

/*
===============================================================================================
                                    main loop
===============================================================================================
*/

/**
 * @brief main loop!
 */
void loop()
{
  vTaskDelay(1); // prevent watchdog from getting upset & for debugging, limit print refresh rate
}

/*
===============================================================================================
                                    debug functions
================================================================================================
*/

/**
 * in-depth debugging for i/o
 */
void PrintIODebug()
{
  Serial.printf("#sats: %d\n", gps.satellites.value());
  Serial.printf("latitude: %f\n", data.latitude);
  Serial.printf("longitude: %f\n", data.longitude);
  Serial.printf("altitude: %f\n", data.altitude);

  Serial.printf("\n\n");
}

/**
 * in-dept debugging for the scheduler
 */
void PrintSchedulerDebug()
{
  // inits
  std::vector<eTaskState> taskStates;
  std::vector<String> taskStatesStrings;
  std::vector<int> taskRefreshRate;
  int uptime = esp_rtc_get_time_us() / 1000000;

  // gather task information
  if (xHandleIO != NULL)
  {
    taskStates.push_back(eTaskGetState(xHandleIO));
  }

  if (xHandleDisplay != NULL)
  {
    taskStates.push_back(eTaskGetState(xHandleDisplay));
  }

  taskRefreshRate.push_back(debugger.gpsTaskCount - debugger.gpsTaskPreviousCount);
  taskRefreshRate.push_back(debugger.displayTaskCount - debugger.displayTaskPreviousCount);

  // make it usable
  for (int i = 0; i < taskStates.size() - 1; ++i)
  {
    taskStatesStrings.push_back(TaskStateToString(taskStates.at(i)));
  }

  // print
  Serial.printf("uptime: %d sec | io: (%u)<%d Hz> | display: (%u)<%d Hz>\r",
                uptime, debugger.gpsTaskCount, taskRefreshRate.at(0),
                debugger.displayTaskCount, taskRefreshRate.at(1));

  // update counters
  debugger.gpsTaskPreviousCount = debugger.gpsTaskCount;
  debugger.displayTaskPreviousCount = debugger.displayTaskCount;

  return;
}