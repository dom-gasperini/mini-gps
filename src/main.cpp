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
#define MAX_SATELLITES 40

// tasks & timers
#define TASK_STACK_SIZE 4096    // in bytes
#define IO_REFRESH_RATE 50      // in RTOS ticks (1 tick = ~1 millisecond)
#define GPS_REFRESH_RATE 5000   // in RTOS ticks (1 tick = ~1 millisecond)
#define DISPLAY_REFRESH_RATE 50 // in RTOS ticks (1 tick = ~1 millisecond)
#define DEBUG_REFRESH_RATE 1000 // in RTOS ticks (1 tick = ~1 millisecond)

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
    .IO_debugEnabled = false,
    .display_debugEnabled = false,
    .scheduler_debugEnable = true,

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

    .speed = 0.0f,

    .year = 0,
    .month = 0,
    .day = 0,

    .hour = 0,
    .minute = 0,
    .second = 0,

    .numSats = 0,
    .satellites = {MAX_SATELLITES},
};

// mutex
SemaphoreHandle_t xMutex = NULL;

// gps
TinyGPSPlus gps;
TinyGPSCustom totalGPGSVMessages(gps, "GPGSV", 1); // $GPGSV sentence, first element
TinyGPSCustom messageNumber(gps, "GPGSV", 2);      // $GPGSV sentence, second element
TinyGPSCustom satellitesInView(gps, "GPGSV", 3);   // $GPGSV sentence, third element

TinyGPSCustom satNumber[4];
TinyGPSCustom elevation[4];
TinyGPSCustom azimuth[4];
TinyGPSCustom snr[4];

HardwareSerial serialGPS(1);

// display
TFT_eSPI tft = TFT_eSPI();

// hardware timer
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

// rtos task handles
TaskHandle_t xHandleIO = NULL;
TaskHandle_t xHandleGPS = NULL;
TaskHandle_t xHandleDisplay = NULL;
TaskHandle_t xHandleDebug = NULL;

/*
===============================================================================================
                                  function declarations
===============================================================================================
*/

// tasks
void IOTask(void *pvParameters);
void GPSTask(void *pvParameters);
void DisplayTask(void *pvParameters);
void DebugTask(void *pvParameters);

// helpers
void TrackSatellites();
void GPSDateTime();
void GPSSpeed();
String TaskStateToString(eTaskState state);

/*
===============================================================================================
                                  setup
===============================================================================================
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
    bool gpsActive = false;
    bool displayActive = false;
  } setup;

  // --------------------------- initialize serial  --------------------------- //
  Serial.begin(9600);
  Serial.printf("\n\n|--- starting setup ---|\n\n");
  // -------------------------------------------------------------------------- //

  // -------------------------- initialize gpio ------------------------------ //
  // analogReadResolution(12);

  // inputs

  // outputs

  // interrupts

  setup.ioActive = true;
  Serial.printf("gpio init [ success ]\n");
  // -------------------------------------------------------------------------- //

  // -------------------------- initialize gps -------------------------------- //
  serialGPS.begin(GPS_BAUD, SERIAL_8N1, GPS_RX, GPS_TX);

  // init sat trackers
  for (int i = 0; i < 4; ++i)
  {
    satNumber[i].begin(gps, "GPGSV", 4 + 4 * i); // offsets 4, 8, 12, 16
    elevation[i].begin(gps, "GPGSV", 5 + 4 * i); // offsets 5, 9, 13, 17
    azimuth[i].begin(gps, "GPGSV", 6 + 4 * i);   // offsets 6, 10, 14, 18
    snr[i].begin(gps, "GPGSV", 7 + 4 * i);       // offsets 7, 11, 15, 19
  }

  setup.gpsActive = true;
  Serial.printf("gps init [ success ]\n");
  // -------------------------------------------------------------------------- //
  // -------------------------- initialize display --------------------------- //
  tft.begin();
  tft.setRotation(1);
  tft.fillScreen(TFT_BLACK);
  tft.setCursor(10, 10);
  tft.printf("test!");

  setup.displayActive = true;
  Serial.printf("display init [ success ]\n");
  // -------------------------------------------------------------------------- //

  // --------------------- scheduler & task status ---------------------------- //
  // init mutex
  xMutex = xSemaphoreCreateMutex();

  // task setup status
  Serial.printf("\ntask setup status:\n");
  Serial.printf("i/o task setup: %s\n", setup.ioActive ? "complete" : "failed");
  Serial.printf("gps task setup: %s\n", setup.gpsActive ? "complete" : "failed");
  Serial.printf("display task setup: %s\n", setup.displayActive ? "complete" : "failed");

  // start tasks
  if (xMutex != NULL)
  {
    if (setup.ioActive)
    {
      xTaskCreate(IOTask, "IO-Task", TASK_STACK_SIZE, NULL, tskIDLE_PRIORITY, &xHandleIO);
    }

    if (setup.gpsActive)
    {
      xTaskCreate(GPSTask, "GPS-Task", TASK_STACK_SIZE, NULL, tskIDLE_PRIORITY, &xHandleGPS);
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
  Serial.printf("\ntask status:\n");
  if (xHandleIO != NULL)
    Serial.printf("i/o task status: %s\n", TaskStateToString(eTaskGetState(xHandleIO)));
  else
    Serial.printf("i/o task status: DISABLED!\n");

  if (xHandleGPS != NULL)
    Serial.printf("gps task status: %s\n", TaskStateToString(eTaskGetState(xHandleGPS)));
  else
    Serial.printf("gps task status: DISABLED!\n");

  if (xHandleDisplay != NULL)
    Serial.printf("display task status: %s\n", TaskStateToString(eTaskGetState(xHandleDisplay)));
  else
    Serial.printf("display task status: DISABLED!\n");

  // scheduler status
  if (xTaskGetSchedulerState() == 2)
  {
    Serial.printf("\nscheduler status: running\n");

    // clock frequency
    rtc_cpu_freq_config_t clock_config;
    rtc_clk_cpu_freq_get_config(&clock_config);
    Serial.printf("soc frequency: %dMHz\n", clock_config.freq_mhz);
  }
  else
  {
    Serial.printf("\nscheduler status: FAILED\nHALTING OPERATIONS");
    while (1)
    {
    };
  }
  Serial.printf("\n\n|--- end setup ---|\n\n");
  // --------------------------------------------------------------------------- //
}

/*
===============================================================================================
                                freertos task functions
===============================================================================================
*/

/**
 * @brief i/o task
 * @param pvParameters parameters passed to task
 */
void IOTask(void *pvParameters)
{
  for (;;)
  {
    // check for mutex availability
    if (xSemaphoreTake(xMutex, (TickType_t)10) == pdTRUE)
    {
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
 * @brief gps task
 * @param pvParameters parameters passed to task
 */
void GPSTask(void *pvParameters)
{
  for (;;)
  {
    // check for mutex availability
    if (xSemaphoreTake(xMutex, (TickType_t)10) == pdTRUE)
    {
      if (serialGPS.available())
      {
        gps.encode(serialGPS.read());

        // sat tracking
        TrackSatellites();

        // current gps position data
        if (gps.location.isUpdated())
        {
          data.latitude = gps.location.lat();
          data.longitude = gps.location.lng();
          data.altitude = gps.altitude.meters();
        }

        // gps date and time data
        if (gps.date.isUpdated())
        {
          data.year = gps.date.year();
          data.month = gps.date.month();
          data.day = gps.date.day();
        }
        if (gps.time.isUpdated())
        {
          data.hour = gps.time.hour();
          data.minute = gps.time.minute();
          data.second = gps.time.second();
        }

        // gps speed data
        if (gps.speed.isUpdated())
        {
          data.speed = gps.speed.mph();
        }
      }

      // release mutex!
      xSemaphoreGive(xMutex);
    }

    // debugging
    if (debugger.debugEnabled)
    {
      debugger.gpsTaskCount++;
    }

    // limit task refresh rate
    vTaskDelay(GPS_REFRESH_RATE);
  }
}

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
      // display gps information
      tft.setTextSize(3);
      tft.setTextColor(TFT_RED);
      tft.setCursor(0, 0);
      tft.printf("gps data:");

      tft.setTextSize(2);
      tft.setTextColor(TFT_CYAN);
      tft.setCursor(0, 30);
      tft.printf("latitude: %f", data.latitude);

      tft.setCursor(0, 50);
      tft.printf("longitude: %f", data.longitude);

      tft.setCursor(0, 70);
      tft.printf("altitude: %f", data.altitude);

      tft.setTextColor(TFT_PURPLE);
      tft.setCursor(0, 90);
      tft.printf("current speed (mph): %.1f", data.speed);

      tft.setTextColor(TFT_GREEN);
      tft.setCursor(0, 120);
      tft.printf("date: %d - %d - %d", data.year, data.month, data.day);

      tft.setCursor(0, 140);
      tft.printf("time: %d : %d : %d", data.hour, data.minute, data.second);

      tft.setTextColor(TFT_WHITE);
      tft.setCursor(0, 200);
      tft.printf("avg conn strength: %.2f", data.avgSignalStrength);

      tft.setCursor(0, 220);
      tft.printf("# satellites: %d", data.numSats);

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

void TrackSatellites()
{
  // gather data about connected satellites
  if (totalGPGSVMessages.isUpdated())
  {
    for (int i = 0; i < 4; ++i)
    {
      int no = atoi(satNumber[i].value());
      if (no >= 1 && no <= MAX_SATELLITES)
      {
        data.satellites[no - 1].elevation = atoi(elevation[i].value());
        data.satellites[no - 1].azimuth = atoi(azimuth[i].value());
        data.satellites[no - 1].snr = atoi(snr[i].value());
        data.satellites[no - 1].active = true;
      }
    }

    int totalMessages = atoi(totalGPGSVMessages.value());
    int currentMessage = atoi(messageNumber.value());
    if (totalMessages == currentMessage)
    {
      // num sats
      data.numSats = 0;
      for (int i = 0; i < MAX_SATELLITES; ++i)
      {
        if (data.satellites[i].active)
        {
          data.numSats++;
        }
      }

      // elevation
      for (int i = 0; i < MAX_SATELLITES; ++i)
      {
        if (data.satellites[i].active)
        {
          // Serial.print(data.satellites[i].elevation);
          // Serial.print(F(" "));
        }
      }

      // azimuth
      for (int i = 0; i < MAX_SATELLITES; ++i)
      {
        if (data.satellites[i].active)
        {
          // Serial.print(data.satellites[i].azimuth);
          // Serial.print(F(" "));
        }
      }

      // signal to noise ratio
      float runSum = 0;
      int count = 0;
      for (int i = 0; i < MAX_SATELLITES; ++i)
      {
        if (data.satellites[i].active)
        {
          // Serial.print(data.satellites[i].snr);
          // Serial.print(F(" "));
          count++;
          runSum += data.satellites[i].snr;
        }
      }
      data.avgSignalStrength = runSum / count;

      // set all to inactive
      for (int i = 0; i < MAX_SATELLITES; ++i)
        data.satellites[i].active = false;
    }
  }
  return;
}

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
    stateStr = "running";
    break;

  case eBlocked:
    stateStr = "blocked";
    break;

  case eSuspended:
    stateStr = "suspended";
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
  Serial.printf("# data.satellites: %d\n", gps.satellites.value());
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

  if (xHandleGPS != NULL)
  {
    taskStates.push_back(eTaskGetState(xHandleGPS));
  }

  if (xHandleDisplay != NULL)
  {
    taskStates.push_back(eTaskGetState(xHandleDisplay));
  }

  taskRefreshRate.push_back(debugger.ioTaskCount - debugger.ioTaskPreviousCount);
  taskRefreshRate.push_back(debugger.gpsTaskCount - debugger.gpsTaskPreviousCount);
  taskRefreshRate.push_back(debugger.displayTaskCount - debugger.displayTaskPreviousCount);

  // make it usable
  for (int i = 0; i < taskStates.size() - 1; ++i)
  {
    taskStatesStrings.push_back(TaskStateToString(taskStates.at(i)));
  }

  // print
  Serial.printf("uptime: %d sec | io: (%u)<%d Hz> | gps:(%u)<%d Hz> | display: (%u)<%d Hz>\r",
                uptime, debugger.ioTaskCount, taskRefreshRate.at(0),
                debugger.gpsTaskCount, taskRefreshRate.at(1),
                debugger.displayTaskCount, taskRefreshRate.at(2));

  // update counters
  debugger.ioTaskPreviousCount = debugger.ioTaskCount;
  debugger.gpsTaskPreviousCount = debugger.gpsTaskCount;
  debugger.displayTaskPreviousCount = debugger.displayTaskCount;

  return;
}