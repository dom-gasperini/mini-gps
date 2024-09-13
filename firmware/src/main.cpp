/**
 * @file main.cpp
 * @author dom gasperini
 * @brief mini-gps
 * @version 2.0
 * @date 2024-06-08
 *
 * @ref https://espregpsSerialif-docs.readthedocs-hosted.com/projects/arduino-esp32/en/latest/libraries.html#apis      (api and hal docs)
 * @ref https://docs.espregpsSerialif.com/projects/esp-idf/en/latest/esp32/_images/esp32-devkitC-v4-pinout.png         (pinout & overview)
 * @ref https://github.com/adafruit/Adafruit_GPS                                                                       (gps library)
 */

/*
===============================================================================================
                                    includes
===============================================================================================
*/

// core
#include <Arduino.h>
#include <vector>
#include "rtc.h"

// functionality
#include "Wire.h"
#include <Adafruit_GPS.h>
#include <TFT_eSPI.h>

// custom
#include <data_types.h>
#include <pin_config.h>

/*
===============================================================================================
                                    definitions
===============================================================================================
*/

// i2c
#define I2C_FREQUENCY 9600
#define I2C_GPS_ADDR 0x10

// display
#define REFRESH_DELAY 5

// gps
#define GPS_BAUD 9600

// tasks
#define IO_WRITE_REFRESH_RATE 1000 // measured in ticks (RTOS ticks interrupt at 1 kHz)
#define IO_READ_REFRESH_RATE 1000  // measured in ticks (RTOS ticks interrupt at 1 kHz)
#define I2C_REFRESH_RATE 1         // measured in ticks (RTOS ticks interrupt at 1 kHz)
#define DISPLAY_REFRESH_RATE 250   // measured in ticks (RTOS ticks interrupt at 1 kHz)
#define DEBUG_REFRESH_RATE 1000    // measured in ticks (RTOS ticks interrupt at 1 kHz)

#define TASK_STACK_SIZE 2048 // in bytes

// debugging
#define ENABLE_DEBUGGING true

/*
===============================================================================================
                                  global variables
===============================================================================================
*/

Data data = {
    .connected = false,
    .wasConnected = false,
    .validTime = false,
    .wasValidTime = false,
    .fixQuality = 0,
    .dtLastFix = 0.0f,
    .dtSinceDate = 0.0f,
    .dtSinceTime = 0.0f,

    .latitude = 0.0f,
    .longitude = 0.0f,
    .altitude = 0.0f,

    .speed = 0.0f,
    .angle = 0.0f,

    .year = 0,
    .month = 0,
    .day = 0,

    .hour = 0,
    .minute = 0,
    .second = 0,
    .timeout = 0,

    .numSats = 0,
};

Debugger debugger = {
    .debugEnabled = ENABLE_DEBUGGING,
    .IO_debugEnabled = false,
    .i2c_debugEnabled = false,
    .display_debugEnabled = false,
    .scheduler_debugEnable = true,

    .debugText = "",

    // scheduler data
    .ioWriteTaskCount = 0,
    .ioReadTaskCount = 0,
    .i2cTaskCount = 0,
    .displayTaskCount = 0,

    .displayRefreshRate = 0,

    .ioReadTaskPreviousCount = 0,
    .ioWriteTaskPreviousCount = 0,
    .i2cTaskPreviousCount = 0,
    .displayTaskPreviousCount = 0,
};

// i2c
TwoWire I2CGPS = TwoWire(0);

// gps
Adafruit_GPS gps(&I2CGPS);

// display
TFT_eSPI tft = TFT_eSPI();
int refreshCounter = 0;
int rtcTestCounter = 0;

// Mutex
SemaphoreHandle_t xMutex = NULL;

// Hardware Timer
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

// RTOS Task Handles
TaskHandle_t xHandleIORead = NULL;
TaskHandle_t xHandleIOWrite = NULL;
TaskHandle_t xHandleI2C = NULL;
TaskHandle_t xHandleDisplay = NULL;
TaskHandle_t xHandleDebug = NULL;

/*
===============================================================================================
                                function declarations
===============================================================================================
*/

// tasks
void IOReadTask(void *pvParameters);
void IOWriteTask(void *pvParameters);
void I2CTask(void *pvParameters);
void DisplayTask(void *pvParameters);
void DebugTask(void *pvParameters);

// helpers
String TaskStateToString(eTaskState state);
bool ValidTime();

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
    // delay startup by 3 seconds
    vTaskDelay(3000);
  }

  // setup managment struct
  struct setup
  {
    bool ioActive = false;
    bool displayActive = false;
    bool i2cActive = false;
  };
  setup setup;

  // --------------------------- initialize serial  --------------------------- //
  Serial.begin(9600);
  Serial.printf("\n\n|--- starting setup ---|\n\n");
  // -------------------------------------------------------------------------- //

  // -------------------------- initialize GPIO ------------------------------ //

  // inputs

  // outputs

  Serial.printf("GPIO INIT [ SUCCESS ]\n");
  setup.ioActive = false;
  // -------------------------------------------------------------------------- //

  // ----------------------- initialize I2C connection --------------------- //
  if (I2CGPS.begin(I2C_SCL_PIN, I2C_SDA_PIN) == true)
  {
    Serial.printf("i2c bus init [ success ]\n");
    setup.i2cActive = true;
  }
  else
  {
    Serial.printf("i2c bus init [ failed ]\n");
  }
  // -------------------------------------------------------------------------- //

  // -------------------------- initialize display --------------------------- //
  tft.init();
  tft.setRotation(3);
  tft.fillScreen(TFT_BLACK);

  setup.displayActive = true;
  Serial.printf("display init [ success ]\n");
  // -------------------------------------------------------------------------- //

  // -------------------------- initialize gps -------------------------------- //
  gps.begin(I2C_GPS_ADDR); // The I2C address to use is 0x10

  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  gps.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);

  // uncomment this line to turn on only the "minimum recommended" data
  // GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  // For parsing data, we don't suggest using anything but either RMC only or RMC+GGA since

  gps.sendCommand(PMTK_SET_NMEA_UPDATE_10HZ);

  // Request updates on antenna status, comment out to keep quiet
  // gps.sendCommand(PGCMD_ANTENNA);

  Serial.printf("gps init [ success ]\n");
  // -------------------------------------------------------------------------- //

  Serial.printf("\n\n|--- end setup ---|\n\n");
  // --------------------------------------------------------------------------- //

  // ------------------------------- Scheduler & Task Status --------------------------------- //
  // init mutex
  xMutex = xSemaphoreCreateMutex();

  // task setup status
  Serial.printf("\ntask setup status:\n");
  Serial.printf("i/o task setup: %s\n", setup.ioActive ? "complete" : "failed");
  Serial.printf("i2c task setup: %s\n", setup.i2cActive ? "complete" : "failed");
  Serial.printf("display task setup %s\n", setup.displayActive ? "complete" : "failed");

  // start tasks
  if (xMutex != NULL)
  {
    if (setup.ioActive)
    {
      xTaskCreate(IOReadTask, "read-io", TASK_STACK_SIZE, NULL, tskIDLE_PRIORITY, &xHandleIORead);
      xTaskCreate(IOWriteTask, "write-io", TASK_STACK_SIZE, NULL, tskIDLE_PRIORITY, &xHandleIOWrite);
    }

    if (setup.i2cActive)
    {
      xTaskCreate(I2CTask, "i2c", TASK_STACK_SIZE, NULL, 16, &xHandleI2C);
    }

    if (setup.displayActive)
    {
      xTaskCreate(DisplayTask, "display-update", TASK_STACK_SIZE, NULL, 4, &xHandleDisplay);
    }

    if (debugger.debugEnabled == true)
    {
      xTaskCreate(DebugTask, "debugger", TASK_STACK_SIZE, NULL, tskIDLE_PRIORITY, &xHandleDebug);
    }
  }
  else
  {
    Serial.printf("failed to init mutex!\nhalting operations!!!");
    while (1)
    {
    };
  }

  // task status
  Serial.printf("\ntask status:\n");
  if (xHandleIORead != NULL)
    Serial.printf("i/o read task status: %s\n", TaskStateToString(eTaskGetState(xHandleIORead)));
  else
    Serial.printf("i/o read task status: disabled!\n");

  if (xHandleIOWrite != NULL)
    Serial.printf("i/o write task status: %s\n", TaskStateToString(eTaskGetState(xHandleIOWrite)));
  else
    Serial.printf("i/o write task status: disabled!\n");

  if (xHandleI2C != NULL)
    Serial.printf("i2c task status: %s\n", TaskStateToString(eTaskGetState(xHandleI2C)));
  else
    Serial.printf("i2c task status: disabled!\n");

  if (xHandleDisplay != NULL)
    Serial.printf("display task status: %s\n", TaskStateToString(eTaskGetState(xHandleDisplay)));
  else
    Serial.printf("display task status: disabled!\n");

  if (xHandleDebug != NULL)
    Serial.printf("debug task status: %s\n", TaskStateToString(eTaskGetState(xHandleDebug)));
  else
    Serial.printf("debug task status: disabled!\n");

  // scheduler status
  if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING)
  {
    Serial.printf("\nscheduler status: runnning\n");

    // clock frequency
    rtc_cpu_freq_config_t clock_config;
    rtc_clk_cpu_freq_get_config(&clock_config);
    Serial.printf("cpu frequency: %dMHz\n", clock_config.freq_mhz);
  }
  else
  {
    Serial.printf("\nscheduler status: failed\nhalting operations!!!");
    while (1)
    {
    };
  }
  Serial.printf("\n\n|--- end setup ---|\n\n");
  // ---------------------------------------------------------------------------------------- //
}

/*
===============================================================================================
                                FreeRTOS Task Functions
===============================================================================================
*/

/**
 * @brief reads I/O
 * @param pvParameters parameters passed to task
 */
void IOReadTask(void *pvParameters)
{
  for (;;)
  {
    // check for mutex availability
    if (xSemaphoreTake(xMutex, (TickType_t)10) == pdTRUE)
    {

      // debugging
      if (debugger.debugEnabled)
      {
        debugger.ioReadTaskCount++;
      }

      // release mutex!
      xSemaphoreGive(xMutex);
    }

    // limit task refresh rate
    vTaskDelay(IO_READ_REFRESH_RATE);
  }
}

/**
 * @brief writes I/O
 * @param pvParameters parameters passed to task
 */
void IOWriteTask(void *pvParameters)
{
  for (;;)
  {
    // check for mutex availability
    if (xSemaphoreTake(xMutex, (TickType_t)10) == pdTRUE)
    {

      // debugging
      if (debugger.debugEnabled)
      {
        debugger.ioWriteTaskCount++;
      }

      // release mutex!
      xSemaphoreGive(xMutex);
    }

    // limit task refresh rate
    vTaskDelay(IO_WRITE_REFRESH_RATE);
  }
}

/**
 * @brief i2c bus
 * @param pvParameters parameters passed to task
 */
void I2CTask(void *pvParameters)
{
  for (;;)
  {
    // check for mutex availability
    if (xSemaphoreTake(xMutex, (TickType_t)5) == pdTRUE)
    {
      // read gps data
      if (gps.read() && gps.newNMEAreceived())
      {
        gps.parse(gps.lastNMEA()); // this also sets the newNMEAreceived() flag to false

        // connection data
        data.connected = gps.fix;
        data.fixQuality = gps.fixquality;

        data.numSats = gps.satellites;
        data.dtLastFix = gps.secondsSinceFix();
        data.dtSinceTime = gps.secondsSinceTime();
        data.dtSinceTime = gps.secondsSinceDate();

        // collect location data
        data.latitude = gps.latitudeDegrees;
        data.longitude = gps.longitudeDegrees;
        data.altitude = gps.altitude;

        // collect speed data
        data.speed = gps.speed * 1.1507795; // speed is given in knots, convert to mph
        if (data.speed < 0.5)               // no need for like 0.1 mph of speed
        {
          data.speed = 0;
        }

        // collect angle data, current heading
        data.angle = gps.angle;

        // collect date data
        data.year = gps.year + 2000;
        data.month = gps.month;
        data.day = gps.day;

        // collect time data
        data.hour = gps.hour;
        data.minute = gps.minute;
        data.second = gps.seconds;

        // data.validTime = ValidTime();
        data.validTime = true;
      }

      // debugging
      if (debugger.debugEnabled)
      {
        debugger.i2cTaskCount++;
      }

      // release mutex!
      xSemaphoreGive(xMutex);
    }

    // limit task refresh rate
    vTaskDelay(I2C_REFRESH_RATE);
  }
}

/**
 * @brief display manager
 * @param pvParameters parameters passed to task
 */
void DisplayTask(void *pvParameters)
{
  for (;;)
  {
    // check for mutex availability
    if (xSemaphoreTake(xMutex, (TickType_t)1) == pdTRUE)
    {
      // clear screen on state change
      if (data.wasConnected != data.connected || data.wasValidTime != data.validTime)
      {
        data.wasConnected = data.connected;
        data.wasValidTime = data.validTime;
        tft.fillScreen(TFT_BLACK);
      }

      // check connection
      if (data.connected || data.validTime)
      {
        // display gps information
        tft.setTextSize(3);
        tft.setTextColor(TFT_RED);
        tft.setCursor(0, 0);
        tft.printf("mini-gps:");

        // outline boxes
        tft.drawRect(0, 25, 320, 155, TFT_DARKCYAN);
        tft.drawRect(0, 185, 320, 55, TFT_MAGENTA);

        // set info font size and color
        tft.setTextSize(2);
        tft.setTextColor(TFT_CYAN, TFT_BLACK, true);

        // location data
        if (data.numSats >= 3)
        {
          tft.setCursor(5, 30);
          tft.printf("latitude: %f", data.latitude);

          tft.setCursor(5, 50);
          tft.printf("longitude: %f", data.longitude);

          tft.setCursor(5, 70);
          tft.printf("altitude: %.1fm   ", data.altitude);
        }
        else
        {
          tft.setCursor(5, 30);
          tft.printf("latitude:  ---.---     ");

          tft.setCursor(5, 50);
          tft.printf("longitude: ---.---     ");

          tft.setCursor(5, 70);
          tft.printf("altitude:  ---.---     ");
        }

        // speed data
        tft.setTextColor(TFT_GOLD, TFT_BLACK, true);
        tft.setCursor(5, 95);
        if (data.numSats >= 4)
        {
          tft.printf("speed (mph): %.1f ", data.speed);
        }
        else
        {
          tft.printf("speed (mph): ---  ", data.speed);
        }

        // angle data
        tft.setTextColor(TFT_GOLD, TFT_BLACK, true);
        tft.setCursor(5, 115);
        if (data.speed > 0.5)
        {
          tft.printf("heading: %.1f ", data.angle);
        }
        else
        {
          tft.printf("heading: ---  ", data.angle);
        }

        // date
        tft.setTextColor(TFT_GREEN, TFT_BLACK, true);
        tft.setCursor(5, 140);
        tft.printf("date: %d / %d / %d    ", data.year, data.month, data.day);

        // time
        tft.setCursor(5, 160);
        tft.printf("time: %d:%d:%d (UTC)      ", data.hour, data.minute, data.second);

        // sat data
        tft.setCursor(5, 190);
        tft.setTextColor(TFT_WHITE, TFT_BLACK, true);
        tft.printf("sats: ");

        if (data.numSats == 0)
          tft.setTextColor(TFT_RED, TFT_BLACK, true);
        if (data.numSats > 0 && data.numSats <= 3)
          tft.setTextColor(TFT_ORANGE, TFT_BLACK, true);
        if (data.numSats >= 3)
          tft.setTextColor(TFT_GREEN, TFT_BLACK, true);

        tft.printf("%d ", data.numSats);

        // connection type
        tft.setCursor(130, 190);
        tft.setTextColor(TFT_WHITE, TFT_BLACK, true);
        tft.printf("status: ");
        switch (data.fixQuality)
        {
        case 0:
          tft.setTextColor(TFT_RED, TFT_BLACK, true);
          tft.printf("NO CONN");
          break;

        case 1:
          tft.setTextColor(TFT_GREEN, TFT_BLACK, true);
          tft.printf("GPS    ");
          break;

        case 2:
          tft.setTextColor(TFT_BLUE, TFT_BLACK, true);
          tft.printf("D-GPS  ");
          break;

        default:
          tft.setTextColor(TFT_RED, TFT_BLACK, true);
          tft.printf("uh-oh  ");
          break;
        }

        // time since last fix
        tft.setCursor(5, 220);
        tft.setTextColor(TFT_WHITE, TFT_BLACK, true);
        tft.printf("dt-fix: ");
        if (data.dtLastFix < 3.0 && data.dtLastFix > 0)
        {
          tft.setTextColor(TFT_GREEN, TFT_BLACK, true);
        }
        else if (data.dtLastFix > 6000) // no fix!
        {
          tft.setTextColor(TFT_RED, TFT_BLACK, true);
        }
        else
        {
          tft.setTextColor(TFT_ORANGE, TFT_BLACK, true);
        }

        if (data.dtLastFix > 120)
        {
          tft.printf("unreliable data", data.dtLastFix);
        }
        else
        {
          tft.printf("%.1f seconds    ", data.dtLastFix);
        }
      }

      // no connection
      else
      {
        // flash a circle top left as activity indicator
        if (refreshCounter <= REFRESH_DELAY)
        {
          tft.fillCircle(10, 10, 10, TFT_GREEN);
        }
        if (refreshCounter >= REFRESH_DELAY)
        {
          tft.fillCircle(10, 10, 10, TFT_BLACK);
        }
        if (refreshCounter >= (REFRESH_DELAY * 2))
        {
          refreshCounter = 0;
        }
        refreshCounter++;

        // text for status indicator
        tft.setTextColor(TFT_RED, TFT_BLACK, true);
        tft.setCursor(30, 5);
        tft.printf("searching...");

        // general info
        tft.setTextSize(2);
        tft.setTextColor(TFT_RED, TFT_BLACK, true);
        tft.setCursor(80, 110);
        tft.printf("< no signal >");
        tft.setCursor(40, 130);
        tft.printf("< rtc data invalid >");
      }

      // debugging
      if (debugger.debugEnabled)
      {
        debugger.displayTaskCount++;
      }

      // release mutex!
      xSemaphoreGive(xMutex);
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

    // i2c
    if (debugger.i2c_debugEnabled)
    {
      PrintI2CDebug();
    }

    // display
    if (debugger.display_debugEnabled)
    {
      PrintDisplayDebug();
    }

    // scheduler
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
                                      main loop
===============================================================================================
*/

void loop()
{
  // everything is managed by RTOS, so nothing happens here!
  vTaskDelay(1); // prevent watchdog from getting upset
}

/*
===============================================================================================
                                    helper functions
===============================================================================================
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

/**
 *
 */
bool ValidTime()
{
  // inits
  bool valid = true;
  uint64_t cutoff = 10; // in seconds

  if (data.hour == 0 && data.minute == 0 && data.second == 0)
  {
    if (data.timeout == 0)
    {
      data.timeout = (long)esp_rtc_get_time_us() + (cutoff * 1000000); // to microseconds
    }

    if (data.timeout <= esp_rtc_get_time_us())
    {
      valid = false;
    }
  }
  else
  {
    data.timeout = 0;
  }

  return valid;
}

/*
===============================================================================================
                                    debug functions
================================================================================================
*/

/**
 * @brief some nice in-depth debugging for I/O
 */
void PrintIODebug()
{
  Serial.printf("\n--- start i/o debug ---\n");

  Serial.printf("\n--- end i/o debug ---\n");
}

void PrintI2CDebug()
{
  Serial.printf("\n--- start i2c debug ---\n");

  debugger.debugText = "";
  for (int i = 0; i < 100; ++i)
  {
    if (Serial.available())
    {
      char c = Serial.read();
      gps.write(c);
    }
    if (gps.available())
    {
      char c = gps.read();
      Serial.write(c);

      debugger.debugText.concat(c);
    }
  }
  Serial.printf("\n\n");

  Serial.printf("fix: %s\n", gps.fix ? "yes" : "no");
  Serial.printf("# sats: %d\n", gps.satellites);

  Serial.printf("time: %d:%d:%d\n", gps.hour, gps.minute, gps.seconds);
  Serial.printf("date: %d-%d-%d\n", gps.year, gps.month, gps.day);

  Serial.printf("lat: %f\n", gps.latitude);
  Serial.printf("long: %f\n", gps.longitude);
  Serial.printf("alt: %f\n", gps.altitude);

  Serial.printf("dt-fix: %f\n", gps.secondsSinceFix());
  Serial.printf("dt-time: %f\n", gps.secondsSinceTime());
  Serial.printf("dt-date: %f\n", gps.secondsSinceDate());

  // Serial.printf("timeout: %ld | clock: %ld\n", data.timeout, (long)esp_rtc_get_time_us());

  Serial.printf("\n--- end i2c debug ---\n");
}

void PrintDisplayDebug()
{
  Serial.printf("\n--- start display debug ---\n");

  // tft.fillScreen(TFT_BLACK);
  // tft.setTextSize(2);
  // tft.setCursor(0, 0);
  // tft.setTextColor(TFT_GREEN, TFT_BLACK, true);
  // tft.printf("%s", debugger.debugText.c_str());

  Serial.printf("\n--- end display debug ---\n");
}

/**
 * @brief scheduler debugging
 */
void PrintSchedulerDebug()
{
  // inits
  std::vector<eTaskState> taskStates;
  std::vector<String> taskStatesStrings;
  std::vector<int> taskRefreshRate;
  int uptime = esp_rtc_get_time_us() / 1000000;

  // gather task information
  if (xHandleIORead != NULL)
  {
    taskStates.push_back(eTaskGetState(xHandleIORead));
  }
  if (xHandleIOWrite != NULL)
  {
    taskStates.push_back(eTaskGetState(xHandleIOWrite));
  }
  if (xHandleI2C != NULL)
  {
    taskStates.push_back(eTaskGetState(xHandleI2C));
  }
  if (xHandleDisplay != NULL)
  {
    taskStates.push_back(eTaskGetState(xHandleDisplay));
  }

  taskRefreshRate.push_back(debugger.ioReadTaskCount - debugger.ioReadTaskPreviousCount);
  taskRefreshRate.push_back(debugger.ioWriteTaskCount - debugger.ioWriteTaskPreviousCount);
  taskRefreshRate.push_back(debugger.i2cTaskCount - debugger.i2cTaskPreviousCount);
  taskRefreshRate.push_back(debugger.displayTaskCount - debugger.displayTaskPreviousCount);

  // make it usable
  for (int i = 0; i < taskStates.size() - 1; ++i)
  {
    taskStatesStrings.push_back(TaskStateToString(taskStates.at(i)));
  }

  // print
  Serial.printf("uptime: %d | read io: <%d Hz> (%d) | write io: <%d Hz> (%d) | i2c: <%d Hz> (%d) | display: <%d Hz> (%d) \r",
                uptime, taskRefreshRate.at(0), debugger.ioReadTaskCount, taskRefreshRate.at(1), debugger.ioWriteTaskCount, taskRefreshRate.at(2),
                debugger.i2cTaskCount, taskRefreshRate.at(3), debugger.displayTaskCount);

  // update counters
  debugger.ioReadTaskPreviousCount = debugger.ioReadTaskCount;
  debugger.ioWriteTaskPreviousCount = debugger.ioWriteTaskCount;
  debugger.i2cTaskPreviousCount = debugger.i2cTaskCount;
  debugger.displayTaskPreviousCount = debugger.displayTaskCount;
}