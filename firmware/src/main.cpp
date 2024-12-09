/**
 * @file main.cpp
 * @author dom gasperini
 * @brief mini-gps
 * @version 2.2
 * @date 2024-12-05
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
#define I2C_FREQUENCY 115200
#define I2C_GPS_ADDR 0x10

// display
#define REFRESH_DELAY 5

// general
#define KNOTS_TO_MPH 1.1507795 // mulitplier for converting knots to mph
#define MIN_SPEED 2.00         // minimum number of knots before displaying speed to due resolution limitations
#define FINAL_VALID_OPERATING_YEAR 2080
#define START_VALID_OPERATING_YEAR 2024

// tasks
#define IO_WRITE_REFRESH_RATE 1000 // measured in ticks (RTOS ticks interrupt at 1 kHz)
#define IO_READ_REFRESH_RATE 1000  // measured in ticks (RTOS ticks interrupt at 1 kHz)
#define I2C_REFRESH_RATE 1         // measured in ticks (RTOS ticks interrupt at 1 kHz)
#define DISPLAY_REFRESH_RATE 250   // measured in ticks (RTOS ticks interrupt at 1 kHz)
#define DEBUG_REFRESH_RATE 1000    // measured in ticks (RTOS ticks interrupt at 1 kHz)

#define TASK_STACK_SIZE 2048 // in bytes

// debugging
#define ENABLE_DEBUGGING false

/*
===============================================================================================
                                  global variables
===============================================================================================
*/

Data data = {
    .connected = false,
    .wasConnected = false,
    .validDate = false,
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
int displayRefreshCounter = 0;

// mutex
SemaphoreHandle_t xMutex = NULL;

// hardware timer
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

// rtos task handles
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
bool IsValidDate();
void ActivityAnimation();

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
  if (I2CGPS.begin(I2C_SCL_PIN, I2C_SDA_PIN, I2C_FREQUENCY) == true)
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

  // display gps information
  tft.setTextSize(3);
  tft.setTextColor(TFT_RED);
  tft.setCursor(0, 0);
  tft.printf("mini-gps:");

  // outline boxes
  tft.drawRect(0, 25, 320, 155, TFT_DARKCYAN);
  tft.drawRect(0, 185, 320, 55, TFT_MAGENTA);

  setup.displayActive = true;
  Serial.printf("display init [ success ]\n");
  // -------------------------------------------------------------------------- //

  // -------------------------- initialize gps -------------------------------- //
  gps.begin(I2C_GPS_ADDR);

  // set gps i2c baud rate
  gps.sendCommand(PMTK_SET_BAUD_115200);

  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  gps.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);

  // set gps to mcu update rate
  gps.sendCommand(PMTK_SET_NMEA_UPDATE_10HZ);

  // set gps position fix rate
  gps.sendCommand(PMTK_API_SET_FIX_CTL_5HZ);

  // request updates on antenna status (don't need this on)
  // gps.sendCommand(PGCMD_ANTENNA);

  Serial.printf("gps init [ success ]\n");
  // -------------------------------------------------------------------------- //

  Serial.printf("\n\n|--- end setup ---|\n\n");
  // --------------------------------------------------------------------------- //

  // ------------------------------- scheduler & task status --------------------------------- //
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
      xTaskCreate(I2CTask, "i2c", TASK_STACK_SIZE, NULL, 16, &xHandleI2C); // 16 seems optimal
    }

    if (setup.displayActive)
    {
      xTaskCreate(DisplayTask, "display-update", TASK_STACK_SIZE, NULL, 8, &xHandleDisplay); // 4 seems optimal
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
                                rtos task functions
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
    if (xSemaphoreTake(xMutex, (TickType_t)1) == pdTRUE)
    {
      // read gps data
      if (gps.read() && gps.newNMEAreceived())
      {
        gps.parse(gps.lastNMEA()); // this sets the newNMEAreceived() flag to false

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

        // collect vector data
        data.speed = gps.speed; // speed is given in knots
        data.angle = gps.angle;

        // not enough resolution for accurately measuring very low speeds
        if (data.speed > MIN_SPEED)
        {
          data.speed = data.speed * KNOTS_TO_MPH; // convert to mph
        }
        else
        {
          data.speed = 0;
          data.angle = 0;
        }

        // collect date data
        data.year = gps.year + 2000;
        data.month = gps.month;
        data.day = gps.day;

        // collect time data
        data.hour = gps.hour;
        data.minute = gps.minute;
        data.second = gps.seconds;

        data.validDate = IsValidDate();
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
    if (xSemaphoreTake(xMutex, (TickType_t)10) == pdTRUE)
    {
      // location data
      tft.setTextSize(2);
      tft.setTextColor(TFT_CYAN, TFT_BLACK, true);
      if (data.numSats >= 3)
      {
        tft.setCursor(5, 30);
        tft.printf("latitude: %.5f", data.latitude);

        tft.setCursor(5, 50);
        tft.printf("longitude: %.5f", data.longitude);

        tft.setCursor(5, 70);
        tft.printf("altitude: %d m        ", (int)data.altitude);
      }
      else
      {
        tft.setCursor(5, 30);
        tft.printf("latitude:  ---.---    ");

        tft.setCursor(5, 50);
        tft.printf("longitude: ---.---    ");

        tft.setCursor(5, 70);
        tft.printf("altitude:  ---.---    ");
      }

      // speed data
      tft.setTextColor(TFT_GOLD, TFT_BLACK, true);
      tft.setCursor(5, 95);
      if (data.numSats > 3)
      {
        tft.printf("speed: %.1f mph", data.speed);
      }
      else
      {
        tft.printf("speed: ---      ", data.speed);
      }

      // angle data
      tft.setTextColor(TFT_GOLD, TFT_BLACK, true);
      tft.setCursor(5, 115);
      if (data.speed > 0.5)
      {
        tft.printf("heading: %d deg", (int)data.angle);
      }
      else
      {
        tft.printf("heading: ---      ", data.angle);
      }

      // date and time
      if (data.validDate)
      {
        tft.setTextColor(TFT_GREEN, TFT_BLACK, true);
        tft.setCursor(5, 140);
        tft.printf("date: %d / %d / %d    ", data.year, data.month, data.day);

        tft.setCursor(5, 160);
        tft.printf("time: %d:%d:%d (UTC)      ", data.hour, data.minute, data.second);
      }
      else
      {
        tft.setTextColor(TFT_RED, TFT_BLACK, true);
        tft.setCursor(5, 140);
        tft.printf("date: -- / -- / --    ", data.year, data.month, data.day);

        tft.setTextColor(TFT_MAGENTA, TFT_BLACK, true);
        tft.setCursor(5, 160);
        tft.printf("time: %d:%d:%d            ", data.hour, data.minute, data.second);
      }

      // sat data
      tft.setCursor(5, 190);
      tft.setTextColor(TFT_WHITE, TFT_BLACK, true);
      tft.printf("sats: ");
      if (data.numSats == 0)
      {
        tft.setTextColor(TFT_RED, TFT_BLACK, true);
      }
      if (data.numSats > 0 && data.numSats <= 3)
      {
        tft.setTextColor(TFT_ORANGE, TFT_BLACK, true);
      }
      if (data.numSats > 3)
      {
        tft.setTextColor(TFT_GREEN, TFT_BLACK, true);
      }
      tft.printf("%d ", data.numSats);

      // connection type
      tft.setCursor(130, 190);
      tft.setTextColor(TFT_WHITE, TFT_BLACK, true);
      tft.printf("status: ");
      switch (data.fixQuality)
      {
      // invalid fix quality
      case 0:
        if (data.validDate)
        {
          tft.setTextColor(TFT_ORANGE, TFT_BLACK, true);
          tft.printf("no fix ");
        }
        else // no fix and no date time data
        {
          tft.setTextColor(TFT_RED, TFT_BLACK, true);
          tft.printf("no conn");
        }
        break;

      // gps fix quality
      case 1:
        tft.setTextColor(TFT_GREEN, TFT_BLACK, true);
        tft.printf("gps    ");
        break;

      // d-gps fixed quality
      case 2:
        tft.setTextColor(TFT_BLUE, TFT_BLACK, true);
        tft.printf("d-gps  ");
        break;

      // catch statement
      default:
        tft.setTextColor(TFT_RED, TFT_BLACK, true);
        tft.printf("err:%d ", data.fixQuality);
        break;
      }

      // time since last fix
      tft.setCursor(5, 220);
      tft.setTextColor(TFT_WHITE, TFT_BLACK, true);
      tft.printf("dt-fix: ");
      if (data.dtLastFix < 1.0)
      {
        tft.setTextColor(TFT_GREEN, TFT_BLACK, true);
        tft.printf("%.2f seconds    ", data.dtLastFix);
      }
      else if (data.dtLastFix < 120 && data.dtLastFix > 1.0) // been a bit since a connection
      {
        tft.setTextColor(TFT_ORANGE, TFT_BLACK, true);
        tft.printf("%.2f seconds    ", data.dtLastFix);
      }
      else // longer than a minute without a fix
      {
        tft.setTextColor(TFT_RED, TFT_BLACK, true);
        if (data.validDate)
        {
          tft.printf("> 2 minutes     ", data.dtLastFix);
        }
        else
        {
          tft.printf("unreliable data", data.dtLastFix);
        }
      }

      // activity animation
      ActivityAnimation();

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
 * @param pvParameters parameters passed to task
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
  // everything is managed by rtos, so nothing happens here!
  vTaskDelay(1); // prevent watchdog from getting upset
}

/*
===============================================================================================
                                    helper functions
===============================================================================================
*/

/**
 * @brief converts an rtos task state to printable string format
 * @param state = rtos task state
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
 * @brief determine if the real-time clock is cold started or warm started
 */
bool IsValidDate()
{
  // test for valid year data
  if (data.year >= FINAL_VALID_OPERATING_YEAR || data.year < START_VALID_OPERATING_YEAR)
  {
    return false; // gps indicating invalid operating year
  }
  else
  {
    return true;
  }
}

/**
 * @brief displays animation about current activity
 */
void ActivityAnimation()
{
  tft.setTextSize(1);
  tft.setCursor(220, 10);

  switch (data.fixQuality)
  {
  // invalid fix quality
  case 0:
    if (data.validDate)
    {
      // arrows building inwards
      tft.setTextColor(TFT_ORANGE, TFT_BLACK, true);
      switch (displayRefreshCounter)
      {
      case 0:
        tft.printf("     <>     ");
        displayRefreshCounter++;
        break;
      case 1:
        tft.printf("~    <>    ~");
        displayRefreshCounter++;
        break;
      case 2:
        tft.printf("~~   <>   ~~");
        displayRefreshCounter++;
        break;
      case 3:
        tft.printf("~~~  <>  ~~~");
        displayRefreshCounter++;
        break;
      case 4:
        tft.printf("~~~> <> <~~~");
        displayRefreshCounter = 0;
        break;

      default:
        displayRefreshCounter = 0;
        break;
      }
    }
    else
    {
      // arrows building outwards
      tft.setTextColor(TFT_RED, TFT_BLACK, true);
      switch (displayRefreshCounter)
      {
      case 0:
        tft.printf("     <>     ");
        displayRefreshCounter++;
        break;
      case 1:
        tft.printf("   - <> -   ");
        displayRefreshCounter++;
        break;
      case 2:
        tft.printf("  -- <> --  ");
        displayRefreshCounter++;
        break;
      case 3:
        tft.printf(" --- <> --- ");
        displayRefreshCounter++;
        break;
      case 4:
        tft.printf("<--- <> --->");
        displayRefreshCounter = 0;
        break;

      default:
        displayRefreshCounter = 0;
        break;
      }
    }
    break;

  // gps fix quality
  case 1:
    // arrows building inwards
    tft.setTextColor(TFT_GREEN, TFT_BLACK, true);
    switch (displayRefreshCounter)
    {
    case 0:
      tft.printf("     <>     ");
      displayRefreshCounter++;
      break;
    case 1:
      tft.printf("-    <>    -");
      displayRefreshCounter++;
      break;
    case 2:
      tft.printf("--   <>   --");
      displayRefreshCounter++;
      break;
    case 3:
      tft.printf("---  <>  ---");
      displayRefreshCounter++;
      break;
    case 4:
      tft.printf("---> <> <---");
      displayRefreshCounter = 0;
      break;

    default:
      displayRefreshCounter = 0;
      break;
    }
    break;

  default:
    break;
  }
}

/*
===============================================================================================
                                    debug functions
================================================================================================
*/

/**
 * @brief i/o debugging
 */
void PrintIODebug()
{
  Serial.printf("\n--- start i/o debug ---\n");

  Serial.printf("\n--- end i/o debug ---\n");
}

/**
 * @brief ic2 debugging
 */
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

  Serial.printf("\n--- end i2c debug ---\n");
}

/**
 * @brief display debugging
 */
void PrintDisplayDebug()
{
  Serial.printf("\n--- start display debug ---\n");

  tft.fillScreen(TFT_BLACK);
  tft.setTextSize(2);
  tft.setCursor(0, 0);
  tft.setTextColor(TFT_GREEN, TFT_BLACK, true);
  tft.printf("%s", debugger.debugText.c_str());

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