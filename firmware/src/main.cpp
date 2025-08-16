/**
 * @file main.cpp
 * @author dom gasperini
 * @brief mini gps
 * @version 4.1
 * @date 2025-08-16
 *
 * @ref https://espregpsSerialif-docs.readthedocs-hosted.com/projects/arduino-esp32/en/latest/libraries.html#apis      (api and hal docs)
 * @ref https://docs.arduino.cc/resources/pinouts/ABX00083-full-pinout.pdf                                             (pinout & overview)
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
#include "driver/rtc_io.h"
#include <SPI.h>

// functionality
#include "Wire.h"
#include <Adafruit_GPS.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>

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

// general
#define KNOTS_TO_MPH 1.1507795 // mulitplier for converting knots to mph
#define MIN_SPEED 2.00         // minimum number of knots before displaying speed to due resolution limitations
#define FINAL_VALID_OPERATING_YEAR 2080
#define START_VALID_OPERATING_YEAR 2024

// tasks
#define GPS_CORE 0
#define DISPLAY_CORE 1
#define IO_REFRESH_RATE 100      // measured in ticks (RTOS ticks interrupt at 1 kHz)
#define I2C_REFRESH_RATE 10      // measured in ticks (RTOS ticks interrupt at 1 kHz)
#define DISPLAY_REFRESH_RATE 100 // measured in ticks (RTOS ticks interrupt at 1 kHz)
#define DEBUG_REFRESH_RATE 1000  // measured in ticks (RTOS ticks interrupt at 1 kHz)

#define TASK_STACK_SIZE 4096 // in bytes

// debugging
#define ENABLE_DEBUGGING true

/*
===============================================================================================
                                  global variables
===============================================================================================
*/

GpsDataType data = {
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
    .ioTaskCount = 0,
    .i2cTaskCount = 0,
    .displayTaskCount = 0,

    .displayRefreshRate = 0,

    .ioTaskPreviousCount = 0,
    .i2cTaskPreviousCount = 0,
    .displayTaskPreviousCount = 0,
};

// gps
TwoWire I2CGPS = TwoWire(0);
Adafruit_GPS gps(&I2CGPS);

// display
static const int spiClk = 240000000; // 1 MHz
SPIClass *hspi = new SPIClass(HSPI);
Adafruit_ILI9341 tft(hspi, SPI_DC_PIN, SPI_RESET_PIN);
int displayRefreshCounter = 0;

// io
int powerButtonCounter = 0;
bool enableDisplayPower = true;
bool enableGpsPower = true;
bool sleepModeEnable = false;
bool lowBatteryModeEnable = false;

// semaphore
SemaphoreHandle_t xSemaphore = NULL;
SemaphoreHandle_t xMutex = NULL;

// hardware timer
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

// rtos task handles
TaskHandle_t xHandleIO = NULL;
TaskHandle_t xHandleI2C = NULL;
TaskHandle_t xHandleDisplay = NULL;
TaskHandle_t xHandleDebug = NULL;

/*
===============================================================================================
                                function declarations
===============================================================================================
*/

// tasks
void IOTask(void *pvParameters);
void I2CTask(void *pvParameters);
void DisplayTask(void *pvParameters);
void DebugTask(void *pvParameters);

// helpers
String TaskStateToString(eTaskState state);
bool IsValidDate();
void ActivityAnimation(GpsDataType gpsData);
void DisplayGpsData(GpsDataType gpsData);
void DisplayLowPowerMode();

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
    bool gpsActive = false;
  };
  setup setup;

  // --------------------------- initialize serial  --------------------------- //
  Serial.begin(9600);
  Serial.printf("\n\n|--- starting setup ---|\n\n");
  // -------------------------------------------------------------------------- //

  // -------------------------- initialize GPIO ------------------------------ //
  // inputs
  // pinMode(POWER_BUTTON_PIN, INPUT);
  // esp_sleep_enable_ext0_wakeup((gpio_num_t)POWER_BUTTON_PIN, LOW);
  // rtc_gpio_pullup_dis((gpio_num_t)POWER_BUTTON_PIN);
  // rtc_gpio_pulldown_en((gpio_num_t)POWER_BUTTON_PIN);

  // outputs
  pinMode(I2C_POWER_TOGGLE, OUTPUT);
  pinMode(DISPLAY_POWER_TOGGLE, OUTPUT);

  Serial.printf("gpio init [ success ]\n");
  setup.ioActive = true;
  // -------------------------------------------------------------------------- //

  // -------------------------- initialize display --------------------------- //
  hspi->begin(SPI_SCLK_PIN, SPI_MISO_PIN, SPI_MOSI_PIN, SPI_CS_PIN);

  pinMode(SPI_CS_PIN, OUTPUT);

  tft.begin();
  tft.setRotation(3);

  // display gps information
  tft.fillScreen(ILI9341_BLACK);
  tft.setTextSize(3);
  tft.setTextColor(ILI9341_RED);
  tft.setCursor(0, 0);
  tft.printf("mini gps:");

  // outline boxes
  tft.drawRect(0, 25, 320, 155, ILI9341_DARKCYAN);
  tft.drawRect(0, 185, 320, 55, ILI9341_MAGENTA);

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
  gps.sendCommand(PMTK_SET_NMEA_UPDATE_5HZ);

  // set gps position fix rate
  gps.sendCommand(PMTK_API_SET_FIX_CTL_5HZ);

  // request updates on antenna status (don't need this on)
  // gps.sendCommand(PGCMD_ANTENNA);

  Serial.printf("gps init [ success ]\n");
  setup.gpsActive = true;
  // -------------------------------------------------------------------------- //

  Serial.printf("\n|--- end setup ---|\n");
  // --------------------------------------------------------------------------- //

  // ------------------------------- scheduler & task status --------------------------------- //
  // init mutex and semaphore
  xMutex = xSemaphoreCreateMutex();
  xSemaphore = xSemaphoreCreateCounting(100, 0);

  // task setup status
  Serial.printf("\ntask setup status:\n");
  Serial.printf("i/o task setup: %s\n", setup.ioActive ? "complete" : "failed");
  Serial.printf("i2c task setup: %s\n", setup.gpsActive ? "complete" : "failed");
  Serial.printf("display task setup %s\n", setup.displayActive ? "complete" : "failed");

  // start tasks
  if (xSemaphore != NULL && xMutex != NULL)
  {
    if (setup.ioActive)
    {
      // xTaskCreatePinnedToCore(IOTask, "io", TASK_STACK_SIZE, NULL, 1, &xHandleIO, DISPLAY_CORE);
    }

    if (setup.gpsActive)
    {
      xTaskCreatePinnedToCore(I2CTask, "i2c", TASK_STACK_SIZE, NULL, 1, &xHandleI2C, GPS_CORE);
    }

    if (setup.displayActive)
    {
      xTaskCreatePinnedToCore(DisplayTask, "display", TASK_STACK_SIZE, NULL, 1, &xHandleDisplay, DISPLAY_CORE);
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
  if (xHandleIO != NULL)
    Serial.printf("i/o task status: %s\n", TaskStateToString(eTaskGetState(xHandleIO)));
  else
    Serial.printf("i/o task status: disabled!\n");

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
  Serial.printf("\n|--- end setup ---|\n\n");
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
void IOTask(void *pvParameters)
{
  // get task tick
  const TickType_t xFrequency = pdMS_TO_TICKS(IO_REFRESH_RATE);
  TickType_t taskLastWakeTick = xTaskGetTickCount();

  for (;;)
  {
    // limit task refresh rate
    vTaskDelayUntil(&taskLastWakeTick, xFrequency);

    // debugging
    if (debugger.debugEnabled)
    {
      debugger.ioTaskCount++;
    }
  }
}

/**
 * @brief i2c bus
 * @param pvParameters parameters passed to task
 */
void I2CTask(void *pvParameters)
{
  // get task tick
  TickType_t taskLastWakeTick = xTaskGetTickCount();

  for (;;)
  {
    // limit task refresh rate
    vTaskDelayUntil(&taskLastWakeTick, I2C_REFRESH_RATE);

    if (xSemaphoreGive(xSemaphore))
    {
      // read gps data
      while (gps.read() != 0)
      {
        // process gps data
        if (gps.newNMEAreceived() == true)
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
            data.speed = 0.0f;
            data.angle = 0.0f;
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
      }

      // debugging
      if (debugger.debugEnabled)
      {
        debugger.i2cTaskCount++;
      }
    }
  }
}

/**
 * @brief display manager
 * @param pvParameters parameters passed to task
 */
void DisplayTask(void *pvParameters)
{
  // get task tick
  TickType_t taskLastWakeTick = xTaskGetTickCount();
  GpsDataType gpsDataCopy;

  for (;;)
  {
    // limit task refresh rate
    vTaskDelayUntil(&taskLastWakeTick, DISPLAY_REFRESH_RATE);

    if (xSemaphoreTake(xSemaphore, (TickType_t)0) == pdTRUE)
    {
      int semaphoreCount = uxSemaphoreGetCount(xSemaphore);
      while (semaphoreCount > 0)
      {
        xSemaphoreTake(xSemaphore, (TickType_t)0);
        semaphoreCount--;
      }

      // copy gps data
      gpsDataCopy = data;

      // --- low battery logic --- //
      if (digitalRead(LOW_POWER_MODE_PIN) == LOW)
      {
        // lowBatteryModeEnable = true;
      }
      // --- low battery logic --- //

      // --- deep sleep logic --- //
      // powerButtonCounter = (digitalRead(POWER_BUTTON_PIN) == LOW) ? powerButtonCounter++ : 0;
      // sleepModeEnable = (powerButtonCounter >= DISPLAY_REFRESH_RATE * 10) ? true : false;

      // enableGpsPower = !sleepModeEnable;
      // enableDisplayPower = !sleepModeEnable;
      // // --- deep sleep logic --- //

      // // --- update power distribution --- //
      // digitalWrite(I2C_POWER_TOGGLE, (enableGpsPower && !lowBatteryModeEnable));
      // digitalWrite(DISPLAY_POWER_TOGGLE, enableDisplayPower);

      // if (sleepModeEnable)
      // {
      //   esp_deep_sleep_start();
      // }
      // --- update power distribution --- //
    }

    // update display
    // if (!sleepModeEnable)
    // {
    DisplayGpsData(gpsDataCopy);
    ActivityAnimation(gpsDataCopy);
    // }
    // else
    // {
    //   DisplayLowPowerMode();
    // }

    // debugging
    if (debugger.debugEnabled)
    {
      debugger.displayTaskCount++;
    }
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
    stateStr = "running";
    break;

  case eBlocked:
    stateStr = "blocked";
    break;

  case eSuspended:
    stateStr = "suspended";
    break;

  case eDeleted:
    stateStr = "deleted";
    break;

  default:
    stateStr = "error";
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
void ActivityAnimation(GpsDataType gpsDataCopy)
{
  tft.setTextSize(1);
  tft.setCursor(220, 10);

  switch (gpsDataCopy.fixQuality)
  {
  // invalid fix quality
  case 0:
    if (gpsDataCopy.validDate)
    {
      // arrows building inwards
      tft.setTextColor(ILI9341_ORANGE, ILI9341_BLACK);
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
      tft.setTextColor(ILI9341_RED, ILI9341_BLACK);
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
    tft.setTextColor(ILI9341_GREEN, ILI9341_BLACK);
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

/**
 *
 */
void DisplayGpsData(GpsDataType dataCopy)
{
  // location
  int minSats = 3;
  tft.setTextSize(2);
  tft.setTextColor(ILI9341_CYAN, ILI9341_BLACK);
  if (dataCopy.numSats >= minSats)
  {
    tft.setCursor(5, 30);
    tft.printf("latitude: %.5f", dataCopy.latitude);

    tft.setCursor(5, 50);
    tft.printf("longitude: %.5f", dataCopy.longitude);

    tft.setCursor(5, 70);
    tft.printf("altitude: %d m        ", (int)dataCopy.altitude);
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

  // speed
  tft.setTextColor(ILI9341_YELLOW, ILI9341_BLACK);
  tft.setCursor(5, 95);
  if (dataCopy.numSats > minSats)
  {
    tft.printf("speed: %.1f mph", dataCopy.speed);
  }
  else
  {
    tft.printf("speed: ---      ");
  }

  // angle
  tft.setTextColor(ILI9341_YELLOW, ILI9341_BLACK);
  tft.setCursor(5, 115);
  if (dataCopy.speed > MIN_SPEED)
  {
    tft.printf("heading: %d deg", (int)dataCopy.angle);
  }
  else
  {
    tft.printf("heading: ---      ");
  }

  // date and time
  if (dataCopy.validDate)
  {
    tft.setTextColor(ILI9341_GREEN, ILI9341_BLACK);
    tft.setCursor(5, 140);
    tft.printf("date: %d / %d / %d    ", dataCopy.year, dataCopy.month, dataCopy.day);

    tft.setCursor(5, 160);
    tft.printf("time: %d:%d:%d (UTC)      ", dataCopy.hour, dataCopy.minute, dataCopy.second);
  }
  else
  {
    tft.setTextColor(ILI9341_RED, ILI9341_BLACK);
    tft.setCursor(5, 140);
    tft.printf("date: -- / -- / --    ");

    tft.setTextColor(ILI9341_MAGENTA, ILI9341_BLACK);
    tft.setCursor(5, 160);
    tft.printf("time: --:--:--            ");
  }

  // sats
  tft.setCursor(5, 190);
  tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
  tft.printf("sats: ");
  if (dataCopy.numSats == 0)
  {
    tft.setTextColor(ILI9341_RED, ILI9341_BLACK);
  }
  if (dataCopy.numSats > 0 && dataCopy.numSats <= minSats)
  {
    tft.setTextColor(ILI9341_ORANGE, ILI9341_BLACK);
  }
  if (dataCopy.numSats > minSats)
  {
    tft.setTextColor(ILI9341_GREEN, ILI9341_BLACK);
  }
  tft.printf("%d ", dataCopy.numSats);

  // connection type
  tft.setCursor(130, 190);
  tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
  tft.printf("status: ");
  switch (dataCopy.fixQuality)
  {
  // invalid fix quality
  case 0:
    if (dataCopy.validDate)
    {
      tft.setTextColor(ILI9341_ORANGE, ILI9341_BLACK);
      tft.printf("no fix ");
    }
    else // no fix and no date time
    {
      tft.setTextColor(ILI9341_RED, ILI9341_BLACK);
      tft.printf("no conn");
    }
    break;

  // gps fix quality
  case 1:
    tft.setTextColor(ILI9341_GREEN, ILI9341_BLACK);
    tft.printf("gps    ");
    break;

  // d-gps fixed quality
  case 2:
    tft.setTextColor(ILI9341_BLUE, ILI9341_BLACK);
    tft.printf("d-gps  ");
    break;

  // catch statement
  default:
    tft.setTextColor(ILI9341_RED, ILI9341_BLACK);
    tft.printf("err:%d ", dataCopy.fixQuality);
    break;
  }

  // time since last fix
  tft.setCursor(5, 220);
  tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
  tft.printf("dt-fix: ");
  if (dataCopy.validDate)
  {
    if (dataCopy.dtLastFix < 1.0)
    {
      tft.setTextColor(ILI9341_GREEN, ILI9341_BLACK);
      tft.printf("%.2f seconds    ", dataCopy.dtLastFix);
    }
    else if (dataCopy.dtLastFix < 120 && dataCopy.dtLastFix > 1.0) // been a bit since a connection
    {
      tft.setTextColor(ILI9341_YELLOW, ILI9341_BLACK);
      tft.printf("%.2f seconds    ", dataCopy.dtLastFix);
    }
    else // longer than a minute without a fix
    {
      tft.setTextColor(ILI9341_ORANGE, ILI9341_BLACK);
      tft.printf("> 2 minutes     ");
    }
  }
  else
  {
    tft.setTextColor(ILI9341_RED, ILI9341_BLACK);
    tft.printf("unreliable data    ");
  }
}

/**
 *
 */
void DisplayLowPowerMode()
{
  tft.setTextColor(ILI9341_RED);
  tft.setCursor(100, 100);
  tft.printf("battery low!");
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

  tft.fillScreen(ILI9341_BLACK);
  tft.setTextSize(2);
  tft.setCursor(0, 0);
  tft.setTextColor(ILI9341_GREEN, ILI9341_BLACK);
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
  if (xHandleIO != NULL)
  {
    taskStates.push_back(eTaskGetState(xHandleIO));
  }
  if (xHandleI2C != NULL)
  {
    taskStates.push_back(eTaskGetState(xHandleI2C));
  }
  if (xHandleDisplay != NULL)
  {
    taskStates.push_back(eTaskGetState(xHandleDisplay));
  }

  taskRefreshRate.push_back(debugger.ioTaskCount - debugger.ioTaskPreviousCount);
  taskRefreshRate.push_back(debugger.i2cTaskCount - debugger.i2cTaskPreviousCount);
  taskRefreshRate.push_back(debugger.displayTaskCount - debugger.displayTaskPreviousCount);

  // make it usable
  for (int i = 0; i < taskStates.size() - 1; ++i)
  {
    taskStatesStrings.push_back(TaskStateToString(taskStates.at(i)));
  }

  // print
  Serial.printf("uptime: %d | read io: <%d Hz> (%d) | i2c: <%d Hz> (%d) | display: <%d Hz> (%d) \n",
                uptime, taskRefreshRate.at(0), debugger.ioTaskCount, taskRefreshRate.at(1), debugger.i2cTaskCount,
                taskRefreshRate.at(2), debugger.displayTaskCount);

  // update counters
  debugger.ioTaskPreviousCount = debugger.ioTaskCount;
  debugger.i2cTaskPreviousCount = debugger.i2cTaskCount;
  debugger.displayTaskPreviousCount = debugger.displayTaskCount;
}