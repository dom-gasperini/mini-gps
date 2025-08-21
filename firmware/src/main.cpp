/**
 * @file main.cpp
 * @author dom gasperini
 * @brief mini gps
 * @version 5.0
 * @date 2025-08-20
 *
 * @ref https://learn.adafruit.com/esp32-s3-reverse-tft-feather/overview      (Adafruit ESP32-S3 Reverse TFT Feather Docs)
 * @ref https://learn.adafruit.com/adafruit-ultimate-gps/overview             (Adafruit Ulitmate GPS Module Docs)
 * @ref https://github.com/adafruit/Adafruit_GPS                              (gps library)
 * @ref https://github.com/adafruit/Adafruit-GFX-Library                      (graphics library)
 */

/*
===============================================================================================
                                    includes
===============================================================================================
*/

// core
#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Preferences.h>
#include <rtc.h>
#include <vector>

// hardware specific
#include <Adafruit_GPS.h>      // gps parsing library
#include <Adafruit_GFX.h>      // graphics library
#include <Adafruit_ST7789.h>   // display driver library
#include <Adafruit_MAX1704X.h> // battery managment chip library

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
#define BATT_MGMT_ADDR 0x36
#define LOW_BATTERY_THRESHOLD 5

// general
#define FIRMWARE_MAJOR 5
#define FIRMWARE_MINOR 0
#define SHORT_PRESS_DURATION_FLOOR 5 // in rtos ticks
#define LONG_PRESS_DURATION_FLOOR 50 // in rtos ticks
#define SLEEP_ENABLE_DELAY 1500      // in rtos ticks
#define SLEEP_COMBO_TIME 100         // in rtos ticks
#define WAKE_COMBO_TIME 5            // in rtos ticks
#define KNOTS_TO_MPH 1.1507795       // mulitplier for converting knots to mph
#define MIN_SPEED 0.5                // minimum number of knots before displaying speed to due resolution limitations
#define INIT_OPERATING_YEAR 2024
#define SLEEP_BUTTON_COMBO_BITMASK 0x000000003 // hex value representing the pins 0-39 as bits (bits 1 and 2 are selected)
#define RADIUS_OF_EARTH 6378.137               // in km
#define WP_1_LAT_NVS_KEY "wp1-lat"
#define WP_1_LONG_NVS_KEY "wp1-long"
#define WP_2_LAT_NVS_KEY "wp2-lat"
#define WP_2_LONG_NVS_KEY "wp2-long"
#define WP_3_LAT_NVS_KEY "wp3-lat"
#define WP_3_LONG_NVS_KEY "wp3-long"
#define WP_4_LAT_NVS_KEY "wp4-lat"
#define WP_4_LONG_NVS_KEY "wp4-long"
#define WP_5_LAT_NVS_KEY "wp5-lat"
#define WP_5_LONG_NVS_KEY "wp5-long"

// tasks
#define EXEC_CORE 0
#define DISPLAY_CORE 1
#define IO_REFRESH_RATE 100      // measured in ticks (RTOS ticks interrupt at 1 kHz)
#define GPS_REFRESH_RATE 10      // measured in ticks (RTOS ticks interrupt at 1 kHz)
#define DISPLAY_REFRESH_RATE 100 // measured in ticks (RTOS ticks interrupt at 1 kHz)
#define DEBUG_REFRESH_RATE 1000  // measured in ticks (RTOS ticks interrupt at 1 kHz)

#define TASK_STACK_SIZE 4096 // in bytes

// debugging
#define ENABLE_DEBUGGING false
#define DEBUG_BOOT_DELAY 3000 // in milliseconds

/*
===============================================================================================
                                  global variables
===============================================================================================
*/

/**
 * @brief
 */
SystemDataType systemManager = {
    .inputFlags =
        {
            .selectShortPress = false,
            .selectLongPress = false,

            .optionShortPress = false,
            .optionLongPress = false,

            .returnShortPress = false,
            .returnLongPress = false,

            .specialShortPress = false,
            .specialLongPress = false,
        },

    .power =
        {
            .lowBatteryModeEnable = false,
            .sleepModeEnable = false,

            .batteryPercent = 0.0f,
            .batteryVoltage = 0.0f,
            .batteryChargeRate = 0.0f,
            .alertStatus = 0,
        },

    .display =
        {
            .displayMode = GPS_MODE,
            .previousDisplayMode = ERROR_MODE,
            .displayRefreshCounter = 0,
        },

    .gps =
        {
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

            .waypoints = {},

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
        }};

DebuggerType debugger = {
    .debugEnabled = ENABLE_DEBUGGING,
    .IO_debugEnabled = false,
    .gps_debugEnabled = false,
    .display_debugEnabled = false,
    .scheduler_debugEnable = true,

    .debugText = "",

    // scheduler data
    .ioTaskCount = 0,
    .gpsTaskCount = 0,
    .displayTaskCount = 0,

    .displayRefreshRate = 0,

    .ioTaskPreviousCount = 0,
    .gpsTaskPreviousCount = 0,
    .displayTaskPreviousCount = 0,
};

// Non-voltile storage
Preferences wpStorage;

// battery management
Adafruit_MAX17048 batteryModule; // hardware dedicated

// gps
Adafruit_GPS gpsModule; // hardware dedicated

// display
Adafruit_ST7789 displayModule = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST); // hardware dedicated

// io
int selectButtonCounter = 0;
bool selectButtonToggle = false;

int optionButtonCounter = 0;
bool optionButtonToggle = false;

int returnButtonCounter = 0;
int specialComboCounter = 0;
int sleepComboCounter = 0;
int waypointSelector = 0;

// semaphore
SemaphoreHandle_t xSemaphore = NULL;
SemaphoreHandle_t xMutex = NULL;

// hardware timer
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

// rtos task handles
TaskHandle_t xHandleIo = NULL;
TaskHandle_t xHandleGps = NULL;
TaskHandle_t xHandleDisplay = NULL;
TaskHandle_t xHandleDebug = NULL;

/*
===============================================================================================
                                function declarations
===============================================================================================
*/

// tasks
void IoTask(void *pvParameters);
void GpsTask(void *pvParameters);
void DisplayTask(void *pvParameters);
void DebugTask(void *pvParameters);

// helpers
float CalculateWaypointDistance(SystemDataType sd, WaypointCoordinatesType wp);
float CalculateWaypointBearing(SystemDataType sd, WaypointCoordinatesType wp);
double DegreesToRadians(float degrees);
String TaskStateToString(eTaskState state);

// task abstractions
InputFlagsType ButtonInputHandler(InputFlagsType iF);
PowerDataType BatteryManager(PowerDataType pm);

// display
void DisplayGpsData(SystemDataType sd);
void DisplayWaypoint(SystemDataType sd);
void DisplayBattery(SystemDataType sd);
void DisplayStatusBar(SystemDataType sd);
void DisplayError(SystemDataType sd);
void DisplayFlashlight(SystemDataType sd);
void DisplayWaypointInputTool(SystemDataType sd);
void DisplaySleepModeAlert(SystemDataType sd);

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
    delay(DEBUG_BOOT_DELAY);
  }

  InitDeviceType setup = {
      .ioActive = false,
      .displayActive = false,
      .gpsActive = false,
  };

  // --------------------------- initialize serial  -------------------------- //
  Serial.begin(9600);
  Serial.printf("\n\n|--- starting setup ---|\n\n");
  // ------------------------------------------------------------------------- //

  // -------------------------- initialize GPIO ------------------------------ //
  // sleep
  gpio_deep_sleep_hold_en();
  gpio_hold_en((gpio_num_t)GPS_ENABLE_PIN);

  // gpio_wakeup_enable((gpio_num_t)RETURN_BUTTON, GPIO_INTR_HIGH_LEVEL); // light sleep single button wake up
  // esp_sleep_enable_ext1_wakeup(SLEEP_BUTTON_COMBO_BITMASK, ESP_EXT1_WAKEUP_ANY_HIGH); // deep sleep muli button wakeup
  esp_sleep_enable_ext0_wakeup((gpio_num_t)RETURN_BUTTON, ESP_EXT1_WAKEUP_ANY_HIGH); // deep sleep single button wake up
  esp_sleep_enable_gpio_wakeup();

  // io
  pinMode(SELECT_BUTTON, INPUT);
  pinMode(OPTION_BUTTON, INPUT);
  pinMode(RETURN_BUTTON, INPUT);

  // gps
  pinMode(GPS_ENABLE_PIN, OUTPUT);    // the enable pin specific to the gps module hardware
  digitalWrite(GPS_ENABLE_PIN, HIGH); // turn on

  // tft
  pinMode(TFT_BACKLITE, OUTPUT);
  digitalWrite(TFT_BACKLITE, LOW); // set backlight off in case of auto pullhigh in hal | TODO: test & remove as needed

  // tft and i2c power toggle
  pinMode(TFT_I2C_POWER, OUTPUT);    // if there are issues with power, try pulling this pin high manually
  digitalWrite(TFT_I2C_POWER, HIGH); // turn on power to the display and I2C(?)
  delay(10);

  Serial.printf("gpio init [ success ]\n");
  setup.ioActive = true;
  // ------------------------------------------------------------------------- //

  // -------------------------- initialize EEPROM ----------------------------- //
  if (wpStorage.begin("wp-storage", false)) // init read/write nvs depot
  {
    Serial.printf("nvs init: [ success ]\n");

    float tmpLat, tmpLong;
    tmpLat = wpStorage.getFloat(WP_1_LAT_NVS_KEY, -99);
    tmpLong = wpStorage.getFloat(WP_1_LONG_NVS_KEY, -99);
    WaypointCoordinatesType wp1 = {tmpLat, tmpLong};

    wpStorage.getFloat(WP_2_LAT_NVS_KEY, -99);
    wpStorage.getFloat(WP_2_LONG_NVS_KEY, -99);
    WaypointCoordinatesType wp2 = {tmpLat, tmpLong};

    wpStorage.getFloat(WP_3_LAT_NVS_KEY, -99);
    wpStorage.getFloat(WP_3_LONG_NVS_KEY, -99);
    WaypointCoordinatesType wp3 = {tmpLat, tmpLong};

    wpStorage.getFloat(WP_4_LAT_NVS_KEY, -99);
    wpStorage.getFloat(WP_4_LONG_NVS_KEY, -99);
    WaypointCoordinatesType wp4 = {tmpLat, tmpLong};

    wpStorage.getFloat(WP_5_LAT_NVS_KEY, -99);
    wpStorage.getFloat(WP_5_LONG_NVS_KEY, -99);
    WaypointCoordinatesType wp5 = {tmpLat, tmpLong};

    // save to dynamic memory
    systemManager.gps.waypoints.at(0) = wp1;
    systemManager.gps.waypoints.at(1) = wp2;
    systemManager.gps.waypoints.at(2) = wp3;
    systemManager.gps.waypoints.at(3) = wp4;
    systemManager.gps.waypoints.at(4) = wp5;
  }
  else
  {
    Serial.printf("nvs init: [ failed ]\n");
  }
  // ------------------------------------------------------------------------- //

  // -------------------------- initialize battery --------------------------- //
  if (batteryModule.begin())
  {
    Serial.printf("battery init [ success ]\n");
    Serial.printf("battery id: %x\n", batteryModule.getChipID());
    Serial.printf("battery voltage: %fv\n", batteryModule.cellVoltage());
    Serial.printf("battery percentage: %f\n", batteryModule.cellPercent());
  }
  else
  {
    Serial.printf("battery init [ failed ]\n");
  }

  // ------------------------------------------------------------------------- //

  // -------------------------- initialize display --------------------------- //
  displayModule.init(135, 240); // ST7789 (240x135)
  displayModule.setRotation(1);
  displayModule.fillScreen(ST77XX_BLACK); // default boot screen fill is white
  // delay(25);  // if the fill screen is too slow uncomment
  digitalWrite(TFT_BACKLITE, HIGH); // turn on display backlight

  setup.displayActive = true;
  Serial.printf("display init [ success ]\n");
  // -------------------------------------------------------------------------- //

  // -------------------------- initialize gps -------------------------------- //
  gpsModule.begin(I2C_GPS_ADDR);

  // set gps i2c baud rate
  gpsModule.sendCommand(PMTK_SET_BAUD_115200);

  // filter
  gpsModule.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);

  // set gps to mcu update rate
  gpsModule.sendCommand(PMTK_SET_NMEA_UPDATE_10HZ);

  // set gps position fix rate
  // gpsModule.sendCommand(PMTK_API_SET_FIX_CTL_5HZ);

  // request updates on antenna status (don't need this on)
  // gps.sendCommand(PGCMD_ANTENNA);

  Serial.printf("gps init [ success ]\n");
  setup.gpsActive = true;
  // -------------------------------------------------------------------------- //

  // ----------------------- scheduler & task status -------------------------- //
  // init semaphore
  xSemaphore = xSemaphoreCreateCounting(100, 0);

  // task setup status
  Serial.printf("\ntask setup status:\n");
  Serial.printf("i/o task setup: %s\n", setup.ioActive ? "complete" : "failed");
  Serial.printf("gps task setup: %s\n", setup.gpsActive ? "complete" : "failed");
  Serial.printf("display task setup %s\n", setup.displayActive ? "complete" : "failed");

  // start tasks
  if (xSemaphore != NULL)
  {
    if (setup.ioActive)
    {
      xTaskCreatePinnedToCore(IoTask, "io", TASK_STACK_SIZE, NULL, 1, &xHandleIo, EXEC_CORE);
    }

    if (setup.gpsActive)
    {
      xTaskCreatePinnedToCore(GpsTask, "gps", TASK_STACK_SIZE, NULL, 1, &xHandleGps, EXEC_CORE);
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
  if (xHandleIo != NULL)
    Serial.printf("i/o task status: %s\n", TaskStateToString(eTaskGetState(xHandleIo)));
  else
    Serial.printf("i/o task status: disabled!\n");

  if (xHandleGps != NULL)
    Serial.printf("gps task status: %s\n", TaskStateToString(eTaskGetState(xHandleGps)));
  else
    Serial.printf("gps task status: disabled!\n");

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
  // -------------------------------------------------------------------------- //
}

/*
===============================================================================================
                                rtos task functions
===============================================================================================
*/

/**
 * @brief reads and writes I/O
 * @param pvParameters parameters passed to task
 */
void IoTask(void *pvParameters)
{
  // inits
  const TickType_t xFrequency = pdMS_TO_TICKS(IO_REFRESH_RATE);
  TickType_t taskLastWakeTick = xTaskGetTickCount();
  PowerDataType powerManager;
  InputFlagsType inputFlags;
  SystemDataType dataframe;

  for (;;)
  {
    // limit task refresh rate
    vTaskDelayUntil(&taskLastWakeTick, xFrequency);

    // buttons
    dataframe.inputFlags = ButtonInputHandler(inputFlags);

    // battery
    dataframe.power = BatteryManager(powerManager);

    // sleep logic
    if (sleepComboCounter >= SLEEP_COMBO_TIME)
    {
      dataframe.power.sleepModeEnable = true;
    }

    if (dataframe.power.sleepModeEnable)
    {
      vTaskDelay(SLEEP_ENABLE_DELAY); // delay sleep actions

      // turn off display and gps module power
      digitalWrite(GPS_ENABLE_PIN, LOW);
      digitalWrite(TFT_I2C_POWER, LOW);

      esp_deep_sleep_start();
    }

    // sync data
    if (xSemaphoreGive(xSemaphore))
    {
      systemManager.inputFlags = inputFlags;
      systemManager.power = powerManager;
    }

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
void GpsTask(void *pvParameters)
{
  // inits
  const TickType_t xFrequency = pdMS_TO_TICKS(GPS_REFRESH_RATE);
  TickType_t taskLastWakeTick = xTaskGetTickCount();
  GpsDataType gpsData;

  for (;;)
  {
    // limit task refresh rate
    vTaskDelayUntil(&taskLastWakeTick, xFrequency);

    // read gps data
    while (gpsModule.read() != 0)
    {
      // process gps data
      if (gpsModule.newNMEAreceived() == true)
      {
        gpsModule.parse(gpsModule.lastNMEA()); // this sets the newNMEAreceived() flag to false

        // connection
        gpsData.connected = gpsModule.fix;
        gpsData.fixQuality = gpsModule.fixquality;

        gpsData.numSats = gpsModule.satellites;
        gpsData.dtLastFix = gpsModule.secondsSinceFix();
        gpsData.dtSinceTime = gpsModule.secondsSinceTime();
        gpsData.dtSinceTime = gpsModule.secondsSinceDate();

        //  location
        gpsData.latitude = gpsModule.latitudeDegrees;
        gpsData.longitude = gpsModule.longitudeDegrees;
        gpsData.altitude = gpsModule.altitude;

        // vector
        systemManager.gps.speed = gpsModule.speed; // speed is given in knots
        gpsData.angle = gpsModule.angle;

        // apply velocity deadband
        if (gpsData.speed > MIN_SPEED)
        {
          gpsData.speed = gpsData.speed * KNOTS_TO_MPH; // convert to mph
        }
        else
        {
          gpsData.speed = 0.0f;
          gpsData.angle = 0.0f;
        }

        // data
        gpsData.year = gpsModule.year + 2000;
        gpsData.month = gpsModule.month;
        gpsData.day = gpsModule.day;

        // time
        gpsData.hour = gpsModule.hour;
        systemManager.gps.minute = gpsModule.minute;
        gpsData.second = gpsModule.seconds;

        gpsData.validDate = (gpsData.year >= INIT_OPERATING_YEAR) ? true : false;
      }
    }

    // sync data
    if (xSemaphoreGive(xSemaphore))
    {
      systemManager.gps = gpsData;
    }

    // debugging
    if (debugger.debugEnabled)
    {
      debugger.gpsTaskCount++;

      char c = gpsModule.read();
      if (c)
        Serial.print(c);
    }
  }
}

/**
 * @brief display manager
 * @param pvParameters parameters passed to task
 */
void DisplayTask(void *pvParameters)
{
  // inits
  const TickType_t xFrequency = pdMS_TO_TICKS(DISPLAY_REFRESH_RATE);
  TickType_t taskLastWakeTick = xTaskGetTickCount();
  SystemDataType sysData;

  for (;;)
  {
    // limit task refresh rate
    vTaskDelayUntil(&taskLastWakeTick, xFrequency);

    if (xSemaphoreTake(xSemaphore, (TickType_t)0) == pdTRUE)
    {
      int semaphoreCount = uxSemaphoreGetCount(xSemaphore);
      while (semaphoreCount > 0)
      {
        xSemaphoreTake(xSemaphore, (TickType_t)0);
        semaphoreCount--;
      }

      sysData = systemManager;
    }

    // update display
    DisplayStatusBar(sysData);

    if (sysData.display.displayMode != sysData.display.previousDisplayMode)
    {
      displayModule.fillScreen(ST77XX_BLACK);
    }
    switch (sysData.display.displayMode)
    {
    case GPS_MODE:
      DisplayGpsData(sysData);
      break;

    case WAYPOINT_MODE:
      DisplayWaypoint(sysData);
      break;

    case BATTERY_MODE:
      DisplayBattery(sysData);
      break;

    case FLASHLIGHT_MODE:
      DisplayFlashlight(sysData);
      break;

    default:
      DisplayError(sysData);
      break;
    }
    sysData.display.previousDisplayMode = sysData.display.displayMode;

    // sleep
    if (sysData.power.sleepModeEnable)
    {
      DisplaySleepModeAlert(sysData);
    }

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
    if (debugger.gps_debugEnabled)
    {
      PrintGpsDebug();
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
 * @brief converts a value in degrees to radians
 */
double DegreesToRadians(float degrees)
{
  return (degrees * PI / 180.0);
}

/**
 * use the haversine formula to calculate the disance between to gps points on earth
 */
float CalculateWaypointDistance(SystemDataType sd, WaypointCoordinatesType waypoint)
{
  // inits
  double distance;
  double currentLatInRad = DegreesToRadians(sd.gps.latitude);
  double currentLongInRad = DegreesToRadians(sd.gps.longitude);
  double waypointLat = DegreesToRadians(waypoint.waypointLatitude);
  double waypointLong = DegreesToRadians(waypoint.waypointLongitude);

  // calculate!
  float firstTerm = pow(sin((waypointLat - currentLatInRad) / 2), 2);
  float fourtTerm = pow(sin((waypointLong - currentLongInRad) / 2), 2);
  distance = (2 * RADIUS_OF_EARTH) * asin(firstTerm + cos(currentLatInRad) * cos(waypointLat) * fourtTerm);

  return distance;
}

/**
 * use the a funky formula to calculate the bearing from gps point 1 to gps point 2
 */
float CalculateWaypointBearing(SystemDataType sd, WaypointCoordinatesType waypoint)
{
  // inits
  double bearing;
  double currentLatInRad = DegreesToRadians(sd.gps.latitude);
  double currentLongInRad = DegreesToRadians(sd.gps.longitude);
  double waypointLat = DegreesToRadians(waypoint.waypointLatitude);
  double waypointLong = DegreesToRadians(waypoint.waypointLongitude);

  // calculate!
  float numerator = sin(waypointLong - currentLatInRad) * cos(waypointLat);
  float denominator = cos(currentLatInRad) * sin(waypointLat) - sin(currentLatInRad) * cos(waypointLat) * cos(waypointLong - currentLatInRad);
  bearing = atan(numerator / denominator);

  return (bearing * 180 / PI + 360); // convert back to degrees
}

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
 * @brief handler for setting flags for button inputs
 */
InputFlagsType ButtonInputHandler(InputFlagsType iF)
{
  // --- select button --- //
  if (digitalRead(SELECT_BUTTON) == LOW)
  {
    selectButtonCounter++;
  }
  else
  {
    selectButtonCounter = 0;
  }

  if (selectButtonCounter > SHORT_PRESS_DURATION_FLOOR && selectButtonCounter < LONG_PRESS_DURATION_FLOOR)
  {
    iF.selectShortPress = true;
  }

  if (selectButtonCounter > LONG_PRESS_DURATION_FLOOR)
  {
    iF.selectLongPress = true;
  }
  // --- select button --- //

  // --- option button --- //
  if (digitalRead(OPTION_BUTTON) == HIGH)
  {
    optionButtonCounter++;
  }
  else
  {
    optionButtonCounter = 0;
  }

  if (optionButtonCounter > SHORT_PRESS_DURATION_FLOOR && optionButtonCounter < LONG_PRESS_DURATION_FLOOR)
  {
    iF.optionShortPress = true;
  }

  if (optionButtonCounter > LONG_PRESS_DURATION_FLOOR)
  {
    iF.optionLongPress = true;
  }
  // --- option button --- //

  // --- return button --- //
  if (digitalRead(RETURN_BUTTON) == HIGH)
  {
    returnButtonCounter++;
  }
  else
  {
    returnButtonCounter = 0;
  }

  if (returnButtonCounter > SHORT_PRESS_DURATION_FLOOR && returnButtonCounter < LONG_PRESS_DURATION_FLOOR)
  {
    iF.returnShortPress = true;
  }

  if (returnButtonCounter > LONG_PRESS_DURATION_FLOOR)
  {
    iF.returnLongPress = true;
  }
  // --- return button --- //

  // --- special combo --- //
  if (digitalRead(SELECT_BUTTON) == LOW && digitalRead(OPTION_BUTTON) == HIGH)
  {
    specialComboCounter++;
  }

  if (specialComboCounter > SHORT_PRESS_DURATION_FLOOR && specialComboCounter < LONG_PRESS_DURATION_FLOOR)
  {
    iF.specialShortPress = true;
    specialComboCounter = 0;
  }

  if (specialComboCounter > LONG_PRESS_DURATION_FLOOR)
  {
    iF.specialLongPress = true;
    specialComboCounter = 0;
  }
  // --- special combo --- //

  // --- sleep combo --- //
  if (digitalRead(RETURN_BUTTON) == LOW && digitalRead(OPTION_BUTTON) == LOW)
  {
    sleepComboCounter++;
  }
  else
  {
    sleepComboCounter = 0;
  }
  // --- sleep combo --- //

  return iF;
}

/**
 * @brief
 */
PowerDataType BatteryManager(PowerDataType pm)
{
  pm.batteryPercent = batteryModule.cellPercent();
  pm.batteryVoltage = batteryModule.cellVoltage();
  pm.batteryChargeRate = batteryModule.chargeRate();

  // --- low battery logic --- //
  if (pm.batteryPercent <= LOW_BATTERY_THRESHOLD)
  {
    pm.lowBatteryModeEnable = true;
  }
  else
  {
    pm.lowBatteryModeEnable = false;
  }
  // --- low battery logic --- //

  return pm;
}

/*
===============================================================================================
                                    display functions
===============================================================================================
*/

/**
 * @brief general gps data screen
 */
void DisplayGpsData(SystemDataType sd)
{
  // location
  int minSats = 3;
  displayModule.setTextSize(1);
  displayModule.setTextColor(ST77XX_CYAN, ST77XX_BLACK);
  if (sd.gps.numSats >= minSats)
  {
    displayModule.setCursor(0, 10);
    displayModule.printf("latitude: %.5f", sd.gps.latitude);

    displayModule.setCursor(0, 20);
    displayModule.printf("longitude: %.5f", sd.gps.longitude);

    displayModule.setCursor(0, 30);
    displayModule.printf("altitude: %d m        ", (int)sd.gps.altitude);
  }
  else
  {
    displayModule.setCursor(0, 10);
    displayModule.printf("latitude:  ---.---    ");

    displayModule.setCursor(0, 20);
    displayModule.printf("longitude: ---.---    ");

    displayModule.setCursor(0, 30);
    displayModule.printf("altitude:  ---.---    ");
  }

  // speed
  displayModule.setTextColor(ST77XX_YELLOW, ST77XX_BLACK);
  displayModule.setCursor(0, 50);
  if (sd.gps.numSats > minSats)
  {
    displayModule.printf("speed: %.1f mph", sd.gps.speed);
  }
  else
  {
    displayModule.printf("speed: ---      ");
  }

  // angle
  displayModule.setTextColor(ST77XX_YELLOW, ST77XX_BLACK);
  displayModule.setCursor(0, 60);
  if (sd.gps.speed > MIN_SPEED)
  {
    displayModule.printf("heading: %d deg", (int)sd.gps.angle);
  }
  else
  {
    displayModule.printf("heading: ---      ");
  }

  // date and time
  if (sd.gps.validDate)
  {
    displayModule.setTextColor(ST77XX_GREEN, ST77XX_BLACK);
    displayModule.setCursor(0, 80);
    displayModule.printf("date: %d / %d / %d    ", sd.gps.year, sd.gps.month, sd.gps.day);

    displayModule.setCursor(0, 90);
    displayModule.printf("time: %d:%d:%d (UTC)      ", sd.gps.hour, sd.gps.minute, sd.gps.second);
  }
  else
  {
    displayModule.setTextColor(ST77XX_RED, ST77XX_BLACK);
    displayModule.setCursor(0, 80);
    displayModule.printf("date: -- / -- / --    ");

    displayModule.setTextColor(ST77XX_MAGENTA, ST77XX_BLACK);
    displayModule.setCursor(0, 90);
    displayModule.printf("time: --:--:--            ");
  }

  // sats
  displayModule.setCursor(0, 110);
  displayModule.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
  displayModule.printf("sats: ");
  if (sd.gps.numSats == 0)
  {
    displayModule.setTextColor(ST77XX_RED, ST77XX_BLACK);
  }
  if (sd.gps.numSats > 0 && sd.gps.numSats <= minSats)
  {
    displayModule.setTextColor(ST77XX_ORANGE, ST77XX_BLACK);
  }
  if (sd.gps.numSats > minSats)
  {
    displayModule.setTextColor(ST77XX_GREEN, ST77XX_BLACK);
  }
  displayModule.printf("%d ", sd.gps.numSats);

  // time since last fix
  displayModule.setCursor(0, 120);
  displayModule.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
  displayModule.printf("dt-fix: ");
  if (sd.gps.validDate)
  {
    if (sd.gps.dtLastFix < 1.0)
    {
      displayModule.setTextColor(ST77XX_GREEN, ST77XX_BLACK);
      displayModule.printf("%.2f seconds    ", sd.gps.dtLastFix);
    }
    else if (sd.gps.dtLastFix < 120 && sd.gps.dtLastFix > 1.0) // been a bit since a connection
    {
      displayModule.setTextColor(ST77XX_YELLOW, ST77XX_BLACK);
      displayModule.printf("%.2f seconds    ", sd.gps.dtLastFix);
    }
    else // longer than a minute without a fix
    {
      displayModule.setTextColor(ST77XX_ORANGE, ST77XX_BLACK);
      displayModule.printf("> 2 minutes     ");
    }
  }
  else
  {
    displayModule.setTextColor(ST77XX_RED, ST77XX_BLACK);
    displayModule.printf("bad data        ");
  }
}

/**
 * @brief the waypoint screen
 */
void DisplayWaypoint(SystemDataType sd)
{
  displayModule.setTextSize(1);
  displayModule.setTextColor(ST77XX_MAGENTA, ST77XX_BLACK);
  displayModule.setCursor(75, 100);
  displayModule.printf("[ waypoints ]");

  // waypoint selector
  // use short option to cycle through preset waypoints
  if (sd.inputFlags.optionShortPress)
  {
    // reset flag
    sd.inputFlags.optionShortPress = false;

    // increment selected waypoint
    waypointSelector++;

    if (waypointSelector > sd.gps.waypoints.size() - 1)
    {
      waypointSelector = 0;
    }
  }
  displayModule.setCursor(0, 20);
  displayModule.setTextColor(ST77XX_BLACK, ST77XX_MAGENTA);
  displayModule.printf("wypt: %d", waypointSelector);

  // display waypoint coordinates
  displayModule.setCursor(0, 30);
  displayModule.printf("lat: %f.4", sd.gps.waypoints.at(waypointSelector).waypointLatitude);

  displayModule.setCursor(0, 40);
  displayModule.printf("long: %f.4", sd.gps.waypoints.at(waypointSelector).waypointLongitude);

  // use long option to open waypoint input
  if (sd.inputFlags.optionLongPress)
  {
    // reset flag
    sd.inputFlags.optionLongPress = false;

    // show waypoint input tool
    displayModule.fillScreen(ST77XX_BLACK);
    DisplayWaypointInputTool(sd);
  }

  // do and then show calcuations on select long press
  if (sd.inputFlags.selectLongPress)
  {
    // reset flag
    sd.inputFlags.selectLongPress = false;
    WaypointCoordinatesType waypoint = sd.gps.waypoints.at(waypointSelector);

    // inits
    displayModule.setCursor(0, 80);
    displayModule.printf("distance: %f.2 km", CalculateWaypointDistance(sd, waypoint));

    displayModule.setCursor(0, 100);
    displayModule.printf("bearing: %f.0 deg", CalculateWaypointBearing(sd, waypoint));
  }
  else
  {
    displayModule.setCursor(0, 80);
    displayModule.printf("distance: ~ km     ");

    displayModule.setCursor(0, 100);
    displayModule.printf("bearing: ~ deg     ");
  }
}

/**
 * @brief the battery information screen
 */
void DisplayBattery(SystemDataType sd)
{
  displayModule.setTextSize(1);
  displayModule.setTextColor(ST77XX_CYAN, ST77XX_BLACK);

  // display battery stats
  displayModule.setCursor(0, 10);
  displayModule.printf("percentage: %f.1%", sd.power.batteryPercent);

  displayModule.setCursor(0, 20);
  displayModule.printf("percentage: %f.2v", sd.power.batteryVoltage);

  displayModule.setCursor(0, 30);
  displayModule.printf("rate: %f.2 %/h", sd.power.batteryChargeRate);

  // display battery manager chip stats
  // displayModule.setCursor(0, 50);
  // displayModule.printf("chip id: %d", sd.power.chipId);
  displayModule.printf("chip id: TODO");

  displayModule.setCursor(0, 60);
  displayModule.printf("alert status: %x", sd.power.alertStatus);

  // firmware verison info
  displayModule.setCursor(0, 100);
  displayModule.printf("firmware version: v%d.%d", FIRMWARE_MAJOR, FIRMWARE_MINOR);
}

/**
 * @brief display a status bar at the top of the screen with important information
 */
void DisplayStatusBar(SystemDataType sd)
{
  // --- mode --- //
  String mode;
  int modeColor;
  int underscoreLength;
  displayModule.setCursor(5, 5);
  displayModule.setTextSize(1);
  switch (sd.display.displayMode)
  {
  case GPS_MODE:
    mode = "gps";
    displayModule.setTextColor(ST77XX_RED, ST77XX_BLACK);
    modeColor = ST77XX_RED;
    break;

  case WAYPOINT_MODE:
    mode = "waypoint";
    displayModule.setTextColor(ST77XX_MAGENTA, ST77XX_BLACK);
    modeColor = ST77XX_MAGENTA;
    break;

  case BATTERY_MODE:
    mode = "battery";
    displayModule.setTextColor(ST77XX_CYAN, ST77XX_BLACK);
    modeColor = ST77XX_CYAN;
    break;

  default:
    mode = "error";
    displayModule.setTextColor(ST77XX_ORANGE, ST77XX_BLACK);
    modeColor = ST77XX_ORANGE;
    break;
  }
  displayModule.printf("< %s >", mode.c_str());

  // underscore mode
  displayModule.drawLine(5, 10, 5, (mode.length() * 3), modeColor); // mulitply text size 1 num width in pixels
  // --- mode --- //

  // --- gps fix status --- //
  if (sd.gps.fixQuality == 1) // fix
  {
    if (sd.display.displayRefreshCounter == 0)
    {
      displayModule.setTextColor(ST77XX_BLACK, ST77XX_GREEN);
      sd.display.displayRefreshCounter++;
    }
    else
    {
      displayModule.setTextColor(ST77XX_GREEN, ST77XX_WHITE);
      sd.display.displayRefreshCounter = 0;
    }
  }
  else if (sd.gps.validDate) // no fix but valid rtc data
  {
    if (sd.display.displayRefreshCounter == 0)
    {
      displayModule.setTextColor(ST77XX_BLACK, ST77XX_ORANGE);
      sd.display.displayRefreshCounter++;
    }
    else
    {
      displayModule.setTextColor(ST77XX_ORANGE, ST77XX_WHITE);
      sd.display.displayRefreshCounter = 0;
    }
  }
  else // no fix and no rtc data
  {
    if (sd.display.displayRefreshCounter == 0)
    {
      displayModule.setTextColor(ST77XX_BLACK, ST77XX_RED);
      sd.display.displayRefreshCounter++;
    }
    else
    {
      displayModule.setTextColor(ST77XX_RED, ST77XX_WHITE);
      sd.display.displayRefreshCounter = 0;
    }
  }
  displayModule.setTextSize(1);
  displayModule.setCursor(150, 10);
  displayModule.printf("gps");
  // --- gps fix status --- //

  // --- battery --- //
  // draw battery symbol
  displayModule.setCursor(220, 10);
  if (sd.power.batteryPercent >= 50.0)
  {
    displayModule.drawRoundRect(220, 10, 25, 10, 2, ST77XX_GREEN);
  }
  else if (sd.power.batteryPercent >= 20.0 && sd.power.batteryPercent < 50.0)
  {
    displayModule.drawRoundRect(220, 10, 25, 10, 2, ST77XX_ORANGE);
  }
  else if (sd.power.batteryPercent < 20.0)
  {
    displayModule.drawRoundRect(220, 10, 25, 10, 2, ST77XX_RED);
  }
  else
  {
    displayModule.printf("error!");
  }

  // write battery percentage
  displayModule.setTextSize(1);
  displayModule.setCursor(220, 10);
  displayModule.printf("%f.0%", sd.power.batteryPercent);
  // --- battery --- //
}

/**
 * @brief display screen that indicates a failure in display modes
 */
void DisplayError(SystemDataType sd)
{
  // init screen
  displayModule.fillScreen(ST77XX_BLACK);

  displayModule.setTextSize(2);
  displayModule.setTextColor(ST77XX_RED);
  displayModule.setCursor(75, 100);
  displayModule.printf("[> mode error <]");
}

/**
 * @brief idk now im just flexing hardware verticalization
 */
void DisplayFlashlight(SystemDataType sd)
{
  if (sd.inputFlags.selectShortPress)
  {
    // reset flag
    sd.inputFlags.selectShortPress = false;
    selectButtonToggle = !selectButtonToggle;
  }

  if (sd.inputFlags.optionShortPress)
  {
    // reset flag
    sd.inputFlags.optionShortPress = false;
    optionButtonToggle = !optionButtonToggle;
  }

  if (selectButtonToggle)
  {
    if (!optionButtonToggle)
    {
      displayModule.fillScreen(ST77XX_WHITE);
    }
    else
    {
      if (sd.display.displayRefreshCounter == 0)
      {
        displayModule.fillScreen(ST77XX_RED);
        sd.display.displayRefreshCounter++;
      }
      else
      {
        displayModule.fillScreen(ST77XX_BLACK);
        sd.display.displayRefreshCounter = 0;
      }

      displayModule.setTextSize(2);
      displayModule.setTextColor(ST77XX_RED);
      displayModule.setCursor(50, 100);
      displayModule.printf("[> emergency light <]");
    }
  }
  else
  {
    displayModule.fillScreen(ST77XX_BLACK);
  }
}

/**
 * @brief gui for inputting a new waypoint
 */
void DisplayWaypointInputTool(SystemDataType sd)
{
  displayModule.setTextSize(1);
  displayModule.setTextColor(ST77XX_RED, ST77XX_BLACK);
  displayModule.setCursor(75, 100);
  displayModule.printf("[> waypoint input tool <]");
}

/**
 * @brief while device is entering sleep mode popup
 */
void DisplaySleepModeAlert(SystemDataType sd)
{
  displayModule.setTextSize(1);
  displayModule.setCursor(30, 100);
  displayModule.setTextColor(ST77XX_BLACK, ST77XX_GREEN);
  displayModule.printf("[> entering hibernation <]");
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
void PrintGpsDebug()
{
  Serial.printf("\n--- start gps debug ---\n");

  // debugger.debugText = "";
  // for (int i = 0; i < 100; ++i)
  // {
  //   if (Serial.available())
  //   {
  //     char c = Serial.read();
  //     gpsModule.write(c);
  //   }
  //   if (gpsModule.available())
  //   {
  //     char c = gpsModule.read();
  //     Serial.write(c);

  //     debugger.debugText.concat(c);
  //   }
  // }
  // Serial.printf("\n\n");

  // Serial.printf("fix: %s\n", gpsModule.fix ? "yes" : "no");
  // Serial.printf("# sats: %d\n", gps.satellites);

  // Serial.printf("time: %d:%d:%d\n", gps.hour, gps.minute, gps.seconds);
  // Serial.printf("date: %d-%d-%d\n", gps.year, gps.month, gps.day);

  // Serial.printf("lat: %f\n", gps.latitude);
  // Serial.printf("long: %f\n", gps.longitude);
  // Serial.printf("alt: %f\n", gps.altitude);

  // Serial.printf("dt-fix: %f\n", gps.secondsSinceFix());
  // Serial.printf("dt-time: %f\n", gps.secondsSinceTime());
  // Serial.printf("dt-date: %f\n", gps.secondsSinceDate());

  Serial.printf("\n--- end gps debug ---\n");
}

/**
 * @brief display debugging
 */
void PrintDisplayDebug()
{
  Serial.printf("\n--- start display debug ---\n");

  // displayModule.fillScreen(ST77XX_BLACK);
  // displayModule.setTextSize(2);
  // displayModule.setCursor(0, 0);
  // displayModule.setTextColor(ST77XX_GREEN, ST77XX_BLACK);
  // displayModule.printf("%s", debugger.debugText.c_str());

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
  if (xHandleIo != NULL)
  {
    taskStates.push_back(eTaskGetState(xHandleIo));
  }
  if (xHandleGps != NULL)
  {
    taskStates.push_back(eTaskGetState(xHandleGps));
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
  Serial.printf("uptime: %d | read io: <%d Hz> (%d) | gps: <%d Hz> (%d) | display: <%d Hz> (%d) \n",
                uptime, taskRefreshRate.at(0), debugger.ioTaskCount, taskRefreshRate.at(1), debugger.gpsTaskCount,
                taskRefreshRate.at(2), debugger.displayTaskCount);

  // update counters
  debugger.ioTaskPreviousCount = debugger.ioTaskCount;
  debugger.gpsTaskPreviousCount = debugger.gpsTaskCount;
  debugger.displayTaskPreviousCount = debugger.displayTaskCount;
}