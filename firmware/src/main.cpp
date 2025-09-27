/**
 * @file main.cpp
 * @author dom gasperini
 * @brief mini gps
 * @version 5.1
 * @date 2025-09-26
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
#define LOW_BATTERY_THRESHOLD 10
#define NUM_DATA_READS 900 // read available data on the i2c bus with this limit

// general
#define FIRMWARE_MAJOR 5
#define FIRMWARE_MINOR 77
#define FIRMWARE_NAME "stargazer"
#define SHORT_PRESS_DURATION 100 // in milliseconds
#define LONG_PRESS_DURATION 200  // in milliseconds
#define SLEEP_ENABLE_DELAY 1000  // in io task cycles
#define SLEEP_COMBO_TIME 2000    // in milliseconds
#define WAKE_COMBO_TIME 50       // in io task cycles
#define KNOTS_TO_MPH 1.1507795   // mulitplier for converting knots to mph
#define MIN_SPEED 0.5            // minimum number of knots before displaying speed to due resolution limitations
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
#define DEBOUNCE_DURATION 50

// tasks
#define EXECUTIVE_CORE 0
#define DISPLAY_CORE 1
#define IO_REFRESH_RATE 20      // measured in ticks (RTOS ticks interrupt at 1 kHz)
#define GPS_REFRESH_RATE 100    // measured in ticks (RTOS ticks interrupt at 1 kHz)
#define DISPLAY_REFRESH_RATE 50 // measured in ticks (RTOS ticks interrupt at 1 kHz)
#define DEBUG_REFRESH_RATE 1000 // measured in ticks (RTOS ticks interrupt at 1 kHz)

#define TASK_STACK_SIZE 4096 // in bytes

// debugging
#define ENABLE_DEBUGGING false
#define DEBUG_BOOT_DELAY 1500 // in milliseconds

/*
===============================================================================================
                                  global variables
===============================================================================================
*/

SystemDataType g_systemData = {
    .inputFlags = {
        .selectShortPress = false,
        .selectLongPress = false,
        .optionShortPress = false,
        .optionLongPress = false,
        .returnShortPress = false,
        .returnLongPress = false,
        .specialShortPress = false,
        .specialLongPress = false,
    },

    .power = {
        .lowBatteryModeEnable = false,
        .sleepModeEnable = false,
        .batteryPercent = 0.0f,
        .batteryVoltage = 0.0f,
        .batteryChargeRate = 0.0f,
        .alertStatus = false,
    },

    .display = {
        .displayMode = GPS_MODE,
        .previousDisplayMode = ERROR_MODE,
        .displayRefreshCounter = 0,
    },
};

GpsDataType g_gpsData = {
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
};

/**
 * @brief debug data
 */
DebuggerType debugger = {
    .debugEnabled = ENABLE_DEBUGGING,
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
Adafruit_MAX17048 batteryModule;

// gps
Adafruit_GPS gpsModule;

// display
Adafruit_ST7789 displayModule = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);
DisplayModeType previousDisplayMode = ERROR_MODE;

// io
bool selectButtonPreviousState = LOW;
unsigned long selectButtonCounter = 0;
bool selectButtonToggle = false;

bool optionButtonPreviousState = LOW;
unsigned long optionButtonCounter = 0;
bool optionButtonToggle = false;

bool returnButtonPreviousState = LOW;
unsigned long returnButtonCounter = 0;

bool confirmSleepMode = false;

unsigned long specialComboCounter = 0;

int waypointSelector = 0;

// queues
QueueHandle_t xIoQueue;
QueueHandle_t xGpsQueue;

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
float CalculateWaypointDistance(GpsDataType sd, WaypointCoordinatesType wp);
float CalculateWaypointBearing(GpsDataType gps, WaypointCoordinatesType wp);
double DegreesToRadians(float degrees);
String TaskStateToString(eTaskState state);
bool IsEqual(InputFlagsType x1, InputFlagsType x2);

// task abstractions
InputFlagsType ButtonInputHandler();
PowerDataType BatteryManager();

// display
void DisplayGpsData(GpsDataType gps);
void DisplayWaypoint(SystemDataType sd, GpsDataType gps);
void DisplaySystem(SystemDataType sd);
void DisplayStatusBar(SystemDataType sd, GpsDataType gps);
void DisplayError(SystemDataType sd);
void DisplayFlashlight(SystemDataType sd);
void DisplayWaypointInputTool(SystemDataType sd);
void DisplaySleepPrompt(SystemDataType sd);

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
  // // sleep
  // gpio_deep_sleep_hold_en();
  // gpio_hold_en((gpio_num_t)GPS_ENABLE_PIN);

  // gpio_wakeup_enable((gpio_num_t)RETURN_BUTTON, GPIO_INTR_HIGH_LEVEL); // light sleep single button wake up
  // esp_sleep_enable_ext1_wakeup(SLEEP_BUTTON_COMBO_BITMASK, ESP_EXT1_WAKEUP_ANY_HIGH); // deep sleep muli button wakeup
  // esp_sleep_enable_ext0_wakeup((gpio_num_t)RETURN_BUTTON, ESP_EXT1_WAKEUP_ANY_HIGH); // deep sleep single button wake up
  // esp_sleep_enable_gpio_wakeup();
  esp_sleep_enable_ext0_wakeup((gpio_num_t)RETURN_BUTTON, HIGH);

  // io
  // pinMode(SELECT_BUTTON, INPUT);
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
  // if (wpStorage.begin("wp-storage", false)) // init read/write nvs depot
  // {
  //   Serial.printf("nvs init: [ success ]\n");

  //   float tmpLat, tmpLong;
  //   tmpLat = wpStorage.getFloat(WP_1_LAT_NVS_KEY, -99);
  //   tmpLong = wpStorage.getFloat(WP_1_LONG_NVS_KEY, -99);
  //   WaypointCoordinatesType wp1 = {tmpLat, tmpLong};

  //   wpStorage.getFloat(WP_2_LAT_NVS_KEY, -99);
  //   wpStorage.getFloat(WP_2_LONG_NVS_KEY, -99);
  //   WaypointCoordinatesType wp2 = {tmpLat, tmpLong};

  //   wpStorage.getFloat(WP_3_LAT_NVS_KEY, -99);
  //   wpStorage.getFloat(WP_3_LONG_NVS_KEY, -99);
  //   WaypointCoordinatesType wp3 = {tmpLat, tmpLong};

  //   wpStorage.getFloat(WP_4_LAT_NVS_KEY, -99);
  //   wpStorage.getFloat(WP_4_LONG_NVS_KEY, -99);
  //   WaypointCoordinatesType wp4 = {tmpLat, tmpLong};

  //   wpStorage.getFloat(WP_5_LAT_NVS_KEY, -99);
  //   wpStorage.getFloat(WP_5_LONG_NVS_KEY, -99);
  //   WaypointCoordinatesType wp5 = {tmpLat, tmpLong};

  //   // save to dynamic memory
  //   systemManager.gps.waypoints.at(0) = wp1;
  //   systemManager.gps.waypoints.at(1) = wp2;
  //   systemManager.gps.waypoints.at(2) = wp3;
  //   systemManager.gps.waypoints.at(3) = wp4;
  //   systemManager.gps.waypoints.at(4) = wp5;
  // }
  // else
  // {
  //   Serial.printf("nvs init: [ failed ]\n");
  // }
  // ------------------------------------------------------------------------- //

  // -------------------------- initialize battery --------------------------- //
  if (batteryModule.begin())
  {
    Serial.printf("battery init [ success ]\n");
    // batteryModule.quickStart();
    batteryModule.enableSleep(true);

    Serial.printf("\tbattery id: 0x%x\n", batteryModule.getChipID());
    Serial.printf("\tbattery voltage: %.1fv\n", batteryModule.cellVoltage());
    Serial.printf("\tbattery percentage: %.1f%\n", batteryModule.cellPercent());
  }
  else
  {
    Serial.printf("battery init [ failed ]\n");
  }

  // ------------------------------------------------------------------------- //

  // -------------------------- initialize display --------------------------- //
  displayModule.init(135, 240); // ST7789 (240x135)
  displayModule.setRotation(3);
  displayModule.fillScreen(ST77XX_BLACK); // default boot screen fill is white
  digitalWrite(TFT_BACKLITE, HIGH);       // turn on display backlight

  setup.displayActive = true;
  Serial.printf("display init [ success ]\n");
  // -------------------------------------------------------------------------- //

  // -------------------------- initialize gps -------------------------------- //
  if (gpsModule.begin(I2C_GPS_ADDR))
  {
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
  }
  else
  {
    Serial.printf("gps init [ failed ]\n");
  }
  // -------------------------------------------------------------------------- //

  // ----------------------- scheduler & task status -------------------------- //
  // init queue
  xIoQueue = xQueueCreate(1, sizeof(SystemDataType));
  xGpsQueue = xQueueCreate(1, sizeof(GpsDataType));

  // task setup status
  Serial.printf("\ntask setup status:\n");
  Serial.printf("i/o task setup: %s\n", setup.ioActive ? "complete" : "failed");
  Serial.printf("gps task setup: %s\n", setup.gpsActive ? "complete" : "failed");
  Serial.printf("display task setup %s\n", setup.displayActive ? "complete" : "failed");

  // start tasks
  if (xIoQueue != NULL && xGpsQueue != NULL)
  {
    if (setup.ioActive)
    {
      xTaskCreatePinnedToCore(IoTask, "io", TASK_STACK_SIZE, NULL, 1, &xHandleIo, EXECUTIVE_CORE);
    }

    if (setup.gpsActive)
    {
      xTaskCreatePinnedToCore(GpsTask, "gps", TASK_STACK_SIZE, NULL, 1, &xHandleGps, EXECUTIVE_CORE);
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

  for (;;)
  {
    // limit task refresh rate
    vTaskDelayUntil(&taskLastWakeTick, xFrequency);

    // collect system information
    g_systemData.inputFlags = ButtonInputHandler();
    g_systemData.power = BatteryManager();

    // display event handler
    switch (g_systemData.display.displayMode)
    {
    case GPS_MODE:
      if (g_systemData.inputFlags.returnShortPress)
      {
        g_systemData.display.displayMode = WAYPOINT_MODE;
      }
      if (g_systemData.inputFlags.returnLongPress)
      {
        g_systemData.display.displayMode = GPS_MODE;
      }
      break;

    case WAYPOINT_MODE:
      if (g_systemData.inputFlags.returnShortPress)
      {
        g_systemData.display.displayMode = SYSTEM_MODE;
      }
      if (g_systemData.inputFlags.returnLongPress)
      {
        g_systemData.display.displayMode = GPS_MODE;
      }
      break;

    case SYSTEM_MODE:
      if (g_systemData.inputFlags.returnShortPress)
      {
        g_systemData.display.displayMode = GPS_MODE;
        g_systemData.power.sleepModeEnable = false;
      }
      if (g_systemData.inputFlags.returnLongPress)
      {
        g_systemData.display.displayMode = GPS_MODE;
        g_systemData.power.sleepModeEnable = false;
      }

      if (g_systemData.inputFlags.selectShortPress)
      {
        g_systemData.display.displayMode = SLEEP_PROMPT_MODE;
      }
      break;

    case SLEEP_PROMPT_MODE:
      if (g_systemData.inputFlags.returnShortPress)
      {
        g_systemData.display.displayMode = SYSTEM_MODE;
        g_systemData.power.sleepModeEnable = false;
      }
      if (g_systemData.inputFlags.optionShortPress)
      {
        g_systemData.power.sleepModeEnable = true;
      }
      break;

    case FLASHLIGHT_MODE:
      // DisplayFlashlight(sysData);
      if (g_systemData.inputFlags.returnShortPress)
      {
        g_systemData.display.displayMode = GPS_MODE;
        g_systemData.power.sleepModeEnable = false;
      }
      break;

    default:
      g_systemData.display.displayMode = ERROR_MODE;

      if (g_systemData.inputFlags.returnShortPress)
      {
        g_systemData.display.displayMode = GPS_MODE;
        g_systemData.power.sleepModeEnable = false;
      }
      break;
    }

    // --- sleep logic --- //
    if (g_systemData.power.sleepModeEnable)
    {
      // turn off display and gps module power
      digitalWrite(GPS_ENABLE_PIN, LOW);
      digitalWrite(TFT_I2C_POWER, LOW);
      batteryModule.sleep(true);

      esp_deep_sleep_start();
    }
    // --- sleep logic --- //

    // send data to the display
    xQueueOverwrite(xIoQueue, &g_systemData);

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
  uint16_t readCounter = 0;

  for (;;)
  {
    // limit task refresh rate
    vTaskDelayUntil(&taskLastWakeTick, xFrequency);

    // read gps data
    while (gpsModule.read() != 0 && readCounter < NUM_DATA_READS)
    {
      // process gps data
      if (gpsModule.newNMEAreceived() == true)
      {
        gpsModule.parse(gpsModule.lastNMEA()); // this sets the newNMEAreceived() flag to false

        // connection
        g_gpsData.connected = gpsModule.fix;
        g_gpsData.fixQuality = gpsModule.fixquality;

        g_gpsData.numSats = gpsModule.satellites;
        g_gpsData.dtLastFix = gpsModule.secondsSinceFix();
        g_gpsData.dtSinceTime = gpsModule.secondsSinceTime();
        g_gpsData.dtSinceTime = gpsModule.secondsSinceDate();

        //  location
        g_gpsData.latitude = gpsModule.latitudeDegrees;
        g_gpsData.longitude = gpsModule.longitudeDegrees;
        g_gpsData.altitude = gpsModule.altitude;

        // vector
        g_gpsData.speed = gpsModule.speed; // speed is given in knots
        g_gpsData.angle = gpsModule.angle;

        // apply velocity deadband
        if (g_gpsData.speed > MIN_SPEED)
        {
          g_gpsData.speed = g_gpsData.speed * KNOTS_TO_MPH; // convert to mph
        }
        else
        {
          g_gpsData.speed = 0.0f;
          g_gpsData.angle = 0.0f;
        }

        // data
        g_gpsData.year = gpsModule.year + 2000;
        g_gpsData.month = gpsModule.month;
        g_gpsData.day = gpsModule.day;

        // time
        g_gpsData.hour = gpsModule.hour;
        g_gpsData.minute = gpsModule.minute;
        g_gpsData.second = gpsModule.seconds;

        g_gpsData.validDate = (g_gpsData.year >= INIT_OPERATING_YEAR) ? true : false;
      }

      // increment read counter
      readCounter++;
    }

    // sync data
    xQueueOverwrite(xGpsQueue, &g_gpsData);

    // debugging
    if (debugger.debugEnabled)
    {
      debugger.gpsTaskCount++;

      // char c = gpsModule.read();
      // if (c)
      //   Serial.print(c);
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
  SystemDataType dataframe;
  GpsDataType gps;

  for (;;)
  {
    // limit task refresh rate
    vTaskDelayUntil(&taskLastWakeTick, xFrequency);

    // copy data
    xQueuePeek(xIoQueue, &dataframe, 0);
    xQueuePeek(xGpsQueue, &gps, 0);

    if (dataframe.display.displayMode != previousDisplayMode)
    {
      displayModule.fillScreen(ST77XX_BLACK);
    }

    // status bar
    DisplayStatusBar(dataframe, gps);

    // main information
    switch (dataframe.display.displayMode)
    {
    case GPS_MODE:
      DisplayGpsData(gps);
      break;

    case WAYPOINT_MODE:
      DisplayWaypoint(dataframe, gps);
      break;

    case SYSTEM_MODE:
      DisplaySystem(dataframe);
      break;

    case SLEEP_PROMPT_MODE:
      DisplaySleepPrompt(dataframe);
      break;

    case FLASHLIGHT_MODE:
      // DisplayFlashlight(dataframe);
      break;

    default:
      DisplayError(dataframe);
      break;
    }
    previousDisplayMode = dataframe.display.displayMode;

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
float CalculateWaypointDistance(GpsDataType gps, WaypointCoordinatesType waypoint)
{
  // inits
  double distance;
  double currentLatInRad = DegreesToRadians(gps.latitude);
  double currentLongInRad = DegreesToRadians(gps.longitude);
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
float CalculateWaypointBearing(GpsDataType gps, WaypointCoordinatesType waypoint)
{
  // inits
  double bearing;
  double currentLatInRad = DegreesToRadians(gps.latitude);
  double currentLongInRad = DegreesToRadians(gps.longitude);
  double waypointLat = DegreesToRadians(waypoint.waypointLatitude);
  double waypointLong = DegreesToRadians(waypoint.waypointLongitude);

  // calculate!
  float numerator = sin(waypointLong - currentLatInRad) * cos(waypointLat);
  float denominator = cos(currentLatInRad) * sin(waypointLat) - sin(currentLatInRad) * cos(waypointLat) * cos(waypointLong - currentLatInRad);
  bearing = atan(numerator / denominator);

  return (bearing * 180 / PI + 360); // convert back to degrees
}

/**
 * @brief handler for setting flags for button inputs
 */
InputFlagsType ButtonInputHandler()
{
  // inits
  InputFlagsType iF = {
      .selectShortPress = false,
      .selectLongPress = false,
      .optionShortPress = false,
      .optionLongPress = false,
      .returnShortPress = false,
      .returnLongPress = false,
      .specialShortPress = false,
      .specialLongPress = false,
  };

  // --- select button --- //
  bool selectButtonState = digitalRead(SELECT_BUTTON);

  if (selectButtonState != selectButtonPreviousState)
  {
    selectButtonCounter = millis();
    selectButtonPreviousState = selectButtonState;

    // check specifically for LOW -> HIGH
    if (selectButtonPreviousState == HIGH)
    {
      if (millis() - selectButtonCounter > SHORT_PRESS_DURATION)
      {
        iF.selectLongPress = true;
      }
      else
      {
        iF.selectShortPress = true;
      }
    }
  }
  selectButtonPreviousState = selectButtonState;
  // --- select button --- //

  // --- option button --- //
  bool optionButtonState = digitalRead(OPTION_BUTTON);

  if (optionButtonState != optionButtonPreviousState)
  {
    optionButtonCounter = millis();
    optionButtonPreviousState = optionButtonState;

    // check specifically for HIGH -> LOW
    if (optionButtonPreviousState == LOW)
    {
      if (millis() - optionButtonCounter > SHORT_PRESS_DURATION)
      {
        iF.optionLongPress = true;
      }
      else
      {
        iF.optionShortPress = true;
      }
    }
  }
  optionButtonPreviousState = optionButtonState;
  // --- option button --- //

  // --- return button --- //
  bool returnButtonState = digitalRead(RETURN_BUTTON);

  if (returnButtonState != returnButtonPreviousState)
  {
    returnButtonCounter = millis();
    returnButtonPreviousState = returnButtonState;

    // check specifically for HIGH -> LOW
    if (returnButtonPreviousState == LOW)
    {
      if (millis() - returnButtonCounter > SHORT_PRESS_DURATION)
      {
        iF.returnLongPress = true;
      }
      else
      {
        iF.returnShortPress = true;
      }
    }
  }
  returnButtonPreviousState = returnButtonState;
  // --- return button --- //

  return iF;
}

/**
 * @brief collect information related to the battery
 */
PowerDataType BatteryManager()
{
  // init
  PowerDataType pm = {
      .lowBatteryModeEnable = false,
      .sleepModeEnable = false,
      .batteryPercent = 0.0f,
      .batteryVoltage = 0.0f,
      .batteryChargeRate = 0.0f,
      .alertStatus = false,
  };

  // poll battery chip
  pm.batteryPercent = batteryModule.cellPercent();
  pm.batteryVoltage = batteryModule.cellVoltage();
  pm.batteryChargeRate = batteryModule.chargeRate();

  // --- low battery logic --- //
  if (pm.batteryPercent <= LOW_BATTERY_THRESHOLD)
  {
    pm.lowBatteryModeEnable = true;
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
void DisplayGpsData(GpsDataType gps)
{
  // location
  int minSats = 3;
  displayModule.setTextSize(1);
  displayModule.setTextColor(ST77XX_CYAN, ST77XX_BLACK);
  if (gps.numSats >= minSats)
  {
    displayModule.setCursor(0, 20);
    displayModule.printf("latitude: %.5f", gps.latitude);

    displayModule.setCursor(0, 30);
    displayModule.printf("longitude: %.5f", gps.longitude);

    displayModule.setCursor(0, 40);
    displayModule.printf("altitude: %d m        ", (int)gps.altitude);
  }
  else
  {
    displayModule.setCursor(0, 20);
    displayModule.printf("latitude:  ---.---    ");

    displayModule.setCursor(0, 30);
    displayModule.printf("longitude: ---.---    ");

    displayModule.setCursor(0, 40);
    displayModule.printf("altitude:  ---.---    ");
  }

  // speed
  displayModule.setTextColor(ST77XX_YELLOW, ST77XX_BLACK);
  displayModule.setCursor(0, 60);
  if (gps.numSats > minSats)
  {
    displayModule.printf("speed: %.1f mph", gps.speed);
  }
  else
  {
    displayModule.printf("speed: ---      ");
  }

  // angle
  displayModule.setTextColor(ST77XX_YELLOW, ST77XX_BLACK);
  displayModule.setCursor(0, 70);
  if (gps.speed > MIN_SPEED)
  {
    displayModule.printf("heading: %d deg", (int)gps.angle);
  }
  else
  {
    displayModule.printf("heading: ---      ");
  }

  // date and time
  if (gps.validDate)
  {
    displayModule.setTextColor(ST77XX_GREEN, ST77XX_BLACK);
    displayModule.setCursor(0, 80);
    displayModule.printf("date: %d / %d / %d    ", gps.year, gps.month, gps.day);

    displayModule.setCursor(0, 90);
    displayModule.printf("time: %d:%d:%d (UTC)      ", gps.hour, gps.minute, gps.second);
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
  if (gps.numSats == 0)
  {
    displayModule.setTextColor(ST77XX_RED, ST77XX_BLACK);
  }
  if (gps.numSats > 0 && gps.numSats <= minSats)
  {
    displayModule.setTextColor(ST77XX_ORANGE, ST77XX_BLACK);
  }
  if (gps.numSats > minSats)
  {
    displayModule.setTextColor(ST77XX_GREEN, ST77XX_BLACK);
  }
  displayModule.printf("%d ", gps.numSats);

  // time since last fix
  displayModule.setCursor(0, 120);
  displayModule.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
  displayModule.printf("dt-fix: ");
  if (gps.validDate)
  {
    if (gps.dtLastFix < 1.0)
    {
      displayModule.setTextColor(ST77XX_GREEN, ST77XX_BLACK);
      displayModule.printf("%.2f seconds    ", gps.dtLastFix);
    }
    else if (gps.dtLastFix < 120 && gps.dtLastFix > 1.0) // been a bit since a connection
    {
      displayModule.setTextColor(ST77XX_YELLOW, ST77XX_BLACK);
      displayModule.printf("%.2f seconds    ", gps.dtLastFix);
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
void DisplayWaypoint(SystemDataType sd, GpsDataType gps)
{
  displayModule.setTextSize(1);
  displayModule.setTextColor(ST77XX_MAGENTA, ST77XX_BLACK);
  // displayModule.setCursor(75, 100);
  // displayModule.printf("[ waypoints ]");

  // waypoint selector
  // use short option to cycle through preset waypoints
  if (sd.inputFlags.optionShortPress)
  {
    // increment selected waypoint
    waypointSelector++;

    if (waypointSelector > gps.waypoints.size() - 1)
    {
      waypointSelector = 0;
    }
  }
  displayModule.setCursor(0, 20);
  displayModule.setTextColor(ST77XX_MAGENTA, ST77XX_BLACK);
  displayModule.printf("wypt: %d", waypointSelector);

  if (!gps.waypoints.empty())
  {
    // display waypoint coordinates
    displayModule.setCursor(0, 30);
    displayModule.printf("lat: %f.4", gps.waypoints.at(waypointSelector).waypointLatitude);

    displayModule.setCursor(0, 40);
    displayModule.printf("long: %f.4", gps.waypoints.at(waypointSelector).waypointLongitude);

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
      WaypointCoordinatesType waypoint = gps.waypoints.at(waypointSelector);

      // inits
      displayModule.setCursor(0, 80);
      displayModule.printf("distance: %f.2 km", CalculateWaypointDistance(gps, waypoint));

      displayModule.setCursor(0, 100);
      displayModule.printf("bearing: %f.0 deg", CalculateWaypointBearing(gps, waypoint));
    }
    else
    {
      displayModule.setCursor(0, 80);
      displayModule.printf("distance: ~ km     ");

      displayModule.setCursor(0, 100);
      displayModule.printf("bearing: ~ deg     ");
    }
  }
  else
  {
    displayModule.setTextColor(ST77XX_RED, ST77XX_BLACK);
    displayModule.setCursor(40, 100);
    displayModule.printf("[ error loading waypoints ]");
  }
}

/**
 * @brief the battery information screen
 */
void DisplaySystem(SystemDataType sd)
{
  displayModule.setTextSize(2);
  displayModule.setTextColor(ST77XX_WHITE, ST77XX_BLACK);

  // display battery percent charge
  if (sd.power.batteryPercent >= 50)
  {
    displayModule.setTextColor(ST77XX_GREEN, ST77XX_BLACK);
  }
  else if (sd.power.batteryPercent > 20)
  {
    displayModule.setTextColor(ST77XX_YELLOW, ST77XX_BLACK);
  }
  else
  {
    displayModule.setTextColor(ST77XX_RED, ST77XX_BLACK);
  }
  displayModule.setCursor(0, 30);
  displayModule.printf("battery: %.1f%%", sd.power.batteryPercent);

  // display battery voltage
  if (sd.power.batteryVoltage >= 3.9)
  {
    displayModule.setTextColor(ST77XX_GREEN, ST77XX_BLACK);
  }
  else if (sd.power.batteryVoltage > 3.4)
  {
    displayModule.setTextColor(ST77XX_YELLOW, ST77XX_BLACK);
  }
  else
  {
    displayModule.setTextColor(ST77XX_RED, ST77XX_BLACK);
  }
  displayModule.setCursor(0, 50);
  displayModule.printf("voltage: %.2fv", sd.power.batteryVoltage);

  if (sd.power.batteryChargeRate >= 0.0f)
  {
    displayModule.setTextColor(ST77XX_GREEN, ST77XX_BLACK);
  }
  else
  {
    displayModule.setTextColor(ST77XX_RED, ST77XX_BLACK);
  }
  displayModule.setCursor(0, 70);
  displayModule.printf("rate: %.1f %%/h  ", sd.power.batteryChargeRate);

  // uptime
  displayModule.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
  displayModule.setTextSize(1);
  displayModule.setCursor(0, 110);

  // parse out time increments
  unsigned long long total_ms = (esp_rtc_get_time_us() + 500ULL) / 1000ULL; // rounded to nearest ms
  unsigned int hours = total_ms / 3600000ULL;
  total_ms %= 3600000ULL;
  unsigned int minutes = total_ms / 60000ULL;
  total_ms %= 60000ULL;
  unsigned int secs = total_ms / 1000ULL;
  total_ms %= 1000ULL;
  unsigned int centis = total_ms / 10ULL; // hundredths of a second (0–99)
  displayModule.printf("uptime: %02u:%02u:%02u:%02u\n", hours, minutes, secs, centis);

  // firmware verison info
  displayModule.setTextColor(ST77XX_MAGENTA, ST77XX_BLACK);
  displayModule.setTextSize(1);
  displayModule.setCursor(0, 120);
  displayModule.printf("firmware: %d.%d \"%s\"", FIRMWARE_MAJOR, FIRMWARE_MINOR, FIRMWARE_NAME);
}

/**
 * @brief display a status bar at the top of the screen with important information
 */
void DisplayStatusBar(SystemDataType sd, GpsDataType gps)
{
  // inits
  int textY = 5;

  // --- mode --- //
  String mode;
  int underscoreLength;
  displayModule.setCursor(5, textY);
  displayModule.setTextSize(1);
  switch (sd.display.displayMode)
  {
  case GPS_MODE:
    mode = "gps";
    displayModule.setTextColor(ST77XX_RED, ST77XX_BLACK);
    break;

  case WAYPOINT_MODE:
    mode = "waypoints";
    displayModule.setTextColor(ST77XX_MAGENTA, ST77XX_BLACK);
    break;

  case SYSTEM_MODE:
    mode = "system";
    displayModule.setTextColor(ST77XX_CYAN, ST77XX_BLACK);
    break;

  case SLEEP_PROMPT_MODE:
    mode = "hibernate";
    displayModule.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
    break;

  default:
    mode = "error";
    displayModule.setTextColor(ST77XX_ORANGE, ST77XX_BLACK);
    break;
  }
  displayModule.printf("< %s >", mode.c_str());
  // --- mode --- //

  // --- gps fix status --- //
  if (gps.fixQuality == 1) // fix
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
  else if (gps.validDate) // no fix but valid rtc data
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
  displayModule.setCursor(90, textY);
  displayModule.printf("gps");
  // --- gps fix status --- //

  // --- time --- //
  if (gps.validDate)
  {
    displayModule.setCursor(140, textY);
    displayModule.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
    displayModule.printf("%d:%d:%d  ", gps.hour, gps.minute, gps.second);
  }
  else
  {
    displayModule.setCursor(140, textY);
    displayModule.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
    displayModule.printf("--:--:--  ");
  }
  // --- time --- //

  // --- battery --- //
  // write battery percentage
  displayModule.setTextSize(1);
  displayModule.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
  String buffer = String((int)sd.power.batteryPercent).substring(0, 3);
  uint16_t strLen = buffer.length();
  uint16_t width = (strLen * 9) + 5;
  uint16_t batteryXBase = 240 - (width + 5);
  displayModule.setCursor(batteryXBase + 6, textY + 2);
  displayModule.printf("%s", buffer.c_str());

  // draw battery symbol
  displayModule.setCursor(batteryXBase, textY);
  if (sd.power.batteryPercent > 50.0)
  {
    displayModule.drawRoundRect(batteryXBase, textY, width, 11, 4, ST77XX_GREEN);
  }
  else if (sd.power.batteryPercent > 20.0 && sd.power.batteryPercent <= 50.0)
  {
    displayModule.drawRoundRect(batteryXBase, textY, width, 11, 4, ST77XX_ORANGE);
  }
  else if (sd.power.batteryPercent <= 20.0)
  {
    displayModule.drawRoundRect(batteryXBase, textY, width, 11, 4, ST77XX_RED);
  }
  else
  {
    displayModule.printf("error!");
  }
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
void DisplaySleepPrompt(SystemDataType sd)
{
  displayModule.setTextSize(2);
  displayModule.setCursor(25, 30);
  displayModule.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
  displayModule.printf("[> hibernate? <]");

  displayModule.setCursor(20, 65);
  displayModule.setTextColor(ST77XX_GREEN, ST77XX_BLACK);
  displayModule.printf("<--- confirm");

  displayModule.setCursor(20, 120);
  displayModule.setTextColor(ST77XX_RED, ST77XX_BLACK);
  displayModule.printf("<--- cancel");
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

  // Serial.printf("select short cntr: %d\n", selectButtonCounter);
  // Serial.printf("option short cntr: %d\n", optionButtonCounter);
  // Serial.printf("return short cntr: %d\n", returnButtonCounter);

  // Serial.printf("select short: %s\n", debugger.inputFlags.selectShortPress ? "on" : "off");
  // Serial.printf("select long: %s\n", systemManager.inputFlags.selectLongPress ? "on" : "off");

  // Serial.printf("option short: %s\n", systemManager.inputFlags.optionShortPress ? "on" : "off");
  // Serial.printf("option long: %s\n", systemManager.inputFlags.optionLongPress ? "on" : "off");

  // Serial.printf("return short: %s\n", systemManager.inputFlags.returnShortPress ? "on" : "off");
  // Serial.printf("return long: %s\n", systemManager.inputFlags.returnLongPress ? "on" : "off");

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

  Serial.printf("\n--- end display debug ---\n");
}

/**
 * @brief scheduler debugging
 */
void PrintSchedulerDebug()
{
  // inits
  std::vector<int> taskRefreshRate;
  int uptime = esp_rtc_get_time_us() / 1000000;

  taskRefreshRate.push_back(debugger.ioTaskCount - debugger.ioTaskPreviousCount);
  taskRefreshRate.push_back(debugger.gpsTaskCount - debugger.gpsTaskPreviousCount);
  taskRefreshRate.push_back(debugger.displayTaskCount - debugger.displayTaskPreviousCount);

  // print
  Serial.printf("uptime: %d | read io: <%d Hz> (%d) | gps: <%d Hz> (%d) | display: <%d Hz> (%d) \n",
                uptime, taskRefreshRate.at(0), debugger.ioTaskCount, taskRefreshRate.at(1), debugger.gpsTaskCount,
                taskRefreshRate.at(2), debugger.displayTaskCount);

  // update counters
  debugger.ioTaskPreviousCount = debugger.ioTaskCount;
  debugger.gpsTaskPreviousCount = debugger.gpsTaskCount;
  debugger.displayTaskPreviousCount = debugger.displayTaskCount;
}