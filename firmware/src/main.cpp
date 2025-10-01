/**
 * @file main.cpp
 * @author dom
 * @brief mini gps
 * @version 6
 * @date 2025-09-30
 *
 * @ref https://learn.adafruit.com/esp32-s3-reverse-tft-feather/overview      (Adafruit ESP32-S3 Reverse TFT Feather docs)
 * @ref https://learn.adafruit.com/adafruit-ultimate-gps/overview             (Adafruit Ulitmate GPS Module docs)
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

// general
#define MIN_SATS 3                 // min number of sats to have a fix
#define KNOTS_TO_MPH 1.1507795     // mulitplier for converting knots to mph
#define METERS_TO_FEET 3.28084     // mulitplier for converting meteres to feet
#define MIN_SPEED 1.5              // minimum number of knots before displaying speed to due resolution limitations
#define INIT_OPERATING_YEAR 2025   // first operational year
#define EOL_YEAR 2079              // end of life operating year
#define RADIUS_OF_EARTH 3958.756   // in miles
#define HALF_BATTERY_CAPACITY 50.0 // in %
#define LOW_BATTERY_CAPACITY 20.0  // in %
#define HIGH_BATTERY_VOLTAGE 3.9   // in volts
#define LOW_BATTERY_VOLTAGE 3.4    // in volts
#define DELTA_FIX_HARDSTOP 100000  // seconds
#define LOW_BATTERY_THRESHOLD 10   // in %

// comms
#define BATT_MGMT_I2C_ADDR 0x36
#define GPS_I2C_ADDR 0x10
#define PTMK_BACKUP_POWER "$PMTK225,4*2F" // command the module to enter backup power mode and maintain date/time/d-fix via coin cell (consumes 15uA from battery)

// system
#define FIRMWARE_MAJOR 6
#define FIRMWARE_MINOR 4
#define FIRMWARE_NAME "astronomer"

// time keeping
#define REFRESH_WAYPOINT_VECTOR_DELAY 5000           // in milliseconds
#define BATTERY_POLL_INTERVAL 2000                   // in milliseconds
#define BATTERY_CHARGING_INDICATION_DELAY 500        // in milliseconds
#define DISPLAY_REFRESH_RATE_CALCULATE_INTERVAL 1000 // in milliseconds
#define SHORT_PRESS_DURATION 100                     // in milliseconds
#define LONG_PRESS_DURATION 200                      // in milliseconds

// nvs
#define NVS_FLASH_WITH_DEFAULT false        // write the default values to the nvs
#define NVS_WAS_SLEEPING_KEY "was-sleeping" // to determine if boot was a cold start or rise from slumber
#define NVS_WP_1_LAT_KEY "wp1-lat"
#define NVS_WP_1_LONG_KEY "wp1-long"
#define NVS_WP_1_NAME_KEY "wp1-name"
#define NVS_WP_2_LAT_KEY "wp2-lat"
#define NVS_WP_2_LONG_KEY "wp2-long"
#define NVS_WP_2_NAME_KEY "wp2-name"
#define NVS_WP_3_LAT_KEY "wp3-lat"
#define NVS_WP_3_LONG_KEY "wp3-long"
#define NVS_WP_3_NAME_KEY "wp3-name"
#define NVS_WP_4_LAT_KEY "wp4-lat"
#define NVS_WP_4_LONG_KEY "wp4-long"
#define NVS_WP_4_NAME_KEY "wp4-name"
#define NVS_WP_5_LAT_KEY "wp5-lat"
#define NVS_WP_5_LONG_KEY "wp5-long"
#define NVS_WP_5_NAME_KEY "wp5-name"

// tasks
#define TASK_STACK_SIZE 4096 // in bytes
#define EXECUTIVE_CORE 0
#define DISPLAY_CORE 1
#define IO_REFRESH_RATE 20      // in RTOS ticks (1 tick = interrupt at 1 kHz)
#define GPS_REFRESH_RATE 20     // in RTOS ticks (1 tick = interrupt at 1 kHz)
#define DISPLAY_REFRESH_RATE 50 // in RTOS ticks (1 tick = interrupt at 1 kHz)
#define DEBUG_REFRESH_RATE 1000 // in RTOS ticks (1 tick = interrupt at 1 kHz)

// debugging
#define DEBUG_BOOT_DELAY 1000  // in milliseconds
#define ENABLE_DEBUGGING false // master debug toggle (does not disable boot output)

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
    },

    .waypointData = {
        .waypoints = {},
        .selectedWaypoint = 0,
    },
    .enableFlashlight = false,
};

GpsDataType g_gpsData = {
    .validDate = false,
    .fixQuality = 0,
    .dtLastFix = 0.0f,
    .dtSinceDate = 0.0f,
    .dtSinceTime = 0.0f,

    .latitude = 0.0f,
    .longitude = 0.0f,
    .altitude = 0.0f,

    .speed = 0.0f,
    .heading = 0.0f,

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
DebuggerType g_debugger = {
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
Adafruit_GPS gpsModule(&Wire);

// display
Adafruit_ST7789 displayModule = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST); // 240x135
DisplayModeType g_previousDisplayMode = ERROR_MODE;
bool g_drewMoonIcon = false;
int g_previousSelectedWaypoint = -1; // init to non-existant index to force first time compute of vector
bool g_updatedFlashlight = false;    // a flag that the flashlight has been enabled
bool g_colorToggleEnable = false;    // flag for alternating colors

// io
bool g_selectButtonPreviousState = LOW;
unsigned long g_selectButtonCounter = 0;
bool g_selectButtonToggle = false;

bool g_optionButtonPreviousState = LOW;
unsigned long g_optionButtonCounter = 0;
bool g_optionButtonToggle = false;

bool g_returnButtonPreviousState = LOW;
unsigned long g_returnButtonCounter = 0;

// time keeping
bool g_wasSleeping = false;
unsigned long g_refreshRateCounter = 0;
unsigned long g_refreshRateLastTime = 0;
unsigned long g_waypointLastComputeTime = 0;
unsigned long g_colorToggleLastTime = 0;
unsigned long g_lastBatteryCheckTime = 0;
unsigned long g_lastBatteryChargeIndicatorTime = 0;

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
float DegreesToRadians(float degrees);
float RadiansToDegrees(float radians);
String TaskStateToString(eTaskState state);
std::pair<uint16_t, uint16_t> FixStatusColorManager(int fixQuality, bool validDate);
void FlashNVS();

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
void DisplaySleepPrompt(SystemDataType sd);
void DrawCloud(int x, int y, int size, int color);

// debug
void PrintDebug();
void PrintGpsDebug();
void PrintIODebug();
void PrintDisplayDebug();
void PrintSchedulerDebug();

/*
===============================================================================================
                                        setup
===============================================================================================
*/

void setup()
{
  // set power configuration
  esp_pm_configure(&power_configuration);

  // init setup manager
  InitDeviceType setup = {
      .ioActive = false,
      .displayActive = false,
      .gpsActive = false,
  };

  // --------------------------- initialize serial  -------------------------- //
  Serial.begin(9600); // serial output over usb-c port

  if (g_debugger.debugEnabled)
  {
    delay(DEBUG_BOOT_DELAY);
  }

  Serial.printf("\n\n|--- starting setup ---|\n\n");
  // ------------------------------------------------------------------------- //

  // --------------------------- initialize IO ------------------------------- //
  // power
  pinMode(TFT_I2C_POWER, OUTPUT);    // turn on power to the gps module
  digitalWrite(TFT_I2C_POWER, HIGH); // turn on power to the display

  // gps
  pinMode(GPS_WAKE_PIN, OUTPUT);           // if the module was in backup mode, this is the only way to wake it
  gpio_hold_dis((gpio_num_t)GPS_WAKE_PIN); // when coming out of deep sleep, the hold needs to be disabled
  digitalWrite(GPS_WAKE_PIN, HIGH);        // wake device

  // tft
  pinMode(TFT_BACKLITE, OUTPUT);

  // sleep
  gpio_deep_sleep_hold_en();
  esp_sleep_enable_ext0_wakeup((gpio_num_t)RETURN_BUTTON, HIGH); // use the return button to wake from deep sleep

  // io
  pinMode(OPTION_BUTTON, INPUT);
  pinMode(RETURN_BUTTON, INPUT); // the select button is set to an input by default, adding it here manually breaks its functionality

  Serial.printf("gpio init [ success ]\n");
  setup.ioActive = true;
  // ------------------------------------------------------------------------- //

  // -------------------- initialize non-volitile storage -------------------- //
  if (wpStorage.begin("wp-storage", false)) // true = read only | false = read/write
  {
    Serial.printf("nvs init: [ success ]\n");

    // flash nvs
    if (NVS_FLASH_WITH_DEFAULT)
    {
      FlashNVS();
      Serial.printf("\tnvs flashed with default data!\n");
    }

    // read nvs
    float tmpLat, tmpLong;
    String tmpName;
    tmpLat = wpStorage.getFloat(NVS_WP_1_LAT_KEY, -99);
    tmpLong = wpStorage.getFloat(NVS_WP_1_LONG_KEY, -99);
    tmpName = wpStorage.getString(NVS_WP_1_NAME_KEY, " ");
    WaypointCoordinatesType wp1 = {tmpLat, tmpLong, tmpName};

    tmpLat = wpStorage.getFloat(NVS_WP_2_LAT_KEY, -99);
    tmpLong = wpStorage.getFloat(NVS_WP_2_LONG_KEY, -99);
    tmpName = wpStorage.getString(NVS_WP_2_NAME_KEY, " ");
    WaypointCoordinatesType wp2 = {tmpLat, tmpLong, tmpName};

    tmpLat = wpStorage.getFloat(NVS_WP_3_LAT_KEY, -99);
    tmpLong = wpStorage.getFloat(NVS_WP_3_LONG_KEY, -99);
    tmpName = wpStorage.getString(NVS_WP_3_NAME_KEY, " ");
    WaypointCoordinatesType wp3 = {tmpLat, tmpLong, tmpName};

    tmpLat = wpStorage.getFloat(NVS_WP_4_LAT_KEY, -99);
    tmpLong = wpStorage.getFloat(NVS_WP_4_LONG_KEY, -99);
    tmpName = wpStorage.getString(NVS_WP_4_NAME_KEY, " ");
    WaypointCoordinatesType wp4 = {tmpLat, tmpLong, tmpName};

    tmpLat = wpStorage.getFloat(NVS_WP_5_LAT_KEY, -99);
    tmpLong = wpStorage.getFloat(NVS_WP_5_LONG_KEY, -99);
    tmpName = wpStorage.getString(NVS_WP_5_NAME_KEY, " ");
    WaypointCoordinatesType wp5 = {tmpLat, tmpLong, tmpName};

    wpStorage.end();

    // save to dynamic memory
    g_systemData.waypointData.waypoints.push_back(wp1);
    g_systemData.waypointData.waypoints.push_back(wp2);
    g_systemData.waypointData.waypoints.push_back(wp3);
    g_systemData.waypointData.waypoints.push_back(wp4);
    g_systemData.waypointData.waypoints.push_back(wp5);
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
    batteryModule.enableSleep(true);

    // collect information
    uint8_t chipId = batteryModule.getChipID();
    g_systemData.power.batteryPercent = batteryModule.cellPercent();
    g_systemData.power.batteryVoltage = batteryModule.cellVoltage();
    Serial.printf("\tchip id: 0x%x\n", chipId);
  }
  else
  {
    Serial.printf("battery init [ failed ]\n");
  }
  // ------------------------------------------------------------------------- //

  // -------------------------- initialize display --------------------------- //
  displayModule.init(135, 240); // set display size
  displayModule.setRotation(3);
  displayModule.fillScreen(ST77XX_BLACK); // ensure display remains dim when backlight is turned on
  digitalWrite(TFT_BACKLITE, HIGH);       // turn on display backlight

  // boot screen
  displayModule.setTextSize(1);
  displayModule.setTextColor(ST77XX_GREEN, ST77XX_BLACK);
  displayModule.setCursor(55, 60);
  displayModule.printf("[> booting mini gps <]");
  delay(1000);

  setup.displayActive = true;
  Serial.printf("display init [ success ]\n");
  // -------------------------------------------------------------------------- //

  // -------------------------- initialize gps -------------------------------- //
  if (gpsModule.begin(GPS_I2C_ADDR))
  {
    gpsModule.wakeup();

    // set data filter
    gpsModule.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGAGSA);

    // set update message rate
    gpsModule.sendCommand(PMTK_SET_NMEA_UPDATE_10HZ);

    // set position fix rate
    gpsModule.sendCommand(PMTK_API_SET_FIX_CTL_5HZ);

    Serial.printf("gps init [ success ]\n");
    setup.gpsActive = true;
  }
  else
  {
    Serial.printf("gps init [ failed ]\n");
  }
  // -------------------------------------------------------------------------- //

  // -------------------------- queues & scheduler ---------------------------- //
  // init queues
  xIoQueue = xQueueCreate(1, sizeof(SystemDataType));
  xGpsQueue = xQueueCreate(1, sizeof(GpsDataType));

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

    if (g_debugger.debugEnabled == true)
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
 * @brief reads and writes i/o
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

    // do time keeping
    unsigned long now = millis();
    if (now - g_lastBatteryCheckTime >= BATTERY_POLL_INTERVAL)
    {
      g_systemData.power = BatteryManager();
      g_lastBatteryCheckTime = now;
    }

    // display event handler
    switch (g_systemData.display.displayMode)
    {
    case GPS_MODE:
      if (g_systemData.inputFlags.returnShortPress)
      {
        g_systemData.display.displayMode = WAYPOINT_MODE;
        g_previousSelectedWaypoint = -1; // force computation of vector
      }
      if (g_systemData.inputFlags.returnLongPress)
      {
        g_systemData.display.displayMode = GPS_MODE;
      }
      break;

    case WAYPOINT_MODE:
      if (g_systemData.inputFlags.selectShortPress)
      {
        g_systemData.waypointData.selectedWaypoint++;
        if (g_systemData.waypointData.selectedWaypoint >= (g_systemData.waypointData.waypoints.size()))
        {
          g_systemData.waypointData.selectedWaypoint = 0;
        }
      }

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
      if (g_systemData.inputFlags.selectShortPress)
      {
        g_systemData.display.displayMode = SLEEP_PROMPT_MODE;
        g_drewMoonIcon = false;
      }

      if (g_systemData.inputFlags.optionShortPress)
      {
        g_systemData.display.displayMode = FLASHLIGHT_MODE;
      }

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
      if (g_systemData.inputFlags.optionShortPress)
      {
        g_systemData.enableFlashlight = !g_systemData.enableFlashlight;
      }

      if (g_systemData.inputFlags.returnShortPress)
      {
        g_systemData.display.displayMode = SYSTEM_MODE;
      }
      break;

    default:
      g_systemData.display.displayMode = ERROR_MODE;

      if (g_systemData.inputFlags.returnShortPress)
      {
        g_systemData.display.displayMode = GPS_MODE;
      }
      break;
    }

    // --- sleep logic --- //
    if (g_systemData.power.sleepModeEnable)
    {
      // turn off display and gps module power
      gpsModule.sendCommand(PTMK_BACKUP_POWER); // backup power command, set module
      digitalWrite(GPS_WAKE_PIN, LOW);
      gpio_hold_en((gpio_num_t)GPS_WAKE_PIN);
      digitalWrite(TFT_I2C_POWER, LOW);
      batteryModule.sleep(true);

      esp_deep_sleep_start();
    }
    // --- sleep logic --- //

    // send data to the display
    xQueueOverwrite(xIoQueue, &g_systemData);

    // debugging
    if (g_debugger.debugEnabled)
    {
      g_debugger.ioTaskCount++;
    }
  }
}

/**
 * @brief parse and save gps data
 * @param pvParameters parameters passed to task
 */
void GpsTask(void *pvParameters)
{
  // inits
  const TickType_t xFrequency = pdMS_TO_TICKS(GPS_REFRESH_RATE);
  TickType_t taskLastWakeTick = xTaskGetTickCount();

  for (;;)
  {
    // limit task refresh rate
    vTaskDelayUntil(&taskLastWakeTick, xFrequency);

    // read gps data
    while (gpsModule.read() != 0)
    {
      // process gps data
      if (gpsModule.newNMEAreceived())
      {
        if (gpsModule.parse(gpsModule.lastNMEA())) // this sets the newNMEAreceived() flag to false
        {
          // connection
          g_gpsData.numSats = gpsModule.satellites;
          g_gpsData.fixQuality = gpsModule.fixquality;
          if (gpsModule.secondsSinceFix() < DELTA_FIX_HARDSTOP)
          {
            g_gpsData.dtLastFix = gpsModule.secondsSinceFix();
          }
          else
          {
            g_gpsData.dtLastFix = -1;
          }

          g_gpsData.dtSinceTime = gpsModule.secondsSinceTime();
          g_gpsData.dtSinceTime = gpsModule.secondsSinceDate();

          //  location
          g_gpsData.latitude = gpsModule.latitudeDegrees;
          g_gpsData.longitude = gpsModule.longitudeDegrees;
          g_gpsData.altitude = gpsModule.altitude * METERS_TO_FEET;

          // vector
          g_gpsData.speed = gpsModule.speed; // speed is given in knots
          g_gpsData.heading = gpsModule.angle;

          // apply velocity deadband
          if (g_gpsData.speed >= MIN_SPEED)
          {
            g_gpsData.speed = g_gpsData.speed * KNOTS_TO_MPH; // convert to mph
          }
          else
          {
            g_gpsData.speed = 0.0f;
            g_gpsData.heading = 0.0f;
          }

          // data
          g_gpsData.year = gpsModule.year + 2000;
          g_gpsData.month = gpsModule.month;
          g_gpsData.day = gpsModule.day;
          g_gpsData.validDate = (g_gpsData.year >= INIT_OPERATING_YEAR && g_gpsData.year < EOL_YEAR) ? true : false; // determine if date time information is valid

          // time
          g_gpsData.hour = gpsModule.hour;
          g_gpsData.minute = gpsModule.minute;
          g_gpsData.second = gpsModule.seconds;
        }
      }
    }

    // sync data
    xQueueOverwrite(xGpsQueue, &g_gpsData);

    // debugging
    if (g_debugger.debugEnabled)
    {
      // Serial.printf("fix: %s\n", gpsModule.fix ? "yes" : "no");
      // Serial.printf("# sats: %d\n", gpsModule.satellites);

      // Serial.printf("time: %d:%d:%d\n", gpsModule.hour, gpsModule.minute, gpsModule.seconds);
      // Serial.printf("date: %d-%d-%d\n", gpsModule.year, gpsModule.month, gpsModule.day);

      // Serial.printf("lat: %f\n", gpsModule.latitude);
      // Serial.printf("long: %f\n", gpsModule.longitude);
      // Serial.printf("alt: %f\n", gpsModule.altitude);

      // Serial.printf("dt-fix: %f\n", gpsModule.secondsSinceFix());
      // Serial.printf("dt-time: %f\n", gpsModule.secondsSinceTime());
      // Serial.printf("dt-date: %f\n", gpsModule.secondsSinceDate());

      // char c = gpsModule.read();
      // if (c)
      //   Serial.print(c);

      g_debugger.gpsTaskCount++;
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

    if (dataframe.display.displayMode != g_previousDisplayMode)
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
      DisplayFlashlight(dataframe);
      break;

    default:
      DisplayError(dataframe);
      break;
    }
    g_previousDisplayMode = dataframe.display.displayMode;

    // debugging
    if (g_debugger.debugEnabled)
    {
      g_debugger.displayTaskCount++;
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
    if (g_debugger.IO_debugEnabled)
    {
      PrintIODebug();
    }

    // i2c
    if (g_debugger.gps_debugEnabled)
    {
      PrintGpsDebug();
    }

    // display
    if (g_debugger.display_debugEnabled)
    {
      PrintDisplayDebug();
    }

    // scheduler
    if (g_debugger.scheduler_debugEnable)
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
 * @param degrees - the value to be convereted to radians
 * @return degree value in radians
 */
float DegreesToRadians(float degrees)
{
  return (degrees * PI / 180.0);
}

/**
 * @brief converts a value in degrees to radian
 * @param radians - the value to be convereted to degrees
 * @return radian value in degrees
 */
float RadiansToDegrees(float rad)
{
  return rad * (180.0 / PI);
}

/**
 * @brief use the haversine formula to calculate the disance between two gps points on earth
 * @param gps - the current gps data
 * @param waypoint - the selected waypoint data
 * @return distance - the distance to the waypoint in miles
 */
float CalculateWaypointDistance(GpsDataType gps, WaypointCoordinatesType waypoint)
{
  // inits
  double dLat = DegreesToRadians(waypoint.latitude - gps.latitude);
  double dLon = DegreesToRadians(waypoint.longitude - gps.longitude);

  // calculate!
  double a = sin(dLat / 2) * sin(dLat / 2) +
             cos(DegreesToRadians(gps.latitude)) * cos(DegreesToRadians(waypoint.longitude)) *
                 sin(dLon / 2) * sin(dLon / 2);

  double c = 2 * atan2(sqrt(a), sqrt(1 - a));

  return RADIUS_OF_EARTH * c; // result in meters
}

/**
 * @brief use the a funky formula to calculate the bearing from gps point 1 to gps point 2
 * @param gps - the current gps data
 * @param waypoint - the selected waypoint
 * @return bearing - the bearing the waypoint in degrees
 */
float CalculateWaypointBearing(GpsDataType gps, WaypointCoordinatesType waypoint)
{
  // inits
  double phi1 = DegreesToRadians(gps.latitude);
  double phi2 = DegreesToRadians(waypoint.latitude);
  double dLon = DegreesToRadians(waypoint.longitude - gps.longitude);

  // calculate!
  double y = sin(dLon) * cos(phi2);
  double x = cos(phi1) * sin(phi2) -
             sin(phi1) * cos(phi2) * cos(dLon);
  double bearing = atan2(y, x);
  bearing = RadiansToDegrees(bearing);

  if (bearing < 0)
  {
    bearing += 360.0; // normalize
  }

  return bearing;
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

  if (selectButtonState != g_selectButtonPreviousState)
  {
    g_selectButtonCounter = millis();
    g_selectButtonPreviousState = selectButtonState;

    // check specifically for LOW -> HIGH
    if (g_selectButtonPreviousState == HIGH)
    {
      if (millis() - g_selectButtonCounter > SHORT_PRESS_DURATION)
      {
        iF.selectLongPress = true;
      }
      else
      {
        iF.selectShortPress = true;
      }
    }
  }
  g_selectButtonPreviousState = selectButtonState;
  // --- select button --- //

  // --- option button --- //
  bool optionButtonState = digitalRead(OPTION_BUTTON);

  if (optionButtonState != g_optionButtonPreviousState)
  {
    g_optionButtonCounter = millis();
    g_optionButtonPreviousState = optionButtonState;

    // check specifically for HIGH -> LOW
    if (g_optionButtonPreviousState == LOW)
    {
      if (millis() - g_optionButtonCounter > SHORT_PRESS_DURATION)
      {
        iF.optionLongPress = true;
      }
      else
      {
        iF.optionShortPress = true;
      }
    }
  }
  g_optionButtonPreviousState = optionButtonState;
  // --- option button --- //

  // --- return button --- //
  bool returnButtonState = digitalRead(RETURN_BUTTON);

  if (returnButtonState != g_returnButtonPreviousState)
  {
    g_returnButtonCounter = millis();
    g_returnButtonPreviousState = returnButtonState;

    // check specifically for HIGH -> LOW
    if (g_returnButtonPreviousState == LOW)
    {
      if (millis() - g_returnButtonCounter > SHORT_PRESS_DURATION)
      {
        iF.returnLongPress = true;
      }
      else
      {
        iF.returnShortPress = true;
      }
    }
  }
  g_returnButtonPreviousState = returnButtonState;
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
 * @param gps - the current gps data
 */
void DisplayGpsData(GpsDataType gps)
{
  // debug
  // gps.validDate = true;
  // gps.fixQuality = 1,
  // gps.dtLastFix = 0.0f,
  // gps.dtSinceDate = 0.0f;
  // gps.dtSinceTime = 0.0f;
  // gps.latitude = 44.4788300;
  // gps.longitude = -73.205870;
  // gps.altitude = 4324.1;
  // gps.speed = 3.1;
  // gps.heading = 271;
  // gps.year = 2025;
  // gps.month = 1;
  // gps.day = 3;
  // gps.hour = 1;
  // gps.minute = 4;
  // gps.second = 3;
  // gps.timeout = 0;
  // gps.numSats = 4;

  // location
  displayModule.setTextSize(2);
  displayModule.setTextColor(ST77XX_CYAN, ST77XX_BLACK);
  if (gps.numSats >= MIN_SATS)
  {
    displayModule.setCursor(5, 30);
    displayModule.printf("%.5f", gps.latitude);

    displayModule.setCursor(130, 30);
    displayModule.printf("%.5f", gps.longitude);

    displayModule.setCursor(90, 55);
    displayModule.printf("%4.1f ft ", gps.altitude);
  }
  else
  {
    displayModule.setCursor(5, 30);
    displayModule.printf("--.-----");

    displayModule.setCursor(140, 30);
    displayModule.printf("--.-----");

    displayModule.setCursor(70, 55);
    displayModule.printf("----.- ft ");
  }

  // speed
  displayModule.setTextSize(2);
  displayModule.setTextColor(ST77XX_MAGENTA, ST77XX_BLACK);
  displayModule.setCursor(5, 90);
  if (gps.numSats > MIN_SATS)
  {
    displayModule.printf("%.1fmph ", gps.speed);
  }
  else
  {
    displayModule.printf("--.-mph ");
  }

  // heading
  displayModule.setTextColor(ST77XX_BLUE, ST77XX_BLACK);
  displayModule.setCursor(190, 90);
  if (gps.speed > MIN_SPEED)
  {
    displayModule.printf("%03d%c", (int)gps.heading, 0xF7);
  }
  else
  {
    displayModule.printf("---%c", 0xF7);
  }

  // TODO: draw compass
  // displayModule.drawCircle(115, 100, 10, ST77XX_WHITE);
  // displayModule.drawLine()

  // date and time
  displayModule.setTextSize(1);
  if (gps.validDate)
  {
    displayModule.setTextColor(ST77XX_GREEN, ST77XX_BLACK);
    displayModule.setCursor(180, 115);
    displayModule.printf("%04d/%02d/%02d", gps.year, gps.month, gps.day);

    displayModule.setCursor(156, 125);
    displayModule.printf("%02d:%02d:%02d (UTC)", gps.hour, gps.minute, gps.second);
  }
  else
  {
    displayModule.setTextColor(ST77XX_RED, ST77XX_BLACK);
    displayModule.setCursor(180, 115);
    displayModule.printf("----/--/--");
    displayModule.setTextColor(ST77XX_MAGENTA, ST77XX_BLACK);
    displayModule.setCursor(156, 125);
    displayModule.printf("--:--:-- (UTC)");
  }

  // sats
  displayModule.setTextSize(1);
  displayModule.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
  displayModule.setCursor(0, 115);
  displayModule.printf("sats: ");

  if (gps.numSats == 0)
  {
    displayModule.setTextColor(ST77XX_RED, ST77XX_BLACK);
  }
  else if (gps.numSats > 0 && gps.numSats <= MIN_SATS)
  {
    displayModule.setTextColor(ST77XX_ORANGE, ST77XX_BLACK);
  }
  else if (gps.numSats > MIN_SATS)
  {
    displayModule.setTextColor(ST77XX_GREEN, ST77XX_BLACK);
  }
  displayModule.printf("%d ", gps.numSats);

  // time since last fix
  displayModule.setTextSize(1);
  displayModule.setCursor(0, 125);
  displayModule.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
  displayModule.printf("dt-fix: ");
  if (gps.validDate)
  {
    if (gps.dtLastFix > 0 && gps.dtLastFix < 1.0)
    {
      displayModule.setTextColor(ST77XX_GREEN, ST77XX_BLACK);
    }
    else if (gps.dtLastFix < 60 && gps.dtLastFix > 1.0) // been a bit since a connection
    {
      displayModule.setTextColor(ST77XX_YELLOW, ST77XX_BLACK);
    }
    else // longer than a minute without a fix
    {
      displayModule.setTextColor(ST77XX_ORANGE, ST77XX_BLACK);
    }
    displayModule.printf("%.2fs       ", gps.dtLastFix);
  }
  else
  {
    displayModule.setTextColor(ST77XX_RED, ST77XX_BLACK);
    displayModule.printf("no rtc      ");
  }
}

/**
 * @brief the waypoint screen
 * @param sd - the current system data
 * @param gps - the current gps data
 */
void DisplayWaypoint(SystemDataType sd, GpsDataType gps)
{
  // gps.validDate = false;
  // gps.fixQuality = 1,
  // gps.dtLastFix = 0.0f,
  // gps.dtSinceDate = 0.0f;
  // gps.dtSinceTime = 0.0f;
  // gps.latitude = 44.4788300;
  // gps.longitude = -73.205870;
  // gps.altitude = 0.0f;
  // gps.speed = 3.1;
  // gps.heading = 271;
  // gps.year = 2025;
  // gps.month = 11;
  // gps.day = 31;
  // gps.hour = 12;
  // gps.minute = 34;
  // gps.second = 53;
  // gps.timeout = 0;
  // gps.numSats = 4;

  // ensure there is valid waypoint data
  if (!sd.waypointData.waypoints.empty())
  {
    // waypoint selector
    displayModule.setTextSize(1);
    displayModule.setCursor(0, 20);
    displayModule.setTextColor(ST77XX_ORANGE, ST77XX_BLACK);
    displayModule.printf("waypoint: %d", sd.waypointData.selectedWaypoint);

    // display waypoint coordinates
    displayModule.setCursor(0, 30);
    displayModule.printf("lat: %.5f", sd.waypointData.waypoints.at(sd.waypointData.selectedWaypoint).latitude);

    displayModule.setCursor(0, 40);
    displayModule.printf("long: %.5f", sd.waypointData.waypoints.at(sd.waypointData.selectedWaypoint).longitude);

    displayModule.setCursor(0, 50);
    displayModule.printf("name: %s                 ", sd.waypointData.waypoints.at(sd.waypointData.selectedWaypoint).name.c_str());

    // display current coordinates
    displayModule.setTextColor(ST77XX_BLUE, ST77XX_BLACK);
    displayModule.setCursor(150, 20);
    displayModule.printf("current:");

    displayModule.setCursor(150, 30);
    displayModule.printf("lat: %.5f", gps.latitude);

    displayModule.setCursor(150, 40);
    displayModule.printf("long: %.5f", gps.longitude);

    // do time keeping
    bool staleVector = false;
    unsigned long now = millis();
    if (now - g_waypointLastComputeTime >= REFRESH_WAYPOINT_VECTOR_DELAY)
    {
      staleVector = true;
      g_waypointLastComputeTime = now;
    }

    // compute and then show calcuations on select short press
    if (sd.waypointData.selectedWaypoint != g_previousSelectedWaypoint || staleVector)
    {
      WaypointCoordinatesType waypoint = sd.waypointData.waypoints.at(sd.waypointData.selectedWaypoint);

      // inits
      displayModule.setTextSize(2);
      displayModule.setCursor(0, 70);
      displayModule.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
      displayModule.printf("vector ->");

      displayModule.setCursor(0, 90);
      displayModule.setTextColor(ST77XX_GREEN, ST77XX_BLACK);
      displayModule.printf("%.2f miles      ", CalculateWaypointDistance(gps, waypoint));
      displayModule.setCursor(0, 110);
      displayModule.printf("@ %.0f%c  ", CalculateWaypointBearing(gps, waypoint), 0xF7);
    }
    g_previousSelectedWaypoint = sd.waypointData.selectedWaypoint;
  }
  else
  {
    displayModule.setTextColor(ST77XX_RED, ST77XX_BLACK);
    displayModule.setCursor(40, 100);
    displayModule.printf("[ error loading waypoints ]");
  }
}

/**
 * @brief system information screen
 * @param sd - the current system data
 */
void DisplaySystem(SystemDataType sd)
{
  displayModule.setTextSize(2);
  displayModule.setTextColor(ST77XX_WHITE, ST77XX_BLACK);

  // display battery percent charge
  if (sd.power.batteryPercent >= HALF_BATTERY_CAPACITY)
  {
    displayModule.setTextColor(ST77XX_GREEN, ST77XX_BLACK);
  }
  else if (sd.power.batteryPercent > LOW_BATTERY_CAPACITY)
  {
    displayModule.setTextColor(ST77XX_YELLOW, ST77XX_BLACK);
  }
  else
  {
    displayModule.setTextColor(ST77XX_RED, ST77XX_BLACK);
  }
  displayModule.setCursor(0, 30);
  displayModule.printf("battery: %.1f%% ", sd.power.batteryPercent);

  // display battery voltage
  if (sd.power.batteryVoltage >= HIGH_BATTERY_VOLTAGE)
  {
    displayModule.setTextColor(ST77XX_GREEN, ST77XX_BLACK);
  }
  else if (sd.power.batteryVoltage > LOW_BATTERY_VOLTAGE)
  {
    displayModule.setTextColor(ST77XX_YELLOW, ST77XX_BLACK);
  }
  else
  {
    displayModule.setTextColor(ST77XX_RED, ST77XX_BLACK);
  }
  displayModule.setCursor(0, 50);
  displayModule.printf("voltage: %.3fv", sd.power.batteryVoltage);

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
  displayModule.printf("uptime: %02u:%02u:%02u:%02u    ", hours, minutes, secs, centis);

  // firmware verison info
  displayModule.setTextColor(ST77XX_MAGENTA, ST77XX_BLACK);
  displayModule.setTextSize(1);
  displayModule.setCursor(0, 120);
  displayModule.printf("firmware: %d.%d \"%s\"", FIRMWARE_MAJOR, FIRMWARE_MINOR, FIRMWARE_NAME);

  // display refresh rate
  g_refreshRateCounter++;
  unsigned long now = millis();
  if (now - g_refreshRateLastTime >= DISPLAY_REFRESH_RATE_CALCULATE_INTERVAL)
  {                                                                                    // 1 second passed
    float refreshRate = g_refreshRateCounter * 1000.0 / (now - g_refreshRateLastTime); // Hz
    g_refreshRateCounter = 0;
    g_refreshRateLastTime = now;

    displayModule.setTextColor(ST77XX_CYAN, ST77XX_BLACK);
    displayModule.setTextSize(1);
    displayModule.setCursor(190, 120);
    displayModule.printf("%.2fHz ", refreshRate);
  }
}

/**
 * @brief display a status bar at the top of the screen with important information
 * @param sd - the current system data
 * @param gps - the current gps data
 */
void DisplayStatusBar(SystemDataType sd, GpsDataType gps)
{
  // inits
  int textY = 5;

  // --- mode --- //
  String mode;
  int underscoreLength;
  displayModule.setCursor(0, textY);
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

  case FLASHLIGHT_MODE:
    mode = "flashlight";
    displayModule.setTextColor(ST77XX_RED, ST77XX_BLACK);
    break;

  default:
    mode = "error";
    displayModule.setTextColor(ST77XX_ORANGE, ST77XX_BLACK);
    break;
  }
  displayModule.printf("<%s>", mode.c_str());
  // --- mode --- //

  // --- gps fix status --- //
  std::pair<uint16_t, uint16_t> colors = FixStatusColorManager(gps.fixQuality, gps.validDate);
  displayModule.setTextColor(colors.first, colors.second);
  displayModule.setTextSize(1);
  displayModule.setCursor(80, textY);
  displayModule.printf("gps");
  // --- gps fix status --- //

  // --- time --- //
  displayModule.setTextSize(1);
  displayModule.setCursor(135, textY);
  if (gps.validDate)
  {
    displayModule.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
    displayModule.printf("%02d:%02d:%02d", gps.hour, gps.minute, gps.second);
  }
  else
  {
    displayModule.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
    displayModule.printf("--:--:--");
  }
  // --- time --- //

  // --- battery --- //
  // write battery percentage
  displayModule.setTextSize(1);
  displayModule.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
  String buffer = String((int)sd.power.batteryPercent).substring(0, 3);
  uint16_t strLen = buffer.length();
  uint16_t batterySymbolWidth = (strLen * 8) + 10;               // +10 for % symbol
  uint16_t batterySymbolX = 240 - (batterySymbolWidth + strLen); // screen width - (text + symbol width)
  uint16_t batteryTextX = batterySymbolX + 5;                    // center the text in the battery symbol
  displayModule.setCursor(batteryTextX, textY);
  displayModule.printf("%s%% ", buffer.c_str());

  // draw battery symbol
  if (sd.power.batteryChargeRate >= 0)
  {
    // do time keeping
    unsigned long now = millis();
    if (now - g_lastBatteryChargeIndicatorTime >= BATTERY_CHARGING_INDICATION_DELAY) // flash the symbol at 2Hz
    {
      displayModule.drawRoundRect(batterySymbolX, 2, batterySymbolWidth, 12, 4, ST77XX_GREEN);
      g_lastBatteryChargeIndicatorTime = now;
    }
    else
    {
      displayModule.drawRoundRect(batterySymbolX, 2, batterySymbolWidth, 12, 4, ST77XX_BLACK);
    }
  }
  else
  {
    if (sd.power.batteryPercent > 50.0)
    {
      displayModule.drawRoundRect(batterySymbolX, 2, batterySymbolWidth, 12, 4, ST77XX_GREEN);
    }
    else if (sd.power.batteryPercent > 20.0 && sd.power.batteryPercent <= 50.0)
    {
      displayModule.drawRoundRect(batterySymbolX, 2, batterySymbolWidth, 12, 4, ST77XX_ORANGE);
    }
    else if (sd.power.batteryPercent <= 20.0)
    {
      displayModule.drawRoundRect(batterySymbolX, 2, batterySymbolWidth, 12, 4, ST77XX_RED);
    }
  }

  // --- battery --- //
}

/**
 * @brief display screen that indicates a failure in display modes
 * @param sd - the current system data
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
 * @brief why not?
 * @param sd - the current system data
 */
void DisplayFlashlight(SystemDataType sd)
{
  if (sd.enableFlashlight)
  {
    if (sd.enableFlashlight != g_updatedFlashlight)
    {
      displayModule.fillScreen(ST77XX_WHITE);
    }
  }
  else
  {
    displayModule.setCursor(5, 65);
    displayModule.setTextColor(ST77XX_RED, ST77XX_BLACK);
    displayModule.setTextSize(2);
    displayModule.printf("<] on / off");
    if (sd.enableFlashlight != g_updatedFlashlight)
    {
      displayModule.fillScreen(ST77XX_BLACK);
    }
  }
  g_updatedFlashlight = sd.enableFlashlight;
}

/**
 * @brief while device is entering sleep mode popup
 * @param sd - the current system data
 */
void DisplaySleepPrompt(SystemDataType sd)
{
  displayModule.setTextSize(2);
  displayModule.setCursor(25, 30);
  displayModule.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
  displayModule.printf("[> hibernate <]");

  displayModule.setCursor(5, 65);
  displayModule.setTextColor(ST77XX_GREEN, ST77XX_BLACK);
  displayModule.printf("<] confirm");

  displayModule.setCursor(5, 120);
  displayModule.setTextColor(ST77XX_RED, ST77XX_BLACK);
  displayModule.printf("<] cancel");

  // draw moon
  if (!g_drewMoonIcon)
  {
    displayModule.fillCircle(180, 85, 30, ST77XX_YELLOW); // moon
    displayModule.fillCircle(170, 75, 30, ST77XX_BLACK);  // moon
    DrawCloud(205, 115, 40, ST77XX_WHITE);
    g_drewMoonIcon = true;
  }
}

/**
 * @brief draw a cloud!
 * @param x - x position on the screen
 * @param y - x position on the screen
 * @param size - the relative size of the clouds in the group
 * @param color - the color of the clouds
 */
void DrawCloud(int x, int y, int size, int color)
{
  // size is the "scale" of the cloud
  int r = size / 3;

  // main center puff
  displayModule.fillCircle(x, y, r + 4, color);

  // surrounding puffs
  displayModule.fillCircle(x - r, y, r, color);
  displayModule.fillCircle(x + r, y, r, color);
  displayModule.fillCircle(x - r / 2, y - r, r, color);
  displayModule.fillCircle(x + r / 2, y - r, r, color);
  displayModule.fillCircle(x, y + r / 2, r, color);
}

/**
 * @brief determine the color of the gps icon in the status bar based on fix and rtc data quality
 * @param fixQuality - the fix status
 * @param validData - if the date time data is good
 * @return text color, text background color
 */
std::pair<uint16_t, uint16_t> FixStatusColorManager(int fixQuality, bool validDate)
{
  // init
  std::pair<uint16_t, uint16_t> colors;
  uint16_t textColor, backgroundColor;

  // do time keeping
  unsigned long now = millis();
  if (now - g_colorToggleLastTime >= 1000) // toggle colors once a second
  {
    g_colorToggleEnable = !g_colorToggleEnable;
    g_colorToggleLastTime = now;
  }

  // determine colors
  switch (fixQuality)
  {
  case 0:          // no fix
    if (validDate) // no fix but valid rtc data
    {
      if (g_colorToggleEnable)
      {
        textColor = ST77XX_BLACK;
        backgroundColor = ST77XX_ORANGE;
      }
      else
      {
        textColor = ST77XX_ORANGE;
        backgroundColor = ST77XX_BLACK;
      }
    }
    else // no fix & no rtc data
    {
      if (g_colorToggleEnable)
      {
        textColor = ST77XX_RED;
        backgroundColor = ST77XX_BLACK;
      }
      else
      {
        textColor = ST77XX_BLACK;
        backgroundColor = ST77XX_RED;
      }
    }
    break;

  case 1: // fix
    if (g_colorToggleEnable)
    {
      textColor = ST77XX_BLACK;
      backgroundColor = ST77XX_GREEN;
    }
    else
    {
      textColor = ST77XX_GREEN;
      backgroundColor = ST77XX_BLACK;
    }
    break;

  case 2: // dgps
    if (g_colorToggleEnable)
    {
      textColor = ST77XX_BLACK;
      backgroundColor = ST77XX_BLUE;
    }
    else
    {
      textColor = ST77XX_BLUE;
      backgroundColor = ST77XX_BLACK;
    }

    break;

  default: // something is wrong i guess
    if (g_colorToggleEnable)
    {
      textColor = ST77XX_BLACK;
      backgroundColor = ST77XX_RED;
    }
    else
    {
      textColor = ST77XX_RED;
      backgroundColor = ST77XX_BLACK;
    }
    break;
  }

  // build color profile
  colors.first = textColor;
  colors.second = backgroundColor;
  return colors;
}

/**
 * @brief default values to flash the nvs with
 */
void FlashNVS()
{
  wpStorage.putFloat(NVS_WP_1_LAT_KEY, 44.47883);
  wpStorage.putFloat(NVS_WP_1_LONG_KEY, -73.20587);
  wpStorage.putString(NVS_WP_1_NAME_KEY, String("btv"));

  wpStorage.putFloat(NVS_WP_2_LAT_KEY, 43.03033);
  wpStorage.putFloat(NVS_WP_2_LONG_KEY, -72.87210);
  wpStorage.putString(NVS_WP_2_NAME_KEY, String("cabin"));

  wpStorage.putFloat(NVS_WP_3_LAT_KEY, 40.36240);
  wpStorage.putFloat(NVS_WP_3_LONG_KEY, -74.04034);
  wpStorage.putString(NVS_WP_3_NAME_KEY, String("fair haven"));

  wpStorage.putFloat(NVS_WP_4_LAT_KEY, -100);
  wpStorage.putFloat(NVS_WP_4_LONG_KEY, -100);
  wpStorage.putString(NVS_WP_4_NAME_KEY, String(" "));

  wpStorage.putFloat(NVS_WP_5_LAT_KEY, -100);
  wpStorage.putFloat(NVS_WP_5_LONG_KEY, -100);
  wpStorage.putString(NVS_WP_5_NAME_KEY, String(" "));

  wpStorage.putBool(NVS_WAS_SLEEPING_KEY, false);
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

  taskRefreshRate.push_back(g_debugger.ioTaskCount - g_debugger.ioTaskPreviousCount);
  taskRefreshRate.push_back(g_debugger.gpsTaskCount - g_debugger.gpsTaskPreviousCount);
  taskRefreshRate.push_back(g_debugger.displayTaskCount - g_debugger.displayTaskPreviousCount);

  // print
  Serial.printf("uptime: %d | io: <%d Hz> (%d) | gps: <%d Hz> (%d) | display: <%d Hz> (%d) \n",
                uptime, taskRefreshRate.at(0), g_debugger.ioTaskCount, taskRefreshRate.at(1), g_debugger.gpsTaskCount,
                taskRefreshRate.at(2), g_debugger.displayTaskCount);

  // update counters
  g_debugger.ioTaskPreviousCount = g_debugger.ioTaskCount;
  g_debugger.gpsTaskPreviousCount = g_debugger.gpsTaskCount;
  g_debugger.displayTaskPreviousCount = g_debugger.displayTaskCount;
}