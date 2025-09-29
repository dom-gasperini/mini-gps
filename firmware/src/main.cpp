/**
 * @file main.cpp
 * @author dom gasperini
 * @brief mini gps
 * @version 5
 * @date 2025-09-28
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

// general
#define MIN_SATS 3             // min number of sats to have a fix
#define KNOTS_TO_MPH 1.1507795 // mulitplier for converting knots to mph
#define MIN_SPEED 2.00         // minimum number of knots before displaying speed to due resolution limitations
#define INIT_OPERATING_YEAR 2024
#define RADIUS_OF_EARTH 3958.756   // in miles
#define HALF_BATTERY_CAPACITY 50.0 // in %
#define LOW_BATTERY_CAPACITY 20.0  // in %
#define HIGH_BATTERY_VOLTAGE 3.9   // in volts
#define LOW_BATTERY_VOLTAGE 3.4    // in volts
#define EOL_YEAR 2079
#define DELTA_FIX_HARDSTOP 100000 // seconds

// comms
#define I2C_FREQUENCY 115200
#define BATT_MGMT_ADDR 0x36
#define LOW_BATTERY_THRESHOLD 10
#define NUM_DATA_READS 900 // read available data on the serial bus with this limit

// system
#define FIRMWARE_MAJOR 5
#define FIRMWARE_MINOR 211
#define FIRMWARE_NAME "stargazer"

// time keeping
#define REFRESH_WAYPOINT_VECTOR_DELAY 5000           // in milliseconds
#define BATTERY_POLL_INTERVAL 2000                   // in milliseconds
#define BATTERY_CHARGING_INDICATION_DELAY 500        // in milliseconds
#define DISPLAY_REFRESH_RATE_CALCULATE_INTERVAL 1000 // in milliseconds
#define SHORT_PRESS_DURATION 100                     // in milliseconds
#define LONG_PRESS_DURATION 200                      // in milliseconds

// nvs
#define FLASH_NVS_WITH_DEFAULT false // write the default values to the nvs
#define WP_1_LAT_NVS_KEY "wp1-lat"
#define WP_1_LONG_NVS_KEY "wp1-long"
#define WP_1_NAME_NVS_KEY "wp1-name"
#define WP_2_LAT_NVS_KEY "wp2-lat"
#define WP_2_LONG_NVS_KEY "wp2-long"
#define WP_2_NAME_NVS_KEY "wp2-name"
#define WP_3_LAT_NVS_KEY "wp3-lat"
#define WP_3_LONG_NVS_KEY "wp3-long"
#define WP_3_NAME_NVS_KEY "wp3-name"
#define WP_4_LAT_NVS_KEY "wp4-lat"
#define WP_4_LONG_NVS_KEY "wp4-long"
#define WP_4_NAME_NVS_KEY "wp4-name"
#define WP_5_LAT_NVS_KEY "wp5-lat"
#define WP_5_LONG_NVS_KEY "wp5-long"
#define WP_5_NAME_NVS_KEY "wp5-name"

// tasks
#define TASK_STACK_SIZE 4096 // in bytes
#define EXECUTIVE_CORE 0
#define DISPLAY_CORE 1
#define IO_REFRESH_RATE 20      // measured in ticks (RTOS ticks interrupt at 1 kHz)
#define GPS_REFRESH_RATE 20     // measured in ticks (RTOS ticks interrupt at 1 kHz)
#define DISPLAY_REFRESH_RATE 50 // measured in ticks (RTOS ticks interrupt at 1 kHz)
#define DEBUG_REFRESH_RATE 1000 // measured in ticks (RTOS ticks interrupt at 1 kHz)

// debugging
#define DEBUG_BOOT_DELAY 1000  // in milliseconds
#define ENABLE_DEBUGGING false // master debug toggle (does not disable boot logging)

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
Adafruit_GPS gpsModule(&Serial1);

// display
Adafruit_ST7789 displayModule = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST); // 240x135
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

bool g_drewMoonIcon = false;

int previousSelectedWaypoint = -1; // init to non-existant index to force first time compute of vector
bool updatedFlashlight = false;    // a flag that the flashlight has been enabled
bool colorToggleEnable = false;    // flag for alternating colors

// time keeping
unsigned long refreshRateCounter = 0;
unsigned long refreshRateLastTime = 0;
unsigned long waypointLastComputeTime = 0;
unsigned long colorToggleLastTime = 0;
unsigned long lastBatteryCheckTime = 0;
unsigned long lastBatteryChargeIndicatorTime = 0;

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
bool IsEqual(InputFlagsType x1, InputFlagsType x2);
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
void DisplayWaypointInputTool(SystemDataType sd);
void DisplaySleepPrompt(SystemDataType sd);
void DrawCloud(int x, int y, int size, int color);

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
  Serial.begin(9600); // debug serial

  if (debugger.debugEnabled)
  {
    delay(DEBUG_BOOT_DELAY);
  }

  Serial1.begin(9600, TX, RX); // gps serial

  Serial.printf("\n\n|--- starting setup ---|\n\n");
  // ------------------------------------------------------------------------- //

  // -------------------------- initialize GPIO ------------------------------ //
  // sleep
  gpio_deep_sleep_hold_en();
  esp_sleep_enable_ext0_wakeup((gpio_num_t)RETURN_BUTTON, HIGH);

  // io
  pinMode(OPTION_BUTTON, INPUT);
  pinMode(RETURN_BUTTON, INPUT);

  // gps
  pinMode(GPS_ENABLE_PIN, OUTPUT); // the enable pin specific to the gps module hardware
  gpio_hold_dis((gpio_num_t)GPS_ENABLE_PIN);
  digitalWrite(GPS_ENABLE_PIN, HIGH); // turn on

  // tft
  pinMode(TFT_BACKLITE, OUTPUT);

  // tft and i2c power toggle
  pinMode(TFT_I2C_POWER, OUTPUT);
  digitalWrite(TFT_I2C_POWER, HIGH); // turn on power to the display

  Serial.printf("gpio init [ success ]\n");
  setup.ioActive = true;
  // ------------------------------------------------------------------------- //

  // -------------------- initialize non-volitile storage -------------------- //
  if (wpStorage.begin("wp-storage", false)) // true = read only | false = read/write
  {
    Serial.printf("nvs init: [ success ]\n");

    // flash nvs
    if (FLASH_NVS_WITH_DEFAULT)
    {
      FlashNVS();
      Serial.printf("\tnvs flashed with default data!\n");
    }

    // read nvs
    float tmpLat, tmpLong;
    String tmpName;
    tmpLat = wpStorage.getFloat(WP_1_LAT_NVS_KEY, -99);
    tmpLong = wpStorage.getFloat(WP_1_LONG_NVS_KEY, -99);
    tmpName = wpStorage.getString(WP_1_NAME_NVS_KEY, " ");
    WaypointCoordinatesType wp1 = {tmpLat, tmpLong, tmpName};

    tmpLat = wpStorage.getFloat(WP_2_LAT_NVS_KEY, -99);
    tmpLong = wpStorage.getFloat(WP_2_LONG_NVS_KEY, -99);
    tmpName = wpStorage.getString(WP_2_NAME_NVS_KEY, " ");
    WaypointCoordinatesType wp2 = {tmpLat, tmpLong, tmpName};

    tmpLat = wpStorage.getFloat(WP_3_LAT_NVS_KEY, -99);
    tmpLong = wpStorage.getFloat(WP_3_LONG_NVS_KEY, -99);
    tmpName = wpStorage.getString(WP_3_NAME_NVS_KEY, " ");
    WaypointCoordinatesType wp3 = {tmpLat, tmpLong, tmpName};

    tmpLat = wpStorage.getFloat(WP_4_LAT_NVS_KEY, -99);
    tmpLong = wpStorage.getFloat(WP_4_LONG_NVS_KEY, -99);
    tmpName = wpStorage.getString(WP_4_NAME_NVS_KEY, " ");
    WaypointCoordinatesType wp4 = {tmpLat, tmpLong, tmpName};

    tmpLat = wpStorage.getFloat(WP_5_LAT_NVS_KEY, -99);
    tmpLong = wpStorage.getFloat(WP_5_LONG_NVS_KEY, -99);
    tmpName = wpStorage.getString(WP_5_NAME_NVS_KEY, " ");
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
  displayModule.init(135, 240);
  displayModule.setRotation(3);
  displayModule.fillScreen(ST77XX_BLACK);
  digitalWrite(TFT_BACKLITE, HIGH); // turn on display backlight

  // boot screen
  displayModule.setTextSize(1);
  displayModule.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
  displayModule.setCursor(55, 60);
  displayModule.printf("[> booting mini gps <]");
  delay(1000);

  setup.displayActive = true;
  Serial.printf("display init [ success ]\n");
  // -------------------------------------------------------------------------- //

  // -------------------------- initialize gps -------------------------------- //
  if (gpsModule.begin(9600)) // default baud rate is 9600
  {
    gpsModule.wakeup();

    // filter
    gpsModule.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);

    // set gps to mcu update rate
    gpsModule.sendCommand(PMTK_SET_NMEA_UPDATE_5HZ);

    // set gps position fix rate
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

    // do time keeping
    unsigned long now = millis();
    if (now - lastBatteryCheckTime >= BATTERY_POLL_INTERVAL)
    {
      g_systemData.power = BatteryManager();
      lastBatteryCheckTime = now;
    }

    // display event handler
    switch (g_systemData.display.displayMode)
    {
    case GPS_MODE:
      if (g_systemData.inputFlags.returnShortPress)
      {
        g_systemData.display.displayMode = WAYPOINT_MODE;
        previousSelectedWaypoint = -1; // force computation of vector
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
      gpsModule.standby();
      digitalWrite(GPS_ENABLE_PIN, LOW);
      gpio_hold_en((gpio_num_t)GPS_ENABLE_PIN);
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
 * @brief gps
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
          g_gpsData.altitude = gpsModule.altitude;

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
    if (debugger.debugEnabled)
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

      debugger.gpsTaskCount++;
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
      DisplayFlashlight(dataframe);
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
 * @param degrees - the value to be convereted to radians
 */
float DegreesToRadians(float degrees)
{
  return (degrees * PI / 180.0);
}

/**
 * @brief converts a value in degrees to radian
 * @param radians - the value to be convereted to degrees
 */
float RadiansToDegrees(float rad)
{
  return rad * (180.0 / PI);
}

/**
 * use the haversine formula to calculate the disance between two gps points on earth
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
 * use the a funky formula to calculate the bearing from gps point 1 to gps point 2
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
  // gps.validDate = true;
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
    displayModule.printf("%.1f m   ", gps.altitude);
  }
  else
  {
    displayModule.setCursor(5, 30);
    displayModule.printf("--.-----");

    displayModule.setCursor(140, 30);
    displayModule.printf("--.-----");

    displayModule.setCursor(70, 55);
    displayModule.printf("----.- m ");
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
  else if (gps.numSats > 0 && gps.numSats < MIN_SATS)
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
    if (now - waypointLastComputeTime >= REFRESH_WAYPOINT_VECTOR_DELAY)
    {
      staleVector = true;
      waypointLastComputeTime = now;
    }

    // compute and then show calcuations on select short press
    if (sd.waypointData.selectedWaypoint != previousSelectedWaypoint || staleVector)
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
    previousSelectedWaypoint = sd.waypointData.selectedWaypoint;
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
  refreshRateCounter++;
  unsigned long now = millis();
  if (now - refreshRateLastTime >= DISPLAY_REFRESH_RATE_CALCULATE_INTERVAL)
  {                                                                                // 1 second passed
    float refreshRate = refreshRateCounter * 1000.0 / (now - refreshRateLastTime); // Hz
    refreshRateCounter = 0;
    refreshRateLastTime = now;

    displayModule.setTextColor(ST77XX_CYAN, ST77XX_BLACK);
    displayModule.setTextSize(1);
    displayModule.setCursor(190, 120);
    displayModule.printf("%.2fHz ", refreshRate);
  }
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
    if (now - lastBatteryChargeIndicatorTime >= BATTERY_CHARGING_INDICATION_DELAY) // flash the symbol at 2Hz
    {
      displayModule.drawRoundRect(batterySymbolX, 2, batterySymbolWidth, 12, 4, ST77XX_GREEN);
      lastBatteryChargeIndicatorTime = now;
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
  if (sd.enableFlashlight)
  {
    if (sd.enableFlashlight != updatedFlashlight)
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
    if (sd.enableFlashlight != updatedFlashlight)
    {
      displayModule.fillScreen(ST77XX_BLACK);
    }
  }
  updatedFlashlight = sd.enableFlashlight;
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
    // moon
    displayModule.fillCircle(180, 85, 30, ST77XX_YELLOW);
    displayModule.fillCircle(170, 75, 30, ST77XX_BLACK);
    DrawCloud(205, 115, 40, ST77XX_WHITE);
    g_drewMoonIcon = true;
  }
}

/**
 * @brief draw a cloud!
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
 * @brief
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
  if (now - colorToggleLastTime >= 1000) // toggle colors once a second
  {
    colorToggleEnable = !colorToggleEnable;
    colorToggleLastTime = now;
  }

  // determine colors
  switch (fixQuality)
  {
  case 0:          // no fix
    if (validDate) // no fix but valid rtc data
    {
      if (colorToggleEnable)
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
      if (colorToggleEnable)
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
    if (colorToggleEnable)
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
    if (colorToggleEnable)
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
    if (colorToggleEnable)
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

void FlashNVS()
{
  wpStorage.putFloat(WP_1_LAT_NVS_KEY, 44.47883);
  wpStorage.putFloat(WP_1_LONG_NVS_KEY, -73.20587);
  wpStorage.putString(WP_1_NAME_NVS_KEY, String("btv"));

  wpStorage.putFloat(WP_2_LAT_NVS_KEY, 43.03033);
  wpStorage.putFloat(WP_2_LONG_NVS_KEY, -72.87210);
  wpStorage.putString(WP_2_NAME_NVS_KEY, String("cabin"));

  wpStorage.putFloat(WP_3_LAT_NVS_KEY, 40.36240);
  wpStorage.putFloat(WP_3_LONG_NVS_KEY, -74.04034);
  wpStorage.putString(WP_3_NAME_NVS_KEY, String("fair haven"));

  wpStorage.putFloat(WP_4_LAT_NVS_KEY, -100);
  wpStorage.putFloat(WP_4_LONG_NVS_KEY, -100);
  wpStorage.putString(WP_4_NAME_NVS_KEY, String(" "));

  wpStorage.putFloat(WP_5_LAT_NVS_KEY, -100);
  wpStorage.putFloat(WP_5_LONG_NVS_KEY, -100);
  wpStorage.putString(WP_5_NAME_NVS_KEY, String(" "));
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
  Serial.printf("uptime: %d | io: <%d Hz> (%d) | gps: <%d Hz> (%d) | display: <%d Hz> (%d) \n",
                uptime, taskRefreshRate.at(0), debugger.ioTaskCount, taskRefreshRate.at(1), debugger.gpsTaskCount,
                taskRefreshRate.at(2), debugger.displayTaskCount);

  // update counters
  debugger.ioTaskPreviousCount = debugger.ioTaskCount;
  debugger.gpsTaskPreviousCount = debugger.gpsTaskCount;
  debugger.displayTaskPreviousCount = debugger.displayTaskCount;
}