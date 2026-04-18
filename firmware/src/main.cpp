/**
 * @file main.cpp
 * @author dom gasperini
 * @brief mini gps
 * @version 7
 * @date 2026-04-17
 *
 * @ref https://learn.adafruit.com/esp32-s3-reverse-tft-feather/overview      (Adafruit ESP32-S3 Reverse TFT Feather docs)
 * @ref https://learn.adafruit.com/adafruit-mini-gps-pa1010d-module/overview  (Adafruit Mini GPS PA1010D Module docs)
 * @ref https://github.com/adafruit/Adafruit_GPS                              (gps library repo)
 * @ref https://github.com/adafruit/Adafruit-GFX-Library                      (graphics library repo)
 */

/*
===============================================================================================
                                    includes
===============================================================================================
*/

// core
#include <Arduino.h>
#include <Wire.h>
#include <Preferences.h> // nvs api
#include <rtc.h>
#include <vector>

// hardware specific
#include <Adafruit_GPS.h>      // gps parsing library
#include <Adafruit_GFX.h>      // graphics library
#include <Adafruit_ST7789.h>   // display driver library
#include <Adafruit_MAX1704X.h> // battery managment chip library

// custom
#include <Data/ExecutiveData.h>
#include <Data/GpsData.h>
#include <Data/IoData.h>

#include <data_types.h>
#include <pin_config.h>

/*
===============================================================================================
                                    definitions
===============================================================================================
*/

// general

// comms
#define BATT_MGMT_I2C_ADDR 0x36
#define GPS_I2C_ADDR 0x10
#define PTMK_STANDBY_MODE "$PMTK161,0*28" // enter standby mode
#define PTMK_BACKUP_MODE "$PMTK225,4*2F"  // command the module to enter backup power mode and maintain date/time/d-fix via coin cell (consumes 15uA from battery)

// system

// nvs
#define NVS_FLASH_WITH_DEFAULT false        // write the default values to the nvs (KEEP DISABLED)
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

#define EXECUTIVE_REFRESH_RATE 2 // in RTOS ticks (1 tick = interrupt at 1 kHz)
#define IO_REFRESH_RATE 2        // in RTOS ticks (1 tick = interrupt at 1 kHz)
#define GPS_REFRESH_RATE 10      // in RTOS ticks (1 tick = interrupt at 1 kHz)
#define DISPLAY_REFRESH_RATE 50  // in RTOS ticks (1 tick = interrupt at 1 kHz)
#define DEBUG_REFRESH_RATE 1000  // in RTOS ticks (1 tick = interrupt at 1 kHz)

// debugging
#define DEBUG_BOOT_DELAY 1000 // in milliseconds
#define ENABLE_DEBUGGING true // master debug toggle (does not disable boot output)

/*
===============================================================================================
                                  global variables
===============================================================================================
*/

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
Preferences g_wpStorage;

// battery management
Adafruit_MAX17048 *g_batteryModule = new Adafruit_MAX17048();

// gps
Adafruit_GPS *g_gpsModule = new Adafruit_GPS(&Wire);

// display
Adafruit_ST7789 g_displayModule = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST); // 240x135
DisplayModeType g_previousDisplayMode = GPS_MODE;

// data
ExecutiveData *g_executiveData = new ExecutiveData();
GpsData *g_gpsData = new GpsData();
IoData *g_ioData = new IoData();

// time keeping
bool g_wasSleeping = false;

// rtos task handles
TaskHandle_t xHandleExecutive = NULL;
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
void ExecutiveTask(void *pvParameters);
void IoTask(void *pvParameters);
void GpsTask(void *pvParameters);
void DisplayTask(void *pvParameters);
void DebugTask(void *pvParameters);

void StateManager(ExecutiveData *executiveData, IoData *ioData);
void ButtonManager(IoData *ioData);
void BatteryManager(Adafruit_MAX17048 *batteryModule, ExecutiveData *executiveData);
void GpsManager(Adafruit_GPS *gpsModule, GpsData *gpsData);
void DisplayManager(Adafruit_ST7789 displayModule, ExecutiveData *executiveData, GpsData *gpsData);

// helpers
void BitGraphics(Adafruit_ST7789 displayModule);
String TaskStateToString(eTaskState state);
void FlashNVS();

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
  Serial.printf("io:\n");
  // power
  pinMode(TFT_I2C_POWER, OUTPUT);
  digitalWrite(TFT_I2C_POWER, HIGH); // turn on power to the display and gps module

  // gps
  // pinMode(GPS_WAKE_PIN, OUTPUT);
  // digitalWrite(GPS_WAKE_PIN, HIGH); // wake module from backup mode

  // tft
  pinMode(TFT_BACKLITE, OUTPUT);

  // sleep
  gpio_deep_sleep_hold_en();
  esp_sleep_enable_ext0_wakeup((gpio_num_t)RETURN_BUTTON, HIGH); // use the return button to wake from deep sleep

  // io
  pinMode(OPTION_BUTTON, INPUT);
  pinMode(RETURN_BUTTON, INPUT); // the select button is set to an input by default, adding it here manually breaks its functionality

  Serial.printf("\tgpio init [ success ]\n");
  setup.ioActive = true;
  // ------------------------------------------------------------------------- //

  // -------------------- initialize non-volitile storage -------------------- //
  Serial.printf("nvs:\n");
  if (g_wpStorage.begin("wp-storage", false)) // true = read only | false = read/write
  {
    Serial.printf("\tnvs init: [ success ]\n");

    // flash nvs
    if (NVS_FLASH_WITH_DEFAULT)
    {
      FlashNVS();
      Serial.printf("\tnvs flashed with default data!\n");
    }

    // read nvs
    float tmpLat, tmpLong;
    String tmpName;
    tmpLat = g_wpStorage.getFloat(NVS_WP_1_LAT_KEY, -99);
    tmpLong = g_wpStorage.getFloat(NVS_WP_1_LONG_KEY, -99);
    tmpName = g_wpStorage.getString(NVS_WP_1_NAME_KEY, " ");
    WaypointCoordinatesType wp1 = {tmpLat, tmpLong, tmpName};
    Serial.printf("wp1: %s\n", wp1.name.c_str());

    tmpLat = g_wpStorage.getFloat(NVS_WP_2_LAT_KEY, -99);
    tmpLong = g_wpStorage.getFloat(NVS_WP_2_LONG_KEY, -99);
    tmpName = g_wpStorage.getString(NVS_WP_2_NAME_KEY, " ");
    WaypointCoordinatesType wp2 = {tmpLat, tmpLong, tmpName};

    tmpLat = g_wpStorage.getFloat(NVS_WP_3_LAT_KEY, -99);
    tmpLong = g_wpStorage.getFloat(NVS_WP_3_LONG_KEY, -99);
    tmpName = g_wpStorage.getString(NVS_WP_3_NAME_KEY, " ");
    WaypointCoordinatesType wp3 = {tmpLat, tmpLong, tmpName};

    tmpLat = g_wpStorage.getFloat(NVS_WP_4_LAT_KEY, -99);
    tmpLong = g_wpStorage.getFloat(NVS_WP_4_LONG_KEY, -99);
    tmpName = g_wpStorage.getString(NVS_WP_4_NAME_KEY, " ");
    WaypointCoordinatesType wp4 = {tmpLat, tmpLong, tmpName};

    tmpLat = g_wpStorage.getFloat(NVS_WP_5_LAT_KEY, -99);
    tmpLong = g_wpStorage.getFloat(NVS_WP_5_LONG_KEY, -99);
    tmpName = g_wpStorage.getString(NVS_WP_5_NAME_KEY, " ");
    WaypointCoordinatesType wp5 = {tmpLat, tmpLong, tmpName};

    g_wasSleeping = g_wpStorage.getBool(NVS_WAS_SLEEPING_KEY, false);
    g_wpStorage.putBool(NVS_WAS_SLEEPING_KEY, false);

    // save to dynamic memory
    std::vector<WaypointCoordinatesType> tmpWps = {wp1, wp2, wp3, wp4, wp5};
    g_executiveData->setWaypoints(tmpWps);
  }
  else
  {
    Serial.printf("\tnvs init: [ failed ]\n");
  }
  // ------------------------------------------------------------------------- //

  // -------------------------- initialize battery --------------------------- //
  Serial.printf("battery:\n");
  if (g_batteryModule->begin())
  {
    Serial.printf("\tbattery init [ success ]\n");
    g_batteryModule->enableSleep(true);

    // collect information
    uint8_t chipId = g_batteryModule->getChipID();
    g_executiveData->setBatteryPercent(g_batteryModule->cellPercent());
    g_executiveData->setBatteryVoltage(g_batteryModule->cellVoltage());
    Serial.printf("\tchip id: 0x%x\n", chipId);
  }
  else
  {
    Serial.printf("\tbattery init [ failed ]\n");
  }
  // ------------------------------------------------------------------------- //

  // -------------------------- initialize display --------------------------- //
  Serial.printf("display:\n");
  g_displayModule.init(135, 240); // set display size
  g_displayModule.setRotation(3);
  g_displayModule.fillScreen(ST77XX_BLACK); // ensure display remains dim when backlight is turned on
  digitalWrite(TFT_BACKLITE, HIGH);         // turn on display backlight

  // boot screen
  g_displayModule.setTextSize(1);
  g_displayModule.setTextColor(ST77XX_GREEN, ST77XX_BLACK);
  g_displayModule.setCursor(55, 60);
  g_displayModule.printf("[> booting mini gps <]");
  delay(1000);

  // b.i.t.
  if (!g_wasSleeping)
  {
    Serial.printf("\tstarting b.i.t.\n");
    BitGraphics(g_displayModule);
  }
  else
  {
    g_displayModule.fillScreen(ST77XX_BLACK); // reset screen for main program
  }

  setup.displayActive = true;
  Serial.printf("\tdisplay init [ success ]\n");
  // -------------------------------------------------------------------------- //

  // -------------------------- initialize gps -------------------------------- //
  Serial.printf("gps:\n");
  if (g_gpsModule->begin(GPS_I2C_ADDR))
  {
    g_gpsModule->sendCommand(""); // wake from standby mode by sending a byte

    // set data filter
    g_gpsModule->sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);

    // set update message rate
    g_gpsModule->sendCommand(PMTK_SET_NMEA_UPDATE_10HZ);

    // set position fix rate
    g_gpsModule->sendCommand(PMTK_API_SET_FIX_CTL_5HZ);

    Serial.printf("\tgps init [ success ]\n");
    setup.gpsActive = true;
  }
  else
  {
    Serial.printf("\tgps init [ failed ]\n");
  }
  // -------------------------------------------------------------------------- //

  // ------------------------------- scheduler -------------------------------- //
  // start tasks
  xTaskCreatePinnedToCore(ExecutiveTask, "executive", TASK_STACK_SIZE, NULL, 1, &xHandleExecutive, EXECUTIVE_CORE);

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

  Serial.printf("\n|--- end setup ---|\n\n");
  // -------------------------------------------------------------------------- //
}

/*
===============================================================================================
                                rtos task functions
===============================================================================================
*/

/**
 *
 */
void ExecutiveTask(void *pvParameters)
{
  // inits
  const TickType_t xFrequency = pdMS_TO_TICKS(EXECUTIVE_REFRESH_RATE);
  TickType_t taskLastWakeTick = xTaskGetTickCount();

  for (;;)
  {
    // limit task refresh rate
    vTaskDelayUntil(&taskLastWakeTick, xFrequency);

    StateManager(g_executiveData, g_ioData);

    // --- sleep logic --- //
    // if (g_executiveData->getSleepModeEnable())
    // {
    //   // turn off display and gps module power
    //   // digitalWrite(GPS_WAKE_PIN, LOW);
    //   // gpsModule.sendCommand(PTMK_BACKUP_MODE); // backup power command, cannot be awkoen via softare
    //   g_gpsModule.sendCommand(PTMK_STANDBY_MODE); // can be awoken from software
    //   g_batteryModule->sleep(true);
    //   g_wpStorage.putBool(NVS_WAS_SLEEPING_KEY, true);
    //   g_wpStorage.end();
    //   digitalWrite(TFT_I2C_POWER, LOW);

    //   esp_deep_sleep_start();
    // }
    // --- sleep logic --- //

    // debugging
    if (g_debugger.debugEnabled)
    {
      g_debugger.executiveTaskCount++;
    }
  }
}
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

    BatteryManager(g_batteryModule, g_executiveData);
    ButtonManager(g_ioData);

    // ----------------------------------------------- //

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

    GpsManager(g_gpsModule, g_gpsData);

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

  for (;;)
  {
    // limit task refresh rate
    vTaskDelayUntil(&taskLastWakeTick, xFrequency);

    DisplayManager(g_displayModule, g_executiveData, g_gpsData);

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
 * @brief default values to flash the nvs with
 */
void FlashNVS()
{
  g_wpStorage.putFloat(NVS_WP_1_LAT_KEY, 44.47883);
  g_wpStorage.putFloat(NVS_WP_1_LONG_KEY, -73.206065);
  g_wpStorage.putString(NVS_WP_1_NAME_KEY, String("btv"));

  g_wpStorage.putFloat(NVS_WP_2_LAT_KEY, 43.03033);
  g_wpStorage.putFloat(NVS_WP_2_LONG_KEY, -72.87210);
  g_wpStorage.putString(NVS_WP_2_NAME_KEY, String("cabin"));

  g_wpStorage.putFloat(NVS_WP_3_LAT_KEY, 40.36240);
  g_wpStorage.putFloat(NVS_WP_3_LONG_KEY, -74.04034);
  g_wpStorage.putString(NVS_WP_3_NAME_KEY, String("fair haven"));

  g_wpStorage.putFloat(NVS_WP_4_LAT_KEY, 40.748048);
  g_wpStorage.putFloat(NVS_WP_4_LONG_KEY, -73.986048);
  g_wpStorage.putString(NVS_WP_4_NAME_KEY, String("nyc"));

  g_wpStorage.putFloat(NVS_WP_5_LAT_KEY, -100);
  g_wpStorage.putFloat(NVS_WP_5_LONG_KEY, -100);
  g_wpStorage.putString(NVS_WP_5_NAME_KEY, String(" "));

  g_wpStorage.putBool(NVS_WAS_SLEEPING_KEY, false);
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

  taskRefreshRate.push_back(g_debugger.executiveTaskCount - g_debugger.executiveTaskPreviousCount);
  taskRefreshRate.push_back(g_debugger.ioTaskCount - g_debugger.ioTaskPreviousCount);
  taskRefreshRate.push_back(g_debugger.gpsTaskCount - g_debugger.gpsTaskPreviousCount);
  taskRefreshRate.push_back(g_debugger.displayTaskCount - g_debugger.displayTaskPreviousCount);

  // print
  Serial.printf("uptime: %d | executive: <%d Hz> (%d) | io: <%d Hz> (%d) | gps: <%d Hz> (%d) | display: <%d Hz> (%d) \n",
                uptime, taskRefreshRate.at(0), g_debugger.executiveTaskCount, taskRefreshRate.at(1), g_debugger.ioTaskCount, taskRefreshRate.at(2), g_debugger.gpsTaskCount,
                taskRefreshRate.at(3), g_debugger.displayTaskCount);

  // update counters
  g_debugger.executiveTaskPreviousCount = g_debugger.executiveTaskCount;
  g_debugger.ioTaskPreviousCount = g_debugger.ioTaskCount;
  g_debugger.gpsTaskPreviousCount = g_debugger.gpsTaskCount;
  g_debugger.displayTaskPreviousCount = g_debugger.displayTaskCount;
}