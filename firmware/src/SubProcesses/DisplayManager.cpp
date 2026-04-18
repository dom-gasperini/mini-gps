/**
 * @file DisplayManager.cpp
 * @brief
 * @date 2026-04-17
 */

/*
===============================================================================================
                                    includes
===============================================================================================
*/

#include <Arduino.h>
#include <rtc.h>

#include <Adafruit_GFX.h>    // graphics library
#include <Adafruit_ST7789.h> // display driver library

#include <Data/ExecutiveData.h>
#include <Data/GpsData.h>

/*
===============================================================================================
                                    definitions
===============================================================================================
*/

#define RADIUS_OF_EARTH 3958.756           // in miles
#define REFRESH_WAYPOINT_VECTOR_DELAY 1000 // in milliseconds

// gps
#define DT_FIX_SOFT_LOST_SIGNAL_TIME_EXPIRED 60.0
#define DT_FIX_SOFT_LOST_SIGNAL_TIME_START 1.0
#define DT_FIX_EXPIRED_TIME 600

// battery info
#define HALF_BATTERY_CAPACITY 50.0 // in %
#define LOW_BATTERY_CAPACITY 20.0  // in %
#define HIGH_BATTERY_VOLTAGE 3.9   // in volts
#define LOW_BATTERY_VOLTAGE 3.4    // in volts

// time keeping
#define BATTERY_CHARGING_INDICATION_DELAY 500        // in milliseconds
#define DISPLAY_REFRESH_RATE_CALCULATE_INTERVAL 1000 // in milliseconds

// system data
#define FIRMWARE_MAJOR 7
#define FIRMWARE_BUILD 3
#define FIRMWARE_NAME "convergence"

/*
===============================================================================================
                                  global variables
===============================================================================================
*/

unsigned long g_lastBatteryChargeIndicatorTime = 0;
int g_previousSelectedWaypoint = -1; // init to non-existant index to force first time compute of vector
unsigned long g_waypointLastComputeTime = 0;
bool g_validWaypointData = false;

unsigned long g_refreshRateCounter = 0;
unsigned long g_refreshRateLastTime = 0;
unsigned long g_colorToggleLastTime = 0;

bool g_drewMoonIcon = false;
bool g_updatedFlashlight = false; // a flag that the flashlight has been enabled
bool g_colorToggleEnable = false; // flag for alternating colors
bool g_previousDebugEnable = false;
bool g_showDebugData = false;

/*
===============================================================================================
                                function declarations
===============================================================================================
*/

void DisplayGpsData(Adafruit_ST7789 displayModule, ExecutiveData *executiveData, GpsData *gpsData);
void DisplayWaypoint(Adafruit_ST7789 displayModule, ExecutiveData *executiveData, GpsData *gpsData);
void DisplaySystem(Adafruit_ST7789 displayModule, ExecutiveData *executiveData);
void DisplayStatusBar(Adafruit_ST7789 displayModule, ExecutiveData *executiveData, GpsData *gpsData);
void DisplayError(Adafruit_ST7789 displayModule, ExecutiveData *executiveData);
void DisplayFlashlight(Adafruit_ST7789 displayModule, ExecutiveData *executiveData);
void DisplaySleepPrompt(Adafruit_ST7789 displayModule, ExecutiveData *executiveData);
void DrawCloud(Adafruit_ST7789 displayModule, int x, int y, int size, int color);
void DisplayDebug(Adafruit_ST7789 displayModule, ExecutiveData *executiveData, GpsData *gpsData);
void BitGraphics(Adafruit_ST7789 displayModule);

void BitFillRectangles(Adafruit_ST7789 displayModule, uint16_t color1, uint16_t color2);
void BitDrawCircles(Adafruit_ST7789 displayModule, uint8_t radius, uint16_t color);
void BitRoundRectangles(Adafruit_ST7789 displayModule);

std::pair<uint16_t, uint16_t> FixStatusColorManager(int fixQuality, bool validDate);

float CalculateWaypointDistance(GpsData *gpsData, WaypointCoordinatesType wp);
float CalculateWaypointBearing(GpsData *gpsData, WaypointCoordinatesType wp);

/*
===============================================================================================
                                    functions
===============================================================================================
*/

/**
 *
 */
void DisplayManager(Adafruit_ST7789 displayModule, ExecutiveData *executiveData, GpsData *gpsData)
{
    if (executiveData->getDisplayMode() != executiveData->getPreviousDisplayMode())
    {
        displayModule.fillScreen(ST77XX_BLACK);
        executiveData->setPreviousDisplayMode(executiveData->getDisplayMode());

        // extras
        g_drewMoonIcon = false;
        g_validWaypointData = !executiveData->getWaypoints().empty();
    }

    // status bar
    DisplayStatusBar(displayModule, executiveData, gpsData);

    // main information
    switch (executiveData->getDisplayMode())
    {
    case GPS_MODE:
        DisplayGpsData(displayModule, executiveData, gpsData);
        break;

    case WAYPOINT_MODE:
        DisplayWaypoint(displayModule, executiveData, gpsData);
        break;

    case SYSTEM_MODE:
        DisplaySystem(displayModule, executiveData);
        break;

    case SLEEP_PROMPT_MODE:
        DisplaySleepPrompt(displayModule, executiveData);
        break;

    case FLASHLIGHT_MODE:
        DisplayFlashlight(displayModule, executiveData);
        break;

    default:
        DisplayError(displayModule, executiveData);
        break;
    }
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
void DisplayGpsData(Adafruit_ST7789 displayModule, ExecutiveData *executiveData, GpsData *gpsData)
{
    // debug data
    DisplayDebug(displayModule, executiveData, gpsData);

    // location
    displayModule.setTextSize(2);
    displayModule.setTextColor(ST77XX_CYAN, ST77XX_BLACK);
    if (gpsData->getNumSats() >= MIN_SATS)
    {
        displayModule.setCursor(5, 30);
        displayModule.printf("%.5f", gpsData->getLatitude());

        displayModule.setCursor(130, 30);
        displayModule.printf("%.5f", gpsData->getLongitude());

        displayModule.setCursor(70, 55);
        displayModule.printf("%4.1f ft ", gpsData->getAltitude());
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
    if (gpsData->getNumSats() > MIN_SATS)
    {
        displayModule.printf("%.1fmph ", gpsData->getSpeed());
    }
    else
    {
        displayModule.printf("--.-mph ");
    }

    // heading
    displayModule.setTextColor(ST77XX_BLUE, ST77XX_BLACK);
    displayModule.setCursor(190, 90);
    if (gpsData->getNumSats() >= MIN_SATS && gpsData->getSpeed() > 0)
    {
        displayModule.printf("%03d%c", (int)gpsData->getHeading(), 0xF7);
    }
    else
    {
        displayModule.printf("---%c", 0xF7);
    }

    // date and time
    displayModule.setTextSize(1);
    if (gpsData->getValidDate())
    {
        displayModule.setTextColor(ST77XX_GREEN, ST77XX_BLACK);
        displayModule.setCursor(180, 115);
        displayModule.printf("%04d/%02d/%02d", gpsData->getYear(), gpsData->getMonth(), gpsData->getDay());

        displayModule.setCursor(156, 125);
        displayModule.printf("%02d:%02d:%02d (UTC)", gpsData->getHour(), gpsData->getMinute(), gpsData->getSecond());
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

    if (gpsData->getNumSats() == 0)
    {
        displayModule.setTextColor(ST77XX_RED, ST77XX_BLACK);
    }
    else if (gpsData->getNumSats() > 0 && gpsData->getNumSats() <= MIN_SATS)
    {
        displayModule.setTextColor(ST77XX_ORANGE, ST77XX_BLACK);
    }
    else if (gpsData->getNumSats() > MIN_SATS)
    {
        displayModule.setTextColor(ST77XX_GREEN, ST77XX_BLACK);
    }
    displayModule.printf("%d ", gpsData->getNumSats());

    // time since last fix
    displayModule.setTextSize(1);
    displayModule.setCursor(0, 125);
    displayModule.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
    if (gpsData->getValidDate())
    {
        if (gpsData->getDtLastFix() < 0) // no fix data
        {
            displayModule.setTextColor(ST77XX_ORANGE, ST77XX_BLACK);
            displayModule.printf("no fix yet       ", gpsData->getDtLastFix());
        }
        else // there is valid fix data
        {
            if (gpsData->getDtLastFix() > 0 && gpsData->getDtLastFix() < DT_FIX_SOFT_LOST_SIGNAL_TIME_START)
            {
                displayModule.setTextColor(ST77XX_GREEN, ST77XX_BLACK);
                displayModule.printf("dt-fix: %.2fs    ", gpsData->getDtLastFix());
            }
            else if (gpsData->getDtLastFix() >= DT_FIX_SOFT_LOST_SIGNAL_TIME_START && gpsData->getDtLastFix() <= DT_FIX_SOFT_LOST_SIGNAL_TIME_EXPIRED)
            {
                displayModule.setTextColor(ST77XX_YELLOW, ST77XX_BLACK);
                displayModule.printf("dt-fix: %.2fs    ", gpsData->getDtLastFix());
            }
            else if (gpsData->getDtLastFix() > DT_FIX_SOFT_LOST_SIGNAL_TIME_EXPIRED && gpsData->getDtLastFix() < DT_FIX_EXPIRED_TIME)
            {
                displayModule.setTextColor(ST77XX_RED, ST77XX_BLACK);
                displayModule.printf("dt-fix: %.2fs    ", gpsData->getDtLastFix());
            }
            else if (gpsData->getDtLastFix() <= DT_FIX_SOFT_LOST_SIGNAL_TIME_EXPIRED)
            {
                displayModule.setTextColor(ST77XX_RED, ST77XX_BLACK);
                displayModule.printf("dt-fix: ages      ", gpsData->getDtLastFix());
            }
        }
    }
    else
    {
        displayModule.setTextColor(ST77XX_RED, ST77XX_BLACK);
        displayModule.printf("no rtc             ");
    }
}

/**
 * @brief the waypoint screen
 * @param displayModule - the display module
 * @param executiveData - the current executive data
 * @param gpsData - the current gps data
 */
void DisplayWaypoint(Adafruit_ST7789 displayModule, ExecutiveData *executiveData, GpsData *gpsData)
{
    // ensure there is valid waypoint data
    if (g_validWaypointData)
    {
        // waypoint selector
        displayModule.setTextSize(1);
        displayModule.setCursor(0, 20);
        displayModule.setTextColor(ST77XX_ORANGE, ST77XX_BLACK);
        displayModule.printf("waypoint: %d", executiveData->getSelectedWaypoint());

        // display waypoint coordinates
        displayModule.setCursor(0, 30);
        displayModule.printf("lat: %.5f", executiveData->getWaypoints().at(executiveData->getSelectedWaypoint()).latitude);

        displayModule.setCursor(0, 40);
        displayModule.printf("long: %.5f", executiveData->getWaypoints().at(executiveData->getSelectedWaypoint()).longitude);

        displayModule.setCursor(0, 50);
        displayModule.printf("name: %s                 ", executiveData->getWaypoints().at(executiveData->getSelectedWaypoint()).name.c_str());

        // display current coordinates
        if (gpsData->getDtLastFix() > 0) // using dt-fix rather than min sat to preserve last fix
        {
            displayModule.setTextColor(ST77XX_BLUE, ST77XX_BLACK);
            displayModule.setCursor(150, 20);
            displayModule.printf("current:");

            displayModule.setCursor(150, 30);
            displayModule.printf("lat: %.5f", gpsData->getLatitude());

            displayModule.setCursor(150, 40);
            displayModule.printf("long: %.5f", gpsData->getLongitude());
        }
        else
        {
            displayModule.setTextColor(ST77XX_BLUE, ST77XX_BLACK);
            displayModule.setCursor(150, 20);
            displayModule.printf("current:");

            displayModule.setCursor(150, 30);
            displayModule.printf("lat: --------");

            displayModule.setCursor(150, 40);
            displayModule.printf("long: --------");
        }

        // do time keeping
        bool staleVector = false;
        unsigned long now = millis();
        if (now - g_waypointLastComputeTime >= REFRESH_WAYPOINT_VECTOR_DELAY)
        {
            staleVector = true;
            g_waypointLastComputeTime = now;
        }

        // compute and then show calcuations on select short press
        if ((executiveData->getSelectedWaypoint() != g_previousSelectedWaypoint || staleVector) && gpsData->getDtLastFix() > 0)
        {
            WaypointCoordinatesType waypoint = executiveData->getWaypoints().at(executiveData->getSelectedWaypoint());

            // inits
            displayModule.setTextSize(2);
            displayModule.setCursor(0, 70);
            displayModule.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
            displayModule.printf("vector ->");

            displayModule.setCursor(0, 90);
            displayModule.setTextColor(ST77XX_GREEN, ST77XX_BLACK);
            displayModule.printf("%.2f miles      ", CalculateWaypointDistance(gpsData, waypoint));
            displayModule.setCursor(0, 110);
            displayModule.printf("@ %.0f%c  ", CalculateWaypointBearing(gpsData, waypoint), 0xF7);
        }
        g_previousSelectedWaypoint = executiveData->getSelectedWaypoint();
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
 * @param displayModule - the display module
 * @param executiveData - the current executive data
 */
void DisplaySystem(Adafruit_ST7789 displayModule, ExecutiveData *executiveData)
{
    displayModule.setTextSize(2);
    displayModule.setTextColor(ST77XX_WHITE, ST77XX_BLACK);

    // display battery percent charge
    if (executiveData->getBatteryPercent() >= HALF_BATTERY_CAPACITY)
    {
        displayModule.setTextColor(ST77XX_GREEN, ST77XX_BLACK);
    }
    else if (executiveData->getBatteryPercent() > LOW_BATTERY_CAPACITY)
    {
        displayModule.setTextColor(ST77XX_YELLOW, ST77XX_BLACK);
    }
    else
    {
        displayModule.setTextColor(ST77XX_RED, ST77XX_BLACK);
    }
    displayModule.setCursor(0, 30);
    displayModule.printf("battery: %.1f%% ", executiveData->getBatteryPercent());

    // display battery voltage
    if (executiveData->getBatteryVoltage() >= HIGH_BATTERY_VOLTAGE)
    {
        displayModule.setTextColor(ST77XX_GREEN, ST77XX_BLACK);
    }
    else if (executiveData->getBatteryVoltage() > LOW_BATTERY_VOLTAGE)
    {
        displayModule.setTextColor(ST77XX_YELLOW, ST77XX_BLACK);
    }
    else
    {
        displayModule.setTextColor(ST77XX_RED, ST77XX_BLACK);
    }
    displayModule.setCursor(0, 50);
    displayModule.printf("voltage: %.3fv", executiveData->getBatteryVoltage());

    if (executiveData->getBatteryChargeRate() >= 0.0f)
    {
        displayModule.setTextColor(ST77XX_GREEN, ST77XX_BLACK);
    }
    else
    {
        displayModule.setTextColor(ST77XX_RED, ST77XX_BLACK);
    }
    displayModule.setCursor(0, 70);
    displayModule.printf("rate: %.1f %%/h  ", executiveData->getBatteryChargeRate());

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
    displayModule.printf("firmware: %d.%d \"%s\"", FIRMWARE_MAJOR, FIRMWARE_BUILD, FIRMWARE_NAME);

    // display refresh rate
    g_refreshRateCounter++;
    unsigned long now = millis();
    if (now - g_refreshRateLastTime >= DISPLAY_REFRESH_RATE_CALCULATE_INTERVAL)
    {
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
 * @param displayModule - the display module
 * @param executiveData - the current executive data
 * @param gpsData - the current gps data
 */
void DisplayStatusBar(Adafruit_ST7789 displayModule, ExecutiveData *executiveData, GpsData *gpsData)
{
    // inits
    int textY = 5;

    // --- mode --- //
    String mode;
    int underscoreLength;
    displayModule.setCursor(0, textY);
    displayModule.setTextSize(1);
    switch (executiveData->getDisplayMode())
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
    std::pair<uint16_t, uint16_t> colors = FixStatusColorManager(gpsData->getFixQuality(), gpsData->getValidDate());
    displayModule.setTextColor(colors.first, colors.second);
    displayModule.setTextSize(1);
    displayModule.setCursor(80, textY);
    displayModule.printf("gps");
    // --- gps fix status --- //

    // --- time --- //
    displayModule.setTextSize(1);
    displayModule.setCursor(135, textY);
    if (gpsData->getValidDate())
    {
        displayModule.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
        displayModule.printf("%02d:%02d:%02d", gpsData->getHour(), gpsData->getMinute(), gpsData->getSecond());
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
    String buffer = String((int)executiveData->getBatteryPercent()).substring(0, 3);
    uint16_t strLen = buffer.length();
    uint16_t batterySymbolWidth = (strLen * 8) + 10;               // +10 for % symbol
    uint16_t batterySymbolX = 240 - (batterySymbolWidth + strLen); // screen width - (text + symbol width)
    uint16_t batteryTextX = batterySymbolX + 5;                    // center the text in the battery symbol
    displayModule.setCursor(batteryTextX, textY);
    displayModule.printf("%s%% ", buffer.c_str());

    // draw battery symbol
    if (executiveData->getBatteryChargeRate() > 0.0f)
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
        if (executiveData->getBatteryPercent() > 50.0)
        {
            displayModule.drawRoundRect(batterySymbolX, 2, batterySymbolWidth, 12, 4, ST77XX_GREEN);
        }
        else if (executiveData->getBatteryPercent() > 20.0 && executiveData->getBatteryPercent() <= 50.0)
        {
            displayModule.drawRoundRect(batterySymbolX, 2, batterySymbolWidth, 12, 4, ST77XX_ORANGE);
        }
        else if (executiveData->getBatteryPercent() <= 20.0)
        {
            displayModule.drawRoundRect(batterySymbolX, 2, batterySymbolWidth, 12, 4, ST77XX_RED);
        }
    }
    // --- battery --- //
}

/**
 * @brief display screen that indicates a failure in display modes
 * @param displayModule - the display module
 * @param executiveData - the current executive data
 */
void DisplayError(Adafruit_ST7789 displayModule, ExecutiveData *executiveData)
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
 * @param displayModule - the display module
 * @param executiveData - the current executive data
 */
void DisplayFlashlight(Adafruit_ST7789 displayModule, ExecutiveData *executiveData)
{
    if (executiveData->getFlashlightEnabled())
    {
        if (executiveData->getFlashlightEnabled() != g_updatedFlashlight)
        {
            displayModule.fillScreen(ST77XX_WHITE);
            g_updatedFlashlight = executiveData->getFlashlightEnabled();
        }
    }
    else
    {
        displayModule.setCursor(5, 65);
        displayModule.setTextColor(ST77XX_RED, ST77XX_BLACK);
        displayModule.setTextSize(2);
        displayModule.printf("<] on / off");
        if (executiveData->getFlashlightEnabled() != g_updatedFlashlight)
        {
            displayModule.fillScreen(ST77XX_BLACK);
        }
    }
    g_updatedFlashlight = executiveData->getFlashlightEnabled();
}

/**
 * @brief while device is entering sleep mode popup
 * @param displayModule - the display module
 * @param executiveData - the current executive data
 */
void DisplaySleepPrompt(Adafruit_ST7789 displayModule, ExecutiveData *executiveData)
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
        DrawCloud(displayModule, 205, 115, 40, ST77XX_WHITE);
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
void DrawCloud(Adafruit_ST7789 displayModule, int x, int y, int size, int color)
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
 * @brief show the refresh rates of the display and gps threads
 * @param sd - the current system data
 * @param gps - the current gps data
 */
void DisplayDebug(Adafruit_ST7789 displayModule, ExecutiveData *executiveData, GpsData *gpsData)
{
    // detect state change
    if (g_previousDebugEnable != executiveData->getDisplayDebugEnabled())
    {
        if (executiveData->getDisplayDebugEnabled())
        {
            g_showDebugData = true;
            displayModule.drawRoundRect(105, 75, 75, 35, 5, ST77XX_RED);
        }
        else
        {
            g_showDebugData = false;
            displayModule.setTextColor(ST77XX_BLACK, ST77XX_BLACK);
            displayModule.setTextSize(1);
            displayModule.fillRect(105, 75, 75, 35, ST77XX_BLACK);
        }

        g_previousDebugEnable = executiveData->getDisplayDebugEnabled();
    }

    // display debug data
    if (g_showDebugData)
    {
        g_refreshRateCounter++;
        unsigned long now = millis();
        if (now - g_refreshRateLastTime >= DISPLAY_REFRESH_RATE_CALCULATE_INTERVAL)
        {
            float refreshRate = g_refreshRateCounter * 1000.0 / (now - g_refreshRateLastTime);
            g_refreshRateCounter = 0;
            g_refreshRateLastTime = now;

            displayModule.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
            displayModule.setTextSize(1);
            displayModule.setCursor(110, 80);
            displayModule.printf("d: %.2fHz ", refreshRate);
        }

        // gps refresh rate
        displayModule.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
        displayModule.setTextSize(1);
        displayModule.setCursor(110, 100);
        displayModule.printf("g: %.2fHz ", gpsData->getRefreshRate());
    }
}

/**
 * @brief run built in test on the display
 */
void BitGraphics(Adafruit_ST7789 displayModule)
{
    BitFillRectangles(displayModule, ST77XX_RED, ST77XX_WHITE);
    delay(150);

    BitDrawCircles(displayModule, 10, ST77XX_WHITE);
    delay(150);

    BitRoundRectangles(displayModule);
    delay(150);

    displayModule.fillScreen(ST77XX_BLACK); // ensure display remains dim when backlight is turned on
    delay(25);
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
    case 0:            // no fix
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

void BitFillRectangles(Adafruit_ST7789 displayModule, uint16_t color1, uint16_t color2)
{
    displayModule.fillScreen(ST77XX_BLACK);
    for (int16_t x = displayModule.width() - 1; x > 6; x -= 6)
    {
        displayModule.fillRect(displayModule.width() / 2 - x / 2, displayModule.height() / 2 - x / 2, x, x,
                               color1);
        displayModule.drawRect(displayModule.width() / 2 - x / 2, displayModule.height() / 2 - x / 2, x, x,
                               color2);
    }
}

void BitDrawCircles(Adafruit_ST7789 displayModule, uint8_t radius, uint16_t color)
{
    displayModule.fillScreen(ST77XX_BLACK);
    for (int16_t x = 0; x < displayModule.width() + radius; x += radius * 2)
    {
        for (int16_t y = 0; y < displayModule.height() + radius; y += radius * 2)
        {
            displayModule.drawCircle(x, y, radius, color);
            delay(5);
        }
    }
}

void BitRoundRectangles(Adafruit_ST7789 displayModule)
{
    displayModule.fillScreen(ST77XX_BLACK);
    uint16_t color = 100;
    int i;
    int t;
    for (t = 0; t <= 4; t += 1)
    {
        int x = 0;
        int y = 0;
        int w = displayModule.width() - 2;
        int h = displayModule.height() - 2;
        for (i = 0; i <= 16; i += 1)
        {
            displayModule.drawRoundRect(x, y, w, h, 5, color);
            x += 2;
            y += 3;
            w -= 4;
            h -= 6;
            color += 1100;
            delay(5);
        }
        color += 100;
    }
}

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
float CalculateWaypointDistance(GpsData *gpsData, WaypointCoordinatesType waypoint)
{
    // inits
    double dLat = DegreesToRadians(waypoint.latitude - gpsData->getLatitude());
    double dLon = DegreesToRadians(waypoint.longitude - gpsData->getLongitude());

    // calculate!
    double a = sin(dLat / 2) * sin(dLat / 2) +
               cos(DegreesToRadians(gpsData->getLatitude())) * cos(DegreesToRadians(waypoint.longitude)) *
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
float CalculateWaypointBearing(GpsData *gpsData, WaypointCoordinatesType waypoint)
{
    // inits
    double phi1 = DegreesToRadians(gpsData->getLatitude());
    double phi2 = DegreesToRadians(waypoint.latitude);
    double dLon = DegreesToRadians(waypoint.longitude - gpsData->getLongitude());

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