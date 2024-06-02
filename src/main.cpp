/**
 * @file main.cpp
 * @author dom gasperini
 * @brief mini-gps
 * @version 1.0
 * @date 2024-05-08
 *
 * @ref https://espregpsSerialif-docs.readthedocs-hosted.com/projects/arduino-esp32/en/latest/libraries.html#apis      (api and hal docs)
 * @ref https://docs.espregpsSerialif.com/projects/esp-idf/en/latest/esp32/_images/esp32-devkitC-v4-pinout.png         (pinout & overview)
 * @ref https://github.com/mikalhart/TinyGPSPlus                                                                (gps library)
 */

/*
===============================================================================================
                                    includes
===============================================================================================
*/

// core
#include <Arduino.h>

// functionality
#include "SoftwareSerial.h"
#include "TinyGPSPlus.h"
#include <TFT_eSPI.h>

// custom
#include <data_types.h>
#include <pin_config.h>

/*
===============================================================================================
                                    definitions
===============================================================================================
*/

// gps
#define GPS_BAUD 9600

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
    .rtcDataValid = true,

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
};

// gps
TinyGPSPlus gps;

// serial connection to the GPS device
SoftwareSerial gpsSerial(GPS_RX, GPS_TX);

// display
TFT_eSPI tft = TFT_eSPI();
int refreshCounter = 0;
int rtcTestCounter = 0;

/*
===============================================================================================
                                function declarations
===============================================================================================
*/

void UpdateDisplay();
void UpdateGPS();

/*
===============================================================================================
                                        setup
===============================================================================================
*/

void setup()
{
  // --------------------------- initialize serial  --------------------------- //
  Serial.begin(9600);
  Serial.printf("\n\n|--- starting setup ---|\n\n");
  // -------------------------------------------------------------------------- //

  // -------------------------- initialize display --------------------------- //
  tft.init();
  tft.setRotation(3);
  tft.fillScreen(TFT_BLACK);

  Serial.printf("display init [ succegpsSerial ]\n");
  // -------------------------------------------------------------------------- //

  // -------------------------- initialize gps -------------------------------- //
  gpsSerial.begin(GPS_BAUD);
  Serial.printf("gps lib version: %s\n", TinyGPSPlus::libraryVersion());

  Serial.printf("gps init [ succegpsSerial ]\n");
  // -------------------------------------------------------------------------- //

  Serial.printf("\n\n|--- end setup ---|\n\n");
  // --------------------------------------------------------------------------- //
}

/*
===============================================================================================
                                      main loop
===============================================================================================
*/

void loop()
{
  // update data every time a new sentence is correctly encoded
  while (gpsSerial.available() > 0)
  {
    if (gps.encode(gpsSerial.read()))
    {
      UpdateGPS();
    }
  }

  delayMicroseconds(10);

  UpdateDisplay();
}

/*
===============================================================================================
                                      functions
===============================================================================================
*/

/**
 *  @brief read and update gps data
 */
void UpdateGPS()
{
  // connection data
  if (gps.satellites.isValid())
  {
    data.numSats = gps.satellites.value();

    if (data.numSats > 0)
    {
      data.connected = true;
    }
    else
    {
      data.connected = false;
    }
  }

  // collect location data
  if (gps.location.isValid())
  {

    data.latitude = gps.location.lat();
    data.longitude = gps.location.lng();
    data.altitude = gps.altitude.feet();
  }

  // collect speed data
  if (gps.speed.isValid())
  {
    data.speed = gps.speed.mph();
  }

  // collect date data
  if (gps.date.isValid())
  {
    data.year = gps.date.year();
    data.month = gps.date.month();
    data.day = gps.date.day();
  }

  // collect time data
  if (gps.time.isValid())
  {
    data.hour = gps.time.hour();
    data.minute = gps.time.minute();
    data.second = gps.time.second();
  }

  // test for active rtc
  if (data.hour == 0 && data.minute == 0 && data.second == 0)
  {
    rtcTestCounter++;

    if (rtcTestCounter >= 10)
    {
      data.rtcDataValid = false;
    }
  }
  else
  {
    data.rtcDataValid = true;

    if (rtcTestCounter > 0)
    {
      rtcTestCounter = 0;
    }
  }

  if (ENABLE_DEBUGGING)
  {
    Serial.print(F("Location: "));
    Serial.print(gps.location.lat(), 6);
    Serial.print(F(","));
    Serial.print(gps.location.lng(), 6);

    Serial.print(F(" | Speed: "));
    Serial.print(gps.speed.mph());

    Serial.print(F(" | Date/Time: "));
    Serial.print(gps.date.month());
    Serial.print(F("/"));
    Serial.print(gps.date.day());
    Serial.print(F("/"));
    Serial.print(gps.date.year());

    Serial.print(F(" "));

    if (gps.time.hour() < 10)
      Serial.print(F("0"));
    Serial.print(gps.time.hour());
    Serial.print(F(":"));
    if (gps.time.minute() < 10)
      Serial.print(F("0"));
    Serial.print(gps.time.minute());
    Serial.print(F(":"));
    if (gps.time.second() < 10)
      Serial.print(F("0"));
    Serial.print(gps.time.second());
    Serial.print(F("."));
    if (gps.time.centisecond() < 10)
      Serial.print(F("0"));
    Serial.print(gps.time.centisecond());

    Serial.printf(" | signal status: %s", data.connected ? "connected" : "not connected");
    Serial.printf(" | rtc status: %s", data.rtcDataValid ? "valid" : "not valid");
    Serial.printf(" | #sats: %d", gps.satellites.value());

    Serial.println();
  }
}

/**
 *  @brief update the display with the newest information and gps state
 */
void UpdateDisplay()
{
  // clear screen on state change
  if ((data.wasConnected != data.connected) || data.rtcDataValid != data.wasRtcDataValid)
  {
    data.wasConnected = data.connected;
    data.wasRtcDataValid = data.rtcDataValid;
    tft.fillScreen(TFT_BLACK);
  }

  // check connection
  if (data.connected || data.rtcDataValid)
  {
    // display gps information
    tft.setTextSize(3);
    tft.setTextColor(TFT_RED);
    tft.setCursor(0, 0);
    tft.printf("gps data:");

    // outline boxes
    tft.drawRect(0, 25, 320, 150, TFT_DARKCYAN);
    tft.drawRect(0, 185, 320, 55, TFT_MAGENTA);

    // set info font size and color
    tft.setTextSize(2);
    tft.setTextColor(TFT_CYAN, TFT_BLACK, true);

    // location data
    if (gps.location.isValid())
    {
      tft.setCursor(5, 30);
      tft.printf("latitude: %f", data.latitude);

      tft.setCursor(5, 50);
      tft.printf("longitude: %f", data.longitude);

      tft.setCursor(5, 70);
      tft.printf("altitude: %f", data.altitude);
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
    tft.setCursor(5, 100);
    if (gps.speed.isValid())
    {
      tft.printf("speed (mph): %.1f", data.speed);
    }
    else
    {
      tft.printf("speed (mph): ---", data.speed);
    }

    // date
    tft.setTextColor(TFT_GREEN, TFT_BLACK, true);
    tft.setCursor(5, 130);
    if (gps.date.isValid() && data.day != 0)
    {
      tft.printf("date: %d / %d / %d    ", data.year, data.month, data.day);
    }
    else
    {
      tft.printf("date: __ / __ / __    ");
    }

    // time
    tft.setCursor(5, 150);
    if (gps.time.isValid())
    {
      tft.printf("time: %d:%d:%d (UTC)    ", data.hour, data.minute, data.second);
    }
    else
    {
      tft.printf("time: ---:---:--- (UTC)", data.hour, data.minute, data.second);
    }

    // connection data
    if (data.connected)
    {
      tft.setCursor(5, 190);
      tft.setTextColor(TFT_GREEN, TFT_BLACK, true);
      tft.printf("satellites connected: %d   ", data.numSats);
    }
    else
    {
      tft.setCursor(80, 190);
      tft.setTextColor(TFT_RED, TFT_BLACK, true);
      tft.printf("< no signal >");
    }

    // time and date downlink status
    tft.setCursor(10, 220);
    if (data.numSats > 0)
    {
      tft.setTextColor(TFT_WHITE, TFT_GREEN, true);
    }
    else
    {
      tft.setTextColor(TFT_WHITE, TFT_RED, true);
    }
    tft.printf("time data");

    // location downlink status
    tft.setCursor(150, 220);
    if (data.numSats > 3)
    {
      tft.setTextColor(TFT_WHITE, TFT_GREEN, true);
    }
    else
    {
      tft.setTextColor(TFT_WHITE, TFT_RED, true);
    }
    tft.printf("location data");
  }

  // no connection and no valid rtc data
  else
  {
    // flash a circle top left as activity indicator
    if (refreshCounter <= 50)
    {
      tft.fillCircle(10, 10, 10, TFT_GREEN);
    }
    if (refreshCounter >= 50)
    {
      tft.fillCircle(10, 10, 10, TFT_BLACK);
    }
    if (refreshCounter >= 100)
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

  if (ENABLE_DEBUGGING)
  {
    // Serial.printf("display updated!\n");
  }
}
