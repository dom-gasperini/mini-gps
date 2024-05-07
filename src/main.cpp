/**
 * @file main.cpp
 * @author dom gasperini
 * @brief mini-gps
 * @version 1.0
 * @date 2024-05-06
 *
 * @ref https://espressif-docs.readthedocs-hosted.com/projects/arduino-esp32/en/latest/libraries.html#apis      (api and hal docs)
 * @ref https://docs.espressif.com/projects/esp-idf/en/latest/esp32/_images/esp32-devkitC-v4-pinout.png         (pinout & overview)
 * @ref https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/system/freertos_idf.html      (FreeRTOS for ESP32 docs)
 */

/*
===============================================================================================
                                    includes
===============================================================================================
*/

// core
#include <Arduino.h>
#include "rtc.h"
#include "rtc_clk_common.h"
#include <vector>
#include <Wire.h>
#include <SPI.h>

// functionality
#include "SoftwareSerial.h"
#include "TinyGPSPlus.h"
#include <TFT_eSPI.h>

// custom headers
#include <data_types.h>
#include <pin_config.h>

/*
===============================================================================================
                                    definitions
===============================================================================================
*/

// display
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

// gps
#define GPS_BAUD 9600
#define MAX_SATELLITES 40

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
TinyGPSCustom totalGPGSVMessages(gps, "GPGSV", 1); // $GPGSV sentence, first element
TinyGPSCustom messageNumber(gps, "GPGSV", 2);      // $GPGSV sentence, second element
TinyGPSCustom satellitesInView(gps, "GPGSV", 3);   // $GPGSV sentence, third element

TinyGPSCustom satNumber[4];
TinyGPSCustom elevation[4];
TinyGPSCustom azimuth[4];
TinyGPSCustom snr[4];

struct
{
  bool active;
  int elevation;
  int azimuth;
  int snr;
} sats[MAX_SATELLITES];

// serial connection to the GPS device
SoftwareSerial ss(GPS_RX, GPS_TX);

// display
TFT_eSPI tft = TFT_eSPI();
int refreshCounter = 0;

/*
===============================================================================================
                                  function declarations
===============================================================================================
*/

void UpdateDisplay();
void UpdateGPS();
float calculateSignalStrength();

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

  Serial.printf("display init [ success ]\n");
  // -------------------------------------------------------------------------- //

  // -------------------------- initialize gps -------------------------------- //
  // init sat trackers
  for (int i = 0; i < 4; ++i)
  {
    satNumber[i].begin(gps, "GPGSV", 4 + 4 * i); // offsets 4, 8, 12, 16
    elevation[i].begin(gps, "GPGSV", 5 + 4 * i); // offsets 5, 9, 13, 17
    azimuth[i].begin(gps, "GPGSV", 6 + 4 * i);   // offsets 6, 10, 14, 18
    snr[i].begin(gps, "GPGSV", 7 + 4 * i);       // offsets 7, 11, 15, 19
  }

  ss.begin(GPS_BAUD);
  Serial.printf("gps lib version: %s\n", TinyGPSPlus::libraryVersion());

  Serial.printf("gps init [ success ]\n");
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
  while (ss.available() > 0)
  {
    if (gps.encode(ss.read()))
    {
      UpdateGPS();
    }
  }

  if (millis() > 5000 && gps.charsProcessed() < 10)
  {
    Serial.println(F("No GPS detected: check wiring."));
    while (true)
      ;
  }

  // update display
  UpdateDisplay();
}

/*
===============================================================================================
                                           functions
===============================================================================================
*/

/**
 *
 */
void UpdateGPS()
{
  if (gps.satellites.isValid())
  {
    data.numSats = gps.satellites.value();

    if (data.numSats == 0)
    {
      data.connected = false;
    }
    else
    {
      data.connected = true;
    }
  }

  if (gps.location.isValid())
  {

    data.latitude = gps.location.lat();
    data.longitude = gps.location.lng();
    data.altitude = gps.altitude.feet();
  }

  if (gps.speed.isValid())
  {
    data.speed = gps.speed.mph();
  }

  if (gps.date.isValid())
  {
    data.year = gps.date.year();
    data.month = gps.date.month();
    data.day = gps.date.day();
  }

  // test for active rtc
  if (gps.date.month() == 0 && gps.date.day() == 0)
  {
    data.rtcDataValid = false;
  }
  else
  {
    data.rtcDataValid = true;
  }

  if (gps.time.isValid())
  {
    data.hour = gps.time.hour();
    data.minute = gps.time.minute();
    data.second = gps.time.second();
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

    Serial.println();
  }
}

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
    tft.drawRect(0, 185, 320, 45, TFT_MAGENTA);

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
      tft.printf("latitude:  ---.---");

      tft.setCursor(5, 50);
      tft.printf("longitude: ---.---");

      tft.setCursor(5, 70);
      tft.printf("altitude:  ---.---");
    }

    // speed data
    tft.setTextColor(TFT_PURPLE, TFT_BLACK, true);
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
    if (gps.date.isValid())
    {
      tft.printf("date: %d/%d/%d", data.year, data.month, data.day);
    }
    else
    {
      tft.printf("date: __/__/__");
    }

    // time
    tft.setCursor(5, 150);
    if (gps.time.isValid())
    {
      tft.printf("time: %d:%d:%d (UTC) ", data.hour, data.minute, data.second); // intentional space at the end to fix visual artifacts with seconds
    }
    else
    {
      tft.printf("time: ---:---:--- (UTC) ", data.hour, data.minute, data.second);
    }

    // connection data
    if (data.connected)
    {
      tft.setCursor(5, 190);
      tft.setTextColor(TFT_GREEN, TFT_BLACK, true);
      tft.printf("satellites connected: %d", data.numSats);
    }
    else
    {
      tft.setCursor(80, 190);
      tft.setTextColor(TFT_RED, TFT_BLACK, true);
      tft.printf("< no signal >");
    }

    // connection stats
    tft.setCursor(5, 210);
    tft.setTextColor(TFT_ORANGE, TFT_BLACK, true);
    if (data.numSats != 0)
    {
      tft.printf("signal strength: %.2f", calculateSignalStrength());
    }
    else
    {
      tft.printf("signal strength: ---", calculateSignalStrength());
    }
  }

  // no connection and no rtc
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

    tft.setTextColor(TFT_RED, TFT_BLACK, true);
    tft.setCursor(30, 5);
    tft.printf("searching...");


    // info
    tft.setTextSize(2);
    tft.setTextColor(TFT_RED, TFT_BLACK, true);
    tft.setCursor(80, 110);
    tft.printf("< no signal >");
    tft.setCursor(40, 130);
    tft.printf("< rtc data invalid >");
  }
}

/**
 *
 */
float calculateSignalStrength()
{
  // inits
  float avg = 0.0f;
  float runSum = 0;

  if (totalGPGSVMessages.isUpdated())
  {
    for (int i = 0; i < 4; ++i)
    {
      int no = atoi(satNumber[i].value());
      // Serial.print(F("SatNumber is ")); Serial.println(no);
      if (no >= 1 && no <= MAX_SATELLITES)
      {
        sats[no - 1].elevation = atoi(elevation[i].value());
        sats[no - 1].azimuth = atoi(azimuth[i].value());
        sats[no - 1].snr = atoi(snr[i].value());
        sats[no - 1].active = true;
      }
    }

    int totalMessages = atoi(totalGPGSVMessages.value());
    int currentMessage = atoi(messageNumber.value());
    if (totalMessages == currentMessage)
    {
      // Serial.print(F("Sats="));
      // Serial.print(gps.satellites.value());
      // Serial.print(F(" Nums="));
      for (int i = 0; i < MAX_SATELLITES; ++i)
        //   if (sats[i].active)
        //   {
        //     Serial.print(i + 1);
        //     Serial.print(F(" "));
        //   }
        // Serial.print(F(" Elevation="));
        // for (int i = 0; i < MAX_SATELLITES; ++i)
        //   if (sats[i].active)
        //   {
        //     Serial.print(sats[i].elevation);
        //     Serial.print(F(" "));
        //   }
        // Serial.print(F(" Azimuth="));
        // for (int i = 0; i < MAX_SATELLITES; ++i)
        //   if (sats[i].active)
        //   {
        //     Serial.print(sats[i].azimuth);
        //     Serial.print(F(" "));
        //   }

        // Serial.print(F(" SNR="));
        for (int i = 0; i < MAX_SATELLITES; ++i)
          if (sats[i].active)
          {
            runSum += sats[i].snr;
            // Serial.print(sats[i].snr);
            // Serial.print(F(" "));
          }
      // Serial.println();

      for (int i = 0; i < MAX_SATELLITES; ++i)
        sats[i].active = false;
    }
  }

  // calcualate average signal strength
  avg = runSum / 4.0; // hardcoded number of sats to parse

  return avg;
}