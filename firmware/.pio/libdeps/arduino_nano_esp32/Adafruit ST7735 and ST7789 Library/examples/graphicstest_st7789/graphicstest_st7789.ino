/**************************************************************************
  This is a library for several Adafruit displays based on ST77* drivers.

  This example works with the 1.14" TFT breakout
    ----> https://www.adafruit.com/product/4383
  The 1.3" TFT breakout
    ----> https://www.adafruit.com/product/4313
  The 1.47" TFT breakout
    ----> https://www.adafruit.com/product/5393
  The 1.54" TFT breakout
    ----> https://www.adafruit.com/product/3787
  The 1.69" TFT breakout
    ----> https://www.adafruit.com/product/5206
  The 1.9" TFT breakout
    ----> https://www.adafruit.com/product/5394
  The 2.0" TFT breakout
    ----> https://www.adafruit.com/product/4311


  Check out the links above for our tutorials and wiring diagrams.
  These displays use SPI to communicate, 4 or 5 pins are required to
  interface (RST is optional).

  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.
  MIT license, all text above must be included in any redistribution
 **************************************************************************/

#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7789.h> // Hardware-specific library for ST7789
#include <SPI.h>

#if defined(ARDUINO_FEATHER_ESP32) // Feather Huzzah32
#define TFT_CS 14
#define TFT_RST 15
#define TFT_DC 32

#elif defined(ESP8266)
#define TFT_CS 4
#define TFT_RST 16
#define TFT_DC 5

#else
// For the breakout board, you can use any 2 or 3 pins.
// These pins will also work for the 1.8" TFT shield.
#define TFT_CS 10
#define TFT_RST 9 // Or set to -1 and connect to Arduino RESET pin
#define TFT_DC 8
#endif

// OPTION 1 (recommended) is to use the HARDWARE SPI pins, which are unique
// to each board and not reassignable. For Arduino Uno: MOSI = pin 11 and
// SCLK = pin 13. This is the fastest mode of operation and is required if
// using the breakout board's microSD card.

Adafruit_ST7789 displayModule = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);

// OPTION 2 lets you interface the display using ANY TWO or THREE PINS,
// tradeoff being that performance is not as fast as hardware SPI above.
// #define TFT_MOSI 11  // Data out
// #define TFT_SCLK 13  // Clock out

// Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST);

float p = 3.1415926;

void setup(void)
{
  Serial.begin(9600);
  Serial.print(F("Hello! ST77xx TFT Test"));

  // Use this initializer (uncomment) if using a 1.3" or 1.54" 240x240 TFT:
  displayModule.init(240, 240); // Init ST7789 240x240

  // OR use this initializer (uncomment) if using a 1.69" 280x240 TFT:
  // tft.init(240, 280);           // Init ST7789 280x240

  // OR use this initializer (uncomment) if using a 2.0" 320x240 TFT:
  // tft.init(240, 320);           // Init ST7789 320x240

  // OR use this initializer (uncomment) if using a 1.14" 240x135 TFT:
  // tft.init(135, 240);           // Init ST7789 240x135

  // OR use this initializer (uncomment) if using a 1.47" 172x320 TFT:
  // tft.init(172, 320);           // Init ST7789 172x320

  // OR use this initializer (uncomment) if using a 1.9" 170x320 TFT:
  // tft.init(170, 320);           // Init ST7789 170x320

  // SPI speed defaults to SPI_DEFAULT_FREQ defined in the library, you can override it here
  // Note that speed allowable depends on chip and quality of wiring, if you go too fast, you
  // may end up with a black screen some times, or all the time.
  // tft.setSPISpeed(40000000);
  Serial.println(F("Initialized"));

  uint16_t time = millis();
  displayModule.fillScreen(ST77XX_BLACK);
  time = millis() - time;

  Serial.println(time, DEC);
  delay(500);

  // large block of text
  displayModule.fillScreen(ST77XX_BLACK);
  testdrawtext("Lorem ipsum dolor sit amet, consectetur adipiscing elit. Curabitur adipiscing ante sed nibh tincidunt feugiat. Maecenas enim massa, fringilla sed malesuada et, malesuada sit amet turpis. Sed porttitor neque ut ante pretium vitae malesuada nunc bibendum. Nullam aliquet ultrices massa eu hendrerit. Ut sed nisi lorem. In vestibulum purus a tortor imperdiet posuere. ", ST77XX_WHITE);
  delay(1000);

  // tft print function!
  tftPrintTest();
  delay(4000);

  // a single pixel
  displayModule.drawPixel(displayModule.width() / 2, displayModule.height() / 2, ST77XX_GREEN);
  delay(500);

  // line draw test
  testlines(ST77XX_YELLOW);
  delay(500);

  // optimized lines
  testfastlines(ST77XX_RED, ST77XX_BLUE);
  delay(500);

  testdrawrects(ST77XX_GREEN);
  delay(500);

  testfillrects(ST77XX_YELLOW, ST77XX_MAGENTA);
  delay(500);

  displayModule.fillScreen(ST77XX_BLACK);
  testfillcircles(10, ST77XX_BLUE);
  testdrawcircles(10, ST77XX_WHITE);
  delay(500);

  testroundrects();
  delay(500);

  testtriangles();
  delay(500);

  mediabuttons();
  delay(500);

  Serial.println("done");
  delay(1000);
}

void loop()
{
  displayModule.invertDisplay(true);
  delay(500);
  displayModule.invertDisplay(false);
  delay(500);
}

void testlines(uint16_t color)
{
  displayModule.fillScreen(ST77XX_BLACK);
  for (int16_t x = 0; x < displayModule.width(); x += 6)
  {
    displayModule.drawLine(0, 0, x, displayModule.height() - 1, color);
    delay(0);
  }
  for (int16_t y = 0; y < displayModule.height(); y += 6)
  {
    displayModule.drawLine(0, 0, displayModule.width() - 1, y, color);
    delay(0);
  }

  displayModule.fillScreen(ST77XX_BLACK);
  for (int16_t x = 0; x < displayModule.width(); x += 6)
  {
    displayModule.drawLine(displayModule.width() - 1, 0, x, displayModule.height() - 1, color);
    delay(0);
  }
  for (int16_t y = 0; y < displayModule.height(); y += 6)
  {
    displayModule.drawLine(displayModule.width() - 1, 0, 0, y, color);
    delay(0);
  }

  displayModule.fillScreen(ST77XX_BLACK);
  for (int16_t x = 0; x < displayModule.width(); x += 6)
  {
    displayModule.drawLine(0, displayModule.height() - 1, x, 0, color);
    delay(0);
  }
  for (int16_t y = 0; y < displayModule.height(); y += 6)
  {
    displayModule.drawLine(0, displayModule.height() - 1, displayModule.width() - 1, y, color);
    delay(0);
  }

  displayModule.fillScreen(ST77XX_BLACK);
  for (int16_t x = 0; x < displayModule.width(); x += 6)
  {
    displayModule.drawLine(displayModule.width() - 1, displayModule.height() - 1, x, 0, color);
    delay(0);
  }
  for (int16_t y = 0; y < displayModule.height(); y += 6)
  {
    displayModule.drawLine(displayModule.width() - 1, displayModule.height() - 1, 0, y, color);
    delay(0);
  }
}

void testdrawtext(char *text, uint16_t color)
{
  displayModule.setCursor(0, 0);
  displayModule.setTextColor(color);
  displayModule.setTextWrap(true);
  displayModule.print(text);
}

void testfastlines(uint16_t color1, uint16_t color2)
{
  displayModule.fillScreen(ST77XX_BLACK);
  for (int16_t y = 0; y < displayModule.height(); y += 5)
  {
    displayModule.drawFastHLine(0, y, displayModule.width(), color1);
  }
  for (int16_t x = 0; x < displayModule.width(); x += 5)
  {
    displayModule.drawFastVLine(x, 0, displayModule.height(), color2);
  }
}

void testdrawrects(uint16_t color)
{
  displayModule.fillScreen(ST77XX_BLACK);
  for (int16_t x = 0; x < displayModule.width(); x += 6)
  {
    displayModule.drawRect(displayModule.width() / 2 - x / 2, displayModule.height() / 2 - x / 2, x, x, color);
  }
}

void testfillrects(uint16_t color1, uint16_t color2)
{
  displayModule.fillScreen(ST77XX_BLACK);
  for (int16_t x = displayModule.width() - 1; x > 6; x -= 6)
  {
    displayModule.fillRect(displayModule.width() / 2 - x / 2, displayModule.height() / 2 - x / 2, x, x, color1);
    displayModule.drawRect(displayModule.width() / 2 - x / 2, displayModule.height() / 2 - x / 2, x, x, color2);
  }
}

void testfillcircles(uint8_t radius, uint16_t color)
{
  for (int16_t x = radius; x < displayModule.width(); x += radius * 2)
  {
    for (int16_t y = radius; y < displayModule.height(); y += radius * 2)
    {
      displayModule.fillCircle(x, y, radius, color);
    }
  }
}

void testdrawcircles(uint8_t radius, uint16_t color)
{
  for (int16_t x = 0; x < displayModule.width() + radius; x += radius * 2)
  {
    for (int16_t y = 0; y < displayModule.height() + radius; y += radius * 2)
    {
      displayModule.drawCircle(x, y, radius, color);
    }
  }
}

void testtriangles()
{
  displayModule.fillScreen(ST77XX_BLACK);
  uint16_t color = 0xF800;
  int t;
  int w = displayModule.width() / 2;
  int x = displayModule.height() - 1;
  int y = 0;
  int z = displayModule.width();
  for (t = 0; t <= 15; t++)
  {
    displayModule.drawTriangle(w, y, y, x, z, x, color);
    x -= 4;
    y += 4;
    z -= 4;
    color += 100;
  }
}

void testroundrects()
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
    }
    color += 100;
  }
}

void tftPrintTest()
{
  displayModule.setTextWrap(false);
  displayModule.fillScreen(ST77XX_BLACK);
  displayModule.setCursor(0, 30);
  displayModule.setTextColor(ST77XX_RED);
  displayModule.setTextSize(1);
  displayModule.println("Hello World!");
  displayModule.setTextColor(ST77XX_YELLOW);
  displayModule.setTextSize(2);
  displayModule.println("Hello World!");
  displayModule.setTextColor(ST77XX_GREEN);
  displayModule.setTextSize(3);
  displayModule.println("Hello World!");
  displayModule.setTextColor(ST77XX_BLUE);
  displayModule.setTextSize(4);
  displayModule.print(1234.567);
  delay(1500);
  displayModule.setCursor(0, 0);
  displayModule.fillScreen(ST77XX_BLACK);
  displayModule.setTextColor(ST77XX_WHITE);
  displayModule.setTextSize(0);
  displayModule.println("Hello World!");
  displayModule.setTextSize(1);
  displayModule.setTextColor(ST77XX_GREEN);
  displayModule.print(p, 6);
  displayModule.println(" Want pi?");
  displayModule.println(" ");
  displayModule.print(8675309, HEX); // print 8,675,309 out in HEX!
  displayModule.println(" Print HEX!");
  displayModule.println(" ");
  displayModule.setTextColor(ST77XX_WHITE);
  displayModule.println("Sketch has been");
  displayModule.println("running for: ");
  displayModule.setTextColor(ST77XX_MAGENTA);
  displayModule.print(millis() / 1000);
  displayModule.setTextColor(ST77XX_WHITE);
  displayModule.print(" seconds.");
}

void mediabuttons()
{
  // play
  displayModule.fillScreen(ST77XX_BLACK);
  displayModule.fillRoundRect(25, 10, 78, 60, 8, ST77XX_WHITE);
  displayModule.fillTriangle(42, 20, 42, 60, 90, 40, ST77XX_RED);
  delay(500);
  // pause
  displayModule.fillRoundRect(25, 90, 78, 60, 8, ST77XX_WHITE);
  displayModule.fillRoundRect(39, 98, 20, 45, 5, ST77XX_GREEN);
  displayModule.fillRoundRect(69, 98, 20, 45, 5, ST77XX_GREEN);
  delay(500);
  // play color
  displayModule.fillTriangle(42, 20, 42, 60, 90, 40, ST77XX_BLUE);
  delay(50);
  // pause color
  displayModule.fillRoundRect(39, 98, 20, 45, 5, ST77XX_RED);
  displayModule.fillRoundRect(69, 98, 20, 45, 5, ST77XX_RED);
  // play color
  displayModule.fillTriangle(42, 20, 42, 60, 90, 40, ST77XX_GREEN);
}
