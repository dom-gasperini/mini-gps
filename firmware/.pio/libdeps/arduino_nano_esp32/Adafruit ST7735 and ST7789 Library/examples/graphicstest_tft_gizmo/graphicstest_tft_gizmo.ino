/**************************************************************************
  This is a library for several Adafruit displays based on ST77* drivers.

  Works with the Adafruit TFT Gizmo
    ----> http://www.adafruit.com/products/4367

  Check out the links above for our tutorials and wiring diagrams.

  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.
  MIT license, all text above must be included in any redistribution
 **************************************************************************/

#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7789.h> // Hardware-specific library for ST7789
#include <SPI.h>

// Because of the limited number of pins available on the Circuit Playground Boards
// Software SPI is used
#define TFT_CS 0
#define TFT_RST -1 // Or set to -1 and connect to Arduino RESET pin
#define TFT_DC 1
#define TFT_BACKLIGHT PIN_A3 // Display backlight pin

// You will need to use Adafruit's CircuitPlayground Express Board Definition
// for Gizmos rather than the Arduino version since there are additional SPI
// ports exposed.
#if (SPI_INTERFACES_COUNT == 1)
SPIClass *spi = &SPI;
#else
SPIClass *spi = &SPI1;
#endif

// OPTION 1 (recommended) is to use the HARDWARE SPI pins, which are unique
// to each board and not reassignable.
Adafruit_ST7789 g_displayModule = Adafruit_ST7789(spi, TFT_CS, TFT_DC, TFT_RST);

// OPTION 2 lets you interface the display using ANY TWO or THREE PINS,
// tradeoff being that performance is not as fast as hardware SPI above.
// #define TFT_MOSI      PIN_WIRE_SDA  // Data out
// #define TFT_SCLK      PIN_WIRE_SCL  // Clock out
// Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST);

float p = 3.1415926;

void setup(void)
{
  Serial.begin(9600);
  Serial.print(F("Hello! Gizmo TFT Test"));

  g_displayModule.init(240, 240); // Init ST7789 240x240
  g_displayModule.setRotation(2);

  Serial.println(F("Initialized"));

  pinMode(TFT_BACKLIGHT, OUTPUT);
  digitalWrite(TFT_BACKLIGHT, HIGH); // Backlight on

  uint16_t time = millis();
  g_displayModule.fillScreen(ST77XX_BLACK);
  time = millis() - time;

  Serial.println(time, DEC);
  delay(500);

  // large block of text
  g_displayModule.fillScreen(ST77XX_BLACK);
  testdrawtext("Lorem ipsum dolor sit amet, consectetur adipiscing elit. Curabitur adipiscing ante sed nibh tincidunt feugiat. Maecenas enim massa, fringilla sed malesuada et, malesuada sit amet turpis. Sed porttitor neque ut ante pretium vitae malesuada nunc bibendum. Nullam aliquet ultrices massa eu hendrerit. Ut sed nisi lorem. In vestibulum purus a tortor imperdiet posuere. ", ST77XX_WHITE);
  delay(1000);

  // tft print function!
  tftPrintTest();
  delay(4000);

  // a single pixel
  g_displayModule.drawPixel(g_displayModule.width() / 2, g_displayModule.height() / 2, ST77XX_GREEN);
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

  g_displayModule.fillScreen(ST77XX_BLACK);
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
  g_displayModule.invertDisplay(true);
  delay(500);
  g_displayModule.invertDisplay(false);
  delay(500);
}

void testlines(uint16_t color)
{
  g_displayModule.fillScreen(ST77XX_BLACK);
  for (int16_t x = 0; x < g_displayModule.width(); x += 6)
  {
    g_displayModule.drawLine(0, 0, x, g_displayModule.height() - 1, color);
    delay(0);
  }
  for (int16_t y = 0; y < g_displayModule.height(); y += 6)
  {
    g_displayModule.drawLine(0, 0, g_displayModule.width() - 1, y, color);
    delay(0);
  }

  g_displayModule.fillScreen(ST77XX_BLACK);
  for (int16_t x = 0; x < g_displayModule.width(); x += 6)
  {
    g_displayModule.drawLine(g_displayModule.width() - 1, 0, x, g_displayModule.height() - 1, color);
    delay(0);
  }
  for (int16_t y = 0; y < g_displayModule.height(); y += 6)
  {
    g_displayModule.drawLine(g_displayModule.width() - 1, 0, 0, y, color);
    delay(0);
  }

  g_displayModule.fillScreen(ST77XX_BLACK);
  for (int16_t x = 0; x < g_displayModule.width(); x += 6)
  {
    g_displayModule.drawLine(0, g_displayModule.height() - 1, x, 0, color);
    delay(0);
  }
  for (int16_t y = 0; y < g_displayModule.height(); y += 6)
  {
    g_displayModule.drawLine(0, g_displayModule.height() - 1, g_displayModule.width() - 1, y, color);
    delay(0);
  }

  g_displayModule.fillScreen(ST77XX_BLACK);
  for (int16_t x = 0; x < g_displayModule.width(); x += 6)
  {
    g_displayModule.drawLine(g_displayModule.width() - 1, g_displayModule.height() - 1, x, 0, color);
    delay(0);
  }
  for (int16_t y = 0; y < g_displayModule.height(); y += 6)
  {
    g_displayModule.drawLine(g_displayModule.width() - 1, g_displayModule.height() - 1, 0, y, color);
    delay(0);
  }
}

void testdrawtext(char *text, uint16_t color)
{
  g_displayModule.setCursor(0, 0);
  g_displayModule.setTextColor(color);
  g_displayModule.setTextWrap(true);
  g_displayModule.print(text);
}

void testfastlines(uint16_t color1, uint16_t color2)
{
  g_displayModule.fillScreen(ST77XX_BLACK);
  for (int16_t y = 0; y < g_displayModule.height(); y += 5)
  {
    g_displayModule.drawFastHLine(0, y, g_displayModule.width(), color1);
  }
  for (int16_t x = 0; x < g_displayModule.width(); x += 5)
  {
    g_displayModule.drawFastVLine(x, 0, g_displayModule.height(), color2);
  }
}

void testdrawrects(uint16_t color)
{
  g_displayModule.fillScreen(ST77XX_BLACK);
  for (int16_t x = 0; x < g_displayModule.width(); x += 6)
  {
    g_displayModule.drawRect(g_displayModule.width() / 2 - x / 2, g_displayModule.height() / 2 - x / 2, x, x, color);
  }
}

void testfillrects(uint16_t color1, uint16_t color2)
{
  g_displayModule.fillScreen(ST77XX_BLACK);
  for (int16_t x = g_displayModule.width() - 1; x > 6; x -= 6)
  {
    g_displayModule.fillRect(g_displayModule.width() / 2 - x / 2, g_displayModule.height() / 2 - x / 2, x, x, color1);
    g_displayModule.drawRect(g_displayModule.width() / 2 - x / 2, g_displayModule.height() / 2 - x / 2, x, x, color2);
  }
}

void testfillcircles(uint8_t radius, uint16_t color)
{
  for (int16_t x = radius; x < g_displayModule.width(); x += radius * 2)
  {
    for (int16_t y = radius; y < g_displayModule.height(); y += radius * 2)
    {
      g_displayModule.fillCircle(x, y, radius, color);
    }
  }
}

void testdrawcircles(uint8_t radius, uint16_t color)
{
  for (int16_t x = 0; x < g_displayModule.width() + radius; x += radius * 2)
  {
    for (int16_t y = 0; y < g_displayModule.height() + radius; y += radius * 2)
    {
      g_displayModule.drawCircle(x, y, radius, color);
    }
  }
}

void testtriangles()
{
  g_displayModule.fillScreen(ST77XX_BLACK);
  uint16_t color = 0xF800;
  int t;
  int w = g_displayModule.width() / 2;
  int x = g_displayModule.height() - 1;
  int y = 0;
  int z = g_displayModule.width();
  for (t = 0; t <= 15; t++)
  {
    g_displayModule.drawTriangle(w, y, y, x, z, x, color);
    x -= 4;
    y += 4;
    z -= 4;
    color += 100;
  }
}

void testroundrects()
{
  g_displayModule.fillScreen(ST77XX_BLACK);
  uint16_t color = 100;
  int i;
  int t;
  for (t = 0; t <= 4; t += 1)
  {
    int x = 0;
    int y = 0;
    int w = g_displayModule.width() - 2;
    int h = g_displayModule.height() - 2;
    for (i = 0; i <= 16; i += 1)
    {
      g_displayModule.drawRoundRect(x, y, w, h, 5, color);
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
  g_displayModule.setTextWrap(false);
  g_displayModule.fillScreen(ST77XX_BLACK);
  g_displayModule.setCursor(0, 30);
  g_displayModule.setTextColor(ST77XX_RED);
  g_displayModule.setTextSize(1);
  g_displayModule.println("Hello World!");
  g_displayModule.setTextColor(ST77XX_YELLOW);
  g_displayModule.setTextSize(2);
  g_displayModule.println("Hello World!");
  g_displayModule.setTextColor(ST77XX_GREEN);
  g_displayModule.setTextSize(3);
  g_displayModule.println("Hello World!");
  g_displayModule.setTextColor(ST77XX_BLUE);
  g_displayModule.setTextSize(4);
  g_displayModule.print(1234.567);
  delay(1500);
  g_displayModule.setCursor(0, 0);
  g_displayModule.fillScreen(ST77XX_BLACK);
  g_displayModule.setTextColor(ST77XX_WHITE);
  g_displayModule.setTextSize(0);
  g_displayModule.println("Hello World!");
  g_displayModule.setTextSize(1);
  g_displayModule.setTextColor(ST77XX_GREEN);
  g_displayModule.print(p, 6);
  g_displayModule.println(" Want pi?");
  g_displayModule.println(" ");
  g_displayModule.print(8675309, HEX); // print 8,675,309 out in HEX!
  g_displayModule.println(" Print HEX!");
  g_displayModule.println(" ");
  g_displayModule.setTextColor(ST77XX_WHITE);
  g_displayModule.println("Sketch has been");
  g_displayModule.println("running for: ");
  g_displayModule.setTextColor(ST77XX_MAGENTA);
  g_displayModule.print(millis() / 1000);
  g_displayModule.setTextColor(ST77XX_WHITE);
  g_displayModule.print(" seconds.");
}

void mediabuttons()
{
  // play
  g_displayModule.fillScreen(ST77XX_BLACK);
  g_displayModule.fillRoundRect(25, 10, 78, 60, 8, ST77XX_WHITE);
  g_displayModule.fillTriangle(42, 20, 42, 60, 90, 40, ST77XX_RED);
  delay(500);
  // pause
  g_displayModule.fillRoundRect(25, 90, 78, 60, 8, ST77XX_WHITE);
  g_displayModule.fillRoundRect(39, 98, 20, 45, 5, ST77XX_GREEN);
  g_displayModule.fillRoundRect(69, 98, 20, 45, 5, ST77XX_GREEN);
  delay(500);
  // play color
  g_displayModule.fillTriangle(42, 20, 42, 60, 90, 40, ST77XX_BLUE);
  delay(50);
  // pause color
  g_displayModule.fillRoundRect(39, 98, 20, 45, 5, ST77XX_RED);
  g_displayModule.fillRoundRect(69, 98, 20, 45, 5, ST77XX_RED);
  // play color
  g_displayModule.fillTriangle(42, 20, 42, 60, 90, 40, ST77XX_GREEN);
}
