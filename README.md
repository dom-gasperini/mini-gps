# mini-gps

a gps location device

## details

- down to 3 meters of precision
- realtime determination of velocity accurate to 0.1 knots
- powered by 2 AA batteries (8-10 hours of battery)

## hardware

- esp32 devkit-c
- Adafruit 2.4" ILI9341 display
- Adafruit GPS Module (MTK33x9 Chipset)

## software

- FreeRTOS
- TFT_eSPI
- Adafruit GPS Library

## todo:

### hardware

- consider different display
- apply power supply upgrade

### gui

- increase display of fix precision
- add cardinal direction with heading (ex: NW @ 345 deg)
- consider EST rather than UTC

### firmware

- add last know time of fix to flash memory
- implement deep sleep / soft power state control
- battery precentage and estimated power on time remaining
-
