# mini gps

a gps location device

## details

- 3 meters precision
- 5Hz fix update rate
- LiIo (8 hours of battery)

## hardware

- esp32 devkit-c
- Adafruit 2.4" ILI9341 display
- Adafruit GPS Module (MTK33x9 Chipset)

## software

- FreeRTOS
- Adafruit GFZ Library
- Adafruit GPS Library

## todo:

### hardware

- different display

### gui

- increase display of fix precision
- add cardinal direction with heading (ex: NW @ 345 deg)
- consider EST rather than UTC

### firmware

- add last know time of fix to flash memory
- implement deep sleep / soft power state control
- battery precentage and estimated power on time remaining
-
