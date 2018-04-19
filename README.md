## Marine Barometer with bonus temperature and humidity
## NMEA2000 CAN Bus

Environment sensor sends ressure, humidity and temperature to N2K bus.
e-Paper scrolling graph or Barograph displays log of pressure, humidity and temperature, displays 48h of readings.

## Power saving
On bus power send readings once per second, update display evey 5 mins.
On battery power take readings based on X scale and fill buffer, update display every 15 mins. 
When battery is below 3.2v log pressure but don't update display

- Light sleep ~16ma
- Read sensor ~20ma
- Display update ~50ma`

## Tested hardware
* ESP32 (NANO32)
* BME280 I2C
* SN65HVD230D CAN tranceiver
* WaveShare 2.13 e-Paper display
* 120K, 22K resistors for vbatt monitoring

## ToDo
* Monitor CAN power/ charging voltage for battery switching
* Finish sleep modes for power saving on battery
* move circular array to RTC SRAM for deep sleep
* allow multiple X scales 12h,24h,48h
* touch sensor buttons for X scale Y scale
