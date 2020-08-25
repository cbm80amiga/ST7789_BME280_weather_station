# ST7789_BME280_weather_station
Mini weather station

YouTube video:

https://youtu.be/MBehHqti35w


## Features

- all 3 values (temperature, humidity and pressure) are displayed on one screen
- 1 button is used to switch display mode
- graph with values from last 48 hours
- software generated 7-segment font saves a lot of memory necessary for high resolution font
- BME280 is connected via I2C
- ST7789 display is connected via SPI

## Notes

- Required: DigiFont and RREFont libraries (available on my GitHub) and Adafruit_BME280 
- In case of compatibility issues use Arduino IDE 1.6.5 
- Use only ENABLE_RRE_16B = 1 in RREFont.h, other ENABLEs should be 0 to save memory

If you find it useful and want to buy me a coffee or a beer:

https://www.paypal.me/cbm80amiga
