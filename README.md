# data_rocket

## Overview
This Arduino is mounted on a rocket and records data from the flight to an SD card. It has three sensors that it pulls data from, detailed in the `Sensors` section below. The data recording begins after sensor calibration and is ended with the press of a button after the flight.

## Sensors

The following is the list of sensors and data pulled from them.

- [BNO055 Absolute Orientation Sensor](https://www.adafruit.com/product/2472)
  - Acceleration
  - Orientation
- [BMP280 Barometric Pressure & Altitude Sensor](https://www.adafruit.com/product/2651)
  - Altitude
  - Pressure
  - Temperature
- [Adafruit Ultimate GPS Breakout - 66 channel w/10 Hz updates - Version 3](https://www.adafruit.com/product/746)
  - Latitude
  - Longitude

## Other components
- [MicroSD card breakout board+](https://www.adafruit.com/product/254)

## Recorded data format

The sensor data is saved to a file on an SD card with the filename `log_file.txt`. The data is in CSV format with columns in the following order:

`AccelX,AccelY,AccelZ,Roll,Pitch,Heading,Temp,Pressure,Altitude,Lat,Long`

Additional information:
- All data types are floats.
- All measurements are metric (meters, m/s^2, celsius, etc.)

## Macros
- `LOG_TO_SD` (`boolean`)
  Determines whether to log data to SD card. Useful if SD card breakout is not available or only `Serial` is needed.
- `POLL_RATE` (`int`)
  Rate in milliseconds to poll the sensors for new data.

## Roadmap
- [X] Pull data from IMU
- [X] Pull data from barometer
- [X] Pull data from GPS
- [X] Write data to SD Card
- [X] [Include timestamp with data](https://github.com/bqrichards/data_rocket/issues/2)
- [ ] Send data over radio
