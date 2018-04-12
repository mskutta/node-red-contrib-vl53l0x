# Node-RED VL53L0X

VL53L0X is a Time-of-Flight (ToF) ranging sensor.  It is produced by STMicroelectronics: [http://www.st.com/en/imaging-and-photonics-solutions/vl53l0x.html](http://www.st.com/en/imaging-and-photonics-solutions/vl53l0x.html)

The carriers for the sensor can be found here:

1. [Pololu](https://www.pololu.com/product/2490)
1. [Adafruit](https://www.adafruit.com/product/3316)
1. [AliExpress](https://www.aliexpress.com/item/VL53L0X-Time-of-Flight-Laser-Distance-Sensor-Breakout-Module-for-Arduino-VL53L0-VL53L0XV2-Carrier-with-Voltage/32801792612.html)

This package contains a Node-RED Node interface for interacting and retrieving the range distance with the VL53L0X sensor.

## Raspberry PI I2C interface

Communicating to the sensor is done over I2C.
I2C must be enabled on the Raspberry PI.
See the [Raspberry Pi SPI and I2C Tutorial](https://learn.sparkfun.com/tutorials/raspberry-pi-spi-and-i2c-tutorial) for instructions on how to enable I2C.

## Node-RED VL53L0X Node

The Node-RED VL53L0X Node has a few parameters that must be configured:

1. **Bus**: This is the bus number the VL53L0X is connected to.  The default is **ic2-1**.
1. **Bus Address**: This is the address of the VL53L0X.  The VL53L0X has an address of 41 (0x29) by default.
1. **Poll Interval**: This is how often the distance in millimeters is read from the VL53L0X
1. **Name**: (optional)

**start** or **stop** must be passed into the node to start or stop the polling.
Push msg = {payload : start | stop} to start | stop the flow.

## Implementation Notes

* Only doing minimal init and running in "default mode" which is good to about 1200 mm.
* Calibration routines are not executed on the VL53L0X. A simple calibration tweak routine on the range data is used instead.
* Using continuous ranging measurements.