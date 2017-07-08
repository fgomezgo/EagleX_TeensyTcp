# EagleX_TeensyTcp
This repository develops a TCP server and client using the Teensy 3.5 and an Ethernet Board with the ENC28J60 chip and the EtherCard library for Arduino for the EagleX's Rover v3.


This project is part of the Eagle X Control project for the Rover v3

The microcontroller (Teensy 3.5) works as an interface between the rover's main computer (Raspberry Pi 3) and the sensors onboard listed below:

- Adafruit 10 DOF:
	1. LSM303DLHC: 	3-axis accelerometer and 3-axis magnetometer (Learning Guide: http://adafru.it/cXW)
	2. L3GD20:		3-axis gyroscope										(Learning Guide: http://adafru.it/cXX)
	3. BMP180:		Barometric pressure sensor							(Learning Guide: http://adafru.it/cXY)
	
- Adafruit Ultimate GPS v3

- Adafruit Temperature Sensor ()

This microcontroller also controls the rover's motor drivers and robotic arm drivers.

How to connect Teensy 3.5 to Ethernet Board ENC28J60 through SPI communication:

| SPI PIN | Teensy 3.5 | ENC28J60 |
|:-------:|------------|----------|
| CS      | 10 (CS0)   | 7        |
| MOSI    | 11 (MOSI0) | 2        |
| MISO    | 12 (MISO0) | 3        |
| SCK     | 13 (SCK0)  | 1        |
| RST     |            | 8        |
| GND     |            | 9        |
| 3.3 V   |            | 10       |

How to connect 10-DOF to Teensy 3.5 to 10-DOF through I2C communication:

| I2C PIN | Teensy 3.5 | 10-DOF |
|:-------:|:----------:|:------:|
| SDA     | 18 (SDA0)  | SDA    |
| SCL     | 19 (SCL0)  | SCL    |
| GND     |            | GND    |
| 3.3 V   |            | VIN    |



