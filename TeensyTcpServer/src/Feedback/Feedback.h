/*
  Eagle X's rover feedback/IMU library Feedback.h - Library for managing feedback signals 
  from encoders, actuators, IMU
*/
#ifndef Feedback_h
#define Feedback_h

#include "Arduino.h"
#include "Adafruit_LIS3DH.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_BMP085_U.h>
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_10DOF.h>
#include <Encoder.h>


class Feedback{
    public:
        Feedback(char LIS3DH_CS[4], char LIS3DH_MOSI, char LIS3DH_MISO, char LIS3DH_CLK);
        bool suspensionImuConf();
        void suspensionImuUpdate();
        float getSuspensionRB();
        float getSuspensionRF();
        float getSuspensionLF();
        float getSuspensionLB();
        bool chassisImuConf();
        void chassisImuUpdate();
        float getChassisRoll();
        float getChassisPitch();
        float getChassisYaw();
        float getHeading();
        
        //* Encoders
        void encodersInit( Encoder *encL1, Encoder *encL2, Encoder *encL3, Encoder *encR1, Encoder *encR2, Encoder *encR3);
        float encodersRead(char i);
        float encodersReadLeft();
        float encodersReadRight();
        void  encodersReadAll();

    private:
        Adafruit_LIS3DH _lis[4];
        float _suspensionsAngle[7];
        float _radsPerChange = 0.0523;
        /* Assign a unique ID to the sensors */
        Adafruit_10DOF                _dof   = Adafruit_10DOF();
        Adafruit_LSM303_Accel_Unified _accel = Adafruit_LSM303_Accel_Unified(30301);
        Adafruit_LSM303_Mag_Unified   _mag   = Adafruit_LSM303_Mag_Unified(30302);
        Adafruit_BMP085_Unified       _bmp   = Adafruit_BMP085_Unified(18001);
        /* Update this with the correct SLP for accurate altitude measurements */
        float _seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA;

        sensors_event_t _accel_event;
        sensors_event_t _mag_event;
        sensors_event_t _bmp_event;
        sensors_vec_t   _orientation;

        // Encoders
        Encoder *_encL1;
        Encoder *_encL2;
        Encoder *_encL3;
        Encoder *_encR1;
        Encoder *_encR2;
        Encoder *_encR3;
        long oldPosition[6] = {-999, -999, -999, -999, -999, -999};
        long newPosition[6];
        float rads[6];
        float _speedLeft;
        float _speedRight;

        float pi = 3.14159;
        float r = 0.1;

};

#endif