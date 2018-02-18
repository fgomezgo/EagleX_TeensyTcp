/*
  Eagle X's rover IMU library Imu.h - Accelerometer library
*/
#ifndef Imu_h
#define Imu_h

#include "Arduino.h"

//#include <i2c_t3.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_10DOF.h>

//Accelerometers
#include <SPI.h>
#include <Adafruit_LIS3DH.h>

class Imu{
    public:
        Imu(int x);
        void updateData();  //Update data from all sensors
        void initSensors(); //Initialize sensors
        float getRoll();    //Get roll of chassis
        float getPitch();   //Get pitch of chassis
        float getYaw();     //Get yaw of chassis
        float getWheel1();  //Get roation of wheel 1
        float getWheel2();  //Get roation of wheel 2
        float getWheel3();  //Get roation of wheel 3
        float getWheel4();  //Get roation of wheel 4

    private:
        Adafruit_10DOF dof;
        Adafruit_LSM303_Accel_Unified accel;
        Adafruit_LSM303_Mag_Unified   mag;
        Adafruit_LIS3DH lis[4];

        float roll;
        float pitch;
        float yaw;
        float wheels[4];
        float radsPerChange=0.0523;
        const int LIS3DH_CS[4] = {23, 22, 21, 20};
        int i;
};

#endif
