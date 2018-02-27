/*
  Eagle X's rover feedback/IMU library Feedback.h - Library for managing feedback signals 
  from encoders, actuators, IMU
*/
#ifndef Feedback_h
#define Feedback_h

#include "Arduino.h"
#include "Adafruit_LIS3DH.h"

class Feedback{
    public:
        Feedback(char LIS3DH_CS[4], char LIS3DH_MOSI, char LIS3DH_MISO, char LIS3DH_CLK);
        bool suspensionImuConf();
        void suspensionImuUpdate();
        float getSuspensionRB();
        float getSuspensionRF();
        float getSuspensionLF();
        float getSuspensionLB();

    private:
        Adafruit_LIS3DH _lis[4];
        float _suspensionsAngle[4];
        float _radsPerChange = 0.0523;
};

#endif