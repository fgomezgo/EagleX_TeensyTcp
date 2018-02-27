/*
  Eagle X's rover feedback/IMU library Feedback.h - Library for managing feedback signals 
  from encoders, actuators, IMU
*/
#include "Arduino.h"
#include "Feedback.h"



Feedback::Feedback(char LIS3DH_CS[4], char LIS3DH_MOSI, char LIS3DH_MISO, char LIS3DH_CLK){
    for(int i = 0; i < 4; i++){
        _lis[i] = Adafruit_LIS3DH(LIS3DH_CS[i], LIS3DH_MOSI, LIS3DH_MISO, LIS3DH_CLK);
    }
}

bool Feedback::suspensionImuConf(){
     bool flag = false;
     for(int i = 0; i < 4; i++){
         if (! _lis[i].begin(0x18)) {   // change this to 0x19 for alternative i2c address
            flag = true;
        }
        delay(10);
     }
     return flag;
}


void Feedback::suspensionImuUpdate(){
    for(int i = 0; i < 4; i++){
        _lis[i].read();
        if ( abs( ((_lis[i].y / 5095.54) - _suspensionsAngle[i])) > _radsPerChange * 2){
            _suspensionsAngle[i] = (_lis[i].y / 5095.54); 
        } 
        delay(5);
    }
}

float Feedback::getSuspensionRB(){
    suspensionImuUpdate();
    return _suspensionsAngle[0];
}

float Feedback::getSuspensionRF(){
    suspensionImuUpdate();
    return _suspensionsAngle[1];
}

float Feedback::getSuspensionLF(){
    suspensionImuUpdate();
    return _suspensionsAngle[2];
}

float Feedback::getSuspensionLB(){
    suspensionImuUpdate();
    return _suspensionsAngle[3];
}