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

bool Feedback::chassisImuConf(){
    bool flag = false;
    if(!_accel.begin())
    {
        /* There was a problem detecting the LSM303 ... check your connections */
        flag = true;
    }
    if(!_mag.begin())
    {
        /* There was a problem detecting the LSM303 ... check your connections */
        flag = true;
    }
    if(!_bmp.begin())
    {
        /* There was a problem detecting the BMP180 ... check your connections */
        flag = true;
    }
    return flag;
}

void Feedback::chassisImuUpdate(){
    _accel.getEvent(&_accel_event);
    if (_dof.accelGetOrientation(&_accel_event, &_orientation)){
        if ( abs(((-_orientation.roll) / 50) -_suspensionsAngle[4]) > _radsPerChange * 2) { //and abs( ((-orientation.roll)/50)-positions[2])<radsPerChange*4) {
            _suspensionsAngle[4]=(-_orientation.roll) / 50;
        }
        if ( abs( ((-_orientation.pitch)/50) -_suspensionsAngle[5]) > _radsPerChange * 2) { //and abs( ((-orientation.pitch)/50)-positions[1])<radsPerChange*4) {
            _suspensionsAngle[5] = _orientation.pitch / 50;  
        }
    }

    _mag.getEvent(&_mag_event);
    if (_dof.magGetOrientation(SENSOR_AXIS_Z, &_mag_event, &_orientation))
    {    
        if ( abs( ((-_orientation.heading)/50) -_suspensionsAngle[6]) > _radsPerChange * 2){ //and abs( ((-orientation.heading)/50)-positions[0])<radsPerChange*4)
            _suspensionsAngle[6] = -_orientation.heading / 50;  
        }
    }
}

float Feedback::getChassisRoll(){
    chassisImuUpdate();
    return _suspensionsAngle[4];
}

float Feedback::getChassisPitch(){
    chassisImuUpdate();
    return _suspensionsAngle[5];
}

float Feedback::getChassisYaw(){
    chassisImuUpdate();
    return _suspensionsAngle[6];
}