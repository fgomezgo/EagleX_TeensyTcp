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
     //encoders[1]=Encoder (3, 5); //Right 1
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

float Feedback::getHeading(){
    if (_dof.fusionGetOrientation(&_accel_event,&_mag_event, &_orientation)){
        return _orientation.heading;
    }
}
//* Encoders --------------

void Feedback::encodersInit( Encoder *encL1, Encoder *encL2, Encoder *encL3, Encoder *encR1, Encoder *encR2, Encoder *encR3){
    /* Assign a unique ID to the sensors */
    _encL1 = encL1;
    _encL2 = encL2;
    _encL3 = encL3;
    _encR1 = encR1;
    _encR2 = encR2;
    _encR3 = encR3;
   /*
    _encoders[0] = encL1;
    _encoders[1] = encL2;
    _encoders[2] = encL3;
    _encoders[3] = encR1;
    _encoders[4] = encR2;
    _encoders[5] = encR3;*/
}



float Feedback::encodersReadLeft(){
    newPosition[0] = _encL1[0].read();
    newPosition[1] = _encL2[0].read();
    newPosition[2] = _encL3[0].read();
    rads[0] = 0;
    rads[1] = 0;
    rads[2] = 0;
    if (newPosition[0] != oldPosition[0]) {
        rads[0] = abs(oldPosition[0]-newPosition[0])*(pi/477.6384);
        rads[0] = rads[0]*r;
        if(rads[0] > 0.63){
            rads[0] = 0.63;
        }
        oldPosition[0] = newPosition[0];
    }else{
        rads[0]=0;
    }

    if (newPosition[1] != oldPosition[1]) {
        rads[1] = abs(oldPosition[1]-newPosition[1])*(pi/477.6384);
        rads[1] = rads[1]*r;
        if(rads[1] > 0.63){
            rads[1] = 0.63;
        }
        oldPosition[1] = newPosition[1];
    }else{
        rads[1]=0;
    }

    if (newPosition[2] != oldPosition[2]) {
        rads[2] = abs(oldPosition[2]-newPosition[2])*(pi/477.6384);
        rads[2] = rads[2]*r;
        if(rads[2] > 0.63){
            rads[2] = 0.63;
        }
        oldPosition[2] = newPosition[2];
    }else{
        rads[2]=0;
    }
    //! Change this to return the AVG of threee encoders
    //return (rads[0] + rads[1] + rads[2]) / 3.0;
    return (rads[0] + rads[1]) / 2.0;
}

float Feedback::encodersReadRight(){
    newPosition[3] = _encR1[0].read();
    newPosition[4] = _encR2[0].read();
    newPosition[5] = _encR3[0].read();

    if (newPosition[3] != oldPosition[3]) {
        rads[3] = abs(oldPosition[3]-newPosition[3])*(pi/477.6384);
        rads[3] = rads[3]*r;
        if(rads[3] > 0.63){
            rads[3] = 0.63;
        }
        oldPosition[3] = newPosition[3];
    }else{
        rads[3]=0;
    }

    if (newPosition[4] != oldPosition[4]) {
        rads[4] = abs(oldPosition[4]-newPosition[4])*(pi/477.6384);
        rads[4] = rads[4]*r;
        if(rads[4] > 0.63){
            rads[4] = 0.63;
        }
        oldPosition[4] = newPosition[4];
    }else{
        rads[4]=0;
    }

    if (newPosition[5] != oldPosition[5]) {
        rads[5] = abs(oldPosition[5]-newPosition[5])*(pi/477.6384);
        rads[5] = rads[5]*r;
        if(rads[5] > 0.63){
            rads[5] = 0.63;
        }
        oldPosition[5] = newPosition[5];
    }else{
        rads[5]=0;
    }
    //! Change this to return the AVG of threee encoders
    return (rads[3] + rads[4] + rads[5]) / 3.0;
    //return rads[3];
    //return rads[3];
    //return rads[5];
 
}

