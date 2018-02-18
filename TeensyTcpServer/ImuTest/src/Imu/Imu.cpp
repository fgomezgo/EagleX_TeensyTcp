/*
  Eagle X's rover IMU library Imu.h - Accelerometer library
*/
#include "Arduino.h"
#include "Imu.h"

// Used for software SPI
#define LIS3DH_CLK 32
#define LIS3DH_MISO 1
#define LIS3DH_MOSI 0



//Joint information
//char *joints[] = {"base_link_to_base_pitch", "base_pitch_to_base_roll", "base_roll_to_chasis", "chasis_to_right", "chasis_to_left", "chasis_to_right2", "chasis_to_left2" };
//float positions[7];

Imu::Imu(int x){
    x=0;
}

void Imu::initSensors() {
    /* Assign a unique ID to the sensors */
    dof   = Adafruit_10DOF();
    accel = Adafruit_LSM303_Accel_Unified(30301);
    mag   = Adafruit_LSM303_Mag_Unified(30302); 

    if (!accel.begin())
    {
    /* There was a problem detecting the LSM303 ... check your connections */
        while (1);
    }
    if (!mag.begin())
    {
    /* There was a problem detecting the LSM303 ... check your connections */
        while (1);
    }

    for(i=0;i<4;i++){
    lis[i] = Adafruit_LIS3DH(LIS3DH_CS[i], LIS3DH_MOSI, LIS3DH_MISO, LIS3DH_CLK);
    if (! lis[i].begin(0x18)) {   
        while (1);
    } 
    lis[i].setRange(LIS3DH_RANGE_4_G);   // 2, 4, 8 or 16 G!
    }

}

void Imu::updateData() {
    sensors_event_t accel_event;
    sensors_event_t mag_event;
    sensors_vec_t   orientation;
    
    /* Calculate pitch and roll from the raw accelerometer data */
    accel.getEvent(&accel_event);
    if (dof.accelGetOrientation(&accel_event, &orientation))
    {    
        //Information is updated in correpsonding variable only if the change is significant enough to be showed
        if ( abs( ((-orientation.roll)/50)-roll)>radsPerChange*2) { 
            roll=(-orientation.roll)/50;
        }
        
        if ( abs( ((-orientation.pitch)/50)-pitch)>radsPerChange*2) {
            pitch=orientation.pitch/50;  
        }
    }

    /* Calculate the heading using the magnetometer */
    mag.getEvent(&mag_event);
    if (dof.magGetOrientation(SENSOR_AXIS_Z, &mag_event, &orientation))
    {    
        if ( abs( ((-orientation.heading)/50)-yaw)>radsPerChange*2){ 
            yaw=-orientation.heading/50;  
        }
    }

    for(i=0;i<4;i++){
        lis[i].read();      // get X Y and Z data at once
        if ( abs( ((lis[i].y/5095.54)-wheels[i]))>radsPerChange*2) 
            wheels[i]=(lis[i].y/5095.54); 
        //Y4 y Y3 por su pedo
        sensors_event_t event; 
        lis[i].getEvent(&event);
    }
}

float Imu::getRoll() { 
    updateData();
    return  roll;
}

float Imu::getPitch() { 
    updateData();
    return  pitch;
}

float Imu::getYaw() { 
    updateData();
    return  yaw;
}

float Imu::getWheel1() { 
    updateData();
    return  wheels[0];
}

float Imu::getWheel2() { 
    updateData();
    return  wheels[1];
}

float Imu::getWheel3() { 
    updateData();
    return  wheels[2];
}

float Imu::getWheel4() { 
    updateData();
    return  wheels[3];
}


