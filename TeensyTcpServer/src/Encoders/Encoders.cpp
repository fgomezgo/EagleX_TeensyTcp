/*
  Eagle X's rover Encoders library Encoders.h - Encoders library
*/
#include "Arduino.h"
#include "Encoders.h"

Encoders::Encoders(int x){
    x=0;
}

void Encoders::initEncoders() {
    /* Assign a unique ID to the sensors */
    encoders[0]=myEncl1;
    encoders[1]=myEncr1;
}

void Encoders::readEncoder(int i) {
    newPosition[i] = encoders[i].read();
    if (newPosition[i] != oldPosition[i]) {
        rads[i] = abs(oldPosition[i]-newPosition[i])*(pi/100.0);
        rads[i] = rads[i]*r;
        oldPosition[i] = newPosition[i];
    }else{
        rads[i]=0;
    }
}

void Encoders::readEncoders() {
    for (int i=0; i<2; i++) {
        newPosition[i] = encoders[i].read();
        if (newPosition[i] != oldPosition[i]) {
            rads[i] = abs(oldPosition[i]-newPosition[i])*(pi/100.0);
            rads[i] = rads[i]*r;
            oldPosition[i] = newPosition[i];
        }else{
            rads[i]=0;
        }
    }
}