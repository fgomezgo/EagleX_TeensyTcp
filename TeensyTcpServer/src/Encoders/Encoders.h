/*
  Eagle X's rover Encoders library Encoders.h - Encoders library
*/
#ifndef Encoders_h
#define Encoders_h

#include "Arduino.h"

#include "Encoder.h"

class Encoders{
    public:
        Encoders(int x);
        void initEncoders ();
        void readEncoder (int i); //Read specific encoder
        void readEncoders();  //Readall encoders

    private:
        Encoder myEncl1(13, 11);
        Encoder myEncr1(3, 5);
        Encoder encoders [6];
        long oldPosition[6]={-999, -999, -999, -999, -999, -999};
        long newPosition[6];
        float rads[6];

        float pi = 3.14159;
        float r = 0.075;

};