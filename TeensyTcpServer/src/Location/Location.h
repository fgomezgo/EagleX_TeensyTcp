/*
  Eagle X's rover communication library Location.h - Library for managing the GPS module
*/
#ifndef Location_h
#define Location_h

#include "Arduino.h"
#include <Adafruit_GPS.h>
#include <math.h>

class Location{
    public:
        Location(char freq);
        void moduleConfigure();
        void updateData();
        double convertDegMinToDecDeg (float degMin);
        void getCoordinates();
        float getLongitude();
        float getLatitude();
        float getHeading();
        float getAltitude();
        bool getFix();
        //void setFrequency(char freq);
    private:
        char _frequency;
};

#endif