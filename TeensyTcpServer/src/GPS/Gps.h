/*
  Eagle X's rover GPS library Comms.h - Library for managing the Adafruit Ultimate GPS module
*/
#ifndef Gps_h
#define Gps_h

#include "Arduino.h"
#include <Adafruit_GPS.h>
#include <math.h>

class Gps{
    public:
        Gps();
        void gpsConfigure();
        double convertDegMinToDecDeg (float degMin);

    private:
};


#endif