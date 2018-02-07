/*
  Eagle X's rover communication library Comms.h - Library for managing the Wiz820io ethernet module
*/
#ifndef Comms_h
#define Comms_h

#include "Arduino.h"

class Morse
{
  public:
    Morse(int pin);
    void dot();
    void dash();
  private:
    int _pin;
};

#endif
