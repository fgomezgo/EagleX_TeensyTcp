/*
  Eagle X's rover motor controllers library Motors.h - Library for setting speeds/getting data form controllers
*/
#ifndef Actuator_h
#define Actuator_h

#include "Arduino.h"

class Actuator{
  public:
    // Constructor, sets the reset and error pins
    Actuator(char reset, char error);
    
    // Utility methods/functions
    void motorConfigureAndReset();
    int motorReadByte();
    unsigned int motorGetVariable(unsigned char variableID, unsigned char device);
    
    // Methods/functions for multi-device actions
    void motorSetAllSpeed(int speedLeft, int speedRight);
    unsigned int motorGetAvgVoltage();
    unsigned int motorGetAvgTemp();
    bool motorGetError();
    void motorExitSafeStart();

    // Methdos/functions for unique devices 
    unsigned int motorGetVoltage(unsigned char device);
    unsigned int motorGetTemp(unsigned char device);
    void motorSetSpeed(int speed, unsigned char device);

    

  private:
    char _resetPin;
    char _errPin;

};

#endif