/*
  Eagle X's rover motor controllers library Motors.h - Library for setting speeds/getting data form controllers
*/
#ifndef Motor_h
#define Motor_h

#include "Arduino.h"

class Motor{
  public:
    Motor(char reset, char error);
    void configureAndReset();
    void setAllSpeed(int speed);
    unsigned int getAvgVoltage();
    unsigned int getAvgTemp();
    bool getError();
    void exitSafeStart();
    int readByte();
    unsigned int getVariable(unsigned char variableID, unsigned char device);
    unsigned int getMotorVoltage(unsigned char device);
    unsigned int getMotorTemp(unsigned char device);
    void setMotorSpeed(int speed, unsigned char device);

  private:
    char _resetPin;
    char _errPin;

};

#endif