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

    /* -------------- Pololu Controllers -------------- */
    // Generic methods/functions required for controlllers comms

    void controllerConfigureReset();
    int controllerReadByte();
    unsigned int controllerGetVariable(unsigned char variableID, unsigned char device);
    
    /* -------------- Drive System -------------- */
    // Methods/fucntions that allow the user to set drive system speed/ get board information

    // Multidevice instructions
    void driveSetAllSpeed(int speedLeft, int speedRight);
    unsigned int driveGetAvgVoltage();
    unsigned int driveGetAvgTemp();
    bool driveGetError();
    void driveExitSafeStart();

    // Unique device instructions
    unsigned int driveGetVoltage(unsigned char device);
    unsigned int driveGetTemp(unsigned char device);
    void driveSetSpeed(int speed, unsigned char device);
    
      
    /* -------------- Arm motor controllers -------------- */
    //TODO: will a delay work good enough?
    void baseRotate(bool heading);

    //TODO: Main idea is to use a closed loop system for shoulder position
    void shoulderRotate(int position);
    void elbowRotate(int position);

    /* -------------- Gripper motor controllers -------------- */
    //TODO: Same as above // Controls the servo
    void forearmRotate(int position);
    //TODO: Same as above
    void wristRotate(int position);
    //TODO: Control it with the tiny wheel thingy
    void gripperMove(bool direction);

    /* -------------- Cooling System -------------- */
    //TODO: Plan is to send byte with fans states encoded?
    void coolingSet(char config);

  private:
    char _resetPin;
    char _errPin;

};

#endif