/*
  Eagle X's rover motor controllers library Motors.h - Library for setting speeds/getting data form controllers
*/
#ifndef Actuator_h
#define Actuator_h

#include "Arduino.h"
#include <Servo.h>

class Actuator{
  public:
    // Constructor, sets the reset and error pins
    Actuator(char reset, char error);

    /* -------------- Pololu Controllers -------------- */
    // Generic methods/functions required for controlllers comms

    void controllerConfigureReset();
    int controllerReadByte();
    unsigned int controllerGetVariable(unsigned char variableID, unsigned char device);
    void controllerExitSafeStart();
    
    /* -------------- Drive System -------------- */
    // Methods/fucntions that allow the user to set drive system speed/ get board information

    // Multidevice instructions
    void driveSetAllSpeed(int speedLeft, int speedRight);

    //TODO Implement  separate functions for drive system temps and arm
    unsigned int driveGetAvgVoltage();
    unsigned int driveGetAvgTemp();
    bool driveGetError();
    

    // Unique device instructions
    unsigned int driveGetVoltage(unsigned char device);
    unsigned int driveGetTemp(unsigned char device);
    void driveSetSpeed(int percentage, unsigned char device);
    
      
    /* -------------- Arm motor controllers -------------- */
    //?: delay works good enough!
    void shoulderYaw(bool direction);
    void shoulderPitch(bool direction);

    void elbowPitch(bool direction);

    /* -------------- Wrist Controller -------------- */
    void wristPitch(bool direction);
    void wristRoll(bool direction);

    /* -------------- Gripper Controller -------------- */
    void gripperRoll(bool direction);


    /* -------------- Cooling System -------------- */
    //TODO: Plan is to send byte with fans states encoded?
    void coolingSet(char config);

  private:
    //TODO: Method for setting controller power [%]
    char _resetPin;
    char _errPin;
    Servo _wristPitch;

};

#endif