/*
  Eagle X's rover motor controllers library Motors.h - Library for setting speeds/getting data form controllers
*/
#include "Arduino.h"
#include "Actuator.h"

#define SMCSerial Serial3
// some variable IDs
#define ERROR_STATUS 0		// No used, but left for future modifications
#define INPUT_VOLTAGE 23	// ID for quering the input voltage of a SMC
#define TEMPERATURE 24		// ID for quering the temperature of a SMC

// Constructor, sets the reset and error pins
Actuator::Actuator(char reset, char error){
	_resetPin = reset;
	_errPin = error;
}

// Utility methods/Functions -----------------

void Actuator::motorConfigureAndReset(){
	// Starts serial comms with the SMC
	Serial3.begin(9600);
	// briefly reset SMC when Arduino starts up (optional)
	pinMode(_resetPin, OUTPUT);
	digitalWrite(_resetPin, LOW); // reset SMC
	delay(1); // wait 1 ms
	pinMode(_resetPin, INPUT); // let SMC run again
	// must wait at least 1 ms after reset before transmitting
	delay(5);
	// this lets us read the state of the SMC ERR pin (optional)
	pinMode(_errPin, INPUT);
	// Requires to identify baudrate
	Serial3.write(0xAA); // send baud-indicator byte
	// clear the safe-start violation and let the motor run
	motorExitSafeStart();
}
// Used to read incoming information from the SMC
int Actuator::motorReadByte(){
	char c;
	if(Serial3.readBytes(&c, 1) == 0){ return -1; }
	return (byte)c;
}
// Used to get different variables from a specific motor
unsigned int Actuator::motorGetVariable(unsigned char variableID, unsigned char device){
	Serial3.write(0xAA);
	Serial3.write(device);
	Serial3.write(0x21);
	Serial3.write(variableID);
	return motorReadByte() + 256 * motorReadByte();
}

// Methods/Functions for multi-device actions ------------

// Sets all motors at one side to said speed
void Actuator::motorSetAllSpeed(int speedLeft, int speedRight){
	for(int i = 0; i < 3; i++){
		motorSetSpeed(speedLeft, i);
		delay(5); // Is it needed?
	}
	for(int i = 3; i < 6; i++){
		motorSetSpeed(-speedRight, i);
		delay(5); // Is it needed?
	}
}

// Returns avg voltage of all SMC at the rover
unsigned int Actuator::motorGetAvgVoltage(){
	unsigned int avg = 0;
	for(int i=0; i < 8; i++){
		avg += motorGetVariable(INPUT_VOLTAGE, i);
		delay(1); // Is it needed?
	}
	return avg/8;
}

// Returns the average temperature of all SMC
unsigned int Actuator::motorGetAvgTemp(){
	unsigned int avg = 0;
	for(int i=0; i < 8; i++){
		avg += motorGetVariable(TEMPERATURE, i);
		delay(1); // Is it needed?
	}
	return avg/8;
}

// Returns value of ERR pin 
bool Actuator::motorGetError(){
	return digitalRead(_errPin);
}

// Starts SMCs
void Actuator::motorExitSafeStart(){
	Serial3.write(0x83);
}


// Methods/Functions for unique devices ------------------
// Gets voltage for unique SMC
unsigned int Actuator::motorGetVoltage(unsigned char device){
	return motorGetVariable(INPUT_VOLTAGE, device);
}

// Gets temperature for unique SMC
unsigned int Actuator::motorGetTemp(unsigned char device){
	return motorGetVariable(TEMPERATURE, device);
}

// Sets SMC individual speed
void Actuator::motorSetSpeed(int speed, unsigned char device){
	Serial3.write(0xAA);
	Serial3.write(device);
	if (speed < 0){
		Serial3.write(0x06); // motor reverse command
		speed = -speed; // make speed positive
	}
	else{
		Serial3.write(0x05); // motor forward command
	}
	Serial3.write(speed & 0x1F);
	Serial3.write(speed >> 5);
}
