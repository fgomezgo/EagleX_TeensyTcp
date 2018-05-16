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

//* Utility methods/Functions -----------------

void Actuator::controllerConfigureReset(){
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
	controllerExitSafeStart();
	_wristPitch.attach(14);
	_wristPitch.write(_servoState);
}
// Used to read incoming information from the SMC
int Actuator::controllerReadByte(){
	char c;
	if(Serial3.readBytes(&c, 1) == 0){ return -1; }
	return (byte)c;
}
// Used to get different variables from a specific motor
unsigned int Actuator::controllerGetVariable(unsigned char variableID, unsigned char device){
	Serial3.write(0xAA);
	Serial3.write(device);
	Serial3.write(0x21);
	Serial3.write(variableID);
	return controllerReadByte() + 256 * controllerReadByte();
}

//* Methods/Functions for multi-device actions ------------

// Sets all motors at one side to said speed
void Actuator::driveSetAllSpeed(int speedLeft, int speedRight){
	for(int i = 1; i < 4; i++){		// Drivers 1-3
		driveSetSpeed(speedLeft, i);
		delay(5); // Is it needed?
	}
	//Drivers 8, 10, 11
	driveSetSpeed(-speedRight, 8);
	delay(5); // Is it needed?
	//Drivers 8, 10, 11
	driveSetSpeed(-speedRight, 10);
	delay(5); // Is it needed?
	//Drivers 8, 10, 11
	driveSetSpeed(-speedRight, 11);
	delay(5); // Is it needed?
	
}

// Returns avg voltage of all SMC at the rover
unsigned int Actuator::driveGetAvgVoltage(){
	unsigned int avg = 0;
	for(int i=0; i < 8; i++){
		avg += controllerGetVariable(INPUT_VOLTAGE, i);
		delay(1); // Is it needed?
	}
	return avg/8;
}

// Returns the average temperature of all SMC
unsigned int Actuator::driveGetAvgTemp(){
	unsigned int avg = 0;
	for(int i=0; i < 8; i++){
		avg += controllerGetVariable(TEMPERATURE, i);
		delay(1); // Is it needed?
	}
	return avg/8;
}

// Returns value of ERR pin 
bool Actuator::driveGetError(){
	return digitalRead(_errPin);
}

// Starts SMCs
void Actuator::controllerExitSafeStart(){
	Serial3.write(0x83);
}


// Methods/Functions for unique devices ------------------
// Gets voltage for unique SMC
unsigned int Actuator::driveGetVoltage(unsigned char device){
	return controllerGetVariable(INPUT_VOLTAGE, device);
}

// Gets temperature for unique SMC
unsigned int Actuator::driveGetTemp(unsigned char device){
	return controllerGetVariable(TEMPERATURE, device);
}

// Sets SMC individual speed
void Actuator::driveSetSpeed(int percentage, unsigned char device){
	Serial3.write(0xAA);
	Serial3.write(device);
	if (percentage < 0){
		Serial3.write(0x06); // motor reverse command
		percentage = -percentage; // make speed positive
	}
	else{
		Serial3.write(0x05); // motor forward command
	}
	Serial3.write(0x00);
	Serial3.write(percentage);
}
/*////////////////////////////////////////////////////////////////////////////////////////////////////////////*/

/* -------------- Arm motor controllers -------------- */
void Actuator::shoulderYaw(bool direction){
	if(direction){
		driveSetSpeed(100, 4);
	}else{
		driveSetSpeed(-100, 4);
	}
	driveSetSpeed(0, 4);
}

void Actuator::shoulderPitch(bool direction){
	if(direction){
		driveSetSpeed(80, 6);
	}else{
		driveSetSpeed(-80, 6);
	}
	driveSetSpeed(0, 6);
}

void Actuator::elbowPitch(bool direction){
	if(direction){
		driveSetSpeed(-80, 5);
	}else{
		driveSetSpeed(80, 5);
	}
	driveSetSpeed(0, 5);
}

/* -------------- Wrist Controller -------------- */
void Actuator::wristPitch(bool direction){
	//TODO implement servo logic
	
	if(direction){
		_servoState = _servoState - 4;
	}else{
		_servoState = _servoState + 4;
	}
	_wristPitch.write(_servoState);
}

void Actuator::wristRoll(bool direction){
	if(direction){
		driveSetSpeed(40, 9);
	}else{
		driveSetSpeed(-40, 9);
	}
	driveSetSpeed(0, 9);
}

/* -------------- Gripper Controller -------------- */
void Actuator::gripperRoll(bool direction){
	delay(100);
	if(direction){
		driveSetSpeed(100, 7);
	}else{
		driveSetSpeed(-100, 7);
	}
	driveSetSpeed(0, 7);
}


/*////////////////////////////////////////////////////////////////////////////////////////////////////////////*/

/* -------------- Cooling System -------------- */

void Actuator::coolingSet(char config){
	pinMode(16, OUTPUT);
	pinMode(17, OUTPUT);
	digitalWrite(16, config & 0x01);
	digitalWrite(17, config >> 1);
}