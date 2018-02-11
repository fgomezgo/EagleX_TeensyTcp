/*
  Eagle X's rover motor controllers library Motors.h - Library for setting speeds/getting data form controllers
*/
#include "Arduino.h"
#include "Motor.h"

#define SMCSerial Serial3
// some variable IDs
#define ERROR_STATUS 0		// No used, but left for future modifications
#define INPUT_VOLTAGE 23
#define TEMPERATURE 24


Motor::Motor(char reset, char error){
	_resetPin = reset;
	_errPin = error;
}

// Utility methods/Functions -----------------

void Motor::configureAndReset(){
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
	Serial3.write(0xAA); // send baud-indicator byte
	// clear the safe-start violation and let the motor run
	exitSafeStart();
}

int Motor::readByte(){
	char c;
	if(Serial3.readBytes(&c, 1) == 0){ return -1; }
	return (byte)c;
}

unsigned int Motor::getVariable(unsigned char variableID, unsigned char device){
	Serial3.write(0xAA);
	Serial3.write(device);
	Serial3.write(0x21);
	Serial3.write(variableID);
	return readByte() + 256 * readByte();
}

// Methods/Functions for multi-device actions ------------
void Motor::setAllSpeed(int speed){

}

unsigned int Motor::getAvgVoltage(){
	unsigned int avg = 0;
	for(int i=0; i < 8; i++){
		avg += getVariable(INPUT_VOLTAGE, i);
		delay(1); // Is it needed?
	}
	return avg/8;
}

unsigned int Motor::getAvgTemp(){
	unsigned int avg = 0;
	for(int i=0; i < 8; i++){
		avg += getVariable(TEMPERATURE, i);
		delay(1); // Is it needed?
	}
	return avg/8;
}

bool Motor::getError(){
	return digitalRead(_errPin);
}

void Motor::exitSafeStart(){
	Serial3.write(0x83);
}


// Methods/Functions for unique devices ------------------
unsigned int Motor::getMotorVoltage(unsigned char device){
	return getVariable(INPUT_VOLTAGE, device);
}

unsigned int Motor::getMotorTemp(unsigned char device){
	return getVariable(TEMPERATURE, device);
}

void Motor::setMotorSpeed(int speed, unsigned char device){
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
