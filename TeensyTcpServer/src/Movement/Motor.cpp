/*
  Eagle X's rover motor controllers library Motors.h - Library for setting speeds/getting data form controllers
  
*/
#include "Arduino.h"
#include "Motor.h"










// ----------------------------------------------------OLD

#define resetPin 11 // pin 5 connects to SMC nRST
#define errPin 12 // pin 6 connects to SMC ERR
// some variable IDs
#define ERROR_STATUS 0
#define LIMIT_STATUS 3
#define TARGET_SPEED 20
#define INPUT_VOLTAGE 23
#define TEMPERATURE 24
// some motor limit IDs
#define FORWARD_ACCELERATION 5
#define REVERSE_ACCELERATION 9
#define DECELERATION 2
// read a serial byte (returns -1 if nothing received after the timeout expires)
int speed=3000;

int readByte()
{
	char c;
	if(Serial1.readBytes(&c, 1) == 0){ return -1; }
	return (byte)c;
}
// required to allow motors to move
// must be called when controller restarts and after any error
void exitSafeStart()
{
	Serial1.write(0x83);
}
// speed should be a number from -3200 to 3200
void setMotorSpeed(int speed)
{
	if (speed < 0)
	{
		Serial1.write(0x86); // motor reverse command
		speed = -speed; // make speed positive
	}
	else
	{
		Serial1.write(0x85); // motor forward command
	}
	Serial1.write(speed & 0x1F);
	Serial1.write(speed >> 5);
}

void setMotorNumbSpeed(int number,int speed)
{
	Serial1.write(0xAA);
	Serial1.write(number);
	
	
	if (speed < 0)
	{
		Serial1.write(0x06); // motor reverse command
		speed = -speed; // make speed positive
	}
	else
	{
		Serial1.write(0x05); // motor forward command
	}
	Serial1.write(speed & 0x1F);
	Serial1.write(speed >> 5);
}

unsigned char setMotorLimit(unsigned char limitID, unsigned int limitValue)
{
	Serial1.write(0xA2);
	Serial1.write(limitID);
	Serial1.write(limitValue & 0x7F);
	Serial1.write(limitValue >> 7);
	return readByte();
}
// returns the specified variable as an unsigned integer.
// if the requested variable is signed, the value returned by this function
// should be typecast as an int.
unsigned int getVariable(unsigned char variableID)
{
	Serial1.write(0xA1);
	Serial1.write(variableID);
	return readByte() + 256 * readByte();
}
void setup()
{
	Serial.begin(9600); // for debugging (optional)
	Serial1.begin(9600);
	// briefly reset SMC when Arduino starts up (optional)
	pinMode(resetPin, OUTPUT);
	digitalWrite(resetPin, LOW); // reset SMC
	delay(1); // wait 1 ms
	pinMode(resetPin, INPUT); // let SMC run again
	// must wait at least 1 ms after reset before transmitting
	delay(5);
	// this lets us read the state of the SMC ERR pin (optional)
	pinMode(errPin, INPUT);
	Serial1.write(0xAA); // send baud-indicator byte
	setMotorLimit(FORWARD_ACCELERATION, 12);
	setMotorLimit(REVERSE_ACCELERATION, 12);
	setMotorLimit(DECELERATION, 12);
	// clear the safe-start violation and let the motor run
	exitSafeStart();
}
void loop()
{
	setMotorNumbSpeed(0,speed);
       // delay(3000);
        setMotorNumbSpeed(1,speed);
       // delay(3000);
        setMotorNumbSpeed(2,speed);
       // delay(3000);
        setMotorNumbSpeed(3,speed);
       // delay(3000);
        setMotorNumbSpeed(4,speed);
        //delay(3000);
        setMotorNumbSpeed(5,speed);
        
//        setMotorNumbSpeed(0,0);
//        delay(3000);
//        setMotorNumbSpeed(1,0);
//        delay(3000);
//        setMotorNumbSpeed(2,0);
//        delay(3000);
//        setMotorNumbSpeed(3,0);
//        delay(3000);
//        setMotorNumbSpeed(4,0);
//        delay(3000);
//        setMotorNumbSpeed(5,0);
        

	//setMotorSpeed(3200); // full-speed forward
	// signed variables must be cast to ints:
	Serial.println((int)getVariable(TARGET_SPEED));
	delay(1000);
	//setMotorSpeed(-3200); // full-speed reverse
	Serial.println((int)getVariable(TARGET_SPEED));
	delay(1000);
	// write input voltage (in millivolts) to the serial monitor
	Serial.print("VIN = ");
	Serial.print(getVariable(INPUT_VOLTAGE));
	Serial.println(" mV");
	// if an error is stopping the motor, write the error status variable
	// and try to re-enable the motor
	
	
	
	if (digitalRead(errPin) == HIGH)
	{
		Serial.print("Error Status: 0x");
		Serial.println(getVariable(ERROR_STATUS), HEX);
		// once all other errors have been fixed,
		// this lets the motors run again
		exitSafeStart();
	}
}