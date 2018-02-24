#include "src/Comms/Comms.h"
#include "src/Location/Location.h"
#include "src/Actuator/Actuator.h"

byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xEE}; //Assign a mac address
IPAddress ip(192, 168, 1, 200); //Assign my IP adress
unsigned int localPort = 5000; //Assign a Port to talk over
unsigned int request;
byte header;

unsigned int instruction;
String datReq; //String for our data
Comms comms(ip, mac, localPort);      //  Ethernet module object
Location location(1);                 // GPS module object
Actuator actuator(5,6);                     // configure SMC  reset and  error pins

// States
typedef enum{
	IDLE,     // Awaits for communication  and gets the id
	LOCATION,
	LOC_GET,
	LOC_LAT,
	LOC_LON,
	LOC_NO_FIX,
	ACT_ARM_SH,
	ACT_ARM_EL,
	TEST,
}ServerStates;

ServerStates cState;    // Current state

void setup() {
	Serial.begin(9600); //Turn on Serial Port

	comms.start();
	location.moduleConfigure();
	actuator.controllerConfigureReset();

	//actuator.motorSetSpeed(3200,3);
	actuator.driveSetAllSpeed(3200, -3200);

	//Set next state
	cState = IDLE;
}

void loop() {
	switch(cState){
		case IDLE:
			if(comms.available()){   // If data is at socket
				//? Thought this was easier to understand 
				//? Reads, parses header and stores request
				request = comms.read();
				header = request & 0xFF;
				request = request >> 8;

				switch(header){  // Toggle state according to header
					case 0x19:
						cState = LOC_LAT;
						break;
					case 0x1A:
						cState = LOC_LON;
						break;
					case 0x07:
						cState = ACT_ARM_SH;
						break;
					case 0x08:
						cState = ACT_ARM_EL;
						break;
				}
			}else{
				cState = LOC_GET;
			}
			break;

		case LOC_GET:   // Gets updated data from GPS when no requests  are present
			location.updateData();
			cState = IDLE;
			break;

		case LOC_LAT:   // Gets latitude from GPS module and returns to client
			if(location.getFix()){
				comms.writePrecision(location.getLatitude(),5);
				cState = IDLE;
			}else{
				cState = LOC_NO_FIX;
			}
			break;

		case LOC_LON:   // Gets longitude from GPS module and returns to client
			if(location.getFix()){
				comms.writePrecision(location.getLongitude(),5);
				cState = IDLE;
			}else{
				cState = LOC_NO_FIX;
			}
			break;

		case ACT_ARM_SH:	// Rotates sholder in direction of request
			actuator.shoulderRotate(request);
			cState = IDLE;
			break;

		case ACT_ARM_EL:	// Rotates elbow in direction of request
			actuator.elbowRotate(request);
			cState = IDLE;
			break;

		case LOC_NO_FIX:  // If there is no FIX send error to host
			// Send no fix error back
			comms.write("-1");
			// Back to idle
			cState = IDLE;
			break;
		case TEST:
			Serial.println(actuator.driveGetVoltage(3));
			break;

		default:
			break;
	}
}
