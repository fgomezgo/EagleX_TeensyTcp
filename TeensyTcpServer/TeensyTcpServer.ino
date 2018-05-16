#include "src/Comms/Comms.h"
#include "src/Location/Location.h"
#include "src/Actuator/Actuator.h"
#include "src/Feedback/Feedback.h"

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

char LIS3DH_CS[4] = {20, 21, 22, 23};		//IMU
Feedback feedback(LIS3DH_CS, 0, 1, 32);

Encoder myEnc(24, 25);

// States
typedef enum{
	IDLE,     // Awaits for communication  and gets the id
	ACT_DRIVE_ALL_SP,		//? Drive System Controllers
	ACT_ARM_SH_YAW,			//? ARM Controllers
	ACT_ARM_SH_PITCH,
	ACT_ARM_EL_PITCH,
	ACT_WRIST_PITCH,		//? Wrist Controllers
	ACT_WRIST_ROLL,
	ACT_GRIPPER_ROLL,		//? Gripper Controller
	ACT_COOLING_SET,		//? Cooling System
	FEE_UPD_SUSPS,			//? IMU
	FEE_GET_SUSP1,
	FEE_GET_SUSP2,
	FEE_GET_SUSP3,
	FEE_GET_SUSP4,
	FEE_GET_CHASS_ROLL,
	FEE_GET_CHASS_PITCH,
	FEE_GET_CHASS_YAW,		
	FEE_GET_AVG_SPEED,			//? Encoders
	LOC_UPDATE,				//? Location
	LOC_GET_LAT,
	LOC_GET_LON,
	LOC_NO_FIX,
	TEST,
}ServerStates;

ServerStates cState;    // Current state

void setup() {
	//enc_arr[0] = myEnc;
	Serial.begin(9600); //Turn on Serial Port

	comms.start();
	location.moduleConfigure();
	delay(1000);
	actuator.controllerConfigureReset();

	if(feedback.suspensionImuConf()){
		Serial.println("ERROR: Accelerometer intitialization");
	}else{
		Serial.println("MSG: Accelerometers detected");
	}
	if(feedback.chassisImuConf()){
		Serial.println("ERROR: 10-DOF intitialization");
	}else{
		Serial.println("MSG: 10-DOF detected");
	}
	feedback.encodersInit(&myEnc);
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
				switch(header){  
					case 0x00:
						cState = ACT_DRIVE_ALL_SP;		//? SET left and right speed 
						break;
					case 0x07:
						cState = ACT_ARM_SH_YAW; 		//? Arm controllers
						break;
					case 0x08:
						cState = ACT_ARM_SH_PITCH; 		
						break;
					case 0x09:
						cState = ACT_ARM_EL_PITCH; 		
						break;
					case 0x0A:
						cState = ACT_WRIST_PITCH; 		
						break;
					case 0x0B:
						cState = ACT_WRIST_ROLL; 		
						break;
					case 0x0C:
						cState = ACT_GRIPPER_ROLL; 		
						break;
					case 0x0D:
						cState = ACT_COOLING_SET;	//? Cooling System
						break;
					case 0x0E:
						cState = FEE_GET_SUSP1;		//? IMU
						break;
					case 0x4E:
						cState = FEE_GET_SUSP2;
						break;
					case 0x8E:
						cState = FEE_GET_SUSP3;
						break;
					case 0xCE:
						cState = FEE_GET_SUSP4;
						break;
					case 0x0F:
						cState = FEE_GET_CHASS_ROLL;
						break;
					case 0x4F:
						cState = FEE_GET_CHASS_PITCH;
						break;
					case 0x8F:
						cState = FEE_GET_CHASS_YAW;
						break;
					case 0x11:
						cState = LOC_GET_LAT;		//? Location
						break;
					case 0x51:
						cState = LOC_GET_LON;
						break;
				}
			}else{
				//cState = LOC_UPDATE;
				cState = FEE_GET_AVG_SPEED;
			}
			break;

		case ACT_DRIVE_ALL_SP:
			int leftSide,rightSide;
			
			rightSide =  request & 0xFF;
			leftSide = request >> 8;
			if (rightSide >> 7){
				rightSide = rightSide & 0x7F;
				rightSide = -rightSide;
			}
			if (leftSide >> 7){
				leftSide = leftSide & 0x7F;
				leftSide = -leftSide;
			}
			Serial.print("left: ");
			Serial.print(leftSide);
			Serial.print(" ");
			Serial.print("right: ");
			Serial.print(rightSide);
			Serial.println();

			actuator.driveSetAllSpeed(leftSide, rightSide);
			cState = IDLE;
			break;
		
		case ACT_ARM_SH_YAW:
			Serial.println("Shoulder YAW");
			actuator.shoulderYaw(request);
			cState = IDLE;
			break;
		
		case ACT_ARM_SH_PITCH:
			Serial.println("Shoulder PITCH");
			actuator.shoulderPitch(request);
			cState = IDLE;
			break;

		case ACT_ARM_EL_PITCH:
			Serial.println("Elbow PITCH");
			actuator.elbowPitch(request);
			cState = IDLE;
			break;

		case ACT_WRIST_PITCH:
			Serial.println("Wrist PITCH");
			actuator.wristPitch(request);
			cState = IDLE;
			break;

		case ACT_WRIST_ROLL:
			Serial.println("Wrist ROLL");
			actuator.wristRoll(request);
			cState = IDLE;
			break;

		case ACT_GRIPPER_ROLL:
			Serial.println("Gripper ROLL");
			actuator.gripperRoll(request);
			cState = IDLE;
			break;

		case ACT_COOLING_SET:
			Serial.println("Cooling SET");
			Serial.println(request, BIN);
			actuator.coolingSet(request);
			cState = IDLE;
			break;

		case FEE_UPD_SUSPS:
			feedback.suspensionImuUpdate();
			cState = IDLE;
			break;

		case FEE_GET_SUSP1:
			comms.writePrecision(feedback.getSuspensionRB(),5);
			cState = IDLE;
			break;

		case FEE_GET_SUSP2:
			comms.writePrecision(feedback.getSuspensionRF(),5);
			cState = IDLE;
			break;

		case FEE_GET_SUSP3:
			comms.writePrecision(feedback.getSuspensionLF(),5);
			cState = IDLE;
			break;

		case FEE_GET_SUSP4:
			comms.writePrecision(feedback.getSuspensionLB(),5);
			cState = IDLE;
			break;

		case FEE_GET_CHASS_ROLL:
			comms.writePrecision(feedback.getChassisRoll(),5);
			cState = IDLE;
			break;

		case FEE_GET_CHASS_PITCH:
			comms.writePrecision(feedback.getChassisPitch(),5);
			cState = IDLE;
			break;

		case FEE_GET_CHASS_YAW:
			comms.writePrecision(feedback.getChassisYaw(),5);
			cState = IDLE;
			break;
		
		case FEE_GET_AVG_SPEED:
			Serial.println(feedback.encodersRead(0));
			//Serial.println(myEnc.read());

			cState = IDLE;
			break;

		case LOC_UPDATE:   // Gets updated data from GPS when no requests  are present
			location.updateData();
			cState = IDLE;
			break;

		case LOC_GET_LAT:   // Gets latitude from GPS module and returns to client
			
			if(location.getFix()){
				Serial.println("Latitude sent");
				comms.writePrecision(location.getLatitude(),5);
				cState = IDLE;
			}else{
				cState = LOC_NO_FIX;
			}
			break;

		case LOC_GET_LON:   // Gets longitude from GPS module and returns to client
			if(location.getFix()){
				Serial.println("Longitude sent");
				comms.writePrecision(location.getLongitude(),5);
				cState = IDLE;
			}else{
				cState = LOC_NO_FIX;
			}
			break;

		case LOC_NO_FIX:  // If there is no FIX send error to host
			// Send no fix error back
			comms.write("-1");
			// Back to idle
			cState = IDLE;
			break;
		case TEST:
			
			break;

		default:
			break;
	}

	delay(100);
}
