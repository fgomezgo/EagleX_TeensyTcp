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

// States
typedef enum{
	IDLE,     // Awaits for communication  and gets the id
	ACT_DRIVE_ALL_SP,		//? Drive System Controllers
	ACT_ARM,			//? ARM Controllers
	ACT_ARM_SH_PITCH,
	ACT_ARM_EL_PITCH,
	ACT_wrist_pitch_en,		//? Wrist Controllers
	ACT_wrist_roll_en,
	ACT_gripper_roll_en,		//? Gripper Controller
	ACT_COOLING_SET,		//? Cooling System
	FEE_UPD_SUSPS,			//? IMU
	FEE_GET_SUSP1,
	FEE_GET_SUSP2,
	FEE_GET_SUSP3,
	FEE_GET_SUSP4,
	FEE_GET_CHASS_ROLL,
	FEE_GET_CHASS_PITCH,
	FEE_GET_CHASS_YAW,
	LOC_UPDATE,				//? Location
	LOC_GET_LAT,
	LOC_GET_LON,
	LOC_NO_FIX,
	TEST,
}ServerStates;

ServerStates cState;    // Current state

void setup() {
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
						cState = ACT_ARM; 		//? Arm controllers
						break;
					/*
					case 0x08:
						cState = ACT_ARM_SH_PITCH; 		
						break;
					case 0x09:
						cState = ACT_ARM_EL_PITCH; 		
						break;
					case 0x0A:
						cState = ACT_wrist_pitch_en; 		
						break;
					case 0x0B:
						cState = ACT_wrist_roll_en; 		
						break;
					case 0x0C:
						cState = ACT_gripper_roll_en; 		
						break;
					case 0x0D:
						cState = ACT_COOLING_SET;	//? Cooling System
						break;
						*/
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
/*////////////////////////////////////////////////////////////////////////////////////////////////////////////*/
		case ACT_ARM:
			//! ///////////////////////////////////////////////////////////////////////////////////////////////////////////*/
			Serial.println(request,BIN);
			//! ///////////////////////////////////////////////////////////////////////////////////////////////////////////*/
			unsigned int shoulder_yaw_en;
			shoulder_yaw_en = request;
			shoulder_yaw_en = shoulder_yaw_en & 0x01;

			unsigned int shoulder_yaw_dir;
			shoulder_yaw_dir = request>>1;
			shoulder_yaw_dir = shoulder_yaw_dir & 0x01;

			unsigned int shoulder_pitch_en;
			shoulder_pitch_en = request>>2;
			shoulder_pitch_en = shoulder_pitch_en & 0x01;

			unsigned int shoulder_pitch_dir;
			shoulder_pitch_dir = request>>3;
			shoulder_pitch_dir = shoulder_pitch_dir & 0x01;

			unsigned int elbow_pitch_en;
			elbow_pitch_en = request>>4;
			elbow_pitch_en = elbow_pitch_en & 0x01;

			unsigned int elbow_pitch_dir;
			elbow_pitch_dir = request>>5;
			elbow_pitch_dir = elbow_pitch_dir & 0x01;

			unsigned int wrist_pitch_en;
			wrist_pitch_en = request>>6;
			wrist_pitch_en = wrist_pitch_en & 0x01;

			unsigned int wrist_pitch_dir;
			wrist_pitch_dir = request>>7;
			wrist_pitch_dir = wrist_pitch_dir & 0x01;

			unsigned int wrist_roll_en;
			wrist_roll_en = request>>8;
			wrist_roll_en = wrist_roll_en & 0x01;

			unsigned int wrist_roll_dir;
			wrist_roll_dir = request>>9;
			wrist_roll_dir = wrist_roll_dir & 0x01;

			unsigned int gripper_roll_en;
			gripper_roll_en = request>>10;
			gripper_roll_en = gripper_roll_en & 0x01;

			unsigned int gripper_roll_dir;
			gripper_roll_dir = request>>11;
			gripper_roll_dir = gripper_roll_dir & 0x01;

			if(shoulder_yaw_en){
				//! ///////////////////////////////////////////////////////////////////////////////////////////////////////////*/
				Serial.println(shoulder_yaw_dir,BIN);
				//! ////////////////////////////////////////////////////////////////////////////////////////////////////////////*/
				Serial.println("Shoulder YAW");
				actuator.shoulderYaw(shoulder_yaw_dir);
				cState = IDLE;
			}

			if(shoulder_pitch_en){
				//! ///////////////////////////////////////////////////////////////////////////////////////////////////////////*/
				Serial.println(shoulder_pitch_dir,BIN);
				//! ////////////////////////////////////////////////////////////////////////////////////////////////////////////*/
				Serial.println("Shoulder PITCH");
				actuator.shoulderPitch(shoulder_pitch_dir);
				cState = IDLE;
			}

			if(elbow_pitch_en){
				//! ///////////////////////////////////////////////////////////////////////////////////////////////////////////*/
				Serial.println(elbow_pitch_dir,BIN);
				//! ////////////////////////////////////////////////////////////////////////////////////////////////////////////*/
				Serial.println("Elbow PITCH");
				actuator.elbowPitch(elbow_pitch_dir);
				cState = IDLE;
			}

			if(wrist_pitch_en){
				//! ///////////////////////////////////////////////////////////////////////////////////////////////////////////*/
				Serial.println(wrist_pitch_dir,BIN);
				//! ////////////////////////////////////////////////////////////////////////////////////////////////////////////*/
				Serial.println("Wrist PITCH");
				actuator.wristPitch(wrist_pitch_dir);
				cState = IDLE;
			}

			if(wrist_roll_en){
				//! ///////////////////////////////////////////////////////////////////////////////////////////////////////////*/
				Serial.println(wrist_roll_dir,BIN);
				//! ////////////////////////////////////////////////////////////////////////////////////////////////////////////*/
				Serial.println("Wrist ROLL");
				actuator.wristRoll(wrist_roll_dir);
				cState = IDLE;
			}

			if(gripper_roll_en){
				//! ///////////////////////////////////////////////////////////////////////////////////////////////////////////*/
				Serial.println(gripper_roll_dir,BIN);
				//! ////////////////////////////////////////////////////////////////////////////////////////////////////////////*/
				Serial.println("Gripper ROLL");
				actuator.gripperRoll(gripper_roll_dir);
				cState = IDLE;
			}
			if(gripper_roll_en | wrist_roll_en | wrist_pitch_en | elbow_pitch_en | shoulder_pitch_en | shoulder_yaw_en){
				delay(200);
			}
			/*
			actuator.shoulderYaw(request);
			cState = IDLE;
			*/
			break;
		/*
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

		case ACT_wrist_pitch_en:
			Serial.println("Wrist PITCH");
			actuator.wristPitch(request);
			cState = IDLE;
			break;

		case ACT_wrist_roll_en:
			Serial.println("Wrist ROLL");
			actuator.wristRoll(request);
			cState = IDLE;
			break;

		case ACT_gripper_roll_en:
			Serial.println("Gripper ROLL");
			actuator.gripperRoll(request);
			cState = IDLE;
			break;
		*/
/*////////////////////////////////////////////////////////////////////////////////////////////////////////////*/
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
			Serial.println(actuator.driveGetVoltage(3));
			break;

		default:
			break;
	}
}
