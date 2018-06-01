#include "Arduino.h"

#include "src/Comms/Comms.h"
#include "src/Location/Location.h"
#include "src/Actuator/Actuator.h"
#include "src/Feedback/Feedback.h"
#define DHTPIN 26
#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321 	
byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xEE}; //Assign a mac address
IPAddress ip(192, 168, 1, 200); //Assign my IP adress
unsigned int localPort = 5000; //Assign a Port to talk over
unsigned long request;
byte header;
unsigned long time_old;
boolean flag_received;
unsigned int instruction;
String datReq; //String for our data
Comms comms(ip, mac, localPort);      //  Ethernet module object
Location location(1);                 // GPS module object
Actuator actuator(5,6);                     // configure SMC  reset and  error pins
String data = "";
char LIS3DH_CS[4] = {20, 21, 22, 23};		//IMU
Feedback feedback(LIS3DH_CS, 0, 1, 32,DHTPIN, DHTTYPE);

// States
typedef enum{
	IDLE,     // Awaits for communication  and gets the id
	ACT_DRIVE_ALL_SP,		//? Drive System Controllers
	ACT_ARM_ALL_SP,			//? ARM Controllers (Arm yaw, Shoulder pitch, Elbow pitch)
	Teporocho,			
	ACT_GRIPPER_ALL_SP,		//? Wrist Controllers
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
	time_old = millis();
}

void loop() {
	switch(cState){
		case IDLE:
			//Reset ethernet every now and then
			flag_received = false;
			if(comms.available()){   // If data is at socket
				//? Thought this was easier to understand 
				//? Reads, parses header and stores request
				flag_received = true;
				request = comms.read();
				header = request & 0xFF;
				request = request >> 8;
				switch(header){  
					case 0x00:
						cState = ACT_DRIVE_ALL_SP;		//? SET left and right speed 
						break;
					case 0x06:
						cState = Teporocho; 		//? Teporocho
						break;
					case 0x07:
						cState = ACT_ARM_ALL_SP; 		//? Arm controllers
						break;
					case 0x08:
						cState = ACT_GRIPPER_ALL_SP; 	//? Gripper Controllers
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
				time_old = millis();
			}else{
				//cState = LOC_UPDATE;
			}
			//Serial.println((millis() - time_old));
			//Serial.println((millis() - time_old));
				if(!flag_received & ((millis() - time_old) > 10000)){
					Serial.println(" ---- Comms Reset ---");
					comms.start();
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
			data = String(actuator.driveGetTemp(2)) + String(actuator.driveGetVoltage(2));
			//Serial.println(actuator.driveGetTemp(2));
			comms.write(data);
			actuator.driveSetAllSpeed(leftSide, rightSide);
			cState = IDLE;
			break;
		
		case Teporocho:{
		 	float h = feedback.readHumidity();
			float t = feedback.readTemperature();
			float f = feedback.readTemperature(true);
			float k = t +273.15;
			h = round(h*100)/100;
			t = round(t*100)/100;
			f = round(f*100)/100;
			k = round(k*100)/100;
			String hs = String(h);
			String ts = String(t);
			String fs = String(f);
			String ks = String(k);
			if (isnan(h) || isnan(t) || isnan(f)) {
				Serial.println("Failed to read from DHT sensor!");
				break;
			}	
			data = String( hs +","+ ts +","+ fs + ","+ks);
			comms.write(data);

			Serial.print("Humedad");
			Serial.print(feedback.readHumidity());
			Serial.print("Temperatura");
			Serial.println(feedback.readTemperature());
			cState = IDLE;
		}break;

		case ACT_ARM_ALL_SP:
			int shoulderYaw_speed, shoulderPitch_speed, elbowPitch_speed;
			Serial.println(request,HEX);
			// Get individual speeds
			shoulderYaw_speed = request & 0xFF;
			shoulderPitch_speed = (request >> 8) & 0xFF;
			elbowPitch_speed = (request >> 16);
			// Parse direction
			if (shoulderYaw_speed >> 7){
				shoulderYaw_speed = shoulderYaw_speed & 0x7F;
				shoulderYaw_speed = -shoulderYaw_speed;
			}
			if (shoulderPitch_speed >> 7){
				shoulderPitch_speed = shoulderPitch_speed & 0x7F;
				shoulderPitch_speed = -shoulderPitch_speed;
			}
			if (elbowPitch_speed >> 7){
				elbowPitch_speed = elbowPitch_speed & 0x7F;
				elbowPitch_speed = -elbowPitch_speed;
			}
			Serial.print("ARM MOVING: ");
			Serial.print("SH Yaw: ");
			Serial.print(shoulderYaw_speed);
			Serial.print(" SH Pitch: ");
			Serial.print(shoulderPitch_speed);
			Serial.print(" EL Pitch: ");
			Serial.println(elbowPitch_speed);
			// Set actuator speed
			actuator.shoulderYaw(shoulderYaw_speed);
			actuator.shoulderPitch(shoulderPitch_speed);
			actuator.elbowPitch(elbowPitch_speed);

			cState = IDLE;
			break;
		
		case ACT_GRIPPER_ALL_SP:
			int wristPitch_speed, extraction_state, wristRoll_speed, gripperRoll_speed;
			Serial.println(request,HEX);
			// Get individual speeds
			wristPitch_speed = request & 0x03;
			extraction_state = (request & 0xFC) >> 2;
			Serial.print("CIENCEEEE ");
			Serial.println(extraction_state);

			wristRoll_speed = (request >> 8) & 0xFF;
			gripperRoll_speed = (request >> 16);
			// Parse direction
			if (wristPitch_speed >> 7){
				wristPitch_speed = wristPitch_speed & 0x7F;
				wristPitch_speed = -wristPitch_speed;
			}
			if (wristRoll_speed >> 7){
				wristRoll_speed = wristRoll_speed & 0x7F;
				wristRoll_speed = -wristRoll_speed;
			}
			if (gripperRoll_speed >> 7){
				gripperRoll_speed = gripperRoll_speed & 0x7F;
				gripperRoll_speed = -gripperRoll_speed;
			}
			Serial.print("ARM MOVING: ");
			Serial.print("Wrist Yaw: ");
			Serial.print(wristPitch_speed);
			Serial.print(" Wrist Roll: ");
			Serial.print(wristRoll_speed);
			Serial.print(" Gripper Roll: ");
			Serial.println(gripperRoll_speed);
			// Set actuator speed
			if(wristPitch_speed == 1){
				actuator.wristPitch(true);
			}else if (wristPitch_speed == 2){
				actuator.wristPitch(false);
			}
			actuator.wristRoll(wristRoll_speed);
			actuator.gripperRoll(gripperRoll_speed);
			actuator.setCache(extraction_state);
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
		case TEST:{

		}break;

		default:
			break;
	}
}
