// Eagle X project based on teensyTcp:
// 2011-01-30 <jc@wippler.nl> http://opensource.org/licenses/mit-license.php
 
#include <EtherCard.h>

//Libraries for 10-DOF
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_BMP085_U.h>
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_10DOF.h>

/* Assign a unique ID to the sensors */
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_LSM303_Mag_Unified   mag   = Adafruit_LSM303_Mag_Unified(30302);
Adafruit_BMP085_Unified       bmp   = Adafruit_BMP085_Unified(18001);
Adafruit_L3GD20_Unified       gyro  = Adafruit_L3GD20_Unified(20);



#define STATIC 0  // set to 1 to disable DHCP (adjust myip/gwip values below)

#if STATIC
// ethernet interface ip address
static byte myip[] = { 192,168,1,200 };
// gateway ip address
static byte gwip[] = { 192,168,1,3};
#endif

// ethernet mac address - must be unique on your network
static byte mymac[] = { 0x74,0x69,0x69,0x2D,0x30,0x31 };

byte Ethernet::buffer[500]; // tcp/ip send and receive buffer

//Integer to store the number descriptor coming from the client
int client_instr;
//String to store the message response to the client
char server_response[100];


void setup(){
  Serial.begin(57600);
  Serial.println("\n[backSoon]");
  
  if (ether.begin(sizeof Ethernet::buffer, mymac) == 0) 
    Serial.println( "Failed to access Ethernet controller");
#if STATIC
  ether.staticSetup(myip, gwip);
#else
  if (!ether.dhcpSetup())
    Serial.println("DHCP failed");
#endif

  ether.printIp("IP:  ", ether.myip);
  ether.printIp("GW:  ", ether.gwip);  
  ether.printIp("DNS: ", ether.dnsip);  

  //10-DOF Initializers
  accel.begin();
  mag.begin();
  bmp.begin();
  gyro.begin();

  
}

void loop(){
  // wait for an incoming TCP packet, but ignore its contents
   uint16_t payloadPos = ether.packetLoop(ether.packetReceive());
 
  if (payloadPos) {
    //Serial.println(payloadPos);
    char* incomingData = (char *) Ethernet::buffer + payloadPos;
    //Serial.println(incomingData);
    sscanf(incomingData,"%d",&client_instr);

/*  | Device ID | Device Description | Instruction ID |      Instruction Description      |
    |:---------:|:------------------:|:--------------:|:---------------------------------:|
    | 10        | 10-DOF             | 10             | Gyroscope Information (x,y,z)     |
    |           |                    | 11             | Accelerometer Information (x,y,z) |
    |           |                    | 12             | Magnetometer Information (x,y,z)  |
    |           |                    | 13             | Pressure and Temperature          |*/

    memset(server_response,0,sizeof server_response);
    switch(client_instr/100){
          case 10: //10-DOF
          
              sensors_event_t event;
              
             switch(client_instr%100){
                  case 10:
                        gyro.getEvent(&event);
                        sprintf(server_response,"%.6f %.6f %.6f",event.gyro.x,event.gyro.y,event.gyro.z);
                  break;
                  case 11:
                       accel.getEvent(&event);
                       sprintf(server_response,"%.6f %.6f %.6f",event.acceleration.x,event.acceleration.y,event.acceleration.z); 
                  break;
                  case 12:
                       mag.getEvent(&event);
                       sprintf(server_response,"%.6f %.6f %.6f",event.magnetic.x,event.magnetic.y,event.magnetic.z);
                  break;
                  case 13:
                       bmp.getEvent(&event);
                       float temperature;
                       bmp.getTemperature(&temperature);
                       sprintf(server_response,"%.6f %.6f",event.pressure,temperature);
                  break;
              } 
              memcpy_P(ether.tcpOffset(), server_response, sizeof server_response);
              ether.httpServerReply(sizeof server_response - 1);
                       
          break;
      
      }

  }
}
