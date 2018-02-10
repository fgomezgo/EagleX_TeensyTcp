#include "src/Comms/Comms.h"
#include "src/Location/Location.h"

byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xEE}; //Assign a mac address
IPAddress ip(192, 168, 1, 200); //Assign my IP adress
unsigned int localPort = 5000; //Assign a Port to talk over
unsigned int request;

String datReq; //String for our data
Comms comms(ip, mac, localPort);
Location location(1);
// States

typedef enum{
  IDLE,     // Awaits for communication  and gets the id
  LOCATION,
  LOC_GET,
  LOC_LAT,
  LOC_NO_FIX,
}ServerStates;

ServerStates cState;    // Current state


void setup() {
  Serial.begin(9600); //Turn on Serial Port
  comms.commsStart();
  location.moduleConfigure();
  cState = IDLE;
}

void loop() {
  switch(cState){
    case IDLE:
      if(comms.commsAvailable()){   // If data is at socket
        switch(comms.commsRead()){  // Read data and send to device 
          case 0x19:
            cState = LOC_LAT;
            break;
        }
      }else{
        cState = LOC_GET;
      }
      break;

    case LOC_GET:
      location.updateData();
      cState = IDLE;
      break;

    case LOC_LAT:
      if(location.getFix()){
        comms.Udp.beginPacket(comms.Udp.remoteIP(), comms.Udp.remotePort());  //Initialize Packet send
        comms.Udp.print(location.getLatitude(),5); //Send string back to client 
        comms.Udp.endPacket(); //Packet has been sent
        cState = IDLE;
      }else{
        cState = LOC_NO_FIX;
      }
      break;

    case LOC_NO_FIX:
      // Send no fix error back
      comms.Udp.beginPacket(comms.Udp.remoteIP(), comms.Udp.remotePort());  //Initialize Packet send
      comms.Udp.print("ERR: No fix"); //Send string back to client 
      comms.Udp.endPacket(); //Packet has been sent
      // Back to idle
      cState = IDLE;
      break;

    default:
      break;
  }
  
  //Serial.println(comms.commsAvailable());
  /*
  if(comms.commsAvailable()){
    Serial.println("Msg");
    comms.commsRead();
  }*/
  
}
