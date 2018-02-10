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
}ServerStates;

ServerStates cState;    // Current state


void setup() {
  Serial.begin(9600); //Turn on Serial Port
  comms.commsStart();
  location.moduleConfigure();
  cState = LOCATION;
}

void loop() {
  switch(cState){
    case IDLE:
      if(comms.commsAvailable()){   // If data is at socket
        switch(comms.commsRead()){  // Read data and send to device 
          case 0x19:
            cState = LOCATION;
            break;
        }
      }else{
        cState = IDLE;
      }
      break;
    case LOCATION:
      location.updateData();
      if(location.getFix()){
        Serial.print(location.getFix());
        Serial.print(" ");
        Serial.print(" ");
        Serial.print(location.getLatitude(),5);
        Serial.print(" ");
        Serial.print(location.getLongitude(),5);
        Serial.print(" ");
        Serial.print(location.getHeading());
        Serial.print(" ");
        Serial.println(location.getAltitude());
        
      }else{
        Serial.println(location.getFix());
      }

    /*
      comms.Udp.beginPacket(comms.Udp.remoteIP(), comms.Udp.remotePort());  //Initialize Packet send
      comms.Udp.print("GPS"); //Send string back to client 
      comms.Udp.endPacket(); //Packet has been sent*/

      
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
