#include "src/Comms/Comms.h"
#include "src/Location/Location.h"

byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xEE}; //Assign a mac address
IPAddress ip(192, 168, 1, 200); //Assign my IP adress
unsigned int localPort = 5000; //Assign a Port to talk over

String datReq; //String for our data
Comms comms(ip, mac, localPort);
Location location(1);
// States

typedef enum{
  IDLE,     // Awaits for communication  and gets the id
  LOC_GET,
  LOC_LAT,
  LOC_LON,
  LOC_NO_FIX,
}ServerStates;

ServerStates cState;    // Current state

void setup() {
  Serial.begin(9600); //Turn on Serial Port
  comms.start();
  location.moduleConfigure();
  cState = IDLE;
}

void loop() {
  switch(cState){
    case IDLE:
      if(comms.available()){   // If data is at socket
        switch(comms.read()){  // Read data and send to device 
          case 0x19:
            cState = LOC_LAT;
            break;
          case 0x1A:
            cState = LOC_LON;
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

    case LOC_NO_FIX:  // If there is no FIX send error to host
      // Send no fix error back
      comms.write("-1");
      // Back to idle
      cState = IDLE;
      break;

    default:
      break;
  }
}
