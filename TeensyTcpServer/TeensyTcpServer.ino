#include "src/Comms/Comms.h"

byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xEE}; //Assign a mac address
IPAddress ip(192, 168, 1, 200); //Assign my IP adress
unsigned int localPort = 5000; //Assign a Port to talk over


String datReq; //String for our data

Comms comms(ip, mac, localPort);

void setup() {
  Serial.begin(9600); //Turn on Serial Port
  comms.startComms();
}

void loop() {
  comms.readComms();
}
