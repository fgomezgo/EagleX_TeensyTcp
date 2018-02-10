/*
  Eagle X's rover communication library Comms.h - Library for managing the Wiz820io ethernet module
*/

#include "Arduino.h"
#include "Comms.h"


Comms::Comms(IPAddress ip, byte *mac, unsigned int localPort)
{
  _ip = ip;
  _mac = mac;
  _localPort = localPort;
  EthernetUDP Udp; //Define UDP Object
}

void Comms::moduleReset()
{
  // Begin reset sequence for WIZ820ip ethernet module
  pinMode(9, OUTPUT);
  digitalWrite(9, LOW);    // begin reset the WIZ820io
  pinMode(10, OUTPUT);
  digitalWrite(10, HIGH);  // de-select WIZ820io
  delay(1);
  digitalWrite(9, HIGH);   // end reset pulse  
}

void Comms::commsStart()
{
   moduleReset();
   Ethernet.begin(_mac, _ip); //Initialize Ethernet
   Udp.begin(_localPort); //Initialize Udp
   delay(1500); //delay
}

boolean Comms::commsAvailable()
{
  packetSize = Udp.parsePacket(); //Read the packetSize
  if(packetSize > 0){
    return 1;
  }else{
    return 0;
  }
}

unsigned int Comms::commsRead()
{
  Udp.read(_packetBuffer, UDP_TX_PACKET_MAX_SIZE); //Reading the data request on the Udp
  String _dataReq(_packetBuffer); //Convert _packetBuffer array to string _dataReq
  unsigned int ID = (_packetBuffer[0] <<24) | (_packetBuffer[1] <<16) | (_packetBuffer[2] <<8) | _packetBuffer[3];
  memset(_packetBuffer, 0, UDP_TX_PACKET_MAX_SIZE);
  return ID;
}


void Comms::commsReadOld()
{
   packetSize = Udp.parsePacket(); //Read the packetSize
   
  if(packetSize>0){ //Check to see if a request is present
  
  Udp.read(_packetBuffer, UDP_TX_PACKET_MAX_SIZE); //Reading the data request on the Udp
  String _dataReq(_packetBuffer); //Convert _packetBuffer array to string _dataReq
  Serial.println("Req");
  if (_dataReq =="Red") { //See if Red was requested
  
    Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());  //Initialize Packet send
    Udp.print("You are Asking for Red"); //Send string back to client 
    Udp.endPacket(); //Packet has been sent
  }
   if (_dataReq =="Green") { //See if Green was requested
  
    Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());  //Initialize Packet send
    Udp.print("You are Asking for Green"); //Send string back to client 
    Udp.endPacket(); //Packet has been sent
   }
    if (_dataReq =="Blue") { //See if Red was requested
  
    Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());  //Initialize Packet send
    Udp.print("You are Asking for Blue"); //Send string back to client 
    Udp.endPacket(); //Packet has been sent
    }
  }
  memset(_packetBuffer, 0, UDP_TX_PACKET_MAX_SIZE);
   
}
