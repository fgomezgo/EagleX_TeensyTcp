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
  EthernetUDP _Udp; //Define UDP Object
}

void Comms::resetModule()
{
  // Begin reset sequence for WIZ820ip ethernet module
  pinMode(9, OUTPUT);
  digitalWrite(9, LOW);    // begin reset the WIZ820io
  pinMode(10, OUTPUT);
  digitalWrite(10, HIGH);  // de-select WIZ820io
  delay(1);
  digitalWrite(9, HIGH);   // end reset pulse  
}

void Comms::startComms()
{
   resetModule();
   Ethernet.begin(_mac, _ip); //Initialize Ethernet
   _Udp.begin(_localPort); //Initialize Udp
   delay(1500); //delay
}


void Comms::readComms()
{
   _packetSize = _Udp.parsePacket(); //Read the packetSize
   
  if(_packetSize>0){ //Check to see if a request is present
  
  _Udp.read(_packetBuffer, UDP_TX_PACKET_MAX_SIZE); //Reading the data request on the Udp
  String _datReq(_packetBuffer); //Convert _packetBuffer array to string _datReq
  Serial.println("Req");
  if (_datReq =="Red") { //See if Red was requested
  
    _Udp.beginPacket(_Udp.remoteIP(), _Udp.remotePort());  //Initialize Packet send
    _Udp.print("You are Asking for Red"); //Send string back to client 
    _Udp.endPacket(); //Packet has been sent
  }
   if (_datReq =="Green") { //See if Green was requested
  
    _Udp.beginPacket(_Udp.remoteIP(), _Udp.remotePort());  //Initialize Packet send
    _Udp.print("You are Asking for Green"); //Send string back to client 
    _Udp.endPacket(); //Packet has been sent
   }
    if (_datReq =="Blue") { //See if Red was requested
  
    _Udp.beginPacket(_Udp.remoteIP(), _Udp.remotePort());  //Initialize Packet send
    _Udp.print("You are Asking for Blue"); //Send string back to client 
    _Udp.endPacket(); //Packet has been sent
    }
  }
  memset(_packetBuffer, 0, UDP_TX_PACKET_MAX_SIZE);
   
}
