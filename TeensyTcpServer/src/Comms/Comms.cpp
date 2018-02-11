/*
  Eagle X's rover communication library h - Library for managing the Wiz820io ethernet module
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

void Comms::start()
{
   moduleReset();
   Ethernet.begin(_mac, _ip); //Initialize Ethernet
   Udp.begin(_localPort); //Initialize Udp
   delay(1500); //delay
}

void Comms::writePrecision(float data, char precision){
  Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());  //Initialize Packet send
  Udp.print(data, precision); //Send string back to client 
  Udp.endPacket(); //Packet has been sent
}

void Comms::write(String data){
  Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());  //Initialize Packet send
  Udp.print(data); //Send string back to client 
  Udp.endPacket(); //Packet has been sent
}

boolean Comms::available()
{
  packetSize = Udp.parsePacket(); //Read the packetSize
  if(packetSize > 0){
    return 1;
  }else{
    return 0;
  }
}

unsigned int Comms::read()
{
  Udp.read(_packetBuffer, UDP_TX_PACKET_MAX_SIZE); //Reading the data request on the Udp
  unsigned int ID = (_packetBuffer[0] <<24) | (_packetBuffer[1] <<16) | (_packetBuffer[2] <<8) | _packetBuffer[3];
  memset(_packetBuffer, 0, UDP_TX_PACKET_MAX_SIZE);
  return ID;
}

