/*
  Eagle X's rover communication library Comms.h - Library for managing the Wiz820io ethernet module
*/
#ifndef Comms_h
#define Comms_h

#include "Arduino.h"
#include <Ethernet.h> //Load Ethernet Library
#include <EthernetUdp.h> //Load UDP Library
#include <SPI.h> //Load the SPI Library

class Comms
{
  public:
    Comms(IPAddress ip, byte *mac, unsigned int localPort);
    void resetModule();
    void startComms();
    void readComms();
    
  private:
    IPAddress _ip;
    byte *_mac;
    unsigned int _localPort;
    EthernetUDP _Udp; //Define UDP Object
    int _packetSize; //Size of Packet
    char _packetBuffer[UDP_TX_PACKET_MAX_SIZE];
    String _datReq; //String for our data
    
};

#endif
