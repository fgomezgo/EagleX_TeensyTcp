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
    void moduleReset();   // Resets module with a fancy routine
    void commsStart();    // Resets module and initializes ethernet settings
    boolean commsAvailable(); // Returns true if data at port
    unsigned int commsRead();
    void commsWrite();
    void commsReadOld();
    EthernetUDP Udp; //Define UDP Object
    int packetSize; //Size of Packet

  private:
    IPAddress _ip;
    byte *_mac;
    unsigned int _localPort;
    char _packetBuffer[UDP_TX_PACKET_MAX_SIZE];
    
};

#endif
