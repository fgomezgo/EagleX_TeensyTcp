#include <Ethernet.h> //Load Ethernet Library
#include <EthernetUdp.h> //Load UDP Library
#include <SPI.h> //Load the SPI Library
#include <Adafruit_GPS.h>
#include <math.h>

// what's the name of the hardware serial port?
#define GPSSerial Serial5

// Connect to the GPS on the hardware port
Adafruit_GPS GPS(&GPSSerial);
     
// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences
#define GPSECHO false

uint32_t timer = millis();

byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xEE}; //Assign a mac address
IPAddress ip(192, 168, 1, 200); //Assign my IP adress
unsigned int localPort = 5000; //Assign a Port to talk over

char packetBuffer[UDP_TX_PACKET_MAX_SIZE];
String datReq; //String for our data
int packetSize; //Size of Packet
EthernetUDP Udp; //Define UDP Object


// converts lat/long from Adafruit
// degree-minute format to decimal-degrees
double convertDegMinToDecDeg (float degMin) {
  double min = 0.0;
  double decDeg = 0.0;
 
  //get the minutes, fmod() requires double
  min = fmod((double)degMin, 100.0);
 
  //rebuild coordinates in decimal degrees
  degMin = (int) ( degMin / 100 );
  decDeg = degMin + ( min / 60 );
 
  return decDeg;
}




 
void setup() {
  // begin reset sequence for WIZ820ip ethernet module
  
  pinMode(9, OUTPUT);
  digitalWrite(9, LOW);    // begin reset the WIZ820io
  pinMode(10, OUTPUT);
  digitalWrite(10, HIGH);  // de-select WIZ820io
  delay(1);
  digitalWrite(9, HIGH);   // end reset pulse
  
  Serial.begin(9600); //Turn on Serial Port
  Ethernet.begin(mac, ip); //Initialize Ethernet
  Udp.begin(localPort); //Initialize Udp
  delay(1500); //delay

  // Begin GPS configuration
  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  GPS.begin(9600);
  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // uncomment this line to turn on only the "minimum recommended" data
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  // For parsing data, we don't suggest using anything but either RMC only or RMC+GGA since
  // the parser doesn't care about other sentences at this time
  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // 1 Hz update rate
  // For the parsing code to work nicely and have time to sort thru the data, and
  // print it out we don't suggest using anything higher than 1 Hz
     
  // Request updates on antenna status, comment out to keep quiet
  GPS.sendCommand(PGCMD_ANTENNA);

  delay(1000);
  
  
  // Ask for firmware version
  GPSSerial.println(PMTK_Q_RELEASE);
  



  
}
 
void loop() {

  // read data from the GPS in the 'main loop'
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
  if (GPSECHO)
    if (c) Serial.print(c);
  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences!
    // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
    //Serial.println(GPS.lastNMEA()); // this also sets the newNMEAreceived() flag to false
    if (!GPS.parse(GPS.lastNMEA())) // this also sets the newNMEAreceived() flag to false
      return; // we can fail to parse a sentence in which case we should just wait for another
  }
  // if millis() or timer wraps around, we'll just reset it
  if (timer > millis()) timer = millis();

  if (millis() - timer > 2000) {
    timer = millis(); // reset the timer
    /*
    Serial.print("\nTime: ");
    Serial.print(GPS.hour, DEC); Serial.print(':');
    Serial.print(GPS.minute, DEC); Serial.print(':');
    Serial.print(GPS.seconds, DEC); Serial.print('.');
    Serial.println(GPS.milliseconds);
    Serial.print("Date: ");
    Serial.print(GPS.day, DEC); Serial.print('/');
    Serial.print(GPS.month, DEC); Serial.print("/20");
    Serial.println(GPS.year, DEC);
    Serial.print("Fix: "); Serial.print((int)GPS.fix);
    Serial.print(" quality: "); Serial.println((int)GPS.fixquality);*/
    if (GPS.fix) {
//      Serial.print("Location: ");
//      Serial.print(convertDegMinToDecDeg(GPS.latitude), 4); //Serial.print(GPS.lat);
//      Serial.print(", ");
//      Serial.print(-convertDegMinToDecDeg(GPS.longitude), 4); //Serial.println(GPS.lon);
      latitude = convertDegMinToDecDeg(GPS.latitude);
      longitude = -convertDegMinToDecDeg(GPS.longitude);
      gpspeed = GPS.speed * 0.5144444;
      //Serial.println(gpspeed);
      dtostrf(gpspeed,5,5,velocidad);
      //Serial.println(velocidad);
      gps_location.latitude = latitude;
      gps_location.longitude = longitude;
      gps_speed.data = gpspeed;
      gps_speed_string.data = velocidad;
      velocida.publish(&gps_speed_string);
      fix.publish(&gps_location);
      velocity.publish(&gps_speed);
      /*
      Serial.print("Speed (knots): "); Serial.println(GPS.speed);
      Serial.print("Angle: "); Serial.println(GPS.angle);
      Serial.print("Altitude: "); Serial.println(GPS.altitude);
      Serial.print("Satellites: "); Serial.println((int)GPS.satellites);*/
    }
  }


  // Ethernet shinanigan
  
  packetSize = Udp.parsePacket(); //Read theh packetSize
  
  if(packetSize>0){ //Check to see if a request is present
  
  Udp.read(packetBuffer, UDP_TX_PACKET_MAX_SIZE); //Reading the data request on the Udp
  String datReq(packetBuffer); //Convert packetBuffer array to string datReq
  
  if (datReq =="Red") { //See if Red was requested
  
    Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());  //Initialize Packet send
    Udp.print("You are Asking for Red"); //Send string back to client 
    Udp.endPacket(); //Packet has been sent
  }
   if (datReq =="Green") { //See if Green was requested
  
    Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());  //Initialize Packet send
    Udp.print("You are Asking for Green"); //Send string back to client 
    Udp.endPacket(); //Packet has been sent
   }
    if (datReq =="Blue") { //See if Red was requested
  
    Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());  //Initialize Packet send
    Udp.print("You are Asking for Blue"); //Send string back to client 
    Udp.endPacket(); //Packet has been sent
    }
  }
  memset(packetBuffer, 0, UDP_TX_PACKET_MAX_SIZE);
}

