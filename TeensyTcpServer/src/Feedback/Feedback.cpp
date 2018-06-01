/*
  Eagle X's rover feedback/IMU library Feedback.h - Library for managing feedback signals 
  from encoders, actuators, IMU
*/
#include "Feedback.h"



Feedback::Feedback(char LIS3DH_CS[4], char LIS3DH_MOSI, char LIS3DH_MISO, char LIS3DH_CLK,uint8_t pin, uint8_t type, uint8_t count){
    for(int i = 0; i < 4; i++){
        _lis[i] = Adafruit_LIS3DH(LIS3DH_CS[i], LIS3DH_MOSI, LIS3DH_MISO, LIS3DH_CLK);
    }
    _pin = pin;
    _type = type;
    #ifdef __AVR
      _bit = digitalPinToBitMask(pin);
      _port = digitalPinToPort(pin);
    #endif
    _maxcycles = microsecondsToClockCycles(1000);  // 1 millisecond timeout for
                                                  // reading pulses from DHT sensor.
    // Note that count is now ignored as the DHT reading algorithm adjusts itself
    // basd on the speed of the processor.
}

bool Feedback::suspensionImuConf(){
     bool flag = false;
     for(int i = 0; i < 4; i++){
         if (! _lis[i].begin(0x18)) {   // change this to 0x19 for alternative i2c address
            flag = true;
        }
        delay(10);
     }
     return flag;
}


void Feedback::suspensionImuUpdate(){
    for(int i = 0; i < 4; i++){
        _lis[i].read();
        if ( abs( ((_lis[i].y / 5095.54) - _suspensionsAngle[i])) > _radsPerChange * 2){
            _suspensionsAngle[i] = (_lis[i].y / 5095.54); 
        } 
        delay(5);
    }
}

float Feedback::getSuspensionRB(){
    suspensionImuUpdate();
    return _suspensionsAngle[0];
}

float Feedback::getSuspensionRF(){
    suspensionImuUpdate();
    return _suspensionsAngle[1];
}

float Feedback::getSuspensionLF(){
    suspensionImuUpdate();
    return _suspensionsAngle[2];
}

float Feedback::getSuspensionLB(){
    suspensionImuUpdate();
    return _suspensionsAngle[3];
}

bool Feedback::chassisImuConf(){
    bool flag = false;
    if(!_accel.begin())
    {
        /* There was a problem detecting the LSM303 ... check your connections */
        flag = true;
    }
    if(!_mag.begin())
    {
        /* There was a problem detecting the LSM303 ... check your connections */
        flag = true;
    }
    if(!_bmp.begin())
    {
        /* There was a problem detecting the BMP180 ... check your connections */
        flag = true;
    }
    return flag;
}

void Feedback::chassisImuUpdate(){
    _accel.getEvent(&_accel_event);
    if (_dof.accelGetOrientation(&_accel_event, &_orientation)){
        if ( abs(((-_orientation.roll) / 50) -_suspensionsAngle[4]) > _radsPerChange * 2) { //and abs( ((-orientation.roll)/50)-positions[2])<radsPerChange*4) {
            _suspensionsAngle[4]=(-_orientation.roll) / 50;
        }
        if ( abs( ((-_orientation.pitch)/50) -_suspensionsAngle[5]) > _radsPerChange * 2) { //and abs( ((-orientation.pitch)/50)-positions[1])<radsPerChange*4) {
            _suspensionsAngle[5] = _orientation.pitch / 50;  
        }
    }

    _mag.getEvent(&_mag_event);
    if (_dof.magGetOrientation(SENSOR_AXIS_Z, &_mag_event, &_orientation))
    {    
        if ( abs( ((-_orientation.heading)/50) -_suspensionsAngle[6]) > _radsPerChange * 2){ //and abs( ((-orientation.heading)/50)-positions[0])<radsPerChange*4)
            _suspensionsAngle[6] = -_orientation.heading / 50;  
        }
    }
}

float Feedback::getChassisRoll(){
    chassisImuUpdate();
    return _suspensionsAngle[4];
}

float Feedback::getChassisPitch(){
    chassisImuUpdate();
    return _suspensionsAngle[5];
}

float Feedback::getChassisYaw(){
    chassisImuUpdate();
    return _suspensionsAngle[6];
}

/*""""      Teporocho 2.0 """"*/




#define MIN_INTERVAL 2000



void Feedback::begin(void) {
  // set up the pins!
  pinMode(_pin, INPUT_PULLUP);
  // Using this value makes sure that millis() - lastreadtime will be
  // >= MIN_INTERVAL right away. Note that this assignment wraps around,
  // but so will the subtraction.
  _lastreadtime = -MIN_INTERVAL;
  DEBUG_PRINT("Max clock cycles: "); DEBUG_PRINTLN(_maxcycles, DEC);
}

//boolean S == Scale.  True == Fahrenheit; False == Celcius
float Feedback::readTemperature(bool S, bool force) {
  float f = NAN;

  if (read(force)) {
    switch (_type) {
    case DHT11:
      f = data[2];
      if(S) {
        f = convertCtoF(f);
      }
      break;
    case DHT22:
    case DHT21:
      f = data[2] & 0x7F;
      f *= 256;
      f += data[3];
      f *= 0.1;
      if (data[2] & 0x80) {
        f *= -1;
      }
      if(S) {
        f = convertCtoF(f);
      }
      break;
    }
  }
  return f;
}

float Feedback::convertCtoF(float c) {
  return c * 1.8 + 32;
}

float Feedback::convertFtoC(float f) {
  return (f - 32) * 0.55555;
}

float Feedback::readHumidity(bool force) {
  float f = NAN;
  if (read()) {
    switch (_type) {
    case DHT11:
      f = data[0];
      break;
    case DHT22:
    case DHT21:
      f = data[0];
      f *= 256;
      f += data[1];
      f *= 0.1;
      break;
    }
  }
  return f;
}

//boolean isFahrenheit: True == Fahrenheit; False == Celcius
float Feedback::computeHeatIndex(float temperature, float percentHumidity, bool isFahrenheit) {
  // Using both Rothfusz and Steadman's equations
  // http://www.wpc.ncep.noaa.gov/html/heatindex_equation.shtml
  float hi;

  if (!isFahrenheit)
    temperature = convertCtoF(temperature);

  hi = 0.5 * (temperature + 61.0 + ((temperature - 68.0) * 1.2) + (percentHumidity * 0.094));

  if (hi > 79) {
    hi = -42.379 +
             2.04901523 * temperature +
            10.14333127 * percentHumidity +
            -0.22475541 * temperature*percentHumidity +
            -0.00683783 * pow(temperature, 2) +
            -0.05481717 * pow(percentHumidity, 2) +
             0.00122874 * pow(temperature, 2) * percentHumidity +
             0.00085282 * temperature*pow(percentHumidity, 2) +
            -0.00000199 * pow(temperature, 2) * pow(percentHumidity, 2);

    if((percentHumidity < 13) && (temperature >= 80.0) && (temperature <= 112.0))
      hi -= ((13.0 - percentHumidity) * 0.25) * sqrt((17.0 - abs(temperature - 95.0)) * 0.05882);

    else if((percentHumidity > 85.0) && (temperature >= 80.0) && (temperature <= 87.0))
      hi += ((percentHumidity - 85.0) * 0.1) * ((87.0 - temperature) * 0.2);
  }

  return isFahrenheit ? hi : convertFtoC(hi);
}

boolean Feedback::read(bool force) {
  // Check if sensor was read less than two seconds ago and return early
  // to use last reading.
  uint32_t currenttime = millis();
  if (!force && ((currenttime - _lastreadtime) < 2000)) {
    return _lastresult; // return last correct measurement
  }
  _lastreadtime = currenttime;

  // Reset 40 bits of received data to zero.
  data[0] = data[1] = data[2] = data[3] = data[4] = 0;

  // Send start signal.  See DHT datasheet for full signal diagram:
  //   http://www.adafruit.com/datasheets/Digital%20humidity%20and%20temperature%20sensor%20AM2302.pdf

  // Go into high impedence state to let pull-up raise data line level and
  // start the reading process.
  digitalWrite(_pin, HIGH);
  delay(250);

  // First set data line low for 20 milliseconds.
  pinMode(_pin, OUTPUT);
  digitalWrite(_pin, LOW);
  delay(20);

  uint32_t cycles[80];
  {
    // Turn off interrupts temporarily because the next sections are timing critical
    // and we don't want any interruptions.
    InterruptLock lock;

    // End the start signal by setting data line high for 40 microseconds.
    digitalWrite(_pin, HIGH);
    delayMicroseconds(40);

    // Now start reading the data line to get the value from the DHT sensor.
    pinMode(_pin, INPUT_PULLUP);
    delayMicroseconds(10);  // Delay a bit to let sensor pull data line low.

    // First expect a low signal for ~80 microseconds followed by a high signal
    // for ~80 microseconds again.
    if (expectPulse(LOW) == 0) {
      DEBUG_PRINTLN(F("Timeout waiting for start signal low pulse."));
      _lastresult = false;
      return _lastresult;
    }
    if (expectPulse(HIGH) == 0) {
      DEBUG_PRINTLN(F("Timeout waiting for start signal high pulse."));
      _lastresult = false;
      return _lastresult;
    }

    // Now read the 40 bits sent by the sensor.  Each bit is sent as a 50
    // microsecond low pulse followed by a variable length high pulse.  If the
    // high pulse is ~28 microseconds then it's a 0 and if it's ~70 microseconds
    // then it's a 1.  We measure the cycle count of the initial 50us low pulse
    // and use that to compare to the cycle count of the high pulse to determine
    // if the bit is a 0 (high state cycle count < low state cycle count), or a
    // 1 (high state cycle count > low state cycle count). Note that for speed all
    // the pulses are read into a array and then examined in a later step.
    for (int i=0; i<80; i+=2) {
      cycles[i]   = expectPulse(LOW);
      cycles[i+1] = expectPulse(HIGH);
    }
  } // Timing critical code is now complete.

  // Inspect pulses and determine which ones are 0 (high state cycle count < low
  // state cycle count), or 1 (high state cycle count > low state cycle count).
  for (int i=0; i<40; ++i) {
    uint32_t lowCycles  = cycles[2*i];
    uint32_t highCycles = cycles[2*i+1];
    if ((lowCycles == 0) || (highCycles == 0)) {
      DEBUG_PRINTLN(F("Timeout waiting for pulse."));
      _lastresult = false;
      return _lastresult;
    }
    data[i/8] <<= 1;
    // Now compare the low and high cycle times to see if the bit is a 0 or 1.
    if (highCycles > lowCycles) {
      // High cycles are greater than 50us low cycle count, must be a 1.
      data[i/8] |= 1;
    }
    // Else high cycles are less than (or equal to, a weird case) the 50us low
    // cycle count so this must be a zero.  Nothing needs to be changed in the
    // stored data.
  }

  DEBUG_PRINTLN(F("Received:"));
  DEBUG_PRINT(data[0], HEX); DEBUG_PRINT(F(", "));
  DEBUG_PRINT(data[1], HEX); DEBUG_PRINT(F(", "));
  DEBUG_PRINT(data[2], HEX); DEBUG_PRINT(F(", "));
  DEBUG_PRINT(data[3], HEX); DEBUG_PRINT(F(", "));
  DEBUG_PRINT(data[4], HEX); DEBUG_PRINT(F(" =? "));
  DEBUG_PRINTLN((data[0] + data[1] + data[2] + data[3]) & 0xFF, HEX);

  // Check we read 40 bits and that the checksum matches.
  if (data[4] == ((data[0] + data[1] + data[2] + data[3]) & 0xFF)) {
    _lastresult = true;
    return _lastresult;
  }
  else {
    DEBUG_PRINTLN(F("Checksum failure!"));
    _lastresult = false;
    return _lastresult;
  }
}

// Expect the signal line to be at the specified level for a period of time and
// return a count of loop cycles spent at that level (this cycle count can be
// used to compare the relative time of two pulses).  If more than a millisecond
// ellapses without the level changing then the call fails with a 0 response.
// This is adapted from Arduino's pulseInLong function (which is only available
// in the very latest IDE versions):
//   https://github.com/arduino/Arduino/blob/master/hardware/arduino/avr/cores/arduino/wiring_pulse.c
uint32_t Feedback::expectPulse(bool level) {
  uint32_t count = 0;
  // On AVR platforms use direct GPIO port access as it's much faster and better
  // for catching pulses that are 10's of microseconds in length:
  #ifdef __AVR
    uint8_t portState = level ? _bit : 0;
    while ((*portInputRegister(_port) & _bit) == portState) {
      if (count++ >= _maxcycles) {
        return 0; // Exceeded timeout, fail.
      }
    }
  // Otherwise fall back to using digitalRead (this seems to be necessary on ESP8266
  // right now, perhaps bugs in direct port access functions?).
  #else
    while (digitalRead(_pin) == level) {
      if (count++ >= _maxcycles) {
        return 0; // Exceeded timeout, fail.
      }
    }
  #endif

  return count;
}

float Feedback::getPressure(){
    updatePressure();
    Serial.print(_bmp_event.pressure);
    return _bmp_event.pressure;
}

float Feedback::getAltitude(){
    float temperature;
    updatePressure();
    _bmp.getTemperature(&temperature);
    Serial.print(_bmp.pressureToAltitude(_seaLevelPressure,
                                        _bmp_event.pressure,
                                        temperature));
    return _bmp.pressureToAltitude(_seaLevelPressure,
                                        _bmp_event.pressure,
                                        temperature);
}

float Feedback::getTempExt(){
    float temperature;
    updatePressure();
    _bmp.getTemperature(&temperature);
    return temperature;
}

void Feedback::updatePressure(){
    _bmp.getEvent(&_bmp_event);
}

void Feedback::begin(veml6070_integrationtime_t itime, TwoWire *twoWire) {
  _i2c = twoWire;

  //default setting
  _commandRegister.reg = 0x02;
  _commandRegister.bit.IT = itime;

  _i2c->begin();
  _i2c->beginTransmission(VEML6070_ADDR_L);
  _i2c->write(_commandRegister.reg);
  _i2c->endTransmission();
  delay(500);
}

/**************************************************************************/
/*! 
    @brief  read the chips UV sensor
    @return the UV reading as a 16 bit integer
*/
/**************************************************************************/
uint16_t Feedback::readUV() {
  if (_i2c->requestFrom(VEML6070_ADDR_H, 1) != 1) return -1;
  uint16_t uvi = _i2c->read();
  uvi <<= 8;
  if (_i2c->requestFrom(VEML6070_ADDR_L, 1) != 1) return -1;
  uvi |= _i2c->read();

  return uvi;  
}

/**************************************************************************/
/*! 
    @brief  enter or exit sleep (shutdown) mode. While in sleep mode
      the chip draws ~1uA
    @param state true to enter sleep mode, false to exit
*/
/**************************************************************************/
void Feedback::sleep(bool state) {
  _commandRegister.bit.SD = state;

  _i2c->beginTransmission(VEML6070_ADDR_L);
  _i2c->write(_commandRegister.reg);
  _i2c->endTransmission();
}