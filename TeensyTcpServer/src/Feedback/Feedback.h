/*
  Eagle X's rover feedback/IMU library Feedback.h - Library for managing feedback signals 
  from encoders, actuators, IMU
*/
#ifndef Feedback_h
#define Feedback_h

#include "Arduino.h"
#include "Adafruit_LIS3DH.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_BMP085_U.h>
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_10DOF.h>

// really unusual way of getting data, your read from two different addrs!
#define VEML6070_ADDR_H 0x39 ///< High address
#define VEML6070_ADDR_L 0x38 ///< Low address


/**************************************************************************/
/*! 
    @brief  integration time definitions
*/
/**************************************************************************/
typedef enum veml6070_integrationtime {
  VEML6070_HALF_T,
  VEML6070_1_T,
  VEML6070_2_T,
  VEML6070_4_T,
} veml6070_integrationtime_t;


//""""  Teporocho 2.0 """"*/

#ifndef DHT_H
#define DHT_H

#if ARDUINO >= 100
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif

#define DEBUG_PRINTER Serial

// Setup debug printing macros.
#ifdef DHT_DEBUG
    #define DEBUG_PRINT(...) { DEBUG_PRINTER.print(__VA_ARGS__); }
    #define DEBUG_PRINTLN(...) { DEBUG_PRINTER.println(__VA_ARGS__); }
#else
    #define DEBUG_PRINT(...) {}
    #define DEBUG_PRINTLN(...) {}
#endif

// Define types of sensors.
#define DHT11 11
#define DHT22 22
#define DHT21 21
#define AM2301 21


class Feedback{
    public:
        Feedback(char LIS3DH_CS[4], char LIS3DH_MOSI, char LIS3DH_MISO, char LIS3DH_CLK,uint8_t pin, uint8_t type, uint8_t count=6);
        bool suspensionImuConf();
        void suspensionImuUpdate();
        float getSuspensionRB();
        float getSuspensionRF();
        float getSuspensionLF();
        float getSuspensionLB();
        bool chassisImuConf();
        void chassisImuUpdate();
        float getChassisRoll();
        float getChassisPitch();
        float getChassisYaw();
        void begin(void);
        float readTemperature(bool S=false, bool force=false);
        float convertCtoF(float);
        float convertFtoC(float);
        float computeHeatIndex(float temperature, float percentHumidity, bool isFahrenheit=true);
        float readHumidity(bool force=false);
        boolean read(bool force=false);
        float getPressure();
        float getAltitude();
        float getTempExt();
        void updatePressure();
        void begin(veml6070_integrationtime_t itime, TwoWire *twoWire = &Wire);
        uint16_t readUV(void);
        void sleep(bool state);

    private:
        TwoWire *_i2c;

        typedef union {
            struct {
            uint8_t SD:1;
            uint8_t :1;
            uint8_t IT:2;
            uint8_t ACK_THD:1;
            uint8_t ACK:1;
            } bit;
            uint8_t reg;

        } commandRegister;

        commandRegister _commandRegister;


        Adafruit_LIS3DH _lis[4];
        float _suspensionsAngle[7];
        float _radsPerChange = 0.0523;
        /* Assign a unique ID to the sensors */
        Adafruit_10DOF                _dof   = Adafruit_10DOF();
        Adafruit_LSM303_Accel_Unified _accel = Adafruit_LSM303_Accel_Unified(30301);
        Adafruit_LSM303_Mag_Unified   _mag   = Adafruit_LSM303_Mag_Unified(30302);
        Adafruit_BMP085_Unified       _bmp   = Adafruit_BMP085_Unified(18001);
        /* Update this with the correct SLP for accurate altitude measurements */
        float _seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA;

        sensors_event_t _accel_event;
        sensors_event_t _mag_event;
        sensors_event_t _bmp_event;
        sensors_vec_t   _orientation;
        uint8_t data[5];
        uint8_t _pin, _type;
        #ifdef __AVR
            // Use direct GPIO access on an 8-bit AVR so keep track of the port and bitmask
            // for the digital pin connected to the DHT.  Other platforms will use digitalRead.
            uint8_t _bit, _port;
        #endif
        uint32_t _lastreadtime, _maxcycles;
        bool _lastresult;
        uint32_t expectPulse(bool level);
};

#endif
class InterruptLock {
  public:
   InterruptLock() {
    noInterrupts();
   }
   ~InterruptLock() {
    interrupts();
   }

};

#endif
