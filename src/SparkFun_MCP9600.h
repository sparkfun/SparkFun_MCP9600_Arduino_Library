/******************************************************************************
SparkFun_MCP9600.h
SparkFun_MCP9600 Library Header File
Fischer Moseley @ SparkFun Electronics
Original Creation Date: July 8, 2019
https://github.com/sparkfunX/Qwiic_MCP9600_Thermocouple

This file prototypes the MCP9600 class, implemented in SparkFun_MCP9600.cpp.

Development environment specifics:
	IDE: Arduino 1.8.9
	Hardware Platform: Arduino Uno/SparkFun Redboard
	MCP9600 Breakout Version: 1.0.0

This code is beerware; if you see me (or any other SparkFun employee) at the
local, and you've found our code helpful, please buy us a round!

Distributed as-is; no warranty is given.
******************************************************************************/

#ifndef __SparkFun_MCP9600_H__
#define __SparkFun_MCP9600_H__

#include <Wire.h>
#include <Arduino.h>

#define DEV_ADDR 0x60 //device address of the MCP9600
#define DEV_ID_UPPER 0x40 //value of the upper half of the device ID register. lower half is used for device revision
#define DEV_RESOLUTION 0.0625 //device resolution (temperature in C that the LSB represents)
#define retryAttempts 3 //how many times to attempt to read a register from the thermocouple before giving up

// register pointers for various device functions
enum MCP9600_Register: uint8_t {
  HOT_JUNC_TEMP = 0x00,
  DELTA_JUNC_TEMP = 0x01,
  COLD_JUNC_TEMP = 0x02,
  RAW_ADC = 0x03,
  SENSOR_STATUS = 0x04,
  THERMO_SENSOR_CONFIG = 0x05,
  DEVICE_CONFIG = 0x06,
  ALERT1_CONFIG = 0x08,
  ALERT2_CONFIG = 0x09,
  ALERT3_CONFIG = 0x0A,
  ALERT4_CONFIG = 0x0B,
  ALERT1_HYSTERESIS = 0x0C,
  ALERT2_HYSTERESIS = 0x0D,
  ALERT3_HYSTERESIS = 0x0E,
  ALERT4_HYSTERESIS = 0x0F,
  ALERT1_LIMIT = 0x10,
  ALERT2_LIMIT = 0x11,
  ALERT3_LIMIT = 0x12,
  ALERT4_LIMIT = 0x13,
  DEVICE_ID = 0x20,
};

enum Thermocouple_Type: uint8_t {
  TYPE_K = 0b000,
  TYPE_J = 0b001,
  TYPE_T = 0b010,
  TYPE_N = 0b011,
  TYPE_S = 0b100,
  TYPE_E = 0b101,
  TYPE_B = 0b110,
  TYPE_R = 0b111,
};

enum Ambient_Resolution: bool {
  RES_ZERO_POINT_0625 = 0,
  RES_ZERO_POINT_25 = 1,
};

enum Thermocouple_Resolution: uint8_t {
  RES_18_BIT = 0b00,
  RES_16_BIT = 0b01,
  RES_14_BIT = 0b10,
  RES_12_BIT = 0b11,
};

enum Burst_Sample: uint8_t {
  SAMPLES_1 = 0b000,
  SAMPLES_2 = 0b001,
  SAMPLES_4 = 0b010,
  SAMPLES_8 = 0b011,
  SAMPLES_16 = 0b100,
  SAMPLES_32 = 0b101,
  SAMPLES_64 = 0b110,
  SAMPLES_128 = 0b111,
};

enum Shutdown_Mode: uint8_t {
  NORMAL = 0x00,
  SHUTDOWN = 0x01,
  BURST = 0x02,
};

class MCP9600{
  public:

  //Device status
  bool begin(uint8_t address = DEV_ADDR, TwoWire &wirePort = Wire); //Sets device I2C address to a user-specified address, over whatever port the user specifies. 
  bool available();                                                 //Returns true if the thermocouple (hot) junction temperature has been updated since we last checked. Also referred to as the data ready bit.
  bool isConnected();                                               //Returns true if the thermocouple will acknowledge over I2C, and false otherwise
  uint16_t deviceID();                                              //Returns the contents of the device ID register. The upper 8 bits are constant, but the lower contain revision data.
  bool checkDeviceID();                                             //Returns true if the constant upper 8 bits in the device ID register are what they should be according to the datasheet.
  bool resetToDefaults();                                           //Resets all device parameters to their default values. Returns 1 if there was an error, zero otherwise.

  //Sensor measurements
  float getThermocoupleTemp(bool units = true);                     //Returns the thermocouple temperature, and clears the data ready bit. Set units to true for Celcius, or false for freedom units (Fahrenheit)
  float getAmbientTemp(bool units = true);                          //Returns the ambient (IC die) temperature. Set units to true for Celcius, or false for freedom units (Fahrenheit)
  float getTempDelta(bool units = true);                            //Returns the difference in temperature between the thermocouple and ambient junctions. Set units to true for Celcius, or false for freedom units (Fahrenheit)
  signed long getRawADC();                                          //Returns the raw contents of the raw ADC register
  bool isInputRangeExceeded();                                      //Returns true if the MCP9600's EMF range has been exceeded, and false otherwise.

  //Measurement configuration
  bool setAmbientResolution(Ambient_Resolution res);                //Changes the resolution on the cold (ambient) junction, for either 0.0625 or 0.25 degree C resolution. Lower resolution reduces conversion time.
  Ambient_Resolution getAmbientResolution();                        //Returns the resolution on the cold (ambient) junction, for either 0.0625 or 0.25 degree C resolution. Lower resolution reduces conversion time.
  bool setThermocoupleResolution(Thermocouple_Resolution res);      //Changes the resolution on the hot (thermocouple) junction, for either 18, 16, 14, or 12-bit resolution. Lower resolution reduces conversion time.
  Thermocouple_Resolution getThermocoupleResolution();              //Returns the resolution on the hot (thermocouple) junction, for either 18, 16, 14, or 12-bit resolution. Lower resolution reduces conversion time.

  uint8_t setThermocoupleType(Thermocouple_Type type);              //Changes the type of thermocouple connected to the MCP9600. Supported types are KJTNSEBR.
  Thermocouple_Type getThermocoupleType();                          //Returns the type of thermocouple connected to the MCP9600 as found in its configuration register. Supported types are KJTNSEBR.
  uint8_t setFilterCoefficient(uint8_t coefficient);               //Changes the weight of the on-chip exponential moving average filter. Set this to 0 for no filter, 1 for minimum filter, and 7 for maximum filter.
  uint8_t getFilterCoefficient();                                  //Returns the weight of the on-chip exponential moving average filter.

  bool setBurstSamples(Burst_Sample samples);                       //Changes the amount of samples to take in burst mode. Returns 0 if set sucessfully, 1 otherwise.
  Burst_Sample getBurstSamples();                                   //Returns the amount of samples to take in burst mode, according to the device's configuration register.  
  bool burstAvailable();                                            //Returns true if all the burst samples have been taken and the results are ready. Returns false otherwise.
  bool startBurst();                                                //Initiates a burst on the MCP9600.
  bool setShutdownMode(Shutdown_Mode mode);                         //Changes the shutdown "operating" mode of the MCP9600. Configurable to Normal, Shutdown, and Burst. Returns 0 if properly set, 1 otherwise.
  Shutdown_Mode getShutdownMode();                                  //Returns the shutdown "operating" mode of the MCP9600. Configurable to Normal, Shutdown, and Burst.


  //Temperature Alerts 
  bool configAlertTemp(uint8_t number, float temp);                 //Configures the temperature at which to trigger the alert for a given alert number.
  bool configAlertJunction(uint8_t number, bool junction);          //Configures the junction to monitor the temperature of to trigger the alert. Set to zero for the thermocouple (hot) junction, or one for the ambient (cold) junction.
  bool configAlertHysteresis(uint8_t number, uint8_t hysteresis);   //Configures the hysteresis to use around the temperature set point, in degrees Celcius.
  bool configAlertEdge(uint8_t number, bool edge);                  //Configures whether to trigger the alert on the rising (cold -> hot) or falling (hot -> cold) edge of the temperature change. Set to 1 for rising, 0 for falling.
  bool configAlertLogicLevel(uint8_t number, bool level);           //Configures whether the hardware alert pin is active-high or active-low. Set to 1 for active-high, 0 for active-low.
  bool configAlertMode(uint8_t number, bool mode);                  //Configures whether the MCP9600 treats the alert like a comparator or an interrrupt. Set to 1 for interrupt, 0 for comparator. More information is on pg. 34 of the datasheet.
  bool configAlertEnable(uint8_t number, bool enable);              //Configures whether or not the interrupt is enabled or not. Set to 1 to enable, or 0 to disable.
  bool clearAlertPin(uint8_t number);                               //Clears the interrupt on the specified alert channel, resetting the value of the pin.           
  bool isTempGreaterThanLimit(uint8_t number);                      //Returns true if the interrupt has been triggered, false otherwise


  //debug
  uint8_t readSingleRegister(MCP9600_Register reg);                 //Attempts to read a single register, will keep trying for retryAttempts amount of times
  uint16_t readDoubleRegister(MCP9600_Register reg);                //Attempts to read two registers, will keep trying for retryAttempts amount of times
  bool writeSingleRegister(MCP9600_Register reg, uint8_t data);     //Attempts to write data into a single 8-bit register. Does not check to make sure it was written successfully. Returns 0 if there wasn't an error on I2C transmission, and 1 otherwise.
  bool writeDoubleRegister(MCP9600_Register reg, uint16_t data);    //Attempts to write data into a double (two 8-bit) registers. Does not check to make sure it was written successfully. Returns 0 if there wasn't an error on I2C transmission, and 1 otherwise.
 
 
  //Internal I2C Abstraction
  private:
  TwoWire *_i2cPort;                                                //Generic connection to user's chosen I2C port
  uint8_t _deviceAddress;                                           //I2C address of the MCP9600
};
#endif