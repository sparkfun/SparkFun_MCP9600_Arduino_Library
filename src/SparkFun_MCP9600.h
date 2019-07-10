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

#define DEV_ADDR 0x66 //device address of the MCP9600
#define DEV_ID_UPPER 0x40 //value of the upper half of the device ID register. lower half is used for device revision
#define DEV_RESOLUTION 0.0625 //device resolution (temperature in C that the LSB represents)
#define retryAttempts 3 //how many times to attempt to read a register from the thermocouple before giving up

// register pointers for various device functions
enum MCP9600_Register {
  HOT_JUNC_TEMP = 0x00,
  DELTA_JUNC_TEMP = 0x01,
  COLD_JUNC_TEMP = 0x02,
  STATUS = 0x04,
  THERMO_SENSOR_CONFIG = 0x05,
  DEVICE_CONFIG = 0x06,
  DEVICE_ID = 0x20,
};

enum thermocoupleType {
  K_type = 0b000,
  J_type = 0b001,
  T_type = 0b010,
  N_type = 0b011,
  S_type = 0b100,
  E_type = 0b101,
  B_type = 0b110,
  R_type = 0b111,
};

enum ambientResolution {
  RES_ZERO_POINT_0625 = 0b0,
  RES_ZERO_POINT_25 = 0b1,
};

enum thermocoupleResolution {
  RES_18_BIT = 0b00,
  RES_16_BIT = 0b01,
  RES_14_BIT = 0b10,
  RES_12_BIT = 0b11,
};

class MCP9600{
  public:

  //Class constructor
  MCP9600(uint8_t address = DEV_ADDR, TwoWire &wirePort = Wire);

  //Device status
  bool isConnected();                                               //Returns true if the thermocouple will acknowledge over I2C, and false otherwise
  uint16_t deviceID();                                              //Returns the contents of the device ID register. The upper 8 bits are constant, but the lower contain revision data.
  bool checkDeviceID();                                             //Returns true if the constant upper 8 bits in the device ID register are what they should be according to the datasheet.
  bool resetToDefaults();                                           //Resets all device parameters to their default values. Returns 1 if there was an error, zero otherwise.

  //Sensor measurements
  float thermocoupleTemp();                                         //Returns the thermocouple temperature in degrees Celcius
  float ambientTemp();                                              //Returns the ambient (IC die) temperature in degrees Celcius
  float tempDelta();                                                //Returns the difference in temperature between the thermocouple and ambient junctions, in degrees Celcius

  //Measurement configuration
  bool setAmbientResolution(ambientResolution res);                 //Changes the resolution on the cold (ambient) junction, for either 0.0625 or 0.25 degree C resolution. Lower resolution reduces conversion time.
  ambientResolution getAmbientResolution();                         //Returns the resolution on the cold (ambient) junction, for either 0.0625 or 0.25 degree C resolution. Lower resolution reduces conversion time.
  bool setThermocoupleResolution(thermocoupleResolution res);       //Changes the resolution on the hot (thermocouple) junction, for either 18, 16, 14, or 12-bit resolution. Lower resolution reduces conversion time.
  thermocoupleResolution getThermocoupleResolution();               //Returns the resolution on the hot (thermocouple) junction, for either 18, 16, 14, or 12-bit resolution. Lower resolution reduces conversion time.

  uint8_t setThermocoupleType(thermocoupleType type);               //Changes the type of thermocouple connected to the MCP9600. Supported types are KJTNSEBR.
  thermocoupleType getThermocoupleType();                           //Returns the type of thermocouple connected to the MCP9600 as found in its configuration register. Supported types are KJTNSEBR.
  uint8_t setFilterCoeffecients(uint8_t coeffecient);               //Changes the weight of the on-chip exponential moving average filter. Set this to 0 for no filter, 1 for minimum filter, and 7 for maximum filter.
  uint8_t getFilterCoeffecients();                                  //Returns the weight of the on-chip exponential moving average filter.


  //Internal I2C Abstraction
  private:
  TwoWire *_i2cPort;                                                //Generic connection to user's chosen I2C port
  uint8_t _deviceAddress;                                           //I2C address of the MCP9600
  uint8_t readSingleRegister(MCP9600_Register reg);                 //Attempts to read a single register, will keep trying for retryAttempts amount of times
  uint16_t readDoubleRegister(MCP9600_Register reg);                //Attempts to read two registers, will keep trying for retryAttempts amount of times
  bool writeSingleRegister(MCP9600_Register reg, uint8_t data);     //Attempts to write data into a single 8-bit register. Does not check to make sure it was written successfully.
};
#endif