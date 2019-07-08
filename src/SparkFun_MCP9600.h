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
  Ktype = 0b000,
  Jtype = 0b001,
  Ttype = 0b010,
  Ntype = 0b011,
  Stype = 0b100,
  Etype = 0b101,
  Btype = 0b110,
  Rtype = 0b111,
};

class MCP9600{
    public:
    MCP9600(uint8_t address = DEV_ADDR, TwoWire &wirePort = Wire);    //Class constructor
    bool isConnected();                                               //Returns true if the thermocouple will acknowledge over I2C, and false otherwise
    uint16_t deviceID();                                              //Returns the contents of the device ID register. The upper 8 bits are constant, but the lower contain revision data.
    bool checkDeviceID();                                             //Returns true if the constant upper 8 bits in the device ID register are what they should be according to the datasheet.

    float thermocoupleTemp();                                         //Returns the thermocouple temperature in degrees Celcius
    float ambientTemp();                                              //Returns the ambient (IC die) temperature in degrees Celcius
    float tempDelta();                                                //Returns the difference in temperature between the thermocouple and ambient junctions, in degrees Celcius

    uint8_t setThermocoupleType(thermocoupleType type);               //Sets the type of thermocouple connected to the MCP9600. Supported types are KJTNSEBR.
    uint8_t setFilterCoeffecients(uint8_t coeffecient);               //Sets how heavy of an exponential moving average filter to use. Set this to 0 for no filter, 1 for minimum filter, and 7 for maximum filter.

    private:
    TwoWire *_i2cPort;                                                //Generic connection to user's chosen I2C port
    uint8_t _deviceAddress;                                           //I2C address of the MCP9600
    uint8_t readSingleRegister(MCP9600_Register reg);                 //Attempts to read a single register, will keep trying for retryAttempts amount of times
    uint16_t readDoubleRegister(MCP9600_Register reg);                //Attempts to read two registers, will keep trying for retryAttempts amount of times
    uint8_t writeSingleRegister(MCP9600_Register reg, uint8_t data);  //Attempts to write data into a single 8-bit register. Does not check to make sure it was written successfully.
};
#endif