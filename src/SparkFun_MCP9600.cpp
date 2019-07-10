/******************************************************************************
SparkFun_MCP9600.h
SparkFun_MCP9600 Library Source File
Fischer Moseley @ SparkFun Electronics
Original Creation Date: July 8, 2019
https://github.com/sparkfunX/Qwiic_MCP9600_Thermocouple

This file implements the MCP9600 class, prototyped in SparkFun_MCP9600.cpp.

Development environment specifics:
	IDE: Arduino 1.8.9
	Hardware Platform: Arduino Uno/SparkFun Redboard
	MCP9600 Breakout Version: 1.0.0

This code is beerware; if you see me (or any other SparkFun employee) at the
local, and you've found our code helpful, please buy us a round!

Distributed as-is; no warranty is given.
******************************************************************************/
#include <Wire.h>
#include <SparkFun_MCP9600.h>

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

MCP9600::MCP9600(uint8_t address, TwoWire &wirePort){
  _deviceAddress = address; //grab the address that the sensor is on
  _i2cPort = &wirePort; //grab the port that the user wants to use
  _i2cPort->begin();
  _i2cPort->setClock(10000);
}

bool MCP9600::isConnected(){
  _i2cPort->beginTransmission(_deviceAddress);
  return (_i2cPort->endTransmission() == 0);
}

uint16_t MCP9600::deviceID(){
  return readDoubleRegister(DEVICE_ID);
}

bool MCP9600::checkDeviceID(){
  return (highByte(deviceID()) == DEV_ID_UPPER);
}

float MCP9600::thermocoupleTemp(){
  int16_t raw = readDoubleRegister(HOT_JUNC_TEMP);
  return ((float)raw * DEV_RESOLUTION);
}

float MCP9600::ambientTemp(){
  int16_t raw = readDoubleRegister(COLD_JUNC_TEMP);
  return ((float)raw * DEV_RESOLUTION);
}

float MCP9600::tempDelta(){
  int16_t raw = readDoubleRegister(DELTA_JUNC_TEMP);
  return ((float)raw * DEV_RESOLUTION);
}

uint8_t MCP9600::setThermocoupleType(thermocoupleType type){
  uint8_t config = readSingleRegister(THERMO_SENSOR_CONFIG);
  bitClear(config, 4); //clear the necessary bits so that they can be set
  bitClear(config, 5);
  bitClear(config, 6);
  config |= (type << 4);  //set the necessary bits in the config register
  if(writeSingleRegister(THERMO_SENSOR_CONFIG, config) != 0) return 1; //if write fails, return 1
  if(readSingleRegister(THERMO_SENSOR_CONFIG) != config) return 2; //if the register didn't take the new value, return 2

  return 0; //otherwise return 0
}

uint8_t MCP9600::setFilterCoeffecients(uint8_t coeffecient){
  if(coeffecient > 7) return 3; //return immediately if the value is too big

  uint8_t config = readSingleRegister(THERMO_SENSOR_CONFIG);
  config = config >> 3;
  config = config << 3;
  config |= coeffecient; //set the necessary bits in the config register

  writeSingleRegister(THERMO_SENSOR_CONFIG, config);
  for(uint8_t attempts = 0; attempts <= retryAttempts; attempts++){
    uint8_t readBack = readSingleRegister(THERMO_SENSOR_CONFIG);
    Serial.println(readBack, BIN);
    if(readBack == config) return 0;
  }
  return 1;
}

uint8_t MCP9600::readSingleRegister(MCP9600_Register reg){
  for(uint8_t attempts; attempts <= retryAttempts; attempts++){
    _i2cPort->beginTransmission(DEV_ADDR);
    _i2cPort->write(reg);
    _i2cPort->endTransmission();
    if(_i2cPort->requestFrom(DEV_ADDR, 1) != 0){
      return _i2cPort->read();     
    }
  }
  return;
}

uint16_t MCP9600::readDoubleRegister(MCP9600_Register reg){
  for(byte attempts; attempts <= retryAttempts; attempts++){
    _i2cPort->beginTransmission(DEV_ADDR);
    _i2cPort->write(reg);
    _i2cPort->endTransmission();

    if(_i2cPort->requestFrom(DEV_ADDR, 2) != 0){
      uint16_t data = _i2cPort->read() << 8;
      data |= _i2cPort->read();
      return data;
    }
  }
  return;
}

uint8_t MCP9600::writeSingleRegister(MCP9600_Register reg, uint8_t data){
  _i2cPort->beginTransmission(_deviceAddress);
  _i2cPort->write(reg);
  _i2cPort->write(data);
  return _i2cPort->endTransmission();
}