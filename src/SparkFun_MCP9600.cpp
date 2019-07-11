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

//Class constructor
MCP9600::MCP9600(uint8_t address, TwoWire &wirePort){
  _deviceAddress = address; //grab the address that the sensor is on
  _i2cPort = &wirePort; //grab the port that the user wants to use
  _i2cPort->begin();
  _i2cPort->setClock(10000);
}


/*-------------------------------- Device Status ------------------------*/

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


/*----------------------------- Sensor Measurements ---------------------*/

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

signed long MCP9600::rawADC(){
  for(byte attempts; attempts <= retryAttempts; attempts++){
    _i2cPort->beginTransmission(DEV_ADDR);
    _i2cPort->write(RAW_ADC);
    _i2cPort->endTransmission();

    if(_i2cPort->requestFrom(DEV_ADDR, 3) != 0){
      signed long data = _i2cPort->read() << 16;
      data |= _i2cPort->read() << 8;
      data |= _i2cPort->read();
      return data;
    }
  }
}


/*--------------------------- Measurement Configuration --------------- */

bool MCP9600::setAmbientResolution(Ambient_Resolution res){
  uint8_t config = readSingleRegister(DEVICE_CONFIG); //get current device configuration so we don't wipe everything else
  bitWrite(config, 7, res); //set the bit that controls the ambient (cold) junction resolution

  bool failed = writeSingleRegister(DEVICE_CONFIG, config); //write new config register to MCP9600
  failed |= (readSingleRegister(DEVICE_CONFIG) != config); //double check that it was set properly
  return failed; //return 1 if the write failed or the register wasn't properly set, 0 otherwise
}

Ambient_Resolution MCP9600::getAmbientResolution(){
  uint8_t config = readSingleRegister(DEVICE_CONFIG); //grab current device configuration 
  return bitRead(config, 7); //return 7th bit from config register
}

bool MCP9600::setThermocoupleResolution(Thermocouple_Resolution res){
  uint8_t config = readSingleRegister(DEVICE_CONFIG); //grab current device configuration so we don't wipe everything else
  bool highResolutionBit = bitRead(res, 1);
  bool lowResolutionBit = bitRead(res, 0);
  bitWrite(config, 6, highResolutionBit); //set 6th bit of config register to 1st bit of the resolution
  bitWrite(config, 5, lowResolutionBit); //set 5th bit of config register to 0th bit of the resolution

  bool failed = writeSingleRegister(DEVICE_CONFIG, config); //write new config register to MCP9600
  failed |= (readSingleRegister(DEVICE_CONFIG) != config); //double check that it was written properly
  return failed; //return 1 if the write failed or the register wasn't properly set, 0 otherwise
}

Thermocouple_Resolution MCP9600::getThermocoupleResolution(){
  uint8_t config = readSingleRegister(DEVICE_CONFIG); //grab current device configuration 
  Thermocouple_Resolution res; //define new thermocoupleResolution enum to return
  bool highResolutionBit = bitRead(config, 6);
  bool lowResolutionBit = bitRead(config, 5);
  bitWrite(res, 1, highResolutionBit); //set 1st bit of the enum to the 6th bit of the config register
  bitWrite(res, 0, lowResolutionBit); //set 0th bit of the enum to the 5th bit of the config register
  return res;
}

uint8_t MCP9600::setThermocoupleType(Thermocouple_Type type){
  uint8_t config = readSingleRegister(THERMO_SENSOR_CONFIG); //grab current device configuration so we don't wipe everything else
  bitClear(config, 4); //clear the necessary bits so that they can be set
  bitClear(config, 5);
  bitClear(config, 6);
  config |= (type << 4);  //set the necessary bits in the config register
  if(writeSingleRegister(THERMO_SENSOR_CONFIG, config)) return 1; //if write fails, return 1
  if(readSingleRegister(THERMO_SENSOR_CONFIG) != config) return 2; //if the register didn't take the new value, return 2

  return 0; //otherwise return 0
}

Thermocouple_Type MCP9600::getThermocoupleType(){
  uint8_t config = readSingleRegister(THERMO_SENSOR_CONFIG);
  return (config >> 4); //clear the non-thermocouple-type bits in the config registe
}

uint8_t MCP9600::setFilterCoeffecients(uint8_t coeffecient){
  if(coeffecient > 7) return 3; //return immediately if the value is too big

  uint8_t config = readSingleRegister(THERMO_SENSOR_CONFIG);
  bitWrite(coeffecient, 3, bitRead(config, 3));
  bitWrite(coeffecient, 4, bitRead(config, 3));
  bitWrite(coeffecient, 5, bitRead(config, 3));
  bitWrite(coeffecient, 6, bitRead(config, 3));
  bitWrite(coeffecient, 7, bitRead(config, 3));

  //config = config >> 3;
  //config = config << 3;
  //config |= coeffecient; //set the necessary bits in the config register

  return writeSingleRegister(THERMO_SENSOR_CONFIG, coeffecient);
}

uint8_t MCP9600::getFilterCoeffecients(){
  uint8_t config = readSingleRegister(THERMO_SENSOR_CONFIG);
  uint8_t coeff = 0;
  bitWrite(coeff, 0, bitRead(config, 0));
  bitWrite(coeff, 1, bitRead(config, 1));
  bitWrite(coeff, 2, bitRead(config, 2));

  return coeff; //clear the non-filter-coeffecients data in the config register
}

bool MCP9600::setBurstSamples(Burst_Sample samples){
  uint8_t config = readSingleRegister(DEVICE_CONFIG);
  bool highResolutionBit = bitRead(samples, 2);
  bool midResolutionBit = bitRead(samples, 1);
  bool lowResolutionBit = bitRead(samples, 0);
  bitWrite(config, 4, highResolutionBit); //write 2nd bit of samples to 4th of config
  bitWrite(config, 3, midResolutionBit); //write 1st bit of samples to 3rd of config
  bitWrite(config, 2, lowResolutionBit); //write 0th bit of samples to 2nd of config

  bool failed = writeSingleRegister(DEVICE_CONFIG, config); //write new config register to MCP9600
  failed |= (readSingleRegister(DEVICE_CONFIG) != config); //double check that it was written properly
  return failed; //return 1 if the write failed or the register wasn't properly set, 0 otherwise
}

Burst_Sample MCP9600::getBurstSamples(){
  uint8_t config = readSingleRegister(DEVICE_CONFIG);
  bool highResolutionBit = bitRead(config, 4);
  bool midResolutionBit = bitRead(config, 3);
  bool lowResolutionBit = bitRead(config, 2);
  Burst_Sample samples;
  bitWrite(samples, 2, highResolutionBit); //write 4th bit of config to 2nd bit of samples
  bitWrite(samples, 1, midResolutionBit); //write 3rd bit of config to 1st bit of samples
  bitWrite(samples, 0, lowResolutionBit); //write 2nd bit of config to 0th bit of samples
  return samples;
}

bool MCP9600::burstAvailable(){
  uint8_t status = readSingleRegister(SENSOR_STATUS);
  return (status >> 7); //return only the 7th bit where the burst complete flag is
}

bool MCP9600::startBurst(){
  uint8_t status = readSingleRegister(SENSOR_STATUS);
  bitWrite(status, 7, 0); //clear the 7th bit of the status register, and send over I2C

  bool failed =  writeSingleRegister(SENSOR_STATUS, status); //return whether the write was successful
  failed |= setShutdownMode(BURST);

  return failed;
}

bool MCP9600::setShutdownMode(Shutdown_Mode mode){
  uint8_t config = readSingleRegister(DEVICE_CONFIG);
  config = (config >> 2) << 2; //clear last two bits of the device config register
  config |= mode;

  bool failed = writeSingleRegister(DEVICE_CONFIG, config); //write new config register to MCP9600
  failed |= (readSingleRegister(DEVICE_CONFIG) != config); //double check that it was written properly
  return failed; //return 1 if the write failed or the register wasn't properly set, 0 otherwise
}

Shutdown_Mode MCP9600::getShutdownMode(){
  uint8_t config = readSingleRegister(DEVICE_CONFIG);
  uint8_t mode = 0;
  bitWrite(mode, 0, bitRead(config, 0));
  bitWrite(mode, 1, bitRead(config, 1));
  return mode; //clear all bits except the last two and return
}

/*------------------------- Internal I2C Abstraction ---------------- */

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

bool MCP9600::writeSingleRegister(MCP9600_Register reg, uint8_t data){
  _i2cPort->beginTransmission(_deviceAddress);
  _i2cPort->write(reg);
  _i2cPort->write(data);
  return (_i2cPort->endTransmission() != 0);
}