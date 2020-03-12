/*
  Temperature Alerts with the MCP9600 Thermocouple Amplifier
  By: Fischer Moseley
  SparkFun Electronics
  Date: July 12, 2019
  License: This code is public domain but you buy me a beer if you use this and we meet someday (Beerware License).

  This example shows how to use alerts on the MCP9600! The following code is intented to print out the temperature
  like normal, but when you touch the thermocouple it triggers an alert! It will also trigger another alert when
  you remove your hand. Both alerts will be printed over serial so that you can see them, and will also trigger pins
  on the MCP9600 to change state. These can be read with a multimeter or oscilloscope.

  Since each alert can only trigger on either a rising (cold -> hot) or falling (hot -> cold) transition, we 
  need to configure two alerts. 

  Hardware Connections:
  Attach the Qwiic Shield to your Arduino/Photon/ESP32 or other
  Plug the sensor onto the shield
  Serial.print it out at 115200 baud to serial monitor.
*/
#include <Wire.h>
#include <SparkFun_MCP9600.h>

MCP9600 tempSensor;
uint8_t risingAlert = 1; //What alert to use for detecting cold -> hot transitions.
uint8_t fallingAlert = 3; //What alert to use for detecting hot -> cold transitions.
                          //These numbers are arbitrary and can be anything from 1 to 4, but just can't be equal!

float alertTemp = 29.5;  //What temperature to trigger the alert at (before hysteresis).
                        //This is about the surface temperature of my finger, but please change this if 
                        //you have colder/warmer hands or if the ambient temperature is different.
uint8_t hysteresis = 2; //How much hysteresis to have, in degrees Celcius. Feel free to adjust this, but 2°C seems to be about right.

void setup(){
    Serial.begin(115200);
    Wire.begin();
    Wire.setClock(100000);
    tempSensor.begin();       // Uses the default address (0x60) for SparkFun Thermocouple Amplifier
    //tempSensor.begin(0x66); // Default address (0x66) for SparkX Thermocouple Amplifier

  //check if the sensor is connected
  if(tempSensor.isConnected()){
    Serial.println("Device will acknowledge!");
  }
  else {
    Serial.println("Device did not acknowledge! Freezing.");
    while(1); //hang forever
  }

  //check if the Device ID is correct
  if(tempSensor.checkDeviceID()){
    Serial.println("Device ID is correct!");        
  }
  else {
    Serial.println("Device ID is not correct! Freezing.");
    while(1); //hang forever
  }

  tempSensor.configAlertHysteresis(risingAlert, hysteresis);
  tempSensor.configAlertTemp(risingAlert, alertTemp);
  tempSensor.configAlertJunction(risingAlert, 0);
  tempSensor.configAlertEdge(risingAlert, HIGH);
  tempSensor.configAlertLogicLevel(risingAlert, HIGH);
  tempSensor.configAlertMode(risingAlert, 1);
  tempSensor.configAlertEnable(risingAlert, true);

  tempSensor.configAlertHysteresis(fallingAlert, hysteresis);
  tempSensor.configAlertTemp(fallingAlert, alertTemp);
  tempSensor.configAlertJunction(fallingAlert, 0);
  tempSensor.configAlertEdge(fallingAlert, LOW);
  tempSensor.configAlertLogicLevel(fallingAlert, HIGH);
  tempSensor.configAlertMode(fallingAlert, 1);
  tempSensor.configAlertEnable(fallingAlert, true);

  Serial.print("alert 1 hysteresis: ");
  Serial.println(tempSensor.readSingleRegister(ALERT1_HYSTERESIS), BIN);
  Serial.print("alert 1 limit: ");
  Serial.println(tempSensor.readDoubleRegister(ALERT1_LIMIT), BIN);
  Serial.print("alert 1 config: ");
  Serial.println(tempSensor.readSingleRegister(ALERT1_CONFIG), BIN);

  Serial.print("alert 3 hysteresis: ");
  Serial.println(tempSensor.readSingleRegister(ALERT3_HYSTERESIS), BIN);
  Serial.print("alert 3 limit: ");
  Serial.println(tempSensor.readDoubleRegister(ALERT3_LIMIT), BIN);
  Serial.print("alert 3 config: ");
  Serial.println(tempSensor.readSingleRegister(ALERT3_CONFIG), BIN);
}

unsigned long clock = millis();
uint16_t updateTime = 20;
void loop(){
  if((clock + updateTime) < millis()){ //update every 20ms without blocking!
    Serial.print("Thermocouple: ");
    Serial.print(tempSensor.getThermocoupleTemp());
    Serial.print(" °C   Ambient: ");
    Serial.print(tempSensor.getAmbientTemp());
    Serial.print(" °C   Temperature Delta: ");
    Serial.print(tempSensor.getTempDelta());
    Serial.print(" °C");

    if(tempSensor.isTempGreaterThanLimit(risingAlert)){
      Serial.print(" Temperature exceeds limit 1!");
    }

    if(tempSensor.isTempGreaterThanLimit(fallingAlert)){
      Serial.print(" Temperature exeeds limit 3!");
    }

    Serial.println(); 
    clock = millis();
  }

  if(Serial.available()){
    byte foo = Serial.read();
    if(foo == '1'){
      Serial.print("clearing alert 1: ");
      Serial.println(tempSensor.clearAlertPin(1));
    }

    if(foo == '3'){
      Serial.print("clearing alert 3: ");
      Serial.println(tempSensor.clearAlertPin(3));
    }
  }
}