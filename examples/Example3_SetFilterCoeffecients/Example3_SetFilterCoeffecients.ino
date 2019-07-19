/*
  Setting Filter Coeffecients with the MCP9600 Thermocouple Amplifier
  By: Fischer Moseley
  SparkFun Electronics
  Date: July 8, 2019
  License: This code is public domain but you buy me a beer if you use this and we meet someday (Beerware License).

  This example outputs the ambient and thermocouple temperatures from the MCP9600 sensor, but allows the filtering
  onboard the MCP9600 to be controlled. The MCP9600 implements an exponential running average filter, the
  "strength" of which is programmable! The setFilterCoeffecients function takes a coeffecient between 0 and 7, 
  where 0 disables the filter, 1 corresponds to minimum filtering, and 7 enables maximum filtering. The "strength"
  of the filter just refers to how long it takes for the filter to respond to a step function input. 

  Hardware Connections:
  Attach the Qwiic Shield to your Arduino/Photon/ESP32 or other
  Plug the sensor onto the shield
  Serial.print it out at 115200 baud to serial monitor.
*/

#include <SparkFun_MCP9600.h>
MCP9600 tempSensor;
uint8_t coeffecient = 3;

void setup(){
    Serial.begin(115200);
    Wire.begin();
    Wire.setClock(100000);

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
        Serial.print("Device ID reported as: 0x");
        Serial.println(tempSensor.readDoubleRegister(DEVICE_ID), HEX);
    }
    else {
        Serial.println("Device ID is not correct! Freezing.");
        while(1); //hang forever
    }

    //print the filter coeffecient that's about to be set
    Serial.print("Setting Filter Coeffecient to ");
    Serial.print(coeffecient);
    Serial.println("!");

    tempSensor.setFilterCoeffecients(coeffecient);

    //tell us if the coeffecient was set sucessfully
    if(tempSensor.getFilterCoeffecients() == coeffecient){
        Serial.println("Filter Coeffecients set sucessfully!");
    }

    else{
        Serial.println("Setting filter coeffecient failed!");
        Serial.println("The value of the coeffecient is: ");
        Serial.println(tempSensor.getFilterCoeffecients(), BIN);
    }
}

void loop(){ //print the thermocouple, ambient and delta temperatures every 200ms
    Serial.print("Thermocouple: ");
    Serial.print(tempSensor.thermocoupleTemp());
    Serial.print(" °C   Ambient: ");
    Serial.print(tempSensor.ambientTemp());
    Serial.print(" °C   Temperature Delta: ");
    Serial.print(tempSensor.tempDelta());
    Serial.print(" °C");
    Serial.print("   Current Coeffecient: ");
    Serial.print(tempSensor.getFilterCoeffecients(), BIN);
    Serial.println();
    delay(200);
}