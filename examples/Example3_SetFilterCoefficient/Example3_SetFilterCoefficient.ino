/*
  Setting Filter Coefficient with the MCP9600 Thermocouple Amplifier
  By: Fischer Moseley
  SparkFun Electronics
  Date: July 8, 2019
  License: This code is public domain but you buy me a beer if you use this and we meet someday (Beerware License).

  This example outputs the ambient and thermocouple temperatures from the MCP9600 sensor, but allows the filtering
  onboard the MCP9600 to be controlled. The MCP9600 implements an exponential running average filter, the
  "strength" of which is programmable! The setFilterCoefficient function takes a coefficient between 0 and 7, 
  where 0 disables the filter, 1 corresponds to minimum filtering, and 7 enables maximum filtering. The "strength"
  of the filter just refers to how long it takes for the filter to respond to a step function input. 

  Quick Note! For some reason the getFilterCoefficient() function is a little wierd about returning the proper
  data. This is a known issue and while we've done our best to fix it, every once in a while it might return a 0,
  or the wrong value entirely. We think this is an issue with the MCP9600, and there's not much we can do about it.
  If you'd like to learn more or contact us, check out this issue on GitHub!

  https://github.com/sparkfun/SparkFun_MCP9600_Arduino_Library/issues/1

  Hardware Connections:
  Attach the Qwiic Shield to your Arduino/Photon/ESP32 or other
  Plug the sensor onto the shield
  Serial.print it out at 115200 baud to serial monitor.
*/

#include <SparkFun_MCP9600.h>
MCP9600 tempSensor;
uint8_t coefficient = 3;

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

    //print the filter coefficient that's about to be set
    Serial.print("Setting Filter coefficient to ");
    Serial.print(coefficient);
    Serial.println("!");

    tempSensor.setFilterCoefficient(coefficient);

    //tell us if the coefficient was set sucessfully
    if(tempSensor.getFilterCoefficient() == coefficient){
        Serial.println("Filter Coefficient set sucessfully!");
    }

    else{
        Serial.println("Setting filter coefficient failed!");
        Serial.println("The value of the coefficient is: ");
        Serial.println(tempSensor.getFilterCoefficient(), BIN);
    }
}

void loop(){ //print the thermocouple, ambient and delta temperatures every 200ms
    if(tempSensor.available()){
        Serial.print("Thermocouple: ");
        Serial.print(tempSensor.getThermocoupleTemp());
        Serial.print(" °C   Ambient: ");
        Serial.print(tempSensor.getAmbientTemp());
        Serial.print(" °C   Temperature Delta: ");
        Serial.print(tempSensor.getTempDelta());
        Serial.print(" °C   Filter Coefficient: ");
        Serial.print(tempSensor.getFilterCoefficient(), DEC);
        Serial.println();
        delay(20); //don't pound on the I2C bus too hard     
    }
}