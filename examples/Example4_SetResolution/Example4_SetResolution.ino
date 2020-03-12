/*
  Setting ADC resolution with the MCP9600 Thermocouple Amplifier
  By: Fischer Moseley
  SparkFun Electronics
  Date: July 10, 2019
  License: This code is public domain but you buy me a beer if you use this and we meet someday (Beerware License).

  This example allows you to change the ADC resolution on the thermocouple (hot) and ambient (cold) junctions. Why
  do this, you ask? Well, setting the resolution lower decreases the sampling time, meaning that you can measure
  high-speed thermal transients! (at the expense of lower resolution, of course).

  Before we start adjusting ADC resolution, a quick word on how this all works. All thermocouple systems have a hot
  and cold junction, and the MCP9600 is no different. Thanks to physics, thermocouples only measure the difference
  in hot and cold junction temperatures, meaning that in order to know the temperature of the hot junction, the
  amplifier needs to know the temperature of cold junction. From there, it will add the cold junction temperature
  to the temperature difference measured by the thermocouple, giving the absolute temperature rather than just the
  relative one.

  This means that the MCP9600 has to take two temperature measurements! Thankfully, the MCP9600 will let us set the 
  resolution on each one independently. SetAmbientResolution and SetThermocoupleResolution configure these measurements
  with a desired resolution for the cold and hot junctions respectively. 

  Cold Junction Possible Resolutions:
  RES_ZERO_POINT_0625   -> Configures the ambient (cold) junction to measure in increments of 0.0625ºC
  RES_ZERO_POINT_25     -> Configures the ambient (cold) junction to measure in increments of 0.25ºC


  Hot Junction Possible Resolutions: 
  RES_18_BIT  -> Reads the hot junction ADC to 18 bits, or 2µV
  RES_16_BIT  -> Reads the hot junction ADC to 16 bits, or 8µV
  RES_14_BIT  -> Reads the hot junction ADC to 14 bits, or 32µV
  RES_12_BIT  -> Reads the hot junction ADC to 12 bits, or 128µV

  It's worth noting that since the thermocouple that serves as the hot junction is arbitrary, we can't provide a 
  specification on how many degrees Celcius precision you will get for a given ADC resolution. 

  Hardware Connections:
  Attach the Qwiic Shield to your Arduino/Photon/ESP32 or other
  Plug the sensor onto the shield
  Serial.print it out at 115200 baud to serial monitor.
*/

#include <SparkFun_MCP9600.h>
MCP9600 tempSensor;

Ambient_Resolution ambientRes = RES_ZERO_POINT_0625;
Thermocouple_Resolution thermocoupleRes = RES_14_BIT;

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

  //set the resolution on the ambient (cold) junction
  tempSensor.setAmbientResolution(ambientRes);
  tempSensor.setThermocoupleResolution(thermocoupleRes);
}

void loop(){ //print the thermocouple, ambient and delta temperatures every 200ms
    if(tempSensor.available()){
        Serial.print("Thermocouple: ");
        Serial.print(tempSensor.getThermocoupleTemp());
        Serial.print(" °C   Ambient: ");
        Serial.print(tempSensor.getAmbientTemp());
        Serial.print(" °C   Temperature Delta: ");
        Serial.print(tempSensor.getTempDelta());
        Serial.print(" °C");
        Serial.println();
        delay(20); //don't pound on the I2C bus too hard
    }
}