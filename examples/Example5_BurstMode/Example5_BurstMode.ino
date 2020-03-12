/*
  Configuring Burst Mode with the MCP9600 Thermocouple Amplifier
  By: Fischer Moseley
  SparkFun Electronics
  Date: July 10, 2019
  License: This code is public domain but you buy me a beer if you use this and we meet someday (Beerware License).

  This example configures the shutdown (or "operating") mode that the MCP9600 runs in. Shutdown mode disables all
  power consuming activities on the MCP9600, including measurements, but it will still respond to I2C commands sent
  over Qwiic. Burst mode is similar, where the MCP9600 is shutdown until the Arduino asks it to wake up and take a 
  number of samples, apply any filtering, update any outputs, and then enter shutdown mode. This example walks
  through that process!

  Hardware Connections:
  Attach the Qwiic Shield to your Arduino/Photon/ESP32 or other
  Plug the sensor onto the shield
  Serial.print it out at 115200 baud to serial monitor.
*/

#include <SparkFun_MCP9600.h>
MCP9600 tempSensor;
Shutdown_Mode mode = BURST;
Burst_Sample samples = SAMPLES_8;

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

  //put the MCP9600 into burst mode!
  tempSensor.setBurstSamples(samples); 
  tempSensor.setShutdownMode(BURST);
}

void loop(){
  if(tempSensor.burstAvailable()){ //if there's a new burst measurement, get the data and print it

    //print the thermocouple, ambient and delta temperatures
    Serial.print("Thermocouple: ");
    Serial.print(tempSensor.getThermocoupleTemp());
    Serial.print(" °C   Ambient: ");
    Serial.print(tempSensor.getAmbientTemp());
    Serial.print(" °C   Temperature Delta: ");
    Serial.print(tempSensor.getTempDelta());
    Serial.print(" °C");
    Serial.println();

    //clear the register and start a new burst cycle!
    tempSensor.startBurst();
  }
}
