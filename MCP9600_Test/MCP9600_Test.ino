#include <Wire.h>

#define DEV_ADDR 0x66
#define RESOLUTION 0.0625

#define retryAttempts 3

// register pointers for various device functions
enum MCP9600_Register {
  HOT_JUNC_TEMP = 0x00,
  DELTA_JUNCT_TEMP = 0x01,
  COLD_JUNC_TEMP = 0x02,
  STATUS = 0x04,
  THERMO_SENSOR_CONFIG = 0x05,
  DEVICE_CONFIG = 0x06,
  DEVICE_ID = 0x20,
};

uint8_t readSingleRegister(MCP9600_Register reg){
  for(byte attempts; attempts <= retryAttempts; attempts++){
    Wire.beginTransmission(DEV_ADDR);
    Wire.write(reg);
    Wire.endTransmission();
    if(Wire.requestFrom(DEV_ADDR, 1) != 0){
      return Wire.read();     
    }
  }
  return;
}

uint16_t readDoubleRegister(MCP9600_Register reg){
  for(byte attempts; attempts <= retryAttempts; attempts++){
    Wire.beginTransmission(DEV_ADDR);
    Wire.write(reg);
    Wire.endTransmission();

    if(Wire.requestFrom(DEV_ADDR, 2) != 0){
      uint16_t data = Wire.read() << 8;
      data |= Wire.read();
      return data;
    }
  }
  return;
}

uint16_t getDeviceID(){
  return readDoubleRegister(DEVICE_ID);
}

int16_t getDelta(){
  int16_t little = readDoubleRegister(DELTA_JUNCT_TEMP);
  int16_t big = lowByte(little);
  big |= highByte(little) << 8;
  return big;
}

bool isConnected(){
  Wire.beginTransmission(DEV_ADDR);
  if(Wire.endTransmission() != 0){
    return false;
  }
  else{
    return true;
  }
}

void setup() {
  Wire.begin();
  Wire.setClock(10000);
  Serial.begin(115200);

  //make sure the device will ack
  if(isConnected()){
    Serial.println("Device Setup Okay");
  }

  else{
    Serial.println("Device Setup Failed");
    while(1); //hang forever
  }
}

void loop() {
  int16_t hot = readDoubleRegister(HOT_JUNC_TEMP);
  int16_t cold = readDoubleRegister(COLD_JUNC_TEMP);
  int16_t delta = getDelta();
  Serial.print("Device ID: ");
  Serial.print(getDeviceID(), BIN);
  Serial.print("  Hot Junction: ");
  Serial.print(hot);
  Serial.print(" (");
  Serial.print((float)hot*RESOLUTION);
  Serial.print(" °C)  Cold Junction: ");
  Serial.print(cold);
  Serial.print(" (");
  Serial.print(cold*RESOLUTION);
  Serial.print(" °C)  Delta Junction: ");
  Serial.print(delta);
  Serial.print(" (");
  Serial.print((float)delta*RESOLUTION);
  Serial.print(" °C)");
  Serial.println();
}
