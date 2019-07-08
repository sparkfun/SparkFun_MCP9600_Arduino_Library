#include <Wire.h>
// register pointers for various device functions
#define DEV_ADDR 0x66
#define RESOLUTION 0.0625

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
  Wire.beginTransmission(DEV_ADDR);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.requestFrom(DEV_ADDR, 1);
  
  if(Wire.available()){
    uint8_t data = Wire.read();
    Wire.endTransmission();
    return data;
  }

  else{
    return 0;
  }
}

uint16_t readDoubleRegister(MCP9600_Register reg){
  Wire.beginTransmission(DEV_ADDR);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.requestFrom(DEV_ADDR, 2);

  while(!(Wire.available() < 2)){
    uint16_t data = Wire.read() << 8;
    data |= Wire.read();
    Wire.endTransmission();
    return data;
  }
}

uint16_t getDeviceID(){
  return readDoubleRegister(DEVICE_ID);
}

int16_t getDelta(){
  Wire.beginTransmission(DEV_ADDR);
  Wire.write(DELTA_JUNCT_TEMP);
  Wire.endTransmission();
  Wire.requestFrom(DEV_ADDR, 2);

  while(!(Wire.available() < 2)){
    int16_t data = Wire.read() << 8;
    data |= Wire.read();
    Wire.endTransmission();
    return data;
  }
}

void setup() {
  Wire.begin();
  Serial.begin(115200);
  Wire.beginTransmission(DEV_ADDR);

  //make sure the device will ack
  if(Wire.endTransmission() != 0){
    Serial.println("Device Setup Failed");
    while(1);
  }

  else{
    Serial.print("Device setup okay");
  }
}

void loop() {
  uint16_t hot = readDoubleRegister(HOT_JUNC_TEMP);
  uint16_t cold = readDoubleRegister(COLD_JUNC_TEMP);
  uint16_t delta = getDelta();
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
  delay(10);
}
