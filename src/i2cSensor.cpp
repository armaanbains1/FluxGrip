#include <Arduino.h>
#include <Wire.h>
#include <iostream>
#include <vector> 
#include "i2cSensor.h"
#define MPU_ADDR 0x68

byte i2cSensor::deviceFinder(){
  byte error, address;
  int nDevices;
  Serial.println("Scanning...");
  nDevices = 0;
  if (nDevices == 0) {
    Serial.println("No I2C devices found\n");
    return 0xFFFF;
  }
  for(address = 1; address < 127; address++ ) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address<16) {
        Serial.print("0");
      }
      Serial.println(address,HEX);
      nDevices++;
      return address;
    }
    else if (error==4) {
      Serial.print("Unknown error at address 0x");
      if (address<16) {
        Serial.print("0");
      }
      Serial.println(address,HEX);
      return 0xFFFF;

    }    
  }
}

void i2cSensor::powerUp(){
    // ----- MPU6050 Initialization -----
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);           // PWR_MGMT_1 register
  Wire.write(0x00);           // Wake up MPU6050 (set sleep = 0)
  Wire.endTransmission(true);

  // Set accelerometer range to ±2g (most stable)
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1C);           // ACCEL_CONFIG register
  Wire.write(0x00);           // ±2g range
  Wire.endTransmission(true);

  // Set sample rate (optional)
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x19);           // SMPLRT_DIV register
  Wire.write(0x07);           // Sample rate = 1kHz / (7+1) = 125Hz
  Wire.endTransmission(true);

  Serial.println("MPU6050 Initialized for Raw Accelerometer Readings");
}

byte i2cSensor::readByte(byte address){
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(address);
    Wire.endTransmission(false); //By putting false here, we essentially tell the wire that were not yet done using the I2C connection to the device
    Wire.requestFrom(MPU_ADDR, 1);
    byte returnByte = Wire.read();
    return returnByte;
}

uint16_t i2cSensor::readHalfWord(byte address){
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(address);
  Wire.endTransmission(false); //By putting false here, we essentially tell the wire that were not yet done using the I2C connection to the device
  Wire.requestFrom(MPU_ADDR, 2);
  uint16_t returnHalfWord = 0;

  for (int i = 0; i <= 3; i+=1){
    uint8_t read = Wire.read();
    returnHalfWord <<= 8;
    returnHalfWord |= read;
  }
  
  return returnHalfWord;
}

uint32_t i2cSensor::readWord(byte address){
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(address);
  Wire.endTransmission(false); //By putting false here, we essentially tell the wire that were not yet done using the I2C connection to the device
  Wire.requestFrom(MPU_ADDR, 4);
  uint32_t returnWord = 0;
  for (int i = 0; i <= 3; i+=1){
    uint8_t read = Wire.read();
    returnWord <<= 8;
    returnWord |= read;
  }
  
  return returnWord;
}
