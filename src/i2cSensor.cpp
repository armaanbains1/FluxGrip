#include <Arduino.h>
#include <Wire.h>
#include <iostream>
#include <vector> 
#include "i2cSensor.h"
#define MPU_ADDR 0x68

byte i2cSensor::deviceFinder(){
  byte error, address;
  int nDevices;
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

void i2cSensor::configureAccelerator(){
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);           
  Wire.write(0x00);           // Wake up MPU6050 (set sleep = 0)
  Wire.endTransmission(true);

  // Set accelerometer range to ±2g (most stable)
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1C);           // ACCEL_CONFIG register
  Wire.write(0x00);           // 2g range
  Wire.endTransmission(true);

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x19);           
  Wire.write(0x07);           // Sample rate = 1kHz / (7+1) = 125Hz
  Wire.endTransmission(true);

}

void i2cSensor::configureGyroscope(){
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);     // Wake up sensor
  Wire.write(0x00);
  Wire.endTransmission(true);

  // Configure gyroscope range to ±250 °/s
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1B);     // GYRO_CONFIG register
  Wire.write(0x00);     // ±250dps
  Wire.endTransmission(true);

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

std::vector<float> i2cSensor::accelometerXYZ(){
  int16_t rawAccX, rawAccY, rawAccZ;
  float AccX, AccY, AccZ;

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 6);

  uint8_t hx = Wire.read(); uint8_t lx = Wire.read();
  rawAccX = (hx << 8) | lx;

  uint8_t hy = Wire.read(); uint8_t ly = Wire.read();
  rawAccY = (hy << 8) | ly;

  uint8_t hz = Wire.read(); uint8_t lz = Wire.read();
  rawAccZ = (hz << 8) | lz;

  AccX = rawAccX / 16384.0;
  AccY = rawAccY / 16384.0;
  AccZ = rawAccZ / 16384.0;

  //Serial.print("AccX: "); Serial.print(AccX);
  //Serial.print(" | AccY: "); Serial.print(AccY);
  //Serial.print(" | AccZ: "); Serial.println(AccZ);

  std::vector<float> accelometerValues = {AccX, AccY, AccZ};

  return accelometerValues;
}

std::vector<float> i2cSensor::galvoXYZ(){
  int16_t rawGyroX, rawGyroY, rawGyroZ;
  float GyroX, GyroY, GyroZ;

  // Request gyroscope registers starting from 0x43
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x43);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 6, true);

  uint8_t hx = Wire.read();
  uint8_t lx = Wire.read();
  rawGyroX = (hx << 8) | lx;

  uint8_t hy = Wire.read();
  uint8_t ly = Wire.read();
  rawGyroY = (hy << 8) | ly;

  uint8_t hz = Wire.read();
  uint8_t lz = Wire.read();
  rawGyroZ = (hz << 8) | lz;

  GyroX = rawGyroX / 131.0;
  GyroY = rawGyroY / 131.0;
  GyroZ = rawGyroZ / 131.0;
  
  //Serial.print("GyroX: "); Serial.print(GyroX);
  //Serial.print(" | GyroY: "); Serial.print(GyroY);
  //Serial.print(" | GyroZ: "); Serial.println(GyroZ);

  std::vector<float> gyroValues = {GyroX, GyroY, GyroZ};

  return gyroValues;
}
