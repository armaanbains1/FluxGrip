#include <Arduino.h>
#include <Wire.h>
#include <iostream>
#include <vector> // 

byte deviceFinder(){
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

byte readByte(int address){
    Wire.requestFrom(address, 1);
    return Wire.read();
}

std::vector<byte> readBytes(int address, int numBytes){
    Wire.requestFrom(address, numBytes);
    std::vectpr
}