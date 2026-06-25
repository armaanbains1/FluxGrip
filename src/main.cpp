#include <Arduino.h>
#include <Wire.h>
#include <iostream>
#include <vector> // 
#include "i2cSensor.h"
#include <iostream>
#include <iomanip> // Required for hex, setw, and setfill
#include <cstdint>

using namespace std;
i2cSensor MPU6050;

void printWord(uint32_t val){
  
  std::cout << "0x" 
              << std::hex          // Switch output to hexadecimal
              << std::uppercase    // Use A-F instead of a-f
              << std::setw(8)      // Set the width of the next output to 8 characters
              << std::setfill('0') // Fill empty space with zeros
              << val               // Finally, print the value
              << std::endl;        // Output: 0x12345678
}

void setup() {
  Wire.begin();
  Serial.begin(115200);
}

void loop() {
  MPU6050.powerUp();
  u_int32_t accelometerTest = MPU6050.readHalfWord(0x3B);
  cout << accelometerTest << endl;
  printWord(accelometerTest);
}

