#include <Arduino.h>
#include <Wire.h>
#include <iostream>
#include <vector> // 
#include "i2cSensor.h"
#include "kinematicsEngine.h"

#include <iostream>
#include <iomanip> // Required for hex, setw, and setfill
#include <cstdint>
std::vector<float> accelometerVals;
std::vector<float> gyroVals;
uint32_t lastClockTime = 0;
using namespace std;
i2cSensor MPU6050;
kinematicsEngine kinEngine;
Quaternion qiNew; 
Quaternion qiPrev;
char mode = 'g';
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
  MPU6050.configureAccelerator();
  accelometerVals = MPU6050.accelometerXYZ();
  MPU6050.configureGyroscope();
  std::tuple<float, float, float> intitialAngles = kinEngine.initialAngleCalculator(accelometerVals[0], accelometerVals[1], accelometerVals[2]);
  cout << "x " << std::get<0>(intitialAngles) << endl;
  cout << "y " << std::get<1>(intitialAngles) << endl;
  cout << "z" << std::get<2>(intitialAngles) << endl;
  qiPrev = kinEngine.quaternionCalculator(intitialAngles);
  gyroVals = MPU6050.galvoXYZ();
  Quaternion initialQw = {0, gyroVals[0], gyroVals[1], gyroVals[2]};
  qiNew = kinEngine.GyroQuaternionUpdater(qiPrev, initialQw, 0.00000001);
  lastClockTime = micros();
  
}



void loop() {
  uint32_t currentClockTime = micros();
  uint32_t clockTicksDT = currentClockTime - lastClockTime;
  lastClockTime = currentClockTime;
  float dt = static_cast<float>(clockTicksDT) / 1000000.0f;
  gyroVals = MPU6050.galvoXYZ();
  Quaternion newQw = {0, gyroVals[0], gyroVals[1], gyroVals[2]};
  qiPrev = qiNew;
  qiNew = kinEngine.GyroQuaternionUpdater(qiPrev, newQw, dt);
  delay(1000);
}

