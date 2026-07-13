#include <Arduino.h>
#include <Wire.h>
#include <iostream>
#include <vector> // 
#include "i2cSensor.h"
#include "kinematicsEngine.h"
#include "workoutEngine.h"
#include "engineClock.h"
#include <iostream>
#include <iomanip> // Required for hex, setw, and setfill
#include <cstdint>
std::vector<float> accelometerVals;
std::vector<float> accelometerValsN;
float accelometerAverageX = 0;
float accelometerAverageY = 0;
float accelometerAverageZ = 0;
float prevAccelometerAverageX = 0;
float prevAccelometerAverageY = 0;
float prevAccelometerAverageZ = 0;
float accelometerCount = 0;
std::vector<float> adjustedAccelometerVals;
std::vector<std::vector<float>> rawAccelometerStream;

Quaternion qAdjustedAccelometerVals;
std::vector<float> adjustedGravityVals;
std::vector<float> velocity = {0,0,0};

Quaternion qAdjustedGravityVals;
Quaternion qGravityVals;

std::vector<float> gyroVals {0, 0, 0};
std::vector<float> gyroValsI = {0,0,0};

std::vector<float> error;

std::vector<float> accolometerTracker {};
std::vector<float> movementTrackerForWorkEngine {0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1};
int movementCounter = 0;
int sampleCount = 0;
float sampleAveragePrev = 0;
float sampleAverageCurr = 0;
float Kp = 0.35;
float Ki = 0.05;

bool enableHighPass = false;

uint32_t lastClockTime = 0;
using namespace std;
i2cSensor MPU6050;
kinematicsEngine kinEngine;
workoutEngine workEngine;
engineClock eClock;

Quaternion qiNew; 
Quaternion qiPrev;
char mode = 'g';

float gyroValsXOffset = 0;
float gyroValsYOffset = 0;
float gyroValsZOffset = 0;

float accValsXOffset = 0;
float accValsYOffset = 0;
float accValsZOffset = 0;

bool sampleClock = false;

bool moving = false;
bool paused = true;
int halfRep;
int rep;

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

  float accelNorm = sqrt(accelometerVals[0] * accelometerVals[0] + 
                        accelometerVals[1] * accelometerVals[1] + 
                        accelometerVals[2] * accelometerVals[2]);

  if (accelNorm > 0.000001f) {
    accelometerVals[0] /= accelNorm;
    accelometerVals[1] /= accelNorm;
    accelometerVals[2] /= accelNorm;
  }
  MPU6050.configureGyroscope();
  std::tuple<float, float, float> intitialAngles = kinEngine.initialAngleCalculator(accelometerVals[0], accelometerVals[1], accelometerVals[2]);
  cout << "x " << std::get<0>(intitialAngles) << endl;
  cout << "y " << std::get<1>(intitialAngles) << endl;
  cout << "z" << std::get<2>(intitialAngles) << endl;
  qiPrev = kinEngine.quaternionCalculator(intitialAngles);

  const int calibrationSamples = 200;

  for (int i = 0; i < calibrationSamples; i++){ // Clean 0 to 199 loop
      gyroVals = MPU6050.galvoXYZ();
      gyroValsXOffset += gyroVals[0];
      gyroValsYOffset += gyroVals[1];
      gyroValsZOffset += gyroVals[2];
      delay(5); // Give the sensor a tiny breath between samples
  }

  gyroValsXOffset /= static_cast<float>(calibrationSamples);
  gyroValsYOffset /= static_cast<float>(calibrationSamples);
  gyroValsZOffset /= static_cast<float>(calibrationSamples);

  //cout << "gyro vals x offset " << gyroValsXOffset << endl;
  //cout << "gyro vals y offset " << gyroValsYOffset << endl;
  //cout << "gyro vals z offset " << gyroValsZOffset << endl;
  
  gyroVals = MPU6050.galvoXYZ();
  gyroVals[0] -= gyroValsXOffset;
  gyroVals[1] -= gyroValsYOffset;
  gyroVals[2] -= gyroValsZOffset;



  for (int i = 0; i < calibrationSamples; i++){ 
      accelometerVals = MPU6050.accelometerXYZ(); // Fixed: Use correct sensor method
      accValsXOffset += accelometerVals[0];
      accValsYOffset += accelometerVals[1];
      accValsZOffset += accelometerVals[2];
      delay(5); 
  }

  accValsXOffset /= static_cast<float>(calibrationSamples);
  accValsYOffset /= static_cast<float>(calibrationSamples);
  accValsZOffset /= static_cast<float>(calibrationSamples);

  // Remove gravity from the Z offset so we only capture the sensor bias error
  accValsZOffset -=   1.0f; 

  accelometerVals = MPU6050.accelometerXYZ();
  accelometerVals[0] -= accValsXOffset;
  accelometerVals[1] -= accValsYOffset;
  accelometerVals[2] -= accValsZOffset;


  Quaternion initialQw = {0, gyroVals[0], gyroVals[1], gyroVals[2]};
  qiNew = kinEngine.GyroQuaternionUpdater(qiPrev, initialQw, 0.00000001);
  //cout << "X: " << adjustedAccelometerVals[0] << "    Y:  " << adjustedAccelometerVals[1] << "    Z:" << adjustedAccelometerVals[2];
  qGravityVals = {0, 0, 0, 1};

  lastClockTime = micros();

for (int i = 0; i < 10; i++){
    uint32_t currentClockTime = micros();
    uint32_t clockTicksDT = currentClockTime - lastClockTime;
    lastClockTime = currentClockTime;
    float dt = static_cast<float>(clockTicksDT) / 1000000.0f;

    gyroVals = MPU6050.galvoXYZ();
    gyroVals[0] = (gyroVals[0] - gyroValsXOffset) * DEG_TO_RAD;
    gyroVals[1] = (gyroVals[1] - gyroValsYOffset) * DEG_TO_RAD;
    gyroVals[2] = (gyroVals[2] - gyroValsZOffset) * DEG_TO_RAD;

    accelometerVals = MPU6050.accelometerXYZ();
    accelometerVals[0] = (accelometerVals[0] - accValsXOffset);
    accelometerVals[1] = (accelometerVals[1] - accValsYOffset);
    accelometerVals[2] = (accelometerVals[2] - accValsZOffset);
    std::vector<float> accelometerValsN = accelometerVals;
    float accelNorm = sqrt(accelometerVals[0] * accelometerVals[0] + 
                          accelometerVals[1] * accelometerVals[1] + 
                          accelometerVals[2] * accelometerVals[2]);

    if (accelNorm > 0.000001f) {
      accelometerValsN[0] /= accelNorm;
      accelometerValsN[1] /= accelNorm;
      accelometerValsN[2] /= accelNorm;
    }

    qGravityVals = kinEngine.quaternionGlobalToLocal(qiNew, {0, 0, 0, 1}); // Global gravity is [0, 0, 1]
    adjustedGravityVals = {qGravityVals.qx, qGravityVals.qy, qGravityVals.qz};
    
    error = kinEngine.vectorCrossProduct(accelometerValsN, adjustedGravityVals);

    gyroValsI[0] += error[0] * Ki * dt;
    gyroValsI[1] += error[1] * Ki * dt;
    gyroValsI[2] += error[2] * Ki * dt;

    float correctedGyroX = gyroVals[0] + (Kp * error[0]) + gyroValsI[0];
    float correctedGyroY = gyroVals[1] + (Kp * error[1]) + gyroValsI[1];
    float correctedGyroZ = gyroVals[2] + (Kp * error[2]) + gyroValsI[2];

    //cout << "X: " << correctedGyroX << "    Y:  " << correctedGyroY << "    Z:" << correctedGyroZ << endl;

    Quaternion newQw = {0, correctedGyroX, correctedGyroY, correctedGyroZ};
    qiPrev = qiNew;
    qiNew = kinEngine.GyroQuaternionUpdater(qiPrev, newQw, dt);

    Quaternion qAccelometerVals = {0, accelometerVals[0], accelometerVals[1], accelometerVals[2]};
    qAdjustedAccelometerVals = kinEngine.quaternionLocalToGlobal(qiNew, qAccelometerVals);
    adjustedAccelometerVals = {qAdjustedAccelometerVals.qx, qAdjustedAccelometerVals.qy, qAdjustedAccelometerVals.qz - 1};
    
    //cout << "X: " << adjustedAccelometerVals[0] << "    Y:  " << adjustedAccelometerVals[1] << "    Z:" << adjustedAccelometerVals[2] << endl;
    //cout << "    Z:" << adjustedAccelometerVals[2];
    accolometerTracker.push_back(adjustedAccelometerVals[2] * 100);

    delay(5);
  }

  for (int i = 0; i<= 99; i++){
    std::vector<float> accelometerValsN = accelometerVals;
    float accelNorm = sqrt(accelometerVals[0] * accelometerVals[0] + 
                          accelometerVals[1] * accelometerVals[1] + 
                          accelometerVals[2] * accelometerVals[2]);

    if (accelNorm > 0.000001f) {
      accelometerValsN[0] /= accelNorm;
      accelometerValsN[1] /= accelNorm;
      accelometerValsN[2] /= accelNorm;
    }
    rawAccelometerStream.push_back(accelometerValsN);

  }

  accelometerCount = 0;
  sampleAveragePrev = workEngine.calculateSampleAverage(accolometerTracker);
  cout << "the size is " << rawAccelometerStream.size() << endl;

}



void loop() {
  // 1. Keep track of timing
  // 1. Enforce the 5000 microsecond (5ms) rate limit
  uint32_t currentClockTime = micros();
  uint32_t clockTicksDT = currentClockTime - lastClockTime;

  // If 5ms have NOT passed yet, stop right here and exit the loop immediately
  if (clockTicksDT < 5000) {
    return; 
  }
  
  // If we get past the 'return', it means exactly 5ms (or slightly more) have passed!
  lastClockTime = currentClockTime; // Save the time for the next check

  float dt = static_cast<float>(clockTicksDT) / 1000000.0f;
  accolometerTracker.push_back(adjustedAccelometerVals[2]-1);
  sampleCount+= 1;
  if (sampleCount == 10){
    sampleAverageCurr = workEngine.calculateSampleAverage(accolometerTracker);
    
    if (sampleAverageCurr > -0.1 && sampleAverageCurr < 0.1){
      sampleAverageCurr = 0;
    }
    accolometerTracker.clear();
    
    cout << sampleAverageCurr << endl;


    movementTrackerForWorkEngine.erase(movementTrackerForWorkEngine.begin());
    movementTrackerForWorkEngine.push_back(sampleAverageCurr);
    
    //cout << movementCounter << endl;
    //cout << movementTrackerForWorkEngine.size()<< endl;
    for (auto &i: movementTrackerForWorkEngine){
      //cout << " " << i << " ";
    }
    //cout << "new" << endl;
    movementCounter += 1;
    if (workEngine.sampleSignChange(sampleAveragePrev, sampleAverageCurr)){
      //cout << "changed" << endl;
    }
    sampleAveragePrev = sampleAverageCurr;
    sampleCount = 0;
  }


  // 2. Read latest sensor data
  gyroVals = MPU6050.galvoXYZ();
  gyroVals[0] = (gyroVals[0] - gyroValsXOffset) * DEG_TO_RAD;
  gyroVals[1] = (gyroVals[1] - gyroValsYOffset) * DEG_TO_RAD;
  gyroVals[2] = (gyroVals[2] - gyroValsZOffset) * DEG_TO_RAD;

  if (enableHighPass){
    gyroVals = {0,0,0};
  }

  accelometerVals = MPU6050.accelometerXYZ();
  accelometerVals[0] = (accelometerVals[0] - accValsXOffset);
  accelometerVals[1] = (accelometerVals[1] - accValsYOffset);
  accelometerVals[2] = (accelometerVals[2] - accValsZOffset);
  std::vector<float> accelometerValsN = accelometerVals;

  float accelNorm = sqrt(accelometerVals[0] * accelometerVals[0] + 
                        accelometerVals[1] * accelometerVals[1] + 
                        accelometerVals[2] * accelometerVals[2]);

  if (accelNorm > 0.000001f) {
    accelometerValsN[0] /= accelNorm;
    accelometerValsN[1] /= accelNorm;
    accelometerValsN[2] /= accelNorm;
  }

  //implementing the zero-velocty update high pass gyro filter
  //goal here is to essentially check the accelometer for a stable point
  //once we get to a stable acceleration, we then go directly to the gyro values, and then, calibrate them, creating a continuous calibration which goes on indefintely

  rawAccelometerStream.push_back(accelometerValsN);
  accelometerCount ++;
  if (accelometerCount == 15){
    float sumX = 0;
    float sumY = 0;
    float sumZ = 0;

    for (auto & i: rawAccelometerStream){
//     cout << sumX << endl;;
//     cout << sumY << endl;;
//      cout << sumZ << endl;;

      sumX += i[0];
      sumY += i[1];
      sumZ += i[2];
    }
    prevAccelometerAverageX = accelometerAverageX;
    prevAccelometerAverageY = accelometerAverageY;
    prevAccelometerAverageZ = accelometerAverageZ;

    accelometerAverageX = sumX / 15.0;
    accelometerAverageY = sumY / 15.0;
    accelometerAverageZ = sumZ / 15.0;
   // cout << "x: " << accelometerAverageX  << "y " << accelometerAverageY << "z " << accelometerAverageZ << endl;
    accelometerCount = 0;
    
    rawAccelometerStream.clear();
    if (kinEngine.getSensorPercentDifference(accelometerAverageX, prevAccelometerAverageX) < 5.30 && kinEngine.getSensorPercentDifference(accelometerAverageY, prevAccelometerAverageY) < 4.10 && kinEngine.getSensorPercentDifference(accelometerAverageZ, prevAccelometerAverageZ) < 0.30){
        //cout << "not moving" << endl;
        enableHighPass = true;

    }
    else{
      //cout << "moving" << endl;
      enableHighPass = false;
    }
    
  }


  // 3. Compute error vector using CURRENT attitude state (qiNew)
  qGravityVals = kinEngine.quaternionGlobalToLocal(qiNew, {0, 0, 0, 1}); // Global gravity is [0, 0, 1]
  adjustedGravityVals = {qGravityVals.qx, qGravityVals.qy, qGravityVals.qz};
  
  // Make sure both vectors are normalized before cross product!
  error = kinEngine.vectorCrossProduct(accelometerValsN, adjustedGravityVals);

  // 4. NOW it is safe to compute the Integral and Proportional corrections
  gyroValsI[0] += error[0] * Ki * dt;
  gyroValsI[1] += error[1] * Ki * dt;
  gyroValsI[2] += error[2] * Ki * dt;

  float correctedGyroX = gyroVals[0] + (Kp * error[0]) + gyroValsI[0];
  float correctedGyroY = gyroVals[1] + (Kp * error[1]) + gyroValsI[1];
  float correctedGyroZ = gyroVals[2] + (Kp * error[2]) + gyroValsI[2];

  //cout << "X: " << correctedGyroX << "    Y:  " << correctedGyroY << "    Z:" << correctedGyroZ << endl;

  // 5. Update the orientation quaternion using the freshly corrected values
  Quaternion newQw = {0, correctedGyroX, correctedGyroY, correctedGyroZ};
  qiPrev = qiNew;
  qiNew = kinEngine.GyroQuaternionUpdater(qiPrev, newQw, dt);

  // 6. Optional: Project raw acceleration into the global frame if needed
  Quaternion qAccelometerVals = {0, accelometerVals[0], accelometerVals[1], accelometerVals[2]};
  qAdjustedAccelometerVals = kinEngine.quaternionLocalToGlobal(qiNew, qAccelometerVals);
  adjustedAccelometerVals = {qAdjustedAccelometerVals.qx, qAdjustedAccelometerVals.qy, qAdjustedAccelometerVals.qz};

  //cout << "X: " << adjustedAccelometerVals[0] << "    Y:  " << adjustedAccelometerVals[1] << "    Z:" << adjustedAccelometerVals[2]+1 << endl;
  ////cout << "    Z:" << adjustedAccelometerVals[2] * 100;
  //cout << "X: " << correctedGyroX << "    Y:  " << correctedGyroY << "    Z:" << correctedGyroZ << endl;


  if (workEngine.checkForPaused(movementTrackerForWorkEngine)){
    if (moving == true){
      halfRep++;
      rep = halfRep / 2;
      cout << "rep count = " << rep << endl; 
    }
    moving = false;
    paused = true;
  }
  else{
    moving = true;
    paused = false;
  }





}

