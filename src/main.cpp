#include <Arduino.h>
#include <Wire.h>
#include <iostream>
#include <vector> // 
#include <unordered_map>
#include "i2cSensor.h"
#include "kinematicsEngine.h"
#include "workoutEngine.h"
#include "engineClock.h"
#include <iostream>
#include <iomanip> // Required for hex, setw, and setfill
#include <cstdint>
#include <string>
#include "edge-impulse-sdk/classifier/ei_run_classifier.h"
#define BUFFER_SIZE (EI_CLASSIFIER_RAW_SAMPLE_COUNT * EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME)


//AI - Model variables
std::unordered_map<std::string, int> aiModel;
std::string maxProbabilityExcercise;
std::string prevMaxProbabilityExcercise;
int timerCount = 0; //counts the number of times we have run the 5ms counter

float input_buf[BUFFER_SIZE] = {};
int head = 0;

static int get_signal_data(size_t offset, size_t length, float *out_ptr) {
    if (out_ptr == NULL) return EIDSP_OUT_OF_MEM;

    size_t start_index = offset * 6;
    size_t count = length * 6;

    for (size_t i = 0; i < count; i++) {
        out_ptr[i] = input_buf[(head + start_index + i) % BUFFER_SIZE];
    }
    return EIDSP_OK;
}

void add_sensor_readings(float readings[8]) {
    for (int i = 0; i < 8; i++) {
        input_buf[head] = readings[i];
        head = (head + 1) % BUFFER_SIZE; // BUFFER_SIZE must be 800!
    }
}

std::vector<float> accelometerVals;
std::vector<float> accelometerValsN;
float accelometerAverageX = 0;
float accelometerAverageY = 0;
float accelometerAverageZ = 0;
float prevAccelometerAverageX = 0;
float prevAccelometerAverageY = 0;
float prevAccelometerAverageZ = 0;
float accelometerCount = 0;

float prevRawX = 0;
float prevRawY = 0;
float prevRawZ = 0;

std::vector<float> adjustedAccelometerVals;
std::vector<std::vector<float>> rawAccelometerStream;
std::vector<int> sets;
std::vector<int> currentSet;

uint32_t halfrepTimerStart;
uint32_t halfrepTime;

Quaternion qAdjustedAccelometerVals;
std::vector<float> adjustedGravityVals;
std::vector<float> velocity = {0,0,0};

Quaternion qAdjustedGravityVals;
Quaternion qGravityVals;

std::vector<float> gyroVals {0, 0, 0};
std::vector<float> gyroValsI = {0,0,0};

std::vector<float> error;

std::vector<float> accolometerTracker {};
std::vector<float> movementTrackerForWorkEngine {0,0,0,0,0,0,0,0,0,0};
int movementCounter = 0;
int sampleCount = 0;
float sampleAveragePrev = 0;
float sampleAverageCurr = 0;
float Kp = 0.35;
float Ki = 0.0;

bool enableHighPass = false;
bool enableLowPass = false;

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
int repCount;
int setNum;
int pauseCount = 0;
bool inSet = false;
bool inRep = false;

void printWord(uint32_t val){
  std::cout << "0x" 
              << std::hex          // Switch output to hexadecimal
              << std::uppercase    // Use A-F instead of a-f
              << std::setw(8)      // Set the width of the next output to 8 characters
              << std::setfill('0') // Fill empty space with zeros
              << val               // Finally, print the value
              << std::endl;        // Output: 0x12345678
}



unsigned long last_inference_time = 0;
const unsigned long INFERENCE_INTERVAL_MS = 100; 

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

  // Figure out what gravity should read at this starting orientation
  // (qiPrev was already computed above from initialAngleCalculator)
  Quaternion expectedGravityLocal = kinEngine.quaternionGlobalToLocal(qiPrev, {0, 0, 0, 1});

  // Subtract expected gravity from each axis, not just a hardcoded 1.0 on Z
  accValsXOffset -= expectedGravityLocal.qx;
  accValsYOffset -= expectedGravityLocal.qy;
  accValsZOffset -= expectedGravityLocal.qz;

  // Remove gravity from the Z offset so we only capture the sensor bias error

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

  for (int i = 0; i<400; i++){
    float readings[6] = {
        adjustedAccelometerVals[0], adjustedAccelometerVals[1], adjustedAccelometerVals[2],
        gyroVals[0], gyroVals[1], gyroVals[2]
    };
    add_sensor_readings(readings);  
  }

   
}



void loop() {

  uint32_t currentClockTime = micros();
  uint32_t clockTicksDT = currentClockTime - lastClockTime;

  // If 5ms have NOT passed yet, stop right here and exit the loop immediately
  if (clockTicksDT < 5000) {
    return; 
  }
  timerCount++;
  lastClockTime = currentClockTime; 

  float dt = static_cast<float>(clockTicksDT) / 1000000.0f;
  accolometerTracker.push_back(adjustedAccelometerVals[2]-1);
  sampleCount+= 1;
  if (sampleCount == 10){
    sampleAverageCurr = workEngine.calculateSampleAverage(accolometerTracker);
    

    if (sampleAverageCurr > -0.06 && sampleAverageCurr < 0.06 ){
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
    if (kinEngine.getSensorPercentDifference(accelometerAverageX, prevAccelometerAverageX) < 5.30 && kinEngine.getSensorPercentDifference(accelometerAverageY, prevAccelometerAverageY) < 4.10 && kinEngine.getSensorPercentDifference(accelometerAverageZ, prevAccelometerAverageZ) < 3.0){
      // this was for the issue with holding the dumbell still, yet still technically "shaking it a bit"
      // it essentially filters out that "shaking" 
      //cout << "not moving" << endl;
        enableHighPass = true;
}
    else{
      //cout << "moving" << endl;
      enableHighPass = false;
    }

    if (kinEngine.getSensorPercentDifference(accelometerVals[0], prevRawX) > 500.0 || 
        kinEngine.getSensorPercentDifference(accelometerVals[1], prevRawY) > 500.0 || 
        kinEngine.getSensorPercentDifference(accelometerVals[2], prevRawZ) > 500.0) {
        
       // enableLowPass = true;  // Impact/Shock detected
    } 
    else {
      //  enableLowPass = false;
    }

    // Save current raw values as the previous raw baseline for the next frame
    prevRawX = accelometerVals[0];
    prevRawY = accelometerVals[1];
    prevRawZ = accelometerVals[2];

  }


  // Compute error vector using curr attitude state (qiNew)
  qGravityVals = kinEngine.quaternionGlobalToLocal(qiNew, {0, 0, 0, 1}); // Global gravity is [0, 0, 1]
  adjustedGravityVals = {qGravityVals.qx, qGravityVals.qy, qGravityVals.qz};
  
  // Make sure both vectors are normalized before cross product
  error = kinEngine.vectorCrossProduct(accelometerValsN, adjustedGravityVals);
  //cout << "error: " << error[0] << " " << error[1] << " " << error[2] << endl;


  // Handle corrections based on high-pass state
  if (enableHighPass || enableLowPass) {
    // Clear and freeze the integral vector so noise can't compound
    //gyroVals = {0.0f, 0.0f, 0.0f}; 
    gyroValsI = {0.0f, 0.0f, 0.0f}; 
  } else {
    // Only integrate gravity/motion errors when actively moving
    gyroValsI[0] += error[0] * Ki * dt;
    gyroValsI[1] += error[1] * Ki * dt;
    gyroValsI[2] += error[2] * Ki * dt;
  }

  if (enableLowPass){
    cout << "TOOO FAST" << endl;

    accelometerValsN = {0,0,1};
  }

  // keep Kp alive to keep orientation quaternion anchored to gravity
  Kp = 0.35f; 

  float correctedGyroX = gyroVals[0] + (Kp * error[0]) + gyroValsI[0];
  float correctedGyroY = gyroVals[1] + (Kp * error[1]) + gyroValsI[1];
  float correctedGyroZ = gyroVals[2] + (Kp * error[2]) + gyroValsI[2];
  //cout << "X: " << correctedGyroX << "    Y:  " << correctedGyroY << "    Z:" << correctedGyroZ << endl;

  //Update the orientation quaternion using the freshly corrected values
  Quaternion newQw = {0, correctedGyroX, correctedGyroY, correctedGyroZ};
  qiPrev = qiNew;
  qiNew = kinEngine.GyroQuaternionUpdater(qiPrev, newQw, dt);

  //Project raw acceleration into the global frame
  Quaternion qAccelometerVals = {0, accelometerVals[0], accelometerVals[1], accelometerVals[2]};
  qAdjustedAccelometerVals = kinEngine.quaternionLocalToGlobal(qiNew, qAccelometerVals);
  adjustedAccelometerVals = {qAdjustedAccelometerVals.qx, qAdjustedAccelometerVals.qy, qAdjustedAccelometerVals.qz};



  //cout << "X: " << adjustedAccelometerVals[0] << "    Y:  " << adjustedAccelometerVals[1] << "    Z:" << adjustedAccelometerVals[2] -1<< endl;
  ////cout << "    Z:" << adjustedAccelometerVals[2] * 100;
  //cout << "X: " << correctedGyroX << "    Y:  " << correctedGyroY << "    Z:" << correctedGyroZ << endl;

  //cout << pauseCount << endl;


  if (workEngine.checkForPaused(movementTrackerForWorkEngine)){
    if (pauseCount >= 2000 && inSet){
      repCount = rep;
      inSet = false;
      setNum++;
      Serial.print("Set #: ");
      Serial.print(setNum);
      Serial.println(" done");
      int currentMax = 0;
      maxProbabilityExcercise = "Idle"; // Default reset
      for (auto&excercise : aiModel){
        if (excercise.second > currentMax){
          currentMax = excercise.second;
          maxProbabilityExcercise = excercise.first;
        }
      }
      if (prevMaxProbabilityExcercise!=maxProbabilityExcercise){
        setNum = 1;
      }
      prevMaxProbabilityExcercise = maxProbabilityExcercise;

      aiModel.clear();
      std::tuple<float, float, float> freshAngles = kinEngine.initialAngleCalculator(accelometerVals[0], accelometerVals[1], accelometerVals[2]);

      qiPrev = kinEngine.quaternionCalculator(freshAngles);

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

      // Figure out what gravity SHOULD read at this starting orientation
      // (qiPrev was already computed above from initialAngleCalculator)
      Quaternion expectedGravityLocal = kinEngine.quaternionGlobalToLocal(qiPrev, {0, 0, 0, 1});

      accValsXOffset -= expectedGravityLocal.qx;
      accValsYOffset -= expectedGravityLocal.qy;
      accValsZOffset -= expectedGravityLocal.qz;

      // Remove gravity from the Z offset so we only capture the sensor bias error

      accelometerVals = MPU6050.accelometerXYZ();
      accelometerVals[0] -= accValsXOffset;
      accelometerVals[1] -= accValsYOffset;
      accelometerVals[2] -= accValsZOffset;


      Quaternion initialQw = {0, gyroVals[0], gyroVals[1], gyroVals[2]};
      qiNew = kinEngine.GyroQuaternionUpdater(qiPrev, initialQw, 0.00000001);

      sets.push_back(rep);
      rep = 0;
      halfRep=0;
    }
    //cout << pauseCount << endl;
    if (moving == true){
      
      halfRep++;
      rep = halfRep / 2;
      if (inRep == true){
        halfrepTime = millis() - halfrepTimerStart;
        //Serial.print("Phase Duration: ");
        //Serial.print(halfrepTime);
        //Serial.println(" ms");
        inRep = false;
      }
      Serial.print("rep count = ");
      Serial.println(rep);
    }
    moving = false;
    paused = true;
    if (pauseCount < 2000){
      pauseCount++;
    }

    //Serial.println(pauseCount);
    
  }
  else{
    if (inRep == false){
      halfrepTimerStart = millis();
      inRep = true;
    }
    moving = true;
    paused = false;
    inSet = true;
    pauseCount = 0;
    //Serial.println(pauseCount);
    
  }


float pitch = atan2(adjustedAccelometerVals[0], sqrt(adjustedAccelometerVals[1]*adjustedAccelometerVals[1] + adjustedAccelometerVals[2]*adjustedAccelometerVals[2])) * 180.0 / M_PI;
float roll  = atan2(adjustedAccelometerVals[1], sqrt(adjustedAccelometerVals[0]*adjustedAccelometerVals[0] + adjustedAccelometerVals[2]*adjustedAccelometerVals[2])) * 180.0 / M_PI;

/*
Serial.print(adjustedAccelometerVals[0]); Serial.print(",");
Serial.print(adjustedAccelometerVals[1]); Serial.print(",");
Serial.print(adjustedAccelometerVals[2]); Serial.print(",");
Serial.print(gyroVals[0]); Serial.print(",");
Serial.print(gyroVals[1]); Serial.print(",");
Serial.print(gyroVals[2]); Serial.print(",");
Serial.print(pitch); Serial.print(",");
Serial.println(roll); // End the line here!
*/
   // 1. Expand array to 8 features including pitch and roll
float readings[8] = {
    adjustedAccelometerVals[0], adjustedAccelometerVals[1], adjustedAccelometerVals[2],
    gyroVals[0],                gyroVals[1],                gyroVals[2],
    pitch,                      roll
};

if (timerCount >= 4) {
    timerCount = 0;

    // 2. Push all 8 features into your buffer
    add_sensor_readings(readings);

    static float flat_buf[BUFFER_SIZE];
    for (size_t i = 0; i < BUFFER_SIZE; i++) {
        flat_buf[i] = input_buf[(head + i) % BUFFER_SIZE];
    }

    signal_t ei_signal;
    int signal_res = numpy::signal_from_buffer(flat_buf, BUFFER_SIZE, &ei_signal);
    if (signal_res != 0) {
        Serial.printf("ERR: Failed to create signal from buffer (%d)\n", signal_res);
        return;
    }

    static ei_impulse_result_t result = { 0 };
    EI_IMPULSE_ERROR res = run_classifier(&ei_signal, &result, false);

    if (res != EI_IMPULSE_OK) {
        Serial.printf("ERR: Failed to run classifier (%d)\n", res);
        return;
    }

// 1. Always evaluate predictions cleanly per frame
    std::string topClass = "Idle";
    float topVal = 0.0f;

    for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
        // Use a small threshold (e.g., > 0.5f or > topVal) so noise doesn't lock a class
        if (result.classification[ix].value > topVal && result.classification[ix].value > 0.6f) {
            topVal = result.classification[ix].value;
            topClass = std::string(result.classification[ix].label);
        }
    }

    if (!workEngine.checkForPausedSensitive(movementTrackerForWorkEngine)) {
        if (topClass != "Idle") {
            aiModel[topClass] += 1;
        }
    } else {
        // If movement stops, force topClass back to Idle
        topClass = "Idle";
    }

    // 2. Stream JSON to Serial
    Serial.printf("{\"maxProbabilityExcercise\":\"%s\",\"conf\":%d,\"rep\":%d,\"setNum\":%d}\n", 
                  topClass.c_str(), 
                  (int)(topVal * 100), 
                  rep, 
                  setNum);
    
}
  
}
