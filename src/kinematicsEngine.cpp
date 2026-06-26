#include "kinematicsEngine.h"
#include <tuple>
#include <cmath>
using namespace std;


std::tuple<float, float, float> kinematicsEngine::initialAngleCalculator(float ax, float ay, float az){
    float a = std::sqrt((ax * ax) + (ay * ay) + (az * az));    
    float alpha = std::acos(ax/a);
    float beta = std::acos(ay/a);
    float gamma = std::acos(az/a);
    std::tuple<float, float, float> angleTuple = {alpha, beta, gamma};
    return angleTuple;
}
 
Quaternion kinematicsEngine::quaternionCalculator(std::tuple<float, float, float> angles){
    float alpha = std::get<0>(angles);
    float beta = std::get<1>(angles);
    float gamma = std::get<2>(angles);

    float cosA = cos(alpha / 2.0f);
    float sinA = sin(alpha / 2.0f);
    float cosB = cos(beta  / 2.0f);
    float sinB = sin(beta  / 2.0f);
    float cosG = cos(gamma / 2.0f);
    float sinG = sin(gamma / 2.0f);

    float qw = (cosA * cosB * cosG) + (sinA * sinB * sinG);
    float qx = (sinA * cosB * cosG) - (cosA * sinB * sinG);
    float qy = (cosA * sinB * cosG) + (sinA * cosB * sinG);
    float qz = (cosA * cosB * sinG) - (sinA * sinB * cosG);
    Quaternion q = {qw,qx,qy,qz};
    return q;
}

Quaternion kinematicsEngine::GyroQuaternionUpdater(Quaternion qi, Quaternion qw, int dt){

}
