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
    q = normalizer(q);
    return q;
}

Quaternion kinematicsEngine::normalizer(Quaternion q) {
    float magnitude = std::sqrt(q.qw * q.qw + q.qx * q.qx + q.qy * q.qy + q.qz * q.qz);

    if (magnitude > 0.000001f) {
        q.qw /= magnitude;
        q.qx /= magnitude;
        q.qy /= magnitude;
        q.qz /= magnitude;
    } else {
        q.qw = 1.0f;
        q.qx = 0.0f;
        q.qy = 0.0f;
        q.qz = 0.0f;
    }

    return q;
}

Quaternion kinematicsEngine::GyroQuaternionUpdater(Quaternion qi, Quaternion qw, int dt){
    //qi + 1 = qi + qwqi*dt/2
    Quaternion qiNew = quarternionAddition(qi, quarternionConstantMultiply(quarternionMultiply(qw, qi),(dt/2)));
    qiNew = normalizer(qiNew);
    return qiNew;
}

Quaternion kinematicsEngine::quarternionMultiply(Quaternion q1, Quaternion q2){
    Quaternion answer;
    answer.qw = q1.qw * q2.qw - q1.qx * q2.qx - q1.qy * q2.qy - q1.qz * q2.qz;
    answer.qx = q1.qw * q2.qx + q2.qw * q1.qx + q1.qy * q2.qz - q1.qz * q2.qy;
    answer.qy = q1.qw * q2.qy + q2.qw * q1.qy + q1.qz * q2.qx - q1.qx * q2.qz;
    answer.qz = q1.qw * q2.qz + q2.qw * q1.qz + q1.qx * q2.qy - q1.qy * q2.qx;
    
    return answer;
}

Quaternion kinematicsEngine::quarternionConstantMultiply(Quaternion q1, float c){
    Quaternion answer;

    answer.qw = q1.qw * c;
    answer.qx = q1.qx * c;
    answer.qy = q1.qy * c;
    answer.qz = q1.qz * c;
    
    return answer;
}

Quaternion kinematicsEngine::quarternionAddition(Quaternion q1, Quaternion q2){
    Quaternion answer;

    answer.qw = q1.qw + q2.qw;
    answer.qx = q1.qx + q2.qx;
    answer.qy = q1.qy + q2.qy;
    answer.qz = q1.qz + q2.qz;
    
    return answer;
}

Quaternion kinematicsEngine::quarternionContantAddition(Quaternion q1, float c){
    Quaternion answer;

    answer.qw = q1.qw + c;
    answer.qx = q1.qx + c;
    answer.qy = q1.qy + c;
    answer.qz = q1.qz + c;
    return answer;
}

Quaternion kinematicsEngine::quarternionSubtraction(Quaternion q1, Quaternion q2){
    Quaternion answer;

    answer.qw = q1.qw - q2.qw;
    answer.qx = q1.qx - q2.qx;
    answer.qy = q1.qy - q2.qy;
    answer.qz = q1.qz - q2.qz;
    
    return answer;
}

Quaternion kinematicsEngine::quarternionConstantSubtraction(Quaternion q1, float c){
    Quaternion answer;

    answer.qw = q1.qw - c;
    answer.qx = q1.qx - c;
    answer.qy = q1.qy - c;
    answer.qz = q1.qz - c;
    
    return answer;
}

Quaternion kinematicsEngine::quaternionLocalToGlobal(Quaternion q1, Quaternion A) {
    Quaternion q1_conjugate;
    q1_conjugate.qw = q1.qw;
    q1_conjugate.qw = -q1.qw;
    q1_conjugate.qw = -q1.qw;
    q1_conjugate.qw = -q1.qw;


    Quaternion intermediate = quarternionMultiply(q1, A);

    Quaternion global_vector = quarternionMultiply(intermediate, q1_conjugate);

    return global_vector;
}