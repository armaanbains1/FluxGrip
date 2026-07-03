#include <workoutEngine.h>

float workoutEngine::calculateSampleAverage(std::vector<float> samples){
    float total = 0;
    for (auto & val : samples){
        total += val;
    }
    return total / 200; 
}

bool workoutEngine::sampleSignChange(float prevAverage, float currentAverage){
    if (prevAverage == 0 && currentAverage != 0){
        return true;
    }
    else{
        return false;
    }
}