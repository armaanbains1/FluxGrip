#include <workoutEngine.h>
#include <algorithm>
#include <iostream>

using namespace std;
float workoutEngine::calculateSampleAverage(std::vector<float> samples){
    float total = 0;
    for (auto & val : samples){
        total += val;
    }
    return total / 10; 
}

bool workoutEngine::sampleSignChange(float prevAverage, float currentAverage){
    if (prevAverage == 0 && currentAverage != 0){
        return true;
    }
    else{
        return false;
    }
}

bool workoutEngine::checkForPaused(std::vector<float> stream){
    //cout << stream.size() << endl;
    int count = std::count(stream.begin(), stream.end(), 0);
    if ((count >= 7)){
        //cout << "trued, count is "  << count << endl;
            
        return true;
    }
    else{
        //cout << "false" << endl;

        return false;
    }
}

bool workoutEngine::checkForPausedSensitive(std::vector<float> stream){
    int count = std::count(stream.begin(), stream.end(), 0);
    if ((count >= 5)){
        //cout << "trued, count is "  << count << endl;
            
        return true;
    }
    else{
        //cout << "false" << endl;

        return false;
    }
}


bool workoutEngine::checkForMoving(std::vector<float> stream){
    //cout << stream.size() << endl;
    int count = std::count(stream.begin(), stream.end(), 0);
    if ((count == 0)){
        //cout << "trued, count is "  << count << endl;
            
        return true;
    }
    else{
        //cout << "false" << endl;

        return false;
    }
}