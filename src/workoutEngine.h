
#include <vector>

class workoutEngine{
    private:

    public:
        float calculateSampleAverage(std::vector<float> samples);
        bool sampleSignChange(float prevAverage, float currentAverage);
        bool checkForPaused(std::vector<float> stream);
        bool checkForMoving(std::vector<float> stream);
        bool checkForPausedSensitive(std::vector<float> stream);
};
