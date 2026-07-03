
#include <vector>

class workoutEngine{
    private:

    public:
        float calculateSampleAverage(std::vector<float> samples);
        bool sampleSignChange(float prevAverage, float currentAverage);

};
