#include <tuple>
struct Quaternion{
    float qw = 1.0f; 
    float qx = 0.0f; 
    float qy = 0.0f; 
    float qz = 0.0f;
    Quaternion(){

    }
    Quaternion(float w, float x, float y, float z){
        qw = w;
        qx = x;
        qy = y; 
        qz = z;
    }
};

class kinematicsEngine{
    private:

    public:
        std::tuple<float, float, float> initialAngleCalculator(float ax, float ay, float az);
        Quaternion quaternionCalculator(std::tuple<float, float, float> angles);
        Quaternion GyroQuaternionUpdater(Quaternion qi, Quaternion qw, int dt);
        Quaternion quarternionMultiply(Quaternion q1, Quaternion q2);
        Quaternion quarternionConstantMultiply(Quaternion q1, float c);
        Quaternion quarternionAddition(Quaternion q1, Quaternion q2);
        Quaternion quarternionContantAddition(Quaternion q1, float c);
        Quaternion quarternionSubtraction(Quaternion q1, Quaternion q2);
        Quaternion quarternionConstantSubtraction(Quaternion q1, float c);
        Quaternion normalizer(Quaternion q);
        Quaternion quaternionLocalToGlobal(Quaternion q1, Quaternion A);




};