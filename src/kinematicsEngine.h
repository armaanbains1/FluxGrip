#include <tuple>
struct Quaternion{
    float qw = 1.0f; 
    float qx = 0.0f; 
    float qy = 0.0f; 
    float qz = 0.0f;

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
        Quaternion GyroQuaternionUpdater(std::tuple<float, float, float, float> angles);


};