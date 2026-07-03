#include <engineClock.h>
#include <chrono>
#include <thread>

void engineClock::samplingClock(bool clock){
    clock = false;
    auto start_time = std::chrono::high_resolution_clock::now();
    auto target_time = std::chrono::duration<double>(0.005);
    while (true) {
        auto current_time = std::chrono::high_resolution_clock::now();
        auto elapsed_time = current_time - start_time;
        clockTrigger = true;

        if (elapsed_time >= target_time){
            break;
        }
    }
    auto start_time1 = std::chrono::high_resolution_clock::now();
    auto target_time1 = std::chrono::duration<double>(1.595);
    while (true) {
        auto current_time1 = std::chrono::high_resolution_clock::now();
        auto elapsed_time1 = current_time1 - start_time1;
        clockTrigger = false;
        if (elapsed_time1 >= target_time1){
            break;
        }
    }
}



