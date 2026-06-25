#include <Arduino.h>
#include <Wire.h>
#include <iostream>
#include <vector> // 
#define MPU_ADDR 0x68

class i2cSensor{
    private:

    public:
        byte deviceFinder();
        void powerUp();
        byte readByte(byte address);
        uint16_t readHalfWord(byte address);
        uint32_t readWord(byte address);
        
};

