#ifndef HALL_SENSOR_HPP
#define HALL_SENSOR_HPP

#include <Arduino.h>
#include <SPI.h>

const int DEVICE_CONFIG = 0x00; //0x0 is used for writes, should really handle this in the read command
const int SENSOR_CONFIG = 0x01;   
const int SYSTEM_CONFIG = 0x02; // this is where we can switch to the 12bit XY sampling mode                                      
const int TEST_CONFIG = 0x0F;
const int X_CH_RESULT = 0x89; //0x8 is used for reads, should really handle this in the write command
const int Y_CH_RESULT = 0x8A;
const int Z_CH_RESULT = 0x8B;
const int TEMP_RESULT = 0x8C;
const int ANGL_RESULT = 0x93;
const int range = 50;

class HallSensor{

    bool invert;
    int pinCS;
    float previousAngle;
    unsigned long previousTime;
    public :
        HallSensor(int pinCS, bool invert);
        double getSpeed(unsigned int *dt, unsigned int *t);
        float getAngle();
        float getAngle360();
        float getAngle360(bool invert);
        float getx();
        float gety();

        void setupSensor();
        void sendSPI(int8_t address, int8_t data1, int8_t data2, int8_t comande);
        int16_t readSPI(int8_t address, int8_t data1, int8_t data2, int8_t comande);
};

#endif // HALL_SENSOR_HPP