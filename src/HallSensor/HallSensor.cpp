#include "HallSensor/HallSensor.h"
#include <Utils.h>

/**
 * @brief Constructor for the HallSensor class.
 * @param CS The chip select pin number for the hall sensor.
*/
HallSensor::HallSensor(int CS, bool invert){
    this->invert = invert;
    this->pinCS = CS;
    this->setupSensor();
    this->previousAngle = this->getAngle();
    this->previousTime = 0;
}

double HallSensor::getSpeed(unsigned int *dt, unsigned int *t){
    float a = this->getAngle();

    *t = micros();

    unsigned int dt_ = *t - this->previousTime;

    if (dt_ > 1000000){
        dt_ = 0;
        this->previousTime = micros();
        *dt = -1;
        this->previousAngle = a;
        return 0;
    }

    this->previousTime = micros();
    float da = utils::normalizeAngle(a-this->previousAngle);
    this->previousAngle = a;

    double speed = da/dt_*1000000.0;
    if (dt != NULL)*dt = dt_;
    return speed;
}

float HallSensor::getAngle(){
    float a = 0;
    if (this->invert)
        a = 2*PI-this->readSPI(ANGL_RESULT, 0x00, 0x00, 0x00)/16.0*2.*PI/360.0;
    else
        a = this->readSPI(ANGL_RESULT, 0x00, 0x00, 0x00)/16.0*2.*PI/360.0;
    //Serial.println("Angle read: " + String(a));
    return utils::normalizeAngle(a);
}

float HallSensor::getAngle360(){
    if (this->invert){
        return 360.0 - this->readSPI(ANGL_RESULT, 0x00, 0x00, 0x00)/16.0;
    }
    return this->readSPI(ANGL_RESULT, 0x00, 0x00, 0x00)/16.0;
}
float HallSensor::getAngle360(bool invert){
    if(invert){
        return 360.0 - this->readSPI(ANGL_RESULT, 0x00, 0x00, 0x00)/16.0;
    }
    return this->readSPI(ANGL_RESULT, 0x00, 0x00, 0x00)/16.0;
}

float HallSensor::getx(){
    return this->readSPI(X_CH_RESULT, 0x00, 0x00, 0x00);
}

float HallSensor::gety(){
    return this->readSPI(Y_CH_RESULT, 0x00, 0x00, 0x00);
}

/**
 * @brief Send a command to the hall sensor using SPI.
 * @param address The address of the register to write to.
 * @param data1 The first byte of data to write.
 * @param data2 The second byte of data to write.
 * @param comande The command to write.
*/
void HallSensor::sendSPI(int8_t address, int8_t data1, int8_t data2, int8_t comande){
    digitalWrite(this->pinCS, LOW);
    SPI.transfer(address);
    SPI.transfer(data1);
    SPI.transfer(data2);
    SPI.transfer(comande);
    digitalWrite(this->pinCS, HIGH);
}

/**
 * @brief Read data from the hall sensor using SPI.
 * @param address The address of the register to read from.
 * @param data1 The first byte of data to read.
 * @param data2 The second byte of data to read.
 * @param comande The command to read.
 * @return The data read from the hall sensor.
*/
int16_t HallSensor::readSPI(int8_t address, int8_t data1, int8_t data2, int8_t comande){
    digitalWrite(this->pinCS, LOW);
    SPI.transfer(address);
    int16_t data = SPI.transfer(data1);
    data = data << 8;
    data |= SPI.transfer(data2);
    //SPI.transfer(comande);    // this is not needed
    digitalWrite(this->pinCS, HIGH);
    //Serial.println("Data read: " + String(float(data)));
    return data;
}

/**
 * @brief Setup the hall sensor.
*/
void HallSensor::setupSensor(){
    pinMode(this->pinCS, OUTPUT);
    delay(10);
    this->sendSPI(TEST_CONFIG, 0x00, 0x04, 0x07);
    delay(50);
    this->sendSPI(SENSOR_CONFIG, 0x42, 0x00, 0x00);
    delay(50);
    this->sendSPI(DEVICE_CONFIG, 0x41, 0x20, 0x00);
    delay(100);
}