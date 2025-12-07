#include <Arduino.h>
#include <Config.h>
#include <Logger/Logger.h>
#include <missionManager/missionManager.h>


void setup() {
    Serial.begin(config::SERIAL_BAUDRATE);
    while (!Serial) {}
    Logger logger(Logger::Level::INFO);
    missionManager mManager(logger);
}

void loop() {

}
