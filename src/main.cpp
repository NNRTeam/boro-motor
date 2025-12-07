#include <Arduino.h>
#include <Config.h>
#include <Logger/Logger.h>
#include <SystemManager/SystemManager.h>

SystemManager systemManager;

void setup() {
    Serial.begin(config::SERIAL_BAUDRATE);
    systemManager.initialize(Logger::Level::INFO);
}

void loop() {
    systemManager.m_robot->run();
}
