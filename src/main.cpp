#include <Arduino.h>
#include <Config.h>
#include <Logger/Logger.h>
#include <SystemManager/SystemManager.h>

SystemManager systemManager;

void setup() {
    Serial.begin(config::SERIAL_BAUDRATE);
    SPI.begin();
    systemManager.initialize(Logger::Level::INFO);
    //systemManager.m_missionManager->addFakeMissionForTest();
}

void loop() {
    systemManager.m_robot->run();
    systemManager.m_serialClient->run();
}