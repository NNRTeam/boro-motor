#include <Arduino.h>
#include <Config.h>
#include <Logger/Logger.h>
#include <SystemManager/SystemManager.h>

SystemManager systemManager;

void setup() {
    delay(5000);
    Serial.begin(config::SERIAL_BAUDRATE);
    systemManager.initialize(Logger::Level::INFO);
    delay(1000);
    //systemManager.m_missionManager->addFakeMissionForTest();
}

void loop() {
    systemManager.m_robot->run();
    systemManager.m_serialClient->run();
}