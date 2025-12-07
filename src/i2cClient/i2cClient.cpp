#include "i2cClient/i2cClient.h"

I2CClient* I2CClient::instanceWrapper = nullptr; // or appropriate initial value

I2CClient::I2CClient(Logger& logger, missionManager* missionManager, Robot* robot) :
    m_logger(logger), m_missionManager(missionManager), m_robot(robot) {}

void I2CClient::begin() {
    Wire.begin(config::I2C_ADD);
    setInstanceWrapper();
    Wire.onReceive(receiveData);
    Wire.onRequest(dataRequested);
    m_logger.info("I2C Client initialized at address " + String(config::I2C_ADD));
}

void I2CClient::receiveData(int length) {
    if (instanceWrapper == nullptr) {
        return; // Safety check
    }
    String receivedData = "";
    char c;
    while (Wire.available()) {
        c = Wire.read();
        receivedData += c;
    }

    if (receivedData.length() == 1) {
        switch (c)
        {
        case 0x01:
            instanceWrapper->m_lastCommand = I2CCommands::ODOM;
            break;
        case 0x02:
            instanceWrapper->m_lastCommand = I2CCommands::MISSION_STATE;
            break;
        default:
            break;
        }
    }
    else {
        instanceWrapper->m_logger.debug("Received I2C data: " + receivedData);
        Mission* mission = instanceWrapper->m_missionManager->parseMissionMessage(receivedData);
        instanceWrapper->m_missionManager->addMission(*mission);
    }
}

void I2CClient::dataRequested() {
    if (instanceWrapper == nullptr) {
        return; // Safety check
    }
    if (instanceWrapper->m_lastCommand == I2CCommands::ODOM)
    {
        float const x = instanceWrapper->m_robot->getX();
        float const y = instanceWrapper->m_robot->getY();
        float const theta = instanceWrapper->m_robot->getTheta();
        Wire.write((uint32_t)(x * 1000)); // Convert to mm
        Wire.write((uint32_t)(y * 1000)); // Convert to mm
        Wire.write((uint32_t)(theta * 10000)); // Convert to 0.0001 rad
    }
    else if (instanceWrapper->m_lastCommand == I2CCommands::MISSION_STATE)
    {
        Mission* currentMission = instanceWrapper->m_missionManager->getCurrentMission();
        Wire.write((uint32_t)currentMission->getId());
        Wire.write((uint32_t)currentMission->getStatus());
    }
}

void I2CClient::setInstanceWrapper() {
    instanceWrapper = this;
}

