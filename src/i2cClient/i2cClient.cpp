#include "i2cClient/i2cClient.h"

I2CClient::I2CClient(Logger& logger, missionManager* missionManager) :
    m_logger(logger), m_missionManager(missionManager) {}

bool I2CClient::begin() {
    Wire.begin(config::I2C_ADD);
    setInstanceWrapper();
    Wire.onReceive(receiveData);
    Wire.onRequest(dataRequested);
    m_logger.info("I2C Client initialized at address " + String(config::I2C_ADD));
    return true;
}

void I2CClient::receiveData(int length) {
    String receivedData = "";
    while (Wire.available()) {
        char c = Wire.read();
        receivedData += c;
    }
    instanceWrapper->m_logger.debug("Received I2C data: " + receivedData);
    Mission* mission = instanceWrapper->m_missionManager->parseMissionMessage(receivedData);
    instanceWrapper->m_missionManager->addMission(*mission);
}

void I2CClient::dataRequested() {
    String dataToSend = "TODO: Implement ODOMETRY DATA";
    for (size_t i = 0; i < dataToSend.length(); ++i) {
        Wire.write(dataToSend.charAt(i));
    }
}

void I2CClient::setInstanceWrapper() {
    instanceWrapper = this;
}

