#ifndef SYSTEM_H
#define SYSTEM_H

#include <Arduino.h>
#include <Config.h>

#include <Logger/Logger.h>
#include <missionManager/missionManager.h>
#include <i2cClient/i2cClient.h>
#include <Robot/Robot.h>

class SystemManager
{
public:
    void initialize(Logger::Level logLevel = Logger::Level::INFO)
    {
        m_logger = new Logger(logLevel);
        m_missionManager = new missionManager(*m_logger);
        m_robot = new Robot(*m_logger);
        m_i2cClient = new I2CClient(*m_logger, m_missionManager, m_robot);
        m_i2cClient->begin();
        if (config::TEST)
            m_missionManager->addFakeMissionForTest();
    }

    Logger* m_logger = nullptr;
    missionManager* m_missionManager = nullptr;
    Robot* m_robot = nullptr;
    I2CClient* m_i2cClient = nullptr;
};




#endif // SYSTEM_H