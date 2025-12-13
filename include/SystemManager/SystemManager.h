#ifndef SYSTEM_H
#define SYSTEM_H

#include <Arduino.h>
#include <Config.h>

#include <Logger/Logger.h>
#include <missionManager/missionManager.h>
#include <SerialClient/SerialClient.h>
#include <Robot/Robot.h>

class SystemManager
{
public:
    void initialize(Logger::Level logLevel = Logger::Level::INFO)
    {
        m_logger = new Logger(logLevel);
        m_missionManager = new missionManager(*m_logger);
        m_robot = new Robot(*m_logger, m_missionManager);
        m_serialClient = new SerialClient(*m_logger, m_missionManager, m_robot);
        if (config::TEST)
            m_missionManager->addFakeMissionForTest();
    }

    Logger* m_logger = nullptr;
    missionManager* m_missionManager = nullptr;
    Robot* m_robot = nullptr;
    SerialClient* m_serialClient = nullptr;
};




#endif // SYSTEM_H