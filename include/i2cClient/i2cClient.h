#ifndef I2CCLIENT_H
#define I2CCLIENT_H

#include <Arduino.h>
#include <Wire.h>
#include <Config.h>
#include <Logger/Logger.h>
#include <missionManager/missionManager.h>
#include <Robot/Robot.h>

class I2CClient
{
public:

    enum class I2CCommands : uint8_t {
        ODOM = 0x01,
        MISSION_STATE = 0x02,
        ALIVE = 0x03
    };

    I2CClient(Logger& logger, missionManager* missionManager, Robot* robot);

    void begin();
    static void receiveData(int length);
    static void dataRequested();
    static I2CClient* instanceWrapper;
private:
    void setInstanceWrapper();
    Logger& m_logger;
    missionManager* m_missionManager;
    Robot* m_robot;
    I2CCommands m_lastCommand = I2CCommands::ODOM;
};

#endif // I2CCLIENT_H