#ifndef I2CCLIENT_H
#define I2CCLIENT_H

#include <Arduino.h>
#include <Wire.h>
#include <Config.h>
#include <Logger/Logger.h>
#include <missionManager/missionManager.h>

class I2CClient
{
public:
    I2CClient(Logger& logger, missionManager* missionManager);

    [[nodiscard]] bool begin();
    static void receiveData(int length);
    static void dataRequested();
private:
    static I2CClient* instanceWrapper;
    void setInstanceWrapper();
    Logger& m_logger;
    missionManager* m_missionManager;
};

#endif // I2CCLIENT_H