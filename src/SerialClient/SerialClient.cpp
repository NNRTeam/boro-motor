#include "SerialClient/SerialClient.h"

SerialClient* SerialClient::instanceWrapper = nullptr; // or appropriate initial value

SerialClient::SerialClient(Logger& logger, missionManager* missionManager, Robot* robot) :
    m_logger(logger), m_missionManager(missionManager), m_robot(robot) {}

void SerialClient::run()
{
    receiveData();
    unsigned long long int const current_time = micros();
    if (m_odomPending || current_time - m_last_odom_send > 1e6/config::ODOM_FREQUENCY_HZ)
    {
        m_odomPending = false;
        m_last_odom_send = current_time;
        sendData();
    }
}

void SerialClient::receiveData() {
    if (!Serial.available())
        return;
    String receivedData = "";
    while (Serial.available()) {
        char c = Serial.read();
        if(c == 'F')
            break;
        receivedData += c;
    }

    if (receivedData[0] == 'R')
    {
        m_robot->resetOdometry();
    }
    else if (receivedData[0] == 'L')
    {
        // emergency stop command, set max deceleration to a high value to stop the robot as fast as possible
        m_robot->emergencyStop(true);
    }
    else if (receivedData[0] == 'O')
    {
        m_robot->parseOdometryData(receivedData);
    }
    else if (receivedData[0] == 'D')
    {
        enterDFUMode();
    }
    else
    {
        Mission mission;
        if (m_missionManager->parseMissionMessage(receivedData, mission)) {
            m_missionManager->addMission(mission);
            m_robot->emergencyStop(false);
        }
    }
}

void SerialClient::enterDFUMode() {
    // Pour Arduino Nano R4 (Renesas RA4M1):
    // Utiliser l'USB pour se déconnecter et forcer un reset bootloader

    Serial.end();  // Fermer le port série
    Serial.flush();
    delay(10);

    // Écrire le magic word pour le bootloader Arduino dans la RAM backup
    volatile uint32_t *const bootKeyAddr = (volatile uint32_t *)0x20007FFC;
    *bootKeyAddr = 0x07738135;  // Magic key Arduino pour double reset

    // Désactiver toutes les interruptions et déclencher un reset logiciel
    __disable_irq();
    NVIC_SystemReset();

    // Ne devrait jamais arriver ici
    while (1) {
        __WFI();  // Wait For Interrupt
    }
}

void writeUint32(uint32_t value) {
    Serial.write((uint8_t)(value >> 24)); // Octet le plus significatif
    Serial.write((uint8_t)(value >> 16));
    Serial.write((uint8_t)(value >> 8));
    Serial.write((uint8_t)value);         // Octet le moins significatif
}

// Fast float-to-buffer append (avoids snprintf overhead on RA4M1)
static char* appendFloat(char* p, float val, signed char width, unsigned char prec) {
    dtostrf(val, width, prec, p);
    while (*p) p++;
    return p;
}

void SerialClient::sendData() {
    float const x = m_robot->getX();
    float const y = m_robot->getY();
    float const theta = m_robot->getTheta();
    float const vl = m_robot->getLinearSpeed();
    float const vr = m_robot->getAngularSpeed();

    // Build packet in stack buffer without any heap allocation.
    // dtostrf is ~5x faster than snprintf for floats on this MCU.
    char buf[64];
    char* p = buf;
    *p++ = 'O';
    p = appendFloat(p, x, 1, 2);  *p++ = ';';
    p = appendFloat(p, y, 1, 2);  *p++ = ';';
    p = appendFloat(p, theta, 1, 2); *p++ = ';';
    p = appendFloat(p, vl, 1, 2); *p++ = ';';
    p = appendFloat(p, vr, 1, 2);
    *p++ = 'F'; *p++ = '\n'; *p = '\0';

    int len = (int)(p - buf);
    // Only send if full packet fits in TX buffer; otherwise retry next loop
    if (Serial.availableForWrite() >= len)
        Serial.write(buf, len);
    else
        m_odomPending = true;
}