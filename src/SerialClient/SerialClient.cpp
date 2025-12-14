#include "SerialClient/SerialClient.h"

SerialClient* SerialClient::instanceWrapper = nullptr; // or appropriate initial value

SerialClient::SerialClient(Logger& logger, missionManager* missionManager, Robot* robot) :
    m_logger(logger), m_missionManager(missionManager), m_robot(robot) {}

void SerialClient::run()
{
    receiveData();
    unsigned long long int const current_time = micros();
    if (current_time - m_last_odom_send > 1e6/config::ODOM_FREQUENCY_HZ)
    {
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
    else if (receivedData[0] == 'D')
    {
        enterDFUMode();
    }
    else
    {
        Mission* mission = m_missionManager->parseMissionMessage(receivedData);
        m_missionManager->addMission(*mission);
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

void SerialClient::sendData() {
    //Mission* currentMission = m_missionManager->getCurrentMission();
    float const x = m_robot->getX();
    float const y = m_robot->getY();
    float const theta = m_robot->getTheta();
    float const vl = m_robot->getLinearSpeed();
    float const vr = m_robot->getAngularSpeed();
    String d = "O" + String(x) + ";" + String(y) + ";" + String(theta) + ";" + String(vl) + ";" + String(vr) + "F";
    Serial.println(d);
}