/**
 * @file motor_controle.cpp
 * @brief Implementation of the Motor class member functions.
 */

#include "Robot/Motor.h"
#include <Config.h>

#define M_PI 3.14159265358979323846

unsigned int calculerTempsEntreSteps(
    double vitesse_angulaire_rad_s,
    int steps_moteur,
    int microstepping)
{
    double vitesse_tours_par_seconde = vitesse_angulaire_rad_s / (2 * PI);
    double steps_par_tour = steps_moteur * microstepping;
    double temps_entre_steps = 1.0 / (vitesse_tours_par_seconde * steps_par_tour); // en secondes
    unsigned int temps_microsecondes = static_cast<unsigned int>(round(temps_entre_steps * 1e6)); // en microsecondes

    return temps_microsecondes;
}


Motor::Motor(int dirPin, int stepPin, int sensorCS, bool invertSensor, double wheelPerimeter, void (*stepCallback)(bool forward))
    : m_stepCallback(stepCallback), PIN_DIR(dirPin), PIN_STEP(stepPin), m_WheelPerimeter(wheelPerimeter), m_is_sensor_inverted(invertSensor)
{
    if (sensorCS != -1)
        m_sensor = new HallSensor(sensorCS, invertSensor);
    pinMode(PIN_DIR, OUTPUT);
    pinMode(PIN_STEP, OUTPUT);
}

void Motor::run()
{
    if(m_speed == 0.0)
        return;
    unsigned long long int currentTime = micros();
    unsigned int interStepTime = calculerTempsEntreSteps(abs(m_speed),
                                                        config::STEP_PER_REVOLUTION,
                                                        config::MICROSTEPS);
    if (currentTime - m_lastStepTime >= interStepTime)
    {
        if (m_speed > 0)
        {
            stepFW();
            m_stepCallback(!m_is_sensor_inverted);
        }
        else if (m_speed < 0)
        {
            stepBW();
            m_stepCallback(m_is_sensor_inverted);
        }

        m_lastStepTime = currentTime;
    }

}

void Motor::setLinearSpeed(float speed)
{
    float speed_rad_s = speed/(m_WheelPerimeter/(2*PI)) * ( m_is_sensor_inverted ? -1 : 1);
    setSpeed(speed_rad_s);
}

void Motor::setSpeed(float speed)
{
    m_speed = speed;
    if (m_speed < 0.0)
        digitalWrite(PIN_DIR, LOW);
    else
        digitalWrite(PIN_DIR, HIGH);
}

void Motor::stepFW()
{
    digitalWrite(PIN_STEP, HIGH);
    delayMicroseconds(2);
    digitalWrite(PIN_STEP, LOW);
}

void Motor::stepBW()
{
    digitalWrite(PIN_STEP, HIGH);
    delayMicroseconds(2);
    digitalWrite(PIN_STEP, LOW);
}

float Motor::getFeedbackSpeed(unsigned int *dt, unsigned int *t){
    float mesuredSpeed = m_sensor->getSpeed(dt, t)*m_WheelPerimeter/(2*PI);
    m_dt = *dt;
    m_t = *t;

    return mesuredSpeed;
}