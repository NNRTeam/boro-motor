/**
 * @file motor_controle.cpp
 * @brief Implementation of the Motor class member functions.
 */

#include "Robot/Motor.h"
#include <Config.h>

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


Motor::Motor(int dirPin, int stepPin, int sensorCS, bool invertSensor, double wheelPerimeter, void (*stepCallback)())
    : m_stepCallback(stepCallback), PIN_DIR(dirPin), PIN_STEP(stepPin), m_WheelPerimeter(wheelPerimeter), m_is_sensor_inverted(invertSensor)
{
    m_sensor = new HallSensor(sensorCS, invertSensor);
    pinMode(PIN_DIR, OUTPUT);
    pinMode(PIN_STEP, OUTPUT);
}

void Motor::run()
{
    if(m_speed == 0.0)
        return;
    unsigned long long int currentTime = micros();
    unsigned int interStepTime = calculerTempsEntreSteps(m_speed,
                                                        config::STEP_PER_REVOLUTION,
                                                        config::MICROSTEPS);
    if (currentTime - m_lastStepTime >= interStepTime)
    {
        if (m_speed > 0)
            stepFW();

        else if (m_speed < 0)
            stepBW();

        m_lastStepTime = currentTime;
    }

}

void Motor::setLinearSpeed(float speed)
{
    setSpeed(speed/(m_WheelPerimeter/(2*PI)));
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