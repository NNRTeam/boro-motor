#ifndef ROBOT_H
#define ROBOT_H

#include <Arduino.h>
#include <Config.h>
#include <Logger/Logger.h>
#include <missionManager/missionManager.h>
#include <Robot/Motor.h>

class Robot
{
    public:
        Robot(Logger logger) : m_logger(logger) {}

        void run();
        [[nodiscard]] float getX() const { return x; }
        [[nodiscard]] float getY() const { return y; }
        [[nodiscard]] float getTheta() const { return theta; }

    protected:
        void Control();
        void updateOdometry();
        void setSpeeds(float linearSpeed, float angularSpeed);
        void samsonUpdateMotors();
        void rotationUpdateMotors();
        void stop();
        bool checkMissionArrived();

        Logger m_logger;

        float x = 0.0; //m
        float y = 0.0; //m
        float theta = 0.0; //rad

        missionManager* m_missionManager = nullptr;

        Motor* motor_left;
        Motor* motor_right;

        float m_linearSpeed = 0.0; //m/s
        float m_angularSpeed = 0.0; //rad/s

        float m_lastLinearSpeed = 0.0; //m/s
        float m_lastAngularSpeed = 0.0; //rad/s

        float m_linearSpeedMotor = 0.0; //m/s
        float m_angularSpeedMotor = 0.0; //rad/s

        float m_lastLinearSpeedMotor = 0.0; //m/s
        float m_lastAngularSpeedMotor = 0.0; //rad/s

        unsigned long long int m_lastControlTime = 0;
    private:
        unsigned int m_dt1 = 0;
        unsigned int m_dt2 = 0;
        unsigned int m_dt = 0;
        unsigned int m_t = 0;

};


#endif // ROBOT_H