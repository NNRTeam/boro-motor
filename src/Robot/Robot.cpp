#include "Robot/Robot.h"
#include <Utils.h>
#include <Arduino.h>

void Robot::run() {
    if (m_missionManager->hasActiveMissions())
        Control();
    motor_left->run();
    motor_right->run();
}

void Robot::Control()
{
    unsigned long long currentTime = micros();
    if (currentTime - m_lastControlTime >= 1e6/config::CONTROL_LOOP_FREQUENCY_HZ) {
        updateOdometry();
        checkMissionArrived();
        Mission* CurentMission = m_missionManager->getCurrentMission();
        if (CurentMission->getType() == Mission::Type::GO)
            samsonUpdateMotors();
        else if (CurentMission->getType() == Mission::Type::TURN)
            rotationUpdateMotors();
        else if (CurentMission->getType() == Mission::Type::STOP || CurentMission->getType() == Mission::Type::WAIT)
            stop();
        m_lastControlTime = currentTime;
    }
}

void Robot::updateOdometry() {
    unsigned int dt1, dt2, t;
    double right_speed = this->motor_right->getFeedbackSpeed(&dt1, &t);
    double left_speed = this->motor_left->getFeedbackSpeed(&dt2, &t);

    m_dt = (dt1+dt2)/2;
    m_t = t;

    m_linearSpeed = (left_speed + right_speed)/2;
    m_angularSpeed = (right_speed-left_speed)/config::OD_WHEEL_BASE_MM;

    // integration selon la methode des trapezes
    float angular_speed = (0.5*m_angularSpeed+0.5*m_lastAngularSpeed);
    float linear_speed = (0.5*m_linearSpeed+0.5*m_lastLinearSpeed);
    if (abs(angular_speed) > 0.0001){ // dans le cas ou la vitesse angulaire est non nulle on utilise la methode de l'arc de cercle
        //methode de l'arc de cercle pour calculer la nouvelle position
        theta = theta + angular_speed*m_dt/1000000.0;
        float R = linear_speed/angular_speed;
        x = x - R*sin(theta) + R*sin(theta+angular_speed*m_dt/1000000.0);
        y = y + R*cos(theta) - R*cos(theta+angular_speed*m_dt/1000000.0);
    }else{
        //methode de la droite pour calculer la nouvelle position
        theta = theta + angular_speed*m_dt/1000000.0;
        x = x + linear_speed*cos(theta)*m_dt/1000000.0;
        y = y + linear_speed*sin(theta)*m_dt/1000000.0;
    }
    m_lastLinearSpeed = m_linearSpeed;
    m_lastAngularSpeed = m_angularSpeed;
    theta = utils::normalizeAngle(theta);
}

void Robot::setSpeeds(float linearSpeed, float angularSpeed)
{
    float v_right = linearSpeed + (angularSpeed*config::M_WHEEL_BASE_MM)/2.0f;
    float v_left = linearSpeed - (angularSpeed*config::M_WHEEL_BASE_MM)/2.0f;

    motor_right->setLinearSpeed(v_right);
    motor_left->setLinearSpeed(v_left);
}

void Robot::samsonUpdateMotors()
{
    float const dx = m_missionManager->getCurrentMission()->getTargetX() - x;
    float const dy = m_missionManager->getCurrentMission()->getTargetY() - y;
    float const distance = sqrt(dx * dx + dy * dy);
    float const angle_cible = atan2(dy, dx);

    bool const forward = m_missionManager->getCurrentMission()->isForward();

    float theta_error;
    if (forward)
        theta_error = angle_cible - theta;
    else
        theta_error = (theta + M_PI) - angle_cible;
    theta_error = utils::normalizeAngle(theta_error);

    float omega = 0.0;
    if (forward) {
        omega = utils::getMin(utils::getMax(theta_error, -config::MAX_ANGULAR_VELOCITY_RAD_S), config::MAX_ANGULAR_VELOCITY_RAD_S);
    } else {
        omega = utils::getMax(utils::getMin(-theta_error, config::MAX_ANGULAR_VELOCITY_RAD_S), -config::MAX_ANGULAR_VELOCITY_RAD_S);
    }

    float v_max;
    if (distance < 0.1) { // Ralentir à l'approche
        v_max = config::MAX_LINEAR_VELOCITY_M_S * (distance / 0.1);
    } else {
        v_max = config::MAX_LINEAR_VELOCITY_M_S;
    }
    float v;
    if (forward) {
        v = m_lastLinearSpeedMotor + config::LINEAR_ACCELERATION_M_S2 * (m_dt / 1000000.0);
        v = utils::getMin(utils::getMax(v, 0.0f), v_max);
    } else {
        v = m_lastLinearSpeedMotor - config::LINEAR_ACCELERATION_M_S2 * (m_dt / 1000000.0);
        v = utils::getMax(utils::getMin(v, 0.0f), -v_max); // Limite à -v_max pour le recul
    }
    if (distance < 0.01) {
        v = 0.0;
        omega = 0.0;
    }
    m_lastLinearSpeedMotor = v;
    m_lastAngularSpeedMotor = omega;
    setSpeeds(v, omega);
}

void Robot::rotationUpdateMotors()
{
    float const target_theta = m_missionManager->getCurrentMission()->getTargetTheta();
    float angle_diff = utils::normalizeAngle(target_theta - theta);
    float omega_desired;
    if (abs(angle_diff) < 0.01) { // Si l'angle est déjà atteint, arrêter le robot
        motor_left->stop();
        motor_right->stop();
        m_lastAngularSpeedMotor = 0.0;
        return;
    }
    // Calcul de la distance d'arrêt
    float stop_distance = (m_lastAngularSpeedMotor * m_lastAngularSpeedMotor) / (2.0f * config::ANGULAR_ACCELERATION_RAD_S2);
    // Calcul de la vitesse angulaire souhaitée
    if (abs(angle_diff) < stop_distance) {
        omega_desired = utils::sign(angle_diff) * sqrt(2.0f * config::ANGULAR_ACCELERATION_RAD_S2 * abs(angle_diff));
    } else {
        omega_desired = utils::sign(angle_diff) * config::MAX_ANGULAR_VELOCITY_RAD_S;
    }
    // Lissage de l'accélération angulaire
    float omega = m_lastAngularSpeedMotor + utils::sign(omega_desired - m_lastAngularSpeedMotor) * config::ANGULAR_ACCELERATION_RAD_S2 * (m_dt / 1000000.0);
    omega = utils::sign(omega_desired) * utils::getMin(abs(omega), abs(omega_desired));
    m_lastAngularSpeedMotor = omega;
    setSpeeds(0.0f, omega);
}

void Robot::stop()
{
    motor_left->stop();
    motor_right->stop();
}

bool Robot::checkMissionArrived()
{
    Mission* currentMission = m_missionManager->getCurrentMission();
    if (currentMission->getType() == Mission::Type::GO) {
        float const dx = currentMission->getTargetX() - x;
        float const dy = currentMission->getTargetY() - y;
        float const distance = sqrt(dx * dx + dy * dy);
        if (distance < config::GO_MISSION_TOLERANCE_M) {
            m_missionManager->endCurrentMission();
            return true;
        }
    } else if (currentMission->getType() == Mission::Type::TURN) {
        float const target_theta = currentMission->getTargetTheta();
        float angle_diff = utils::normalizeAngle(target_theta - theta);
        if (abs(angle_diff) < config::TURN_MISSION_TOLERANCE_RAD) {
            m_missionManager->endCurrentMission();
            return true;
        }
    }
    return false;
}