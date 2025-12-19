#include "Robot/Robot.h"
#include <Utils.h>
#include <Arduino.h>
#include <Config.h>

Robot* Robot::instance = nullptr;

Robot::Robot(Logger logger, missionManager *missionManager) : m_logger(logger), m_missionManager(missionManager)
{
    pinMode(config::M_EN_PIN, OUTPUT);
    digitalWrite(config::M_EN_PIN, HIGH); // free motors
    if (config::MOTOR_ODOM_ONLY) {
        motor_left = new Motor(config::M1_DIR_PIN,
                                config::M1_STEP_PIN,
                                -1,
                                false,
                                config::M1_WHEEL_DIAMETER_MM * M_PI / 1000.0,
                                Robot::leftMotorStepNotify);

        motor_right = new Motor(config::M2_DIR_PIN,
                                config::M2_STEP_PIN,
                                -1,
                                true,
                                config::M2_WHEEL_DIAMETER_MM * M_PI / 1000.0,
                                Robot::rightMotorStepNotify);
    }
    else {
        motor_left = new Motor(config::M1_DIR_PIN,
                                config::M1_STEP_PIN,
                                config::OD1_CS_PIN,
                                false,
                                config::M1_WHEEL_DIAMETER_MM * M_PI / 1000.0,
                                Robot::leftMotorStepNotify);

        motor_right = new Motor(config::M2_DIR_PIN,
                                config::M2_STEP_PIN,
                                config::OD2_CS_PIN,
                                true,
                                config::M2_WHEEL_DIAMETER_MM * M_PI / 1000.0,
                                Robot::rightMotorStepNotify);
    }
    Robot::instance = this;
}

void Robot::run() {
    Control();
    motor_left->run();
    motor_right->run();
}

void Robot::resetOdometry() {
    x = 0.0;
    y = 0.0;
    theta = 0.0;

    m_motorX = 0.0;
    m_motorY = 0.0;
    m_motorTheta = 0.0;

    m_lastLinearSpeed = 0.0;
    m_lastAngularSpeed = 0.0;
}

void Robot::Control()
{
    unsigned long long currentTime = micros();

    if (!config::MOTOR_ODOM_ONLY && currentTime - m_lastOdomTime > 1e6/config::ODOM_MEASURE_FREQUENCY_HZ) {
        updateOdometry();
        m_lastOdomTime = currentTime;
    }

    if (currentTime - m_lastControlTime >= 1e6/config::CONTROL_LOOP_FREQUENCY_HZ) {
        bool hasActiveMission = m_missionManager->hasActiveMissions();
        if (!hasActiveMission && m_missionManager->hasMissions())
        {
            m_missionManager->startNextMission();
            hasActiveMission = m_missionManager->hasActiveMissions();
            if (hasActiveMission && m_missionManager->getCurrentMission()->getType() == Mission::Type::GO) {
                Mission* CurrentMission = m_missionManager->getCurrentMission();
                float const dx = m_missionManager->getCurrentMission()->getTargetX() - getX();
                float const dy = m_missionManager->getCurrentMission()->getTargetY() - getY();
                float const angle_cible = atan2(dy, dx);
                CurrentMission->setThetaGoTo(angle_cible);
            }
        }
        if (hasActiveMission)
        {
            digitalWrite(config::M_EN_PIN, LOW);
            checkMissionArrived();
            Mission* CurrentMission = m_missionManager->getCurrentMission();
            if (CurrentMission == nullptr) {
                stop();
                m_lastControlTime = currentTime;
                return;
            }
            else if (CurrentMission->isActive() == false) {
                m_missionManager->startNextMission();
                float const dx = m_missionManager->getCurrentMission()->getTargetX() - getX();
                float const dy = m_missionManager->getCurrentMission()->getTargetY() - getY();
                float const angle_cible = atan2(dy, dx);
                m_missionManager->getCurrentMission()->setThetaGoTo(angle_cible);
            }

            if (CurrentMission->getType() == Mission::Type::GO)
                samsonUpdateMotors();
            else if (CurrentMission->getType() == Mission::Type::TURN)
                rotationUpdateMotors();
            else if (CurrentMission->getType() == Mission::Type::STOP || CurrentMission->getType() == Mission::Type::WAIT)
                stop();
            m_lastControlTime = currentTime;

            m_logger.debug("Robot Position - X: " + String(getX(), 4) + " m, Y: " + String(getY(), 4) + " m, Theta: " + String(getTheta(), 4) + " rad");
            m_logger.debug("Robot Speed - Linear: " + String(getLinearSpeed(), 4) + " m/s, Angular: " + String(getAngularSpeed(), 4) + " rad/s");
        }
        else if (currentTime - m_lastControlTime > 10*1e6) {
            digitalWrite(config::M_EN_PIN, HIGH); // free motors
        }
    }
}

void Robot::updateOdometry() {
    unsigned int dt1, dt2, t;
    double right_speed = this->motor_right->getFeedbackSpeed(&dt1, &t);
    double left_speed = this->motor_left->getFeedbackSpeed(&dt2, &t);

    m_dt = (dt1+dt2)/2;
    m_t = t;

    m_linearSpeed = (left_speed + right_speed)/2;
    m_angularSpeed = (right_speed-left_speed)/(config::OD_WHEEL_BASE_MM/1000.0);

    // Serial.print("Left speed: " + String(left_speed) + " m/s, Right speed: " + String(right_speed) + " m/s ");
    // Serial.println("=> Linear speed: " + String(m_linearSpeed) + " m/s, Angular speed: " + String(m_angularSpeed) + " rad/s");

    // integration selon la methode des trapezes
    float angular_speed = (0.5*m_angularSpeed+0.5*m_lastAngularSpeed);
    float linear_speed = (0.5*m_linearSpeed+0.5*m_lastLinearSpeed);
    if (abs(angular_speed) > 0.0001){ // dans le cas ou la vitesse angulaire est non nulle on utilise la methode de l'arc de cercle
        //methode de l'arc de cercle pour calculer la nouvelle position
        theta = theta + angular_speed*m_dt/(float)1e6;
        float R = linear_speed/angular_speed;
        x = x - R*sin(theta) + R*sin(theta+angular_speed*m_dt/(float)1e6);
        y = y + R*cos(theta) - R*cos(theta+angular_speed*m_dt/(float)1e6);
    }else{
        //methode de la droite pour calculer la nouvelle position
        theta = theta + angular_speed*m_dt/(float)1e6;
        x = x + linear_speed*cos(theta)*m_dt/(float)1e6;
        y = y + linear_speed*sin(theta)*m_dt/(float)1e6;
    }
    m_lastLinearSpeed = m_linearSpeed;
    m_lastAngularSpeed = m_angularSpeed;
    theta = utils::normalizeAngle(theta);
}

void Robot::setSpeeds(float linearSpeed, float angularSpeed)
{
    float v_right = linearSpeed + (angularSpeed*config::M_WHEEL_BASE_MM/1000.0)/2.0f;
    float v_left = linearSpeed - (angularSpeed*config::M_WHEEL_BASE_MM/1000.0)/2.0f;

    motor_right->setLinearSpeed(v_right);
    motor_left->setLinearSpeed(v_left);
}

void Robot::samsonUpdateMotors()
{
    float const dx = m_missionManager->getCurrentMission()->getTargetX() - getX();
    float const dy = m_missionManager->getCurrentMission()->getTargetY() - getY();
    float const distance = sqrt(dx * dx + dy * dy);
    //float const angle_cible = atan2(dy, dx);
    float const angle_cible =  m_missionManager->getCurrentMission()->getTargetTheta();
    bool const forward = m_missionManager->getCurrentMission()->isForward();

    // Calculer la distance latérale (perpendiculaire) à la droite de référence
    // La droite est définie par le point (targetX, targetY) et l'angle angle_cible
    // Distance signée perpendiculaire = (x - targetX)*sin(angle_cible) - (y - targetY)*cos(angle_cible)
    float const lateral_error = (getX() - m_missionManager->getCurrentMission()->getTargetX()) * sin(angle_cible)
                               - (getY() - m_missionManager->getCurrentMission()->getTargetY()) * cos(angle_cible);

    // Calculer la distance longitudinale (le long de la droite de référence)
    float const longitudinal_distance = (getX() - m_missionManager->getCurrentMission()->getTargetX()) * cos(angle_cible)
                                       + (getY() - m_missionManager->getCurrentMission()->getTargetY()) * sin(angle_cible);

    float theta_error;
    if (forward)
        theta_error = angle_cible - getTheta();
    else
        theta_error = (getTheta() + M_PI) - angle_cible;
    theta_error = utils::normalizeAngle(theta_error);

    // Contrôleur Samson amélioré avec prise en compte de l'erreur latérale
    float const kp_angular = 5.0f;     // Gain proportionnel pour la correction angulaire
    float const kp_lateral = 2.0f;     // Gain proportionnel pour la correction latérale
    float const k_damping = 0.5f;      // Gain d'amortissement pour stabiliser l'approche

    // Vitesse angulaire désirée : correction angulaire + correction latérale + amortissement
    float omega_desired = kp_angular * theta_error
                        - kp_lateral * lateral_error
                        - k_damping * lateral_error * abs(longitudinal_distance) / (distance + 0.01f);

    // Limiter la vitesse angulaire désirée
    if (forward) {
        omega_desired = utils::getMin(utils::getMax(omega_desired, -config::MAX_ANGULAR_VELOCITY_RAD_S), config::MAX_ANGULAR_VELOCITY_RAD_S);
    } else {
        omega_desired = utils::getMax(utils::getMin(omega_desired, config::MAX_ANGULAR_VELOCITY_RAD_S), -config::MAX_ANGULAR_VELOCITY_RAD_S);
    }

    // Lisser l'accélération angulaire
    float omega = m_angularSpeedMotor;
    float omega_diff = omega_desired - omega;
    float max_omega_change = config::ANGULAR_ACCELERATION_RAD_S2 * (m_dt / (float)1e6) * 2;
    if (abs(omega_diff) > max_omega_change) {
        omega += utils::sign(omega_diff) * max_omega_change;
    } else {
        omega = omega_desired;
    }

    // Calculer la vitesse maximale cible
    float v_max;
    float const distance_to_slow = config::MAX_LINEAR_VELOCITY_M_S * config::MAX_LINEAR_VELOCITY_M_S / config::LINEAR_ACCELERATION_M_S2 / 2;
    if (distance < distance_to_slow) { // Ralentir à l'approche
        v_max = config::MAX_LINEAR_VELOCITY_M_S * (distance / distance_to_slow);
    } else {
        v_max = config::MAX_LINEAR_VELOCITY_M_S;
    }

    // Calculer l'accélération désirée en fonction de l'écart de vitesse
    float v_error = v_max - m_linearSpeedMotor;
    float accel_desired;

    if (forward) {
        if (v_error > 0) {
            accel_desired = config::LINEAR_ACCELERATION_M_S2;  // Accélérer
        } else if (v_error < 0) {
            accel_desired = -config::LINEAR_ACCELERATION_M_S2; // Décélérer
        } else {
            accel_desired = 0.0f; // Maintenir la vitesse
        }
    } else {
        if (v_error < 0) {
            accel_desired = -config::LINEAR_ACCELERATION_M_S2; // Accélérer en arrière
        } else if (v_error > 0) {
            accel_desired = config::LINEAR_ACCELERATION_M_S2;  // Décélérer
        } else {
            accel_desired = 0.0f; // Maintenir la vitesse
        }
    }

    // Limiter le jerk (variation de l'accélération)
    float accel_diff = accel_desired - m_currentLinearAccel;
    float max_jerk_change = config::LINEAR_JERK_M_S3 * (m_dt / (float)1e6);
    if (abs(accel_diff) > max_jerk_change) {
        m_currentLinearAccel += utils::sign(accel_diff) * max_jerk_change;
    } else {
        m_currentLinearAccel = accel_desired;
    }

    // Appliquer l'accélération lissée pour calculer la nouvelle vitesse
    float v = m_linearSpeedMotor + m_currentLinearAccel * (m_dt / (float)1e6);

    // Contraindre la vitesse dans les limites
    if (forward) {
        v = utils::getMax(v, 0.0f);
        v = utils::getMin(v, config::MAX_LINEAR_VELOCITY_M_S);
    } else {
        v = utils::getMin(v, 0.0f);
        v = utils::getMax(v, -config::MAX_LINEAR_VELOCITY_M_S);
    }

    // Arrêt final proche de la cible
    if (distance < config::GO_MISSION_TOLERANCE_M/2.0f) {
        v = 0.0;
        omega = 0.0;
        m_currentLinearAccel = 0.0;
    }

    if (distance < 2 * config::GO_MISSION_TOLERANCE_M)
        omega = 0.0f;

    m_linearSpeedMotor = v;
    m_angularSpeedMotor = omega;
    setSpeeds(v, omega);
}

void Robot::rotationUpdateMotors()
{
    float target_theta = m_missionManager->getCurrentMission()->getTargetTheta();
    bool const forward = m_missionManager->getCurrentMission()->isForward();

    if (!forward) {
        target_theta = utils::normalizeAngle(target_theta + M_PI);
    }

    float angle_diff = utils::normalizeAngle(target_theta - getTheta());
    float omega_desired;
    if (abs(angle_diff) < 0.01) { // Si l'angle est déjà atteint, arrêter le robot
        motor_left->stop();
        motor_right->stop();
        m_angularSpeedMotor = 0.0f;
        return;
    }
    // Calcul de la distance d'arrêt
    float stop_distance = (m_angularSpeedMotor * m_angularSpeedMotor) / (2.0f * config::ANGULAR_ACCELERATION_RAD_S2);
    // Calcul de la vitesse angulaire souhaitée
    if (abs(angle_diff) < stop_distance) {
        omega_desired = utils::sign(angle_diff) * sqrt(2.0f * config::ANGULAR_ACCELERATION_RAD_S2 * abs(angle_diff));
    } else {
        omega_desired = utils::sign(angle_diff) * config::MAX_ANGULAR_VELOCITY_RAD_S;
    }
    // Lissage de l'accélération angulaire
    float omega = m_angularSpeedMotor + utils::sign(omega_desired - m_angularSpeedMotor) * config::ANGULAR_ACCELERATION_RAD_S2 * (m_dt / (float)1e6);
    omega = utils::sign(omega_desired) * utils::getMin(abs(omega), abs(omega_desired));
    m_angularSpeedMotor = omega;
    m_linearSpeedMotor = 0.0;
    setSpeeds(0.0f, omega);
}

void Robot::stop()
{
    motor_left->stop();
    motor_right->stop();
    m_linearSpeedMotor = 0.0;
    m_angularSpeedMotor = 0.0;
    m_lastAngularSpeed = 0.0;
    m_lastLinearSpeed = 0.0;
}

bool Robot::checkMissionArrived()
{
    Mission* currentMission = m_missionManager->getCurrentMission();
    if (currentMission->getType() == Mission::Type::GO) {
        float const dx = currentMission->getTargetX() - getX();
        float const dy = currentMission->getTargetY() - getY();
        float const distance = sqrt(dx * dx + dy * dy);
        if (distance < config::GO_MISSION_TOLERANCE_M) {
            m_missionManager->endCurrentMission();
            m_angularSpeedMotor = 0.0;
            m_linearSpeedMotor = 0.0;
            return true;
        }
    } else if (currentMission->getType() == Mission::Type::TURN) {
        float const target_theta = currentMission->getTargetTheta();
        float angle_diff = utils::normalizeAngle(target_theta - getTheta());
        if (abs(angle_diff) < config::TURN_MISSION_TOLERANCE_RAD) {
            m_missionManager->endCurrentMission();
            m_angularSpeedMotor = 0.0;
            m_linearSpeedMotor = 0.0;
            return true;
        }
    }
    return false;
}

void Robot::parseOdometryData(String const &message)
{
    // Message format: "Ox.x;y.y;theta.t" or "Ox.x;y.y;theta.tF"
    String data = message.substring(1); // Remove 'O' prefix
    int firstSemicolon = data.indexOf(';');
    int secondSemicolon = data.indexOf(';', firstSemicolon + 1);

    m_motorX = data.substring(0, firstSemicolon).toFloat();
    m_motorY = data.substring(firstSemicolon + 1, secondSemicolon).toFloat();
    m_motorTheta = utils::normalizeAngle(data.substring(secondSemicolon + 1).toFloat());

    x = m_motorX;
    y = m_motorY;
    theta = m_motorTheta;
}

void Robot::leftMotorStepNotify(bool forward)
{
    if (instance == nullptr) return;
    float distance_motor = (instance->motor_left->getWheelPerimeter()) / (config::STEP_PER_REVOLUTION * config::MICROSTEPS); // in mm
    if (!forward)
        distance_motor = -distance_motor;
    float const robot_angle_change = distance_motor / config::M_WHEEL_BASE_MM * 1000.0; // in rad
    float const avg_distance = distance_motor/2.0f; // in m
    instance->m_motorTheta -= robot_angle_change; // in rad
    instance->m_motorTheta = utils::normalizeAngle(instance->m_motorTheta); // Normaliser entre -π et π
    instance->m_motorX += avg_distance * cos(instance->m_motorTheta); // in m
    instance->m_motorY += avg_distance * sin(instance->m_motorTheta); // in m
}

void Robot::rightMotorStepNotify(bool forward)
{
    if (instance == nullptr) return;
    float distance_motor = (instance->motor_right->getWheelPerimeter()) / (config::STEP_PER_REVOLUTION * config::MICROSTEPS); // in mm
    if (!forward)
        distance_motor = -distance_motor;
    float const robot_angle_change = distance_motor / config::M_WHEEL_BASE_MM * 1000.0; // in rad
    float const avg_distance = distance_motor/2.0f; // in m
    instance->m_motorTheta += robot_angle_change; // in rad
    instance->m_motorTheta = utils::normalizeAngle(instance->m_motorTheta); // Normaliser entre -π et π
    instance->m_motorX += avg_distance * cos(instance->m_motorTheta); // in m
    instance->m_motorY += avg_distance * sin(instance->m_motorTheta); // in m
}