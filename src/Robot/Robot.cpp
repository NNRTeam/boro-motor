#include "Robot/Robot.h"
#include <Utils.h>
#include <Arduino.h>
#include <Config.h>

Robot* Robot::instance = nullptr;

Robot::Robot(Logger& logger, missionManager *missionManager) : m_logger(logger), m_missionManager(missionManager)
{
    pinMode(config::M_EN_PIN, OUTPUT);
    digitalWrite(config::M_EN_PIN, HIGH); // free motors
    if (config::MOTOR_ODOM_ONLY) {
        motor_left = new Motor(config::M1_DIR_PIN,
                                config::M1_STEP_PIN,
                                -1,
                                false,
                                config::M1_INVERT_DIR,
                                config::M1_WHEEL_DIAMETER_MM * M_PI / 1000.0,
                                Robot::leftMotorStepNotify);

        motor_right = new Motor(config::M2_DIR_PIN,
                                config::M2_STEP_PIN,
                                -1,
                                true,
                                config::M2_INVERT_DIR,
                                config::M2_WHEEL_DIAMETER_MM * M_PI / 1000.0,
                                Robot::rightMotorStepNotify);
    }
    else {
        motor_left = new Motor(config::M1_DIR_PIN,
                                config::M1_STEP_PIN,
                                config::OD1_CS_PIN,
                                false,
                                config::M1_INVERT_DIR,
                                config::M1_WHEEL_DIAMETER_MM * M_PI / 1000.0,
                                Robot::leftMotorStepNotify);

        motor_right = new Motor(config::M2_DIR_PIN,
                                config::M2_STEP_PIN,
                                config::OD2_CS_PIN,
                                true,
                                config::M2_INVERT_DIR,
                                config::M2_WHEEL_DIAMETER_MM * M_PI / 1000.0,
                                Robot::rightMotorStepNotify);
    }
    Robot::instance = this;
}

void Robot::run() {
    // Appeler run() en premier pour minimiser la latence entre deux steps,
    // indépendamment du temps pris par Control().
    motor_left->run();
    motor_right->run();
    Control();
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
    m_linearSpeedMotor = 0.0;
    m_angularSpeedMotor = 0.0;
    m_currentLinearAccel = 0.0;
    m_leftStepCount = 0;
    m_rightStepCount = 0;
}

void Robot::Control()
{
    unsigned long long currentTime = micros();

    if (!config::MOTOR_ODOM_ONLY && currentTime - m_lastOdomTime > 1e6/config::ODOM_MEASURE_FREQUENCY_HZ) {
        updateOdometry();
        m_lastOdomTime = currentTime;
    }

    if (currentTime - m_lastControlTime >= 1e6/config::CONTROL_LOOP_FREQUENCY_HZ) {
        // Plafonner m_dt pour éviter une explosion au premier tick ou après une longue pause
        unsigned long long elapsed = currentTime - m_lastControlTime;
        unsigned int const maxDt = (unsigned int)(2.0f * 1e6f / config::CONTROL_LOOP_FREQUENCY_HZ);
        m_dt = (elapsed > maxDt) ? maxDt : (unsigned int)elapsed;
        m_lastControlTime = currentTime;
        if (config::MOTOR_ODOM_ONLY) {
            updateMotorOdometry();
        }
        if (m_isEmergencyStop) {
            setEmergencyStopMotorSpeed();
            return;
        }
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
            m_lastAssignedMission = currentTime;
            digitalWrite(config::M_EN_PIN, LOW);
            checkMissionArrived();
            Mission* CurrentMission = m_missionManager->getCurrentMission();
            if (CurrentMission == nullptr) {
                stop();
                return;
            }
            else if (CurrentMission->isActive() == false) {
                Mission* nextMission = m_missionManager->startNextMission();
                if (nextMission == nullptr) {
                    stop();
                    return;
                }
                float const dx = nextMission->getTargetX() - getX();
                float const dy = nextMission->getTargetY() - getY();
                float const angle_cible = atan2(dy, dx);
                nextMission->setThetaGoTo(angle_cible);
            }

            if (CurrentMission->getType() == Mission::Type::GO)
                samsonUpdateMotors();
            else if (CurrentMission->getType() == Mission::Type::TURN)
                rotationUpdateMotors();
            else if (CurrentMission->getType() == Mission::Type::STOP || CurrentMission->getType() == Mission::Type::WAIT)
                stop();

            //m_logger.debug("Robot Position - X: " + String(getX(), 4) + " m, Y: " + String(getY(), 4) + " m, Theta: " + String(getTheta(), 4) + " rad");
            //m_logger.debug("Robot Speed - Linear: " + String(getLinearSpeed(), 4) + " m/s, Angular: " + String(getAngularSpeed(), 4) + " rad/s");
        }
        else {
            // Libérer les moteurs après 10s d'inactivité
            if (currentTime - m_lastAssignedMission > 10ULL * 1000000ULL) {
                digitalWrite(config::M_EN_PIN, HIGH);
            }
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
    float dTheta = angular_speed * m_dt / (float)1e6;
    if (abs(dTheta) > 1e-6f) {
        // Méthode de l'arc de cercle
        float R = linear_speed / angular_speed;
        x += R * (sin(theta + dTheta) - sin(theta));
        y += -R * (cos(theta + dTheta) - cos(theta));
    } else {
        // Méthode de la droite
        x += linear_speed * cos(theta) * m_dt / (float)1e6;
        y += linear_speed * sin(theta) * m_dt / (float)1e6;
    }
    theta = utils::normalizeAngle(theta + dTheta);
    m_lastLinearSpeed = m_linearSpeed;
    m_lastAngularSpeed = m_angularSpeed;
}

void Robot::emergencyStop(bool enable)
{
    if (enable)
        m_missionManager->cancelAllMissions();
    m_isEmergencyStop = enable;
}

void Robot::setSpeeds(float linearSpeed, float angularSpeed)
{
    float v_right = linearSpeed + (angularSpeed*config::M_WHEEL_BASE_MM/1000.0)/2.0f;
    float v_left = linearSpeed - (angularSpeed*config::M_WHEEL_BASE_MM/1000.0)/2.0f;

    motor_right->setLinearSpeed(v_right);
    motor_left->setLinearSpeed(v_left);
}

void Robot::setEmergencyStopMotorSpeed()
{
    if (m_angularSpeedMotor > 0) {
        m_angularSpeedMotor = utils::getMax(m_angularSpeedMotor - config::ANGULAR_EMERGENCY_MAX_DECELEATION * (m_dt / (float)1e6), 0.0f);
    } else if (m_angularSpeedMotor < 0) {
        m_angularSpeedMotor = utils::getMin(m_angularSpeedMotor + config::ANGULAR_EMERGENCY_MAX_DECELEATION * (m_dt / (float)1e6), 0.0f);
    }
    
    if (m_linearSpeedMotor > 0) {
        m_linearSpeedMotor = utils::getMax(m_linearSpeedMotor - config::LINEAR_EMERGENCY_MAX_DECELEATION * (m_dt / (float)1e6), 0.0f);
    } else if (m_linearSpeedMotor < 0) {
        m_linearSpeedMotor = utils::getMin(m_linearSpeedMotor + config::LINEAR_EMERGENCY_MAX_DECELEATION * (m_dt / (float)1e6), 0.0f);
    }
    m_lastLinearSpeed = m_linearSpeed;
    m_lastAngularSpeed = m_angularSpeed;
    setSpeeds(m_linearSpeedMotor, m_angularSpeedMotor);
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

    // Pour la marche arrière : l'arrière du robot doit pointer vers angle_cible.
    // L'erreur est angle_cible - (theta + π), soit l'opposé de ce que l'on avait.
    float theta_error;
    if (forward)
        theta_error = angle_cible - getTheta();
    else
        theta_error = angle_cible - getTheta() - M_PI;
    theta_error = utils::normalizeAngle(theta_error);

    // Contrôleur Samson amélioré avec prise en compte de l'erreur latérale
    float const kp_angular = 5.0f;     // Gain proportionnel pour la correction angulaire
    float const kp_lateral = 2.0f;     // Gain proportionnel pour la correction latérale
    float const k_damping = 0.5f;      // Gain d'amortissement pour stabiliser l'approche

    // En marche arrière (v < 0), l'effet de ω sur la déviation latérale est inversé,
    // donc les termes de correction latérale changent de signe.
    float const lateral_sign = forward ? -1.0f : 1.0f;
    float omega_desired = kp_angular * theta_error
                        + lateral_sign * kp_lateral * lateral_error
                        + lateral_sign * k_damping * lateral_error * abs(longitudinal_distance) / (distance + 0.01f);

    // Limiter la vitesse angulaire désirée
    if (forward) {
        omega_desired = utils::getMin(utils::getMax(omega_desired, -config::MAX_ANGULAR_VELOCITY_RAD_S), config::MAX_ANGULAR_VELOCITY_RAD_S);
    } else {
        omega_desired = utils::getMax(utils::getMin(omega_desired, config::MAX_ANGULAR_VELOCITY_RAD_S), -config::MAX_ANGULAR_VELOCITY_RAD_S);
    }

    // Lisser l'accélération angulaire
    float omega = m_angularSpeedMotor;
    float omega_diff = omega_desired - omega;
    float max_omega_change = config::ANGULAR_ACCELERATION_RAD_S2 * (m_dt / (float)1e6);
    if (abs(omega_diff) > max_omega_change) {
        omega += utils::sign(omega_diff) * max_omega_change;
    } else {
        omega = omega_desired;
    }

    // Distance restante utile le long de la trajectoire.
    // `longitudinal_distance` est toujours négative tant que la cible n'est pas atteinte
    // (angle_cible pointe vers la cible, donc le robot est « derrière » elle).
    // La formule est identique en marche avant et en marche arrière.
    float remaining_distance = utils::getMax(-longitudinal_distance, 0.0f);

    // Distance de freinage avec prise en compte du jerk.
    // Avec profil trapézoïdal en accélération (jerk limité) :
    //   d_frein = v²/(2a) + v*a/(2j)
    // Le terme v*a/(2j) représente la distance supplémentaire parcourue
    // pendant que la décélération monte en rampe de 0 à a_max.
    float const a_max = config::LINEAR_ACCELERATION_M_S2;
    float const j_max = config::LINEAR_JERK_M_S3;
    float const braking_margin = utils::getMax(remaining_distance - config::GO_MISSION_TOLERANCE_M*0.5, 0.0f);

    // Résoudre la vitesse max admissible pour freiner sur braking_margin
    // d = v²/(2a) + v*a/(2j)  =>  (1/(2a)) v² + (a/(2j)) v - d = 0
    // Coefficients : A = 1/(2a), B = a/(2j), C = -d
    // v = (-B + sqrt(B² + 4*A*d)) / (2*A)
    float const A = 1.0f / (2.0f * a_max);
    float const B = a_max / (2.0f * j_max);
    float const discriminant = B * B + 4.0f * A * braking_margin;
    float v_max;
    if (discriminant > 0.0f) {
        v_max = (-B + sqrt(discriminant)) / (2.0f * A);
    } else {
        v_max = 0.0f;
    }
    v_max = utils::getMin(v_max, config::MAX_LINEAR_VELOCITY_M_S);
    float const v_max_braking = v_max; // limite dure pour la distance de freinage

    // Réduire aussi la vitesse si l'erreur latérale ou angulaire est importante.
    // Utilisé uniquement comme cible douce (via le jerk limiter) pour éviter
    // les à-coups si l'erreur de cap fluctue légèrement.
    float const heading_slowdown = 1.0f / (1.0f + 2.0f * abs(theta_error));
    float const lateral_slowdown = 1.0f / (1.0f + 4.0f * abs(lateral_error));
    float const v_max_soft = v_max * utils::getMax(0.2f, heading_slowdown * lateral_slowdown);

    // Vitesse cible signée (cible douce)
    float const target_speed = forward ? v_max_soft : -v_max_soft;

    // Accélération désirée : proportionnelle à l'écart de vitesse,
    // saturée à ±a_max. Ceci gère symétriquement accélération et décélération.
    float const v_error = target_speed - m_linearSpeedMotor;
    float const kp_accel = 10.0f; // gain pour convertir erreur de vitesse en consigne d'accélération
    float accel_desired = utils::getMin(utils::getMax(kp_accel * v_error, -a_max), a_max);

    // Limiter le jerk (variation de l'accélération) symétriquement
    // en accélération ET en décélération
    float const dt_s = m_dt / (float)1e6;
    float const max_jerk_change = j_max * dt_s;
    float accel_diff = accel_desired - m_currentLinearAccel;
    if (abs(accel_diff) > max_jerk_change) {
        m_currentLinearAccel += utils::sign(accel_diff) * max_jerk_change;
    } else {
        m_currentLinearAccel = accel_desired;
    }

    // Limiter l'accélération courante à ±a_max
    m_currentLinearAccel = utils::getMin(utils::getMax(m_currentLinearAccel, -a_max), a_max);

    // Appliquer l'accélération lissée pour calculer la nouvelle vitesse
    float v = m_linearSpeedMotor + m_currentLinearAccel * dt_s;

    // Contraindre la vitesse : limites absolues ET profil de freinage (v_max)
    // v_max intègre déjà MAX_LINEAR_VELOCITY_M_S, ce clamp assure que le robot
    // ne dépasse jamais la vitesse admissible pour freiner à temps (déplacements courts).
    if (forward) {
        v = utils::getMax(v, 0.0f);
        v = utils::getMin(v, v_max_braking); // clamp dur : distance de freinage uniquement
    } else {
        v = utils::getMin(v, 0.0f);
        v = utils::getMax(v, -v_max_braking);
    }
    // Resynchroniser m_currentLinearAccel avec la vitesse réellement appliquée,
    // pour éviter que l'état interne du jerk limiter reste en avance sur la réalité.
    if (dt_s > 1e-9f) {
        float const implied_accel = (v - m_linearSpeedMotor) / dt_s;
        m_currentLinearAccel = utils::getMin(utils::getMax(implied_accel, -a_max), a_max);
    }

    // Arrêt final seulement quand il ne reste quasiment plus rien à parcourir.
    if (remaining_distance < config::GO_MISSION_TOLERANCE_M / 4.0f && abs(v) < 0.02f) {
        v = 0.0;
        omega = 0.0;
        m_currentLinearAccel = 0.0;
    }

    if (remaining_distance < 2 * config::GO_MISSION_TOLERANCE_M) {
        // Ramener omega vers 0 progressivement pour éviter un à-coup latéral
        float const max_omega_final = config::ANGULAR_ACCELERATION_RAD_S2 * dt_s;
        if (abs(omega) > max_omega_final)
            omega -= utils::sign(omega) * max_omega_final;
        else
            omega = 0.0f;
    }

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
    m_currentLinearAccel = 0.0;
    m_lastAngularSpeed = 0.0;
    m_lastLinearSpeed = 0.0;
}

bool Robot::checkMissionArrived()
{
    Mission* currentMission = m_missionManager->getCurrentMission();
    if (currentMission->getType() == Mission::Type::GO) {
        float const angle_cible = currentMission->getTargetTheta();
        float const longitudinal_distance = (getX() - currentMission->getTargetX()) * cos(angle_cible)
                                           + (getY() - currentMission->getTargetY()) * sin(angle_cible);
        float const remaining_distance = utils::getMax(-longitudinal_distance, 0.0f);
        if (remaining_distance < config::GO_MISSION_TOLERANCE_M) {
            m_missionManager->endCurrentMission();
            m_angularSpeedMotor = 0.0;
            m_linearSpeedMotor = 0.0;
            m_currentLinearAccel = 0.0;
            return true;
        }
    } else if (currentMission->getType() == Mission::Type::TURN) {
        float const target_theta = currentMission->getTargetTheta();
        float angle_diff = utils::normalizeAngle(target_theta - getTheta());
        if (abs(angle_diff) < config::TURN_MISSION_TOLERANCE_RAD) {
            m_missionManager->endCurrentMission();
            m_angularSpeedMotor = 0.0;
            m_linearSpeedMotor = 0.0;
            m_currentLinearAccel = 0.0;
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
    if (forward) instance->m_leftStepCount++;
    else instance->m_leftStepCount--;
}

void Robot::rightMotorStepNotify(bool forward)
{
    if (instance == nullptr) return;
    if (forward) instance->m_rightStepCount++;
    else instance->m_rightStepCount--;
}

void Robot::updateMotorOdometry()
{
    // Lire et remettre à zéro les compteurs de pas (section critique)
    noInterrupts();
    long leftSteps = m_leftStepCount;
    long rightSteps = m_rightStepCount;
    m_leftStepCount = 0;
    m_rightStepCount = 0;
    interrupts();

    // Calculer la distance parcourue par chaque roue
    float const stepsPerRev = config::STEP_PER_REVOLUTION * config::MICROSTEPS;
    float distLeft = leftSteps * motor_left->getWheelPerimeter() / stepsPerRev;
    float distRight = rightSteps * motor_right->getWheelPerimeter() / stepsPerRev;

    // Cinématique différentielle
    float dCenter = (distLeft + distRight) / 2.0f;
    float dTheta = (distRight - distLeft) / (config::M_WHEEL_BASE_MM / 1000.0f);

    // Mise à jour de la position par arc de cercle ou ligne droite
    if (abs(dTheta) > 1e-6f) {
        float R = dCenter / dTheta;
        m_motorX += R * (sin(m_motorTheta + dTheta) - sin(m_motorTheta));
        m_motorY += -R * (cos(m_motorTheta + dTheta) - cos(m_motorTheta));
    } else {
        m_motorX += dCenter * cos(m_motorTheta);
        m_motorY += dCenter * sin(m_motorTheta);
    }
    m_motorTheta = utils::normalizeAngle(m_motorTheta + dTheta);
}