#ifndef motor_control_hpp
#define motor_control_hpp

#include "HallSensor/HallSensor.h"
#include "Arduino.h"


class Motor{
    public:
        Motor(int dirPin, int stepPin, int sensorCS, bool invertSensor, double wheelPerimeter);
        void run();
        void setLinearSpeed(float speed);
        void stop() { m_speed = 0.0; }
        void UpdateWeelPerimeter(double weelPerimeter) { m_WheelPerimeter = weelPerimeter; }
        [[nodiscard]] float getFeedbackSpeed(unsigned int *dt, unsigned int *t);
    protected:

        void setSpeed(float speed);
        void stepFW();
        void stepBW();

        int PIN_DIR;          //Pin de direction
        int PIN_STEP;         //Pin de step

        float m_speed;         //Consigne de vitesse angulaire [rd.sec-1]

        uint64_t m_dt=0.0;                //delta t
        uint64_t m_t=0.0;                 //temps

        double m_WheelPerimeter;     //perimetre de la roue [mm]

        HallSensor* m_sensor;
        bool m_is_sensor_inverted;

        unsigned long long int m_lastStepTime = 0;
};





#endif