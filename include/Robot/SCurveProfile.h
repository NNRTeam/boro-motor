#ifndef SCURVE_PROFILE_H
#define SCURVE_PROFILE_H

class SCurveProfile {
public:
    /// Compute a full 7-phase S-curve for the given distance.
    void compute(float distance, float v_max, float a_acc, float a_dec, float j_acc, float j_dec);

    /// Get the (unsigned) speed at time t seconds since profile start.
    float getSpeed(float t) const;

    /// Get distance traveled at time t.
    float getDistance(float t) const;

    float getTotalTime() const { return m_totalTime; }
    float getTotalDistance() const { return m_totalDistance; }
    bool  isFinished(float t) const { return t >= m_totalTime; }

private:
    struct Phase {
        float t_start;   // cumulative start time
        float duration;
        float jerk;
        float a0;        // acceleration at phase start
        float v0;        // velocity at phase start
        float s0;        // distance at phase start
    };

    float accelDistance(float v_target, float a_max, float j) const;
    float decelDistance(float v_target, float a_max, float j) const;

    Phase m_phases[7];
    int   m_numPhases    = 0;
    float m_totalTime    = 0.0f;
    float m_totalDistance = 0.0f;
};

#endif // SCURVE_PROFILE_H
