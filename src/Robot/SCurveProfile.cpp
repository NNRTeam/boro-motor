#include "Robot/SCurveProfile.h"
#include <math.h>

// ---------------------------------------------------------------------------
// Distance covered during the acceleration part (phases 1-2-3) to reach
// velocity v_target, given max acceleration a_max and jerk j.
// ---------------------------------------------------------------------------
float SCurveProfile::accelDistance(float v_target, float a_max, float j) const {
    if (v_target <= 0.0f) return 0.0f;

    float v_jerk = a_max * a_max / j;  // v reachable with jerk-only (no const-accel)
    float a_peak, t1, t2, t3;

    if (v_target <= v_jerk) {
        // Cannot reach a_max — use reduced peak acceleration
        a_peak = sqrtf(v_target * j);
        t1 = a_peak / j;
        t2 = 0.0f;
        t3 = t1;
    } else {
        a_peak = a_max;
        t1 = a_max / j;
        t2 = (v_target - v_jerk) / a_max;
        t3 = t1;
    }

    // Phase 1: j = +j, a0 = 0, v0 = 0
    float v1 = 0.5f * j * t1 * t1;
    float d1 = (1.0f / 6.0f) * j * t1 * t1 * t1;

    // Phase 2: j = 0, a0 = a_peak, v0 = v1
    float v2 = v1 + a_peak * t2;
    float d2 = v1 * t2 + 0.5f * a_peak * t2 * t2;

    // Phase 3: j = -j, a0 = a_peak, v0 = v2
    float d3 = v2 * t3 + 0.5f * a_peak * t3 * t3 - (1.0f / 6.0f) * j * t3 * t3 * t3;

    return d1 + d2 + d3;
}

// ---------------------------------------------------------------------------
// Distance covered during the deceleration part (phases 5-6-7) from
// velocity v_target down to 0.
// ---------------------------------------------------------------------------
float SCurveProfile::decelDistance(float v_target, float a_max, float j) const {
    if (v_target <= 0.0f) return 0.0f;

    float v_jerk = a_max * a_max / j;
    float a_peak, t5, t6, t7;

    if (v_target <= v_jerk) {
        a_peak = sqrtf(v_target * j);
        t5 = a_peak / j;
        t6 = 0.0f;
        t7 = t5;
    } else {
        a_peak = a_max;
        t5 = a_max / j;
        t6 = (v_target - v_jerk) / a_max;
        t7 = t5;
    }

    // Phase 5: j = -j, a0 = 0, v0 = v_target
    float v5 = v_target - 0.5f * j * t5 * t5;
    float d5 = v_target * t5 - (1.0f / 6.0f) * j * t5 * t5 * t5;

    // Phase 6: j = 0, a0 = -a_peak, v0 = v5
    float v6 = v5 - a_peak * t6;
    float d6 = v5 * t6 - 0.5f * a_peak * t6 * t6;

    // Phase 7: j = +j, a0 = -a_peak, v0 = v6
    float d7 = v6 * t7 - 0.5f * a_peak * t7 * t7 + (1.0f / 6.0f) * j * t7 * t7 * t7;

    return d5 + d6 + d7;
}

// ---------------------------------------------------------------------------
// Compute the full 7-phase S-curve.
// ---------------------------------------------------------------------------
void SCurveProfile::compute(float distance, float v_max, float a_acc, float a_dec, float j_acc, float j_dec) {
    m_numPhases    = 0;
    m_totalTime    = 0.0f;
    m_totalDistance = 0.0f;

    if (distance <= 0.0f || v_max <= 0.0f) return;

    // --- Find achievable peak velocity ----------------------------------
    float v_peak = v_max;
    float d_acc  = accelDistance(v_peak, a_acc, j_acc);
    float d_dec  = decelDistance(v_peak, a_dec, j_dec);

    if (d_acc + d_dec > distance) {
        // Binary search for the peak velocity that fits the distance
        float lo = 0.0f, hi = v_max;
        for (int i = 0; i < 25; i++) {
            float mid = (lo + hi) * 0.5f;
            if (accelDistance(mid, a_acc, j_acc) + decelDistance(mid, a_dec, j_dec) > distance)
                hi = mid;
            else
                lo = mid;
        }
        v_peak = lo;
        d_acc  = accelDistance(v_peak, a_acc, j_acc);
        d_dec  = decelDistance(v_peak, a_dec, j_dec);
    }

    float cruise_dist = distance - d_acc - d_dec;
    if (cruise_dist < 0.0f) cruise_dist = 0.0f;

    // --- Phase parameters -----------------------------------------------
    // Acceleration side
    float v_jerk_acc = a_acc * a_acc / j_acc;
    float a_peak_acc, t_j_acc, t_const_acc;
    if (v_peak <= v_jerk_acc) {
        a_peak_acc  = sqrtf(v_peak * j_acc);
        t_j_acc     = a_peak_acc / j_acc;
        t_const_acc = 0.0f;
    } else {
        a_peak_acc  = a_acc;
        t_j_acc     = a_acc / j_acc;
        t_const_acc = (v_peak - v_jerk_acc) / a_acc;
    }

    // Deceleration side
    float v_jerk_dec = a_dec * a_dec / j_dec;
    float a_peak_dec, t_j_dec, t_const_dec;
    if (v_peak <= v_jerk_dec) {
        a_peak_dec  = sqrtf(v_peak * j_dec);
        t_j_dec     = a_peak_dec / j_dec;
        t_const_dec = 0.0f;
    } else {
        a_peak_dec  = a_dec;
        t_j_dec     = a_dec / j_dec;
        t_const_dec = (v_peak - v_jerk_dec) / a_dec;
    }

    float t_cruise = (v_peak > 1e-6f && cruise_dist > 0.0f) ? cruise_dist / v_peak : 0.0f;

    // --- Build phase table ----------------------------------------------
    //  Phase 1: jerk +j   (acceleration ramp-up)
    //  Phase 2: jerk  0   (constant acceleration)
    //  Phase 3: jerk -j   (acceleration ramp-down)
    //  Phase 4: jerk  0   (cruise)
    //  Phase 5: jerk -j   (deceleration ramp-up)
    //  Phase 6: jerk  0   (constant deceleration)
    //  Phase 7: jerk +j   (deceleration ramp-down)
    float durations[7] = { t_j_acc, t_const_acc, t_j_acc,
                           t_cruise,
                           t_j_dec, t_const_dec, t_j_dec };
    float jerks[7]     = { j_acc, 0.0f, -j_acc,
                           0.0f,
                          -j_dec, 0.0f,  j_dec };

    float t = 0.0f, v = 0.0f, a = 0.0f, s = 0.0f;

    for (int i = 0; i < 7; i++) {
        float dur = durations[i];
        if (dur < 1e-7f) continue;

        float jrk = jerks[i];
        m_phases[m_numPhases].t_start  = t;
        m_phases[m_numPhases].duration = dur;
        m_phases[m_numPhases].jerk     = jrk;
        m_phases[m_numPhases].a0       = a;
        m_phases[m_numPhases].v0       = v;
        m_phases[m_numPhases].s0       = s;

        // Advance kinematics to end of this phase
        s += v * dur + 0.5f * a * dur * dur + (1.0f / 6.0f) * jrk * dur * dur * dur;
        v += a * dur + 0.5f * jrk * dur * dur;
        a += jrk * dur;
        t += dur;
        m_numPhases++;
    }

    m_totalTime    = t;
    m_totalDistance = s;
}

// ---------------------------------------------------------------------------
float SCurveProfile::getSpeed(float t) const {
    if (m_numPhases == 0 || t <= 0.0f) return 0.0f;
    if (t >= m_totalTime) return 0.0f;

    // Find the active phase
    int idx = m_numPhases - 1;
    for (int k = 0; k < m_numPhases - 1; k++) {
        if (t < m_phases[k].t_start + m_phases[k].duration) {
            idx = k;
            break;
        }
    }

    float dt = t - m_phases[idx].t_start;
    float spd = m_phases[idx].v0 + m_phases[idx].a0 * dt + 0.5f * m_phases[idx].jerk * dt * dt;
    return (spd > 0.0f) ? spd : 0.0f;
}

// ---------------------------------------------------------------------------
float SCurveProfile::getDistance(float t) const {
    if (m_numPhases == 0 || t <= 0.0f) return 0.0f;
    if (t >= m_totalTime) return m_totalDistance;

    int idx = m_numPhases - 1;
    for (int k = 0; k < m_numPhases - 1; k++) {
        if (t < m_phases[k].t_start + m_phases[k].duration) {
            idx = k;
            break;
        }
    }

    float dt = t - m_phases[idx].t_start;
    return m_phases[idx].s0
         + m_phases[idx].v0 * dt
         + 0.5f * m_phases[idx].a0 * dt * dt
         + (1.0f / 6.0f) * m_phases[idx].jerk * dt * dt * dt;
}
