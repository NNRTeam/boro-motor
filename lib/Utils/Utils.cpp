#include "Utils.h"

namespace utils {
    float normalizeAngle(float angle) {
        while (angle > 3.14159265f) {
            angle -= 2.0f * 3.14159265f;
        }
        while (angle < -3.14159265f) {
            angle += 2.0f * 3.14159265f;
        }
        return angle;
    }

    float getMin(float a, float b) {
        return (a < b) ? a : b;
    }

    float getMax(float a, float b) {
        return (a > b) ? a : b;
    }

    int sign(float value) {
        if (value > 0.0f) return 1;
        if (value < 0.0f) return -1;
        return 0;
    }
}