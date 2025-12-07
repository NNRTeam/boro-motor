#ifndef UTILS_H
#define UTILS_H


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
} // namespace utils



#endif // UTILS_H