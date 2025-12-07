#ifndef CONFIG_H
#define CONFIG_H

namespace config {
    
    // HARDWARE CONFIGURATION

    int constexpr M_EN_PIN = 2;

    int constexpr M1_DIR_PIN = 4;
    int constexpr M1_STEP_PIN = 3;

    int constexpr M2_DIR_PIN = 6;
    int constexpr M2_STEP_PIN = 5;

    int constexpr OD1_CS_PIN = 10;
    int constexpr OD2_CS_PIN = 9;

    // MECHANICAL CONFIGURATION

    float constexpr M1_WHEEL_DIAMETER_MM = 60.0f; // Motor Wheel diameter in millimeters
    float constexpr M2_WHEEL_DIAMETER_MM = 60.0f; // Motor Wheel diameter in millimeters

    float constexpr OD1_WHEEL_DIAMETER_MM = 50.0f; // Odom Wheel diameter in millimeters
    float constexpr OD2_WHEEL_DIAMETER_MM = 50.0f; // Odom Wheel diameter in millimeters

    float constexpr M_WHEEL_BASE_MM = 150.0f;    // Distance between the two motors wheels in millimeters
    float constexpr OD_WHEEL_BASE_MM = 200.0f;   // Distance between the two odom wheels in millimeters

    // MOTION CONFIGURATION

    float constexpr MAX_LINEAR_VELOCITY_M_S = 0.4f;    // Maximum linear velocity in metter per second
    float constexpr MAX_ANGULAR_VELOCITY_RAD_S = 2.0f;    // Maximum angular velocity in radians per second

    float constexpr LINEAR_ACCELERATION_M_S2 = 0.4f;      // Linear acceleration in metter per second squared
    float constexpr ANGULAR_ACCELERATION_RAD_S2 = 2.0f;     // Angular acceleration in radians per second squared

    // SOFTWARE CONFIGURATION

    float constexpr CONTROL_LOOP_FREQUENCY_HZ = 25.0f; // Control loop frequency in Hertz
    float constexpr ODOMETRY_PUBLISH_FREQUENCY_HZ = 10.0f; // Odometry publish frequency in Hertz

    unsigned long int constexpr SERIAL_BAUDRATE = 115200; // Serial communication baudrate
    bool constexpr ENABLE_SERIAL_DEBUG = true; // Enable or disable serial debug messages

    int constexpr I2C_ADD = 0x10; // I2C address for the robot

} // namespace config

#endif // CONFIG_H