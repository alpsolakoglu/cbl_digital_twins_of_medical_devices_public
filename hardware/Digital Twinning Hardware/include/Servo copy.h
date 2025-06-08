// #ifndef SERVOCONTROLLER_H
// #define SERVOCONTROLLER_H

// #include <IAxis.h>
// #include <TCAMultiplexer.h>

// #include <RotaryEncoder.h>
// #include <ESP32Servo.h>
// #include <stdint.h>
// #include <string>

// namespace DT {
//     class Servo : public IAxis {
//     private:
//         uint8_t m_pin; // Pin number for the servo
//         double m_virtualZeroInServoAngle; // Virtual zero with respect to the servo
//         bool m_positiveClockwise; // Direction of rotation: true for positive clockwise, false for negative clockwise
//         double m_servoAngleOffsetFromVirtualZero; // Offset of the servo angle from the virtual zero position
//         double m_rotaryEncoderAngleOffsetFromServoAngle; // Offset of the rotary encoder angle from the servo angle

//         ::Servo m_servo; // ESP32Servo instance for controlling the servo
//         double m_virtualAngleDegrees; // Virtual angle in degrees, which is the angle of the servo with respect to the virtual zero position
//     public:
//         // Constructor to initialize the servo on a specific pin
//         Servo(uint8_t pin, double virtualZeroInServoAngle,
//             bool positiveClockwise,
//             double servoAngleOffsetFromVirtualZero,
//             double rotaryEncoderAngleOffsetFromServoAngle);

//         // Initialize the servo
//         bool start() override;

//         // Move the servo to a specified position
//         bool setAngle(double angleDegrees) override;

//         // Get the position last written to the servo
//         double getAngle() const override;

//         // Get the name of the servo (for identification)
//         std::string getAxisName() const override;
//     };
// }

// #endif