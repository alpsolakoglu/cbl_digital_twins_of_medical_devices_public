#ifndef SERVO_H
#define SERVO_H

#include <IAxis.h>
#include <TCAMultiplexer.h>

#include <ESP32Servo.h>
#include <stdint.h>
#include <string>

class Servo : public IAxis {
private:
    uint8_t m_pin; // Pin number for the servo
    uint8_t m_channel; // Channel number for the servo
    double m_angleDegrees; // Current position of the servo in degree
    ::Servo m_servo; // ESP32Servo instance
public:
    // Constructor to initialize the servo on a specific pin
    Servo(uint8_t pin, uint8_t channel, bool positiveClockwise, double ServoAngleOffsetFromVirtualZero, double RotaryEncoderAngleOffsetFromServoAngle);

    // Initialize the servo
    bool attach() override;

    // Move the servo to a specified position
    bool setAngle(double angleDegrees) override;

    // Get the position last written to the servo
    double getAngle() const override;

    // Get the name of the servo (for identification)
    std::string getName() const override;
};

#endif