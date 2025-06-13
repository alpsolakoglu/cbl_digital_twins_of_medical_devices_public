#ifndef SERVOCONTROLLER_H
#define SERVOCONTROLLER_H

#include "interfaces/IPositionable.h"
#include "utils/Angle.h"

#include <ESP32Servo.h>
#include <stdint.h>
#include <string>

namespace DT
{
    class Servo : public IPositionable
    {
    protected:
        uint8_t m_pin; // Pin number for the servo
        Angle m_initialAngle; // Initial angle to set the servo to
        uint16_t m_minPulseWidth; // Minimum pulse width for the servo
        uint16_t m_maxPulseWidth; // Maximum pulse width for the servo

        ::Servo m_servo;        // ESP32Servo instance for controlling the servo
        Angle m_lastSetAngle;   // Last angle set to the servo
        bool m_started = false; // Flag to check if the servo is initialized
    public:
        // Constructor to initialize the servo on a specific pin
        Servo(uint8_t pin, Angle initialAngle = Angle::fromDegrees(90), uint16_t minPulseWidth = 500, uint16_t maxPulseWidth = 2500);

        // Initialize the servo
        bool start() override;

        // Move the servo to a specified position
        bool setAngle(Angle angle);
    
        // Get the position last written to the servo
        Angle getAngle();
    };
}

#endif