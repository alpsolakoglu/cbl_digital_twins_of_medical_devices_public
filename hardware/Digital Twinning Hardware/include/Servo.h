#ifndef SERVOCONTROLLER_H
#define SERVOCONTROLLER_H

#include <IAxis.h>
#include <Angle.h>

#include <ESP32Servo.h>
#include <stdint.h>
#include <string>

namespace DT
{
    class Servo : public IAxis
    {
    private:
        uint8_t m_pin; // Pin number for the servo
        std::string m_axisName; // Name of the servo axis for identification
        Angle m_initialAngle; // Initial angle to set the servo to
        uint16_t m_minPulseWidth; // Minimum pulse width for the servo
        uint16_t m_maxPulseWidth; // Maximum pulse width for the servo

        ::Servo m_servo;        // ESP32Servo instance for controlling the servo
        Angle m_lastSetAngle;   // Last angle set to the servo
        bool m_started = false; // Flag to check if the servo is initialized
    public:
        // Constructor to initialize the servo on a specific pin
        Servo(uint8_t pin, std::string axisName, Angle initialAngle, uint16_t minPulseWidth, uint16_t maxPulseWidth);

        // Initialize the servo
        bool start() override;

        // Move the servo to a specified position
        bool setAngle(Angle angle) override;
    
        // Get the position last written to the servo
        Angle getAngle() override;

        // Get the name of the servo (for identification)
        std::string getAxisName() const override;
    };
}

#endif