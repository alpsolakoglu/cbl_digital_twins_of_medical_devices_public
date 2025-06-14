#include "devices/Servo.h"

#include <ESP32Servo.h>

namespace DT
{
    // Constructor to initialize the servo on a specific pin
    Servo::Servo(uint8_t pin, bool positiveClockwise, Angle initialAngle, uint16_t minPulseWidth, uint16_t maxPulseWidth)
        : m_pin(pin),
          m_positiveClockwise(positiveClockwise),
          m_lastSetAngle(initialAngle),
          m_initialAngle(initialAngle),
          m_minPulseWidth(minPulseWidth),
          m_maxPulseWidth(maxPulseWidth) {}

    // Initialize the servo
    bool Servo::start()
    {
        if (m_started)
        {
            Serial.println("Servo already started on pin " + String(m_pin));
            return true; // Already started, no need to reinitialize
        }

        m_servo.attach(m_pin, m_minPulseWidth, m_maxPulseWidth); // Attach the servo to the specified pin with pulse width limits

        if (m_servo.attached() == false)
        {
            Serial.println("Failed to attach servo on pin " + String(m_pin));
            return false; // Failed to attach the servo
        }

        m_started = true; // Mark the servo as started
        return true;
    }

    // Move the servo to a specified position
    bool Servo::setAngle(Angle angle)
    {
        if (angle.getInDegrees() < 0 || angle.getInDegrees() > 180)
        {
            Serial.println("Angle out of range: " + String(angle.getInDegrees()) + " degrees. Servo angle must be between 0 and 180 degrees.");
            return false; // Angle is out of range
        }

        int pulseWidth = Angle::map(angle, Angle::fromDegrees(0), Angle::fromDegrees(180), m_minPulseWidth, m_maxPulseWidth);
        m_servo.writeMicroseconds(pulseWidth);
        return true;
    }

    // Get the current position of the servo
    Angle Servo::getAngle()
    {
        if (!m_started)
        {
            throw std::runtime_error("Servo not started. Call start() first.");
        }

        return Angle::fromDegrees(m_servo.read());
    }

    bool Servo::getPositiveClockwise() const
    {
        return m_positiveClockwise;
    }
}
