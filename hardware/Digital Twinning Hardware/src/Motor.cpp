#include <Motor.h>

namespace DT
{
    // Constructor to initialize the servo on a specific pin
    Motor::Motor(uint8_t pin, std::string axisName, uint16_t minPulseWidth, uint16_t maxPulseWidth)
        : m_pin(pin), m_axisName(axisName), m_minPulseWidth(minPulseWidth), m_maxPulseWidth(maxPulseWidth) {}

    // Initialize the servo
    bool Motor::start()
    {
        if (m_started)
        {
            Serial.println("Servo already started on pin " + String(m_pin));
            return true; // Already started, no need to reinitialize
        }

        m_motor.attach(m_pin, m_minPulseWidth, m_maxPulseWidth); // Attach the servo to the specified pin with pulse width limits

        if (m_motor.attached() == false)
        {
            Serial.println("Failed to attach servo on pin " + String(m_pin));
            return false; // Failed to attach the servo
        }

        m_started = true; // Mark the servo as started
        return true;
    }

    bool Motor::drive(uint16_t pulseWidth)
    {
        if (!m_started)
        {
            Serial.println("Motor not started. Call start() first.");
            return false; // Motor not started
        }

        if (pulseWidth < m_minPulseWidth || pulseWidth > m_maxPulseWidth)
        {
            Serial.println("Invalid pulse width: " + String(pulseWidth) + ". Must be between " + String(m_minPulseWidth) + " and " + String(m_maxPulseWidth));
            return false; // Invalid pulse width
        }

        Serial.println("Setting motor pulse width to: " + String(pulseWidth));
        m_motor.writeMicroseconds(pulseWidth);
        return true;
    }

    std::string Motor::getAxisName() const
    {
        return m_axisName;
    }

    uint16_t Motor::getMinPulseWidth() const { return m_minPulseWidth; }

    uint16_t Motor::getMaxPulseWidth() const { return m_maxPulseWidth; }
}
