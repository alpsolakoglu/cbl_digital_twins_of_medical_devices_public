#include "devices/Motor.h"

namespace DT
{
    // Constructor to initialize the servo on a specific pin
    Motor::Motor(uint8_t pin, bool positiveClockwise, uint16_t minPulseWidth, uint16_t maxPulseWidth)
        : m_pin(pin),
          m_positiveClockwise(positiveClockwise),
          m_minPulseWidth(minPulseWidth),
          m_maxPulseWidth(maxPulseWidth),
          m_middlePulseWidth(minPulseWidth + (maxPulseWidth - minPulseWidth) / 2) {}

    // Initialize the servo
    bool Motor::start()
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

        // Serial.println("Setting motor pulse width to: " + String(pulseWidth));
        m_servo.writeMicroseconds(pulseWidth);
        return true;
    }

    bool Motor::move(double speed, bool clockwise)
    {
        if (!m_started)
        {
            Serial.println("Motor not started. Call start() first.");
            return false; // Motor not started
        }

        if (speed < 0.0 || speed > 1.0)
        {
            Serial.println("Invalid speed: " + String(speed) + ". Must be between 0.0 and 1.0");
            return false; // Invalid speed
        }

        uint16_t pulseWidth;
        uint16_t pulseWidthOffset = ((double)(m_maxPulseWidth - m_minPulseWidth)) / 2.0 * speed;

        if (m_positiveClockwise && clockwise || !m_positiveClockwise && !clockwise)
        {
            pulseWidth = m_middlePulseWidth + pulseWidthOffset;
            if (pulseWidth > m_maxPulseWidth)
            {
                pulseWidth = m_maxPulseWidth; // Cap to max pulse width (even though it should not go over with the validated speed)
            }
        }
        else if (m_positiveClockwise && !clockwise || !m_positiveClockwise && clockwise)
        {
            pulseWidth = m_middlePulseWidth - pulseWidthOffset;
            if (pulseWidth < m_minPulseWidth)
            {
                pulseWidth = m_minPulseWidth; // Cap to min pulse width (even though it should not go under with the validated speed)
            }
        }

        return drive(pulseWidth); // Set the motor
    }

    bool Motor::stop()
    {
        if (!m_started)
        {
            Serial.println("Motor not started. Call start() first.");
            return false; // Motor not started
        }

        Serial.println("Stopping motor on pin " + String(m_pin));
        drive(m_middlePulseWidth); // Set to no movement
        return true;
    }

    std::string Motor::getAxisName() const
    {
        return m_axisName;
    }

    uint16_t Motor::getMinPulseWidth() const { return m_minPulseWidth; }

    uint16_t Motor::getMaxPulseWidth() const { return m_maxPulseWidth; }
}
