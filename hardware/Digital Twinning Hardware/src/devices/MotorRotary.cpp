#include "devices/MotorRotary.h"

namespace DT
{
    MotorRotary::MotorRotary(uint8_t pin, uint8_t channel, std::string axisName,
                             bool positiveClockwise,
                             Angle initialAngle,
                             uint16_t minPulseWidth, uint16_t maxPulseWidth)
        : m_motor(pin, minPulseWidth, maxPulseWidth),
          m_rotaryEncoder(channel, positiveClockwise),
          m_axisName(axisName),
          m_initialAngle(initialAngle) {};

    bool MotorRotary::start()
    {
        if (!m_motor.start())
        {
            Serial.println("Failed to start MotorRotary: Motor initialization failed.");
            return false; // Failed to start the motor
        }

        // Start the rotary encoder
        if (!m_rotaryEncoder.start())
        {
            Serial.println("Failed to start RotaryEncoder: Rotary encoder initialization failed.");
            return false; // Failed to start the rotary encoder
        }

        m_started = true; // Mark as started
        Serial.println("MotorRotary started successfully.");
        return true; // Successfully started both motor and rotary encoder
    }

    bool MotorRotary::setRotaryEncoderZero()
    {
        if (!m_started)
        {
            Serial.println("MotorRotary not started. Call start() first.");
            return false;
        }

        // Configure the rotary encoder with the motor's initial angle
        if (!m_rotaryEncoder.configure())
        {
            Serial.println("Failed to configure RotaryEncoder.");
            return false; // Failed to configure the rotary encoder
        }

        Serial.println("RotaryEncoder zero set successfully.");
        return true; // Successfully configured the rotary encoder
    }

    bool MotorRotary::move(double speed, bool clockwise)
    {
        if (!m_started)
        {
            Serial.println("MotorRotary not started. Call start() first.");
            return false; // Motor not started
        }

        return m_motor.move(speed, clockwise);
    }

    bool MotorRotary::stop()
    {
        if (!m_started)
        {
            Serial.println("MotorRotary not started. Call start() first.");
            return false; // Motor not started
        }

        return m_motor.stop();
    }

    Angle MotorRotary::getAngle()
    {
        // Get the current angle from the rotary encoder
        return m_rotaryEncoder.readAngle();
    }

    std::string MotorRotary::getAxisName() const
    {
        return m_axisName;
    }
}