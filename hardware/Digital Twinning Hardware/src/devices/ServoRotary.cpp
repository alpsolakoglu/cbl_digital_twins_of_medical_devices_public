#include "devices/ServoRotary.h"

namespace DT
{
    ServoRotary::ServoRotary(uint8_t pin, uint8_t channel, std::string axisName,
                             bool servoPositiveClockwise,
                             bool rotaryEncoderPositiveClockwise,
                             Angle initialAngle,
                             uint16_t minPulseWidth, uint16_t maxPulseWidth)
        : m_servo(pin, servoPositiveClockwise, initialAngle, minPulseWidth, maxPulseWidth),
          m_rotaryEncoder(channel, rotaryEncoderPositiveClockwise),
          m_axisName(axisName),
          m_initialAngle(initialAngle) {};

    bool ServoRotary::start()
    {
        if (!m_servo.start())
        {
            Serial.println("Failed to start ServoRotary: Servo initialization failed.");
            return false; // Failed to start the servo
        }

        // Start the rotary encoder
        if (!m_rotaryEncoder.start())
        {
            Serial.println("Failed to start RotaryEncoder: Rotary encoder initialization failed.");
            return false; // Failed to start the rotary encoder
        }

        m_started = true; // Mark as started
        Serial.println("ServoRotary started successfully.");
        return true; // Successfully started both servo and rotary encoder
    }

    bool ServoRotary::setToInitialAngle() {
        return m_servo.setAngle(m_initialAngle); // Set the servo to the initial angle
    }

    bool ServoRotary::setRotaryEncoderZero()
    {
        if (!m_started)
        {
            Serial.println("ServoRotary not started. Call start() first.");
            return false;
        }

        // Configure the rotary encoder with the servo's initial angle
        if (!m_rotaryEncoder.configure())
        {
            Serial.println("Failed to configure RotaryEncoder.");
            return false; // Failed to configure the rotary encoder
        }

        Serial.println("RotaryEncoder zero set successfully.");
        return true; // Successfully configured the rotary encoder
    }

    bool ServoRotary::setAngle(Angle desiredAngle)
    {
        if (!m_started)
        {
            Serial.println("ServoRotary not started. Call start() first.");
            return false;
        }

        Angle angleWithServoOffset = desiredAngle; // Start with the input angle
        if (m_servo.getPositiveClockwise())
        {
            angleWithServoOffset = Angle::fromDegrees(m_initialAngle.getInDegrees() + desiredAngle.getInDegrees());
        }
        else
        {
            angleWithServoOffset = Angle::fromDegrees(m_initialAngle.getInDegrees() - desiredAngle.getInDegrees());
        }

        // Set the servo angle
        if (!m_servo.setAngle(angleWithServoOffset))
        {
            Serial.println("Failed to set servo angle: " + String(angleWithServoOffset.getInDegrees()) + " degrees");
            return false; // Failed to set the angle
        }

        // // Wait for the rotary encoder to stabilize
        // unsigned long startTime = millis();
        // while (millis() - startTime < timeoutSeconds * 1000)
        // {
        //     Angle currentAngle = m_rotaryEncoder.readAngle();
        //     m_virtualAngle = currentAngle; // Update the virtual angle
        //     if (Angle::isWithinDelta(currentAngle, desiredAngle, Angle::fromDegrees(5.0)))
        //     {
        //         Serial.println("Rotary encoder " + String(m_axisName.c_str()) + " matched desired angle (within epsilon)");
        //         Serial.println("Desired: " + String(desiredAngle.getInDegrees()) + " degrees | Actual: " + String(currentAngle.getInDegrees()) + " degrees");
        //         return true; // Successfully set the angle and verified with rotary encoder
        //     }
        //     delay(100); // Polling delay
        // }

        // Serial.println("Timeout waiting for rotary encoder to match angle: " + String(desiredAngle.getInDegrees()) + " degrees");
        // Serial.println("ServoRotary " + String(m_axisName.c_str()) + " set to absolute angle: " + String(angleWithServoOffset.getInDegrees()) + " degrees");
        return true; // Timeout reached without matching angle
    }

    

    Angle ServoRotary::getAngle()
    {
        return m_rotaryEncoder.readAngle();
    }
}
