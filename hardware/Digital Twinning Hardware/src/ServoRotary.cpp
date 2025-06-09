#include <ServoRotary.h>
#include <ESP32Servo.h>

namespace DT
{
    ServoRotary::ServoRotary(uint8_t pin, uint8_t channel, std::string axisName,
                             bool positiveClockwise,
                             Angle initialAngle,
                             uint16_t minPulseWidth, uint16_t maxPulseWidth)
        : Servo(pin, axisName, initialAngle, minPulseWidth, maxPulseWidth),
          m_rotaryEncoder(channel, axisName, positiveClockwise),
          m_positiveClockwise(positiveClockwise) {};

    bool ServoRotary::start() {
        Servo::start(); // Start the servo first
        Servo::setAngle(m_initialAngle); // Set the initial angle of the servo
        delay(1500); // Allow time for the servo to reach the initial position
        if (!m_rotaryEncoder.start()) {
            Serial.println("Failed to start rotary encoder on channel " + String(m_rotaryEncoder.getChannel()));
            return false; // Failed to start the rotary encoder
        }
        m_rotaryEncoder.configure(); // Configure the rotary encoder
        return true;
    }
    
    bool ServoRotary::setAngleWithRotary(Angle desiredAngle, uint8_t timeoutSeconds)
    {
        if (!m_started)
        {
            Serial.println("ServoRotary not started. Call start() first.");
            return false;
        }

        Angle angleWithServoOffset = desiredAngle; // Start with the input angle
        if (m_positiveClockwise) {
           angleWithServoOffset = Angle::fromDegrees(m_initialAngle.getInDegrees() - desiredAngle.getInDegrees());
        } else {
           angleWithServoOffset = Angle::fromDegrees(m_initialAngle.getInDegrees() + desiredAngle.getInDegrees());
        }
         
        // Set the servo angle
        if (!setAngle(angleWithServoOffset))
        {
            Serial.println("Failed to set servo angle: " + String(angleWithServoOffset.getInDegrees()) + " degrees");
            return false; // Failed to set the angle
        }

        // Wait for the rotary encoder to stabilize
        unsigned long startTime = millis();
        while (millis() - startTime < timeoutSeconds * 1000)
        {
            Angle currentAngle = m_rotaryEncoder.readAngle();
            m_virtualAngle = currentAngle; // Update the virtual angle
            if (Angle::isWithinDelta(currentAngle, desiredAngle, Angle::fromDegrees(5.0)))
            {
                return true; // Successfully set the angle and verified with rotary encoder
            }
            delay(100); // Polling delay
        }

        Serial.println("Timeout waiting for rotary encoder to match angle: " + String(desiredAngle.getInDegrees()) + " degrees");
        return false; // Timeout reached without matching angle
    }

    Angle ServoRotary::getAngleWithRotary() {
        return m_rotaryEncoder.readAngle();
    }
}
