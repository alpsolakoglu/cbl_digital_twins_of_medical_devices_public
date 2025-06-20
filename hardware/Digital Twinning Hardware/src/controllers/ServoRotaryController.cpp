#include "controllers/ServoRotaryController.h"
#include "devices/ServoRotary.h"

namespace DT
{
    ServoRotaryController::ServoRotaryController(uint16_t pin,
                                                 uint8_t channel,
                                                 std::string axisName,
                                                 bool servoPositiveClockwise,
                                                 bool rotaryEncoderPositiveClockwise,
                                                 Angle initialAngle,
                                                 uint16_t minPulseWidth,
                                                 uint16_t maxPulseWidth,
                                                 uint16_t configureWaitTimeMs,
                                                 uint16_t commandTimeoutMs,
                                                 uint16_t awaitingCommandDelayMs,
                                                 Potentiometer *potentiometer)
        : IController(axisName,
                      initialAngle,
                      configureWaitTimeMs,
                      commandTimeoutMs,
                      awaitingCommandDelayMs,
                      potentiometer),
          m_servoRotary(pin,
                        channel,
                        axisName,
                        servoPositiveClockwise,
                        rotaryEncoderPositiveClockwise,
                        initialAngle,
                        minPulseWidth,
                        maxPulseWidth) {};

    bool ServoRotaryController::start()
    {
        if (m_state != ControllerState::START)
        {
            Serial.println("ServoRotaryController is not in the START state, can only start in START state.");
            return false; // Cannot start if not in the correct state
        }

        if (!m_servoRotary.start())
        {
            Serial.println("Failed to start ServoRotary.");
            return false; // Failed to start the servo
        }
        m_state = ControllerState::CONFIGURE; // Change state to CONFIGURATION after starting
        Serial.println("ServoRotaryController started successfully.");
        return true; // Successfully started the servo
    }

    Angle ServoRotaryController::getCurrentAngle()
    {
        return m_servoRotary.getAngle(); // Return the current angle of the motor
    }

    void ServoRotaryController::onStart()
    {
        return; // No action needed in START state
    }

    void ServoRotaryController::onConfigure()
    {
        if (!m_configured)
        {
            if (!m_servoRotary.setToInitialAngle())
            {
                Serial.println("Failed to set ServoRotary to initial angle.");
                m_state = ControllerState::ERROR; // Change state to ERROR if setting angle fails
                return;
            }

            m_configured = true;         // Mark that the servo is set to the initial angle
            m_configureStartTime = millis(); // Record the time when the servo was set to the initial angle
        }

        // Check if the servo has been set to the initial angle for the required time
        if (millis() - m_configureStartTime >= m_configureWaitTimeMs)
        {
            // Set the rotary encoder to zero after the wait time
            m_servoRotary.setRotaryEncoderZero();

            m_awaitingCommandStartTime = millis(); // Record the start time for the awaiting command state
            m_state = ControllerState::HOLD_CONTROLLER_INPUT;
            // m_state = ControllerState::AWAITING_COMMAND; // Change state to AWAITING_COMMAND after configuration
            Serial.println("ServoRotaryController configured successfully.");
        }
    }

    void ServoRotaryController::onAwaitingCommand()
    {
        // if (millis() - m_awaitingCommandStartTime < m_awaitingCommandDelayMs)
        // {
        //     // Delay before awaiting command to go easy on the servos and to let the system stabilize
        //     return;
        // }

        if (!m_angleQueue.empty())
        {
            m_commandAngle = m_angleQueue.front(); // Get the next angle from the queue
            m_angleQueue.pop();                    // Remove it from the queue

            Serial.println("Command received with angle: " + String(m_commandAngle.getInDegrees()) + " degrees");

            if (!m_servoRotary.setAngle(m_commandAngle))
            {
                Serial.println("Failed to set ServoRotary to command angle.");
                m_state = ControllerState::ERROR; // Change state to ERROR if setting angle fails
            }

            m_commandStartTime = millis();                // Record the start time for the command
            m_state = ControllerState::EXECUTING_COMMAND; // Change state to EXECUTING_COMMAND after setting angle
        }
    }

    void ServoRotaryController::onExecutingCommand()
    {
        if (millis() - m_commandStartTime >= m_commandTimeoutMs)
        {
            Serial.println("Command timeout reached for ServoRotaryController.");

            m_awaitingCommandStartTime = millis();       // Record the start time for the awaiting command state
            m_state = ControllerState::AWAITING_COMMAND; // Change state to ERROR if command timeout occurs
            return;
        }

        Angle maxDelta = Angle::fromDegrees(2.0);
        // Check if the servo has reached the command angle
        Angle currentAngle = m_servoRotary.getAngle();
        if (Angle::isWithinDelta(currentAngle, m_commandAngle, maxDelta))
        {
            Serial.println("ServoRotary for " + String(m_axisName.c_str()) + " matched desired angle (within " + String(maxDelta.getInDegrees()) + " degrees)");
            Serial.println("Desired: " + String(m_commandAngle.getInDegrees()) + " degrees | Actual: " + String(currentAngle.getInDegrees()) + " degrees");

            m_awaitingCommandStartTime = millis();       // Record the start time for the awaiting command state
            m_state = ControllerState::AWAITING_COMMAND; // Change state back to AWAITING_COMMAND after executing the command
            return;
        }
    }

    void ServoRotaryController::onHoldControllerInput()
    {
        m_servoRotary.setAngle(m_controllerInputAngle); // Keep the servo at the command angle
        return;
    }

    void ServoRotaryController::onError()
    {
        return; // Handle error state, can be implemented later
    }
}
