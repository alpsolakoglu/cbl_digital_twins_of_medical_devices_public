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
                                                 uint16_t awaitingCommandDelayMs)
        : m_servoRotary(pin,
                        channel,
                        axisName,
                        servoPositiveClockwise,
                        rotaryEncoderPositiveClockwise,
                        initialAngle,
                        minPulseWidth,
                        maxPulseWidth),
          m_configureWaitTimeMs(configureWaitTimeMs),
          m_commandTimeoutMs(commandTimeoutMs),
          m_awaitingCommandDelayMs(awaitingCommandDelayMs),
          m_commandAngle(Angle::fromDegrees(0.0)),
          m_axisName(axisName),
          m_initialAngle(initialAngle),
          m_controllerInputAngle(Angle::fromDegrees(0.0)) {};

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

    bool ServoRotaryController::configure()
    {
        if (m_state != ControllerState::CONFIGURE)
        {
            Serial.println("ServoRotaryController is not in the CONFIGURATION state, can only configure in CONFIGURATION state.");
            return false; // Cannot configure if not in the correct state
        }

        if (!m_servoRotary.setRotaryEncoderZero())
        {
            Serial.println("Failed to set RotaryEncoder zero.");
            return false; // Failed to set the rotary encoder zero
        }

        m_state = ControllerState::AWAITING_COMMAND; // Change state to AWAITING_COMMAND after configuration
        Serial.println("ServoRotaryController configured successfully.");
        return true; // Successfully configured the servo and rotary encoder
    }

    void ServoRotaryController::update()
    {
        // Serial.println(m_state == ControllerState::START ? "ServoRotaryController is in START state." :
        //                m_state == ControllerState::CONFIGURE ? "ServoRotaryController is in CONFIGURE state." :
        //                m_state == ControllerState::AWAITING_COMMAND ? "ServoRotaryController is in AWAITING_COMMAND state." :
        //                m_state == ControllerState::EXECUTING_COMMAND ? "ServoRotaryController is in EXECUTING_COMMAND state." :
        //                m_state == ControllerState::ERROR ? "ServoRotaryController is in ERROR state." :
        //                "Unknown ServoRotaryController state.");
        switch (m_state)
        {
        case ControllerState::START:
        {
            onStart();
            break; // No action needed in START state
        }
        case ControllerState::CONFIGURE:
        {
            onConfigure();
            break;
        }
        case ControllerState::AWAITING_COMMAND:
        {
            onAwaitingCommand();
            break;
        }
        case ControllerState::EXECUTING_COMMAND:
        {
            onExecutingCommand();
            break;
        }
        case ControllerState::HOLD_CONTROLLER_INPUT:
        {
            onHoldControllerInput();
            break;
        }
        case ControllerState::ERROR:
        {
            onError();
            break;
        }
        default:
        {
            Serial.println("Unknown ServoRotaryController state.");
            break;
        }
        }
    }

    void ServoRotaryController::addAngleToQueue(Angle angle)
    {
        m_angleQueue.push(angle); // Add the angle to the queue
        Serial.println("Angle added to queue: " + String(angle.getInDegrees()) + " degrees");
    }

    void ServoRotaryController::onStart()
    {
        return; // No action needed in START state
    }

    void ServoRotaryController::onConfigure()
    {
        if (!m_servoSetToInitialAngle)
        {
            if (!m_servoRotary.setToInitialAngle())
            {
                Serial.println("Failed to set ServoRotary to initial angle.");
                m_state = ControllerState::ERROR; // Change state to ERROR if setting angle fails
                return;
            }

            m_servoSetToInitialAngle = true;         // Mark that the servo is set to the initial angle
            m_servoSetToInitialAngleTime = millis(); // Record the time when the servo was set to the initial angle
        }

        // Check if the servo has been set to the initial angle for the required time
        if (millis() - m_servoSetToInitialAngleTime >= m_configureWaitTimeMs)
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

    void ServoRotaryController::setHoldControllerInputAngle(Angle angle)
    {
        if (Angle::isWithinDelta(angle, m_controllerInputAngle, Angle::fromDegrees(5.0)))
        {
            Serial.println("Controller input angle is within delta, no change needed.");
            return; // No change needed if the angle is within a small delta
        }

        m_controllerInputAngle = angle; // Set the angle for the controller input mode
    }

    Angle ServoRotaryController::getCurrentAngle()
    {
        return m_servoRotary.getAngle(); // Return the current angle of the motor
    }

    ControllerState ServoRotaryController::getState() const
    {
        return m_state; // Return the current state of the controller
    }
}
