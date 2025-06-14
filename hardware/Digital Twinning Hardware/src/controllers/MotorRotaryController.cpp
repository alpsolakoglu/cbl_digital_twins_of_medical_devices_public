#include "controllers/MotorRotaryController.h"

namespace DT
{
    MotorRotaryController::MotorRotaryController(uint16_t pin,
                                                 uint8_t channel,
                                                 std::string axisName,
                                                 bool motorPositiveClockwise,
                                                 bool rotaryEncoderPositiveClockwise,
                                                 Angle initialAngle,
                                                 uint16_t minPulseWidth,
                                                 uint16_t maxPulseWidth,
                                                 uint16_t commandTimeoutMs,
                                                 uint16_t configureWaitTimeMs,
                                                 uint16_t awaitingCommandDelayMs)
        : m_motorRotary(pin,
                        channel,
                        axisName,
                        motorPositiveClockwise,
                        rotaryEncoderPositiveClockwise,
                        initialAngle,
                        minPulseWidth,
                        maxPulseWidth),
          m_configureWaitTimeMs(configureWaitTimeMs),
          m_commandTimeoutMs(commandTimeoutMs),
          m_awaitingCommandDelayMs(awaitingCommandDelayMs),
          m_commandAngle(Angle::fromDegrees(0.0)),
          m_axisName(axisName),
          m_initialAngle(initialAngle) {};

    bool MotorRotaryController::start()
    {
        if (m_state != ControllerState::START)
        {
            Serial.println("MotorRotaryController is not in the START state, can only start in START state.");
            return false; // Cannot start if not in the correct state
        }

        if (!m_motorRotary.start())
        {
            Serial.println("Failed to start MotorRotary.");
            return false; // Failed to start the servo
        }
        m_state = ControllerState::CONFIGURE; // Change state to CONFIGURATION after starting
        Serial.println("MotorRotaryController started successfully.");
        return true; // Successfully started the servo
    }

    bool MotorRotaryController::configure()
    {
        if (m_state != ControllerState::CONFIGURE)
        {
            Serial.println("MotorRotaryController is not in the CONFIGURATION state, can only configure in CONFIGURATION state.");
            return false; // Cannot configure if not in the correct state
        }

        if (!m_motorRotary.setRotaryEncoderZero())
        {
            Serial.println("Failed to set RotaryEncoder zero.");
            return false; // Failed to set the rotary encoder zero
        }

        m_state = ControllerState::AWAITING_COMMAND; // Change state to AWAITING_COMMAND after configuration
        Serial.println("MotorRotaryController configured successfully.");
        return true; // Successfully configured the servo and rotary encoder
    }

    void MotorRotaryController::update()
    {
        // Temporary safety measure
        if (m_state == ControllerState::AWAITING_COMMAND || m_state == ControllerState::EXECUTING_COMMAND)
        {
            // Check if the motor has exceeded the testing limits
            Angle currentAngle = m_motorRotary.getAngle();
            double currentAngleDegrees = currentAngle.getInDegrees();
            if (60.0 < currentAngleDegrees && currentAngleDegrees < 300.0)
            {
                Serial.println("Motor exceeded testing limits, stopping motor and entering error state.");
                Serial.println("Current angle: " + String(currentAngleDegrees) + " degrees. Must be between 0 and 60 degrees or between 300 and 360 degrees.");
                m_motorRotary.stop(); // Stop the motor if it exceeds the testing limits

                m_state = ControllerState::ERROR;
                return;
            }
        }

        // Serial.println(m_state == ControllerState::START ? "MotorRotaryController is in START state." :
        //                m_state == ControllerState::CONFIGURE ? "MotorRotaryController is in CONFIGURE state." :
        //                m_state == ControllerState::AWAITING_COMMAND ? "MotorRotaryController is in AWAITING_COMMAND state." :
        //                m_state == ControllerState::EXECUTING_COMMAND ? "MotorRotaryController is in EXECUTING_COMMAND state." :
        //                m_state == ControllerState::ERROR ? "MotorRotaryController is in ERROR state." :
        //                "Unknown MotorRotaryController state.");
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
        case ControllerState::ERROR:
        {
            onError();
            break;
        }
        default:
        {
            Serial.println("Unknown MotorRotaryController state.");
            break;
        }
        }
    }

    void MotorRotaryController::addAngleToQueue(Angle angle)
    {
        m_angleQueue.push(angle); // Add the angle to the queue
        Serial.println("Angle added to queue: " + String(angle.getInDegrees()) + " degrees");
    }

    void MotorRotaryController::onStart()
    {
        return; // No action needed in START state
    }

    void MotorRotaryController::onConfigure()
    {
        if (!m_configured)
        {
            // Set the rotary encoder to zero after the wait time
            m_motorRotary.setRotaryEncoderZero();
            Serial.println("MotorRotaryController setting rotary encoder zero.");
            m_configured = true;             // Mark as configured
            m_configureStartTime = millis(); // Record the start time for the configuration state
        }

        if (millis() - m_configureStartTime >= m_configureWaitTimeMs)
        {
            // Configuration complete, change state to AWAITING_COMMAND
            Serial.println("MotorRotaryController configuration complete, moving to AWAITING_COMMAND state.");

            m_awaitingCommandStartTime = millis();       // Record the start time for the awaiting command state
            m_state = ControllerState::AWAITING_COMMAND; // Change state to AWAITING_COMMAND after configuration
        }
    }

    void MotorRotaryController::onAwaitingCommand()
    {
        if (millis() - m_awaitingCommandStartTime < m_awaitingCommandDelayMs)
        {
            // Delay before awaiting command to go easy on the servos and to let the system stabilize
            return;
        }

        if (!m_angleQueue.empty())
        {
            m_commandAngle = m_angleQueue.front(); // Get the next angle from the queue
            m_angleQueue.pop();                    // Remove it from the queue

            Serial.println("Command received with angle: " + String(m_commandAngle.getInDegrees()) + " degrees");

            if (m_commandAngle.getInDegrees() < 0 || m_commandAngle.getInDegrees() > 360)
            {
                Serial.println("Command angle out of bounds: " + String(m_commandAngle.getInDegrees()) + " degrees. Must be between 0 and 360 degrees.");
                m_state = ControllerState::ERROR; // Change state to ERROR if angle is out of bounds
                return;
            }

            m_commandStartTime = millis();                // Record the start time for the command
            m_state = ControllerState::EXECUTING_COMMAND; // Change state to EXECUTING_COMMAND after setting angle
        }
    }

    void MotorRotaryController::onExecutingCommand()
    {
        if (millis() - m_commandStartTime >= m_commandTimeoutMs)
        {
            Serial.println("Command timeout reached for MotorRotaryController.");
            m_state = ControllerState::AWAITING_COMMAND; // Change state to ERROR if command timeout occurs
            return;
        }

        if (angleReachedWithinDelta(m_commandAngle, Angle::fromDegrees(1.0)))
        {
            m_motorRotary.stop(); // Stop the motor if the command angle is reached within the delta
            Serial.println("Command angle reached: " + String(m_commandAngle.getInDegrees()) + " degrees");
            m_awaitingCommandStartTime = millis();       // Record the start time for the awaiting command state
            m_state = ControllerState::AWAITING_COMMAND; // Change state back to AWAITING_COMMAND after executing the command
            return;
        }

        setMotorToMoveTowardsAngle(m_commandAngle); // Move the motor towards the command angle


        // if (0 <= currentAngle.getInDegrees() && currentAngle.getInDegrees() < 180)
        // {
        //     // Command angle on right side of start pos
        //     if (0 <= m_commandAngle.getInDegrees() && m_commandAngle.getInDegrees() < 180)
        //     {
        //         commandAngleClockwiseFromCurrentAngle = m_commandAngle.getInDegrees() > currentAngle.getInDegrees();
        //     }
        //     // Command angle in left side of start pos
        //     else
        //     {
        //         // Rotate counter-clockwise
        //     }
        // }
        // // Current angle in left side of start pos
        // else
        // {
        //     // Command angle in right side of start pos
        //     if (0 <= m_commandAngle.getInDegrees() && m_commandAngle.getInDegrees() < 180)
        //     {
        //         // Rotate clockwise
        //     }
        //     // Command angle in left side of start pos
        //     else
        //     {
        //         commandAngleClockwiseFromCurrentAngle = m_commandAngle.getInDegrees() > currentAngle.getInDegrees();
        //     }
        // }
    }

    void MotorRotaryController::onError()
    {
        return; // Handle error state, can be implemented later
    }

    void MotorRotaryController::setMotorToMoveTowardsAngle(Angle desiredAngle)
    {
        Angle currentAngle = m_motorRotary.getAngle(); // Get the current angle of the motor

        bool commandAngleClockwiseFromCurrentAngle;

        bool currentAngleInRightHalf = 0 <= currentAngle.getInDegrees() && currentAngle.getInDegrees() < 180;
        bool commandAngleInRightHalf = 0 <= m_commandAngle.getInDegrees() && m_commandAngle.getInDegrees() < 180;

        if ((currentAngleInRightHalf && commandAngleInRightHalf) || (!currentAngleInRightHalf && !commandAngleInRightHalf))
        {
            commandAngleClockwiseFromCurrentAngle = m_commandAngle.getInDegrees() > currentAngle.getInDegrees();
        }
        else
        {
            commandAngleClockwiseFromCurrentAngle = !currentAngleInRightHalf && commandAngleInRightHalf;
        }

        double minSpeed = 0.05;
        Angle delta = Angle::delta(currentAngle, m_commandAngle);
        double deltaDegrees = delta.getInDegrees();
        double speedModifier = deltaDegrees > 90.0 ? 1.0 : deltaDegrees / 90.0;
        // Serial.println("Current angle R: " + String(currentAngle.getInDegrees()) + " degrees");

        m_motorRotary.drive(minSpeed + speedModifier * (1.0 - minSpeed), commandAngleClockwiseFromCurrentAngle); // Move the motor to the command angle
    }

    bool MotorRotaryController::angleReachedWithinDelta(Angle desiredAngle, Angle maxDelta)
    {
        // Check if the motor has reached the command angle
        Angle currentAngle = m_motorRotary.getAngle();
        Serial.println("Current angle: " + String(currentAngle.getInDegrees()) + " degrees, Desired angle: " + String(desiredAngle.getInDegrees()) + " degrees, Max delta: " + String(maxDelta.getInDegrees()) + " degrees");
        return Angle::isWithinDelta(currentAngle, desiredAngle, maxDelta);
    }
}
