#include "controllers/MotorRotaryController.h"

namespace DT
{
    MotorRotaryController::MotorRotaryController(uint16_t pin,
                                                 uint8_t channel,
                                                 std::string axisName,
                                                 bool motorPositiveClockwise,
                                                 bool rotaryEncoderPositiveClockwise,
                                                 Angle initialAngle,
                                                 Potentiometer *potentiometer,
                                                 double beta,
                                                 uint16_t minPulseWidth,
                                                 uint16_t maxPulseWidth,
                                                 uint16_t commandTimeoutMs,
                                                 uint16_t configureWaitTimeMs,
                                                 uint16_t awaitingCommandDelayMs)
        : IController(axisName,
                      initialAngle,
                      configureWaitTimeMs,
                      commandTimeoutMs,
                      awaitingCommandDelayMs,
                      potentiometer,
                      beta),
          m_motorRotary(pin,
                        channel,
                        axisName,
                        motorPositiveClockwise,
                        rotaryEncoderPositiveClockwise,
                        initialAngle,
                        minPulseWidth,
                        maxPulseWidth) {};

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

    Angle MotorRotaryController::getCurrentAngle()
    {
        return m_motorRotary.getAngle(); // Return the current angle of the motor
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

            m_awaitingCommandStartTime = millis(); // Record the start time for the awaiting command state
            // m_state = ControllerState::AWAITING_COMMAND; // Change state to AWAITING_COMMAND after configuration
            m_state = ControllerState::HOLD_CONTROLLER_INPUT; // Change state to HOLD_CONTROLLER_INPUT after configuration
        }
    }

    void MotorRotaryController::onAwaitingCommand()
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
            m_motorRotary.stop();                        // Stop the motor if command timeout occurs
            m_state = ControllerState::AWAITING_COMMAND; // Change state to ERROR if command timeout occurs
            return;
        }

        if (Angle::isWithinDelta(m_motorRotary.getAngle(), m_commandAngle, Angle::fromDegrees(0.1)))
        {
            m_motorRotary.stop(); // Stop the motor if the command angle is reached within the delta
            Serial.println("Command angle reached: " + String(m_commandAngle.getInDegrees()) + " degrees");
            m_awaitingCommandStartTime = millis();       // Record the start time for the awaiting command state
            m_state = ControllerState::AWAITING_COMMAND; // Change state back to AWAITING_COMMAND after executing the command
            return;
        }

        setMotorToMoveTowardsAngle(m_commandAngle); // Move the motor towards the command angle
    }

    void MotorRotaryController::onHoldControllerInput()
    {
        Serial.println("RobotAngle" + String(m_axisName.c_str()) + ":" + m_motorRotary.getAngle().getInDegrees());

        updateControllerInputAngle();

        setMotorToMoveTowardsAngle(m_controllerInputAngle);
        return;
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
        bool commandAngleInRightHalf = 0 <= desiredAngle.getInDegrees() && desiredAngle.getInDegrees() < 180;

        if ((currentAngleInRightHalf && commandAngleInRightHalf) || (!currentAngleInRightHalf && !commandAngleInRightHalf))
        {
            commandAngleClockwiseFromCurrentAngle = desiredAngle.getInDegrees() > currentAngle.getInDegrees();
        }
        else
        {
            commandAngleClockwiseFromCurrentAngle = !currentAngleInRightHalf && commandAngleInRightHalf;
        }

        double minSpeed = 0.05;
        Angle delta = Angle::delta(currentAngle, desiredAngle);
        double deltaDegrees = delta.getInDegrees();
        // Serial.println("Delta angle: " + String(deltaDegrees) + " degrees");
        double speedModifier = deltaDegrees > 90.0 ? 1.0 : deltaDegrees / 90.0;
        // Serial.println("Speed modifier: " + String(speedModifier));
        // Serial.println("Current angle R: " + String(currentAngle.getInDegrees()) + " degrees");

        Serial.println("Drive Val:" + String(minSpeed + speedModifier * (1.0 - minSpeed)));

        m_motorRotary.drive(minSpeed + speedModifier * (1.0 - minSpeed), commandAngleClockwiseFromCurrentAngle); // Move the motor to the command angle
    }

    // // Temporary safety measure
    // if (m_state == ControllerState::AWAITING_COMMAND || m_state == ControllerState::EXECUTING_COMMAND || m_state == ControllerState::HOLD_CONTROLLER_INPUT)
    // {
    //     // Check if the motor has exceeded the testing limits
    //     Angle currentAngle = m_motorRotary.getAngle();
    //     double currentAngleDegrees = currentAngle.getInDegrees();
    //     if (70.0 < currentAngleDegrees && currentAngleDegrees < 290.0)
    //     {
    //         Serial.println("Motor exceeded testing limits, stopping motor and entering error state.");
    //         Serial.println("Current angle: " + String(currentAngleDegrees) + " degrees. Must be between 0 and 60 degrees or between 300 and 360 degrees.");
    //         m_motorRotary.stop(); // Stop the motor if it exceeds the testing limits

    //         m_state = ControllerState::ERROR;
    //         return;
    //     }
    // }
}
