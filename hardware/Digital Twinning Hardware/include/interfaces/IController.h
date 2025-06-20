#ifndef I_CONTROLLER_H
#define I_CONTROLLER_H

#include "controllers/states/ControllerStates.h"
#include "devices/MotorRotary.h"
#include <devices/Potentiometer.h>

#include <queue>

namespace DT
{
    class IController
    {
    private:
        virtual void onStart() = 0;
        virtual void onConfigure() = 0;
        virtual void onAwaitingCommand() = 0;
        virtual void onExecutingCommand() = 0;
        virtual void onHoldControllerInput() = 0;
        virtual void onError() = 0;

    public:
        Potentiometer *m_potentiometer; // Potentiometer for reading angles
        double m_beta; // Beta value for EWMA

        // MotorRotary m_motorRotary;
        ControllerState m_state = ControllerState::START; // Reference to the current state of the axis controller

        bool m_configured = false;                // Flag to check if the motor is configured
        unsigned long m_configureStartTime;       // Start time for the configuration state
        unsigned long m_commandStartTime;         // Start time for the current command
        unsigned long m_awaitingCommandStartTime; // Start time for the awaiting command state

        std::queue<Angle> m_angleQueue; // Queue to hold angles for the axis
        Angle m_initialAngle;
        Angle m_commandAngle; // The angle to be set for the motor

        Angle m_controllerInputAngle; // The angle for the controller input mode

        uint16_t m_configureWaitTimeMs;    // Wait time for configuration in seconds
        uint16_t m_commandTimeoutMs;       // Timeout duration for the controller
        uint16_t m_awaitingCommandDelayMs; // Delay before awaiting command in seconds
        std::string m_axisName;            // Name of the axis for identification

        IController(
            std::string axisName,
            Angle initialAngle = Angle::fromDegrees(90),
            uint16_t configureWaitTimeMs = 3000,
            uint16_t commandTimeoutMs = 5000,
            uint16_t awaitingCommandDelayMs = 750,
            Potentiometer *potentiometer = nullptr,
            double beta = 0.9) : m_configureWaitTimeMs(configureWaitTimeMs),
                                 m_commandTimeoutMs(commandTimeoutMs),
                                 m_awaitingCommandDelayMs(awaitingCommandDelayMs),
                                 m_commandAngle(Angle::fromDegrees(0.0)),
                                 m_axisName(axisName),
                                 m_initialAngle(initialAngle),
                                 m_controllerInputAngle(Angle::fromDegrees(0.0)),
                                 m_potentiometer(potentiometer),
                                 m_beta(beta) {};

        virtual bool start() = 0;

        ControllerState getState() const
        {
            return m_state; // Return the current state of the controller
        }

        void addAngleToQueue(Angle angle)
        {
            m_angleQueue.push(angle); // Add the angle to the queue
            Serial.println("Angle added to queue: " + String(angle.getInDegrees()) + " degrees");
        }

        void updateControllerInputAngle()
        {
            double potentiometerAngleDegrees = m_potentiometer->getCurrentVirtual();

            double controllerInputAngleDegrees = m_controllerInputAngle.getInDegrees();
            double controllerInputAngleScaled = controllerInputAngleDegrees <= 180 ? controllerInputAngleDegrees : controllerInputAngleDegrees - 360;

            Serial.println("potangledegrees");
            Serial.println(potentiometerAngleDegrees);
            Serial.println("contscaled");
            Serial.println(controllerInputAngleScaled);
            m_controllerInputAngle = Angle::fromDegrees(m_beta * controllerInputAngleScaled + (1 - m_beta) * potentiometerAngleDegrees);
            // m_controllerInputAngle = Angle::fromDegrees(potentiometerAngleDegrees);
        }

        void update()
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
    };
}

#endif