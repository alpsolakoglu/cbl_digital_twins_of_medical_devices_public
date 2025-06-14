#ifndef SERVO_ROTARY_CONTROLLER_H
#define SERVO_ROTARY_CONTROLLER_H

#include "controllers/states/ControllerStates.h"
#include "devices/ServoRotary.h"

#include <queue>

namespace DT
{
    class ServoRotaryController
    {
    private:
        ServoRotary m_servoRotary;
        ControllerState m_state = ControllerState::START; // Reference to the current state of the axis controller

        bool m_servoSetToInitialAngle = false;      // Flag to check if the servo is set to the initial angle
        unsigned long m_servoSetToInitialAngleTime; // Start time for setting the servo to the initial angle
        unsigned long m_commandStartTime;           // Start time for the current command
        unsigned long m_awaitingCommandStartTime;   // Start time for the awaiting command state

        std::queue<Angle> m_angleQueue; // Queue to hold angles for the axis
        Angle m_initialAngle;
        Angle m_commandAngle; // The angle to be set for the servo

        uint16_t m_configureWaitTimeMs;    // Wait time for configuration in seconds
        uint16_t m_commandTimeoutMs;       // Timeout duration for the controller
        uint16_t m_awaitingCommandDelayMs; // Delay before awaiting command in seconds
        std::string m_axisName;            // Name of the axis for identification

        // bool setAngle(Angle angle);

        // bool getAngle(Angle angle);

        bool configure();

        void onStart();
        void onConfigure();
        void onAwaitingCommand();
        void onExecutingCommand();
        void onError();

    public:
        ServoRotaryController(uint16_t pin,
                              uint8_t channel,
                              std::string axisName,
                              bool servoPositiveClockwise,
                              bool rotaryEncoderPositiveClockwise,
                              Angle initialAngle = Angle::fromDegrees(90),
                              uint16_t minPulseWidth = 500,
                              uint16_t maxPulseWidth = 2500,
                              uint16_t configureWaitTimeMs = 3000,
                              uint16_t commandTimeoutMs = 5000,
                              uint16_t awaitingCommandDelayMs = 750);

        bool start();

        void update();

        void addAngleToQueue(Angle angle);
    };

}

#endif