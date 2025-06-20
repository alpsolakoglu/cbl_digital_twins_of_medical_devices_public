#ifndef MOTOR_ROTARY_CONTROLLER_H
#define MOTOR_ROTARY_CONTROLLER_H

#include "controllers/states/ControllerStates.h"
#include "devices/MotorRotary.h"
#include "devices/Potentiometer.h"
#include "interfaces/IController.h"

#include <queue>

namespace DT
{
    class MotorRotaryController : public IController
    {
    private:
        MotorRotary m_motorRotary;

        void onStart();
        void onConfigure();
        void onAwaitingCommand();
        void onExecutingCommand();
        void onHoldControllerInput();
        void onError();
    public:
        MotorRotaryController(uint16_t pin,
                              uint8_t channel,
                              std::string axisName,
                              bool motorPositiveClockwise,
                              bool rotaryEncoderPositiveClockwise,
                              Angle initialAngle = Angle::fromDegrees(90),
                              uint16_t minPulseWidth = 500,
                              uint16_t maxPulseWidth = 2500,
                              uint16_t configureWaitTimeMs = 3000,
                              uint16_t commandTimeoutMs = 5000,
                              uint16_t awaitingCommandDelayMs = 750,
                              Potentiometer *potentiometer = nullptr);

        bool start();

        Angle getCurrentAngle();

        void setMotorToMoveTowardsAngle(Angle desiredAngle);
    };

}

#endif