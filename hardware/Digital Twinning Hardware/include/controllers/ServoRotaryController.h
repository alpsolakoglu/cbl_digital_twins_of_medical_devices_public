#ifndef SERVO_ROTARY_CONTROLLER_H
#define SERVO_ROTARY_CONTROLLER_H

#include "controllers/states/ControllerStates.h"
#include "devices/ServoRotary.h"
#include "devices/Potentiometer.h"
#include "interfaces/IController.h"

#include <queue>

namespace DT
{
    class ServoRotaryController : public IController
    {
    private:
        ServoRotary m_servoRotary;

        void onStart();
        void onConfigure();
        void onAwaitingCommand();
        void onExecutingCommand();
        void onHoldControllerInput();
        void onError();

    public:
        ServoRotaryController(uint16_t pin,
                              uint8_t channel,
                              std::string axisName,
                              bool servoPositiveClockwise,
                              bool rotaryEncoderPositiveClockwise,
                              Angle initialAngle = Angle::fromDegrees(90),
                              Potentiometer *potentiometer = nullptr,
                              double beta = 0.9,
                              uint16_t minPulseWidth = 500,
                              uint16_t maxPulseWidth = 2500,
                              uint16_t configureWaitTimeMs = 3000,
                              uint16_t commandTimeoutMs = 5000,
                              uint16_t awaitingCommandDelayMs = 750);

        bool start();

        Angle getCurrentAngle();
    };

}

#endif