#ifndef MOTOR_ROTARY_H
#define MOTOR_ROTARY_H

#include "devices/Motor.h"
#include "devices/RotaryEncoder.h"
#include "interfaces/IDriveable.h"

#include <stdint.h>
#include <string>

namespace DT
{
    class MotorRotary : public IDriveable
    {
    private:
        std::string m_axisName;   // Name of the axis for identification
        Angle m_initialAngle;     // Initial angle for reference

        bool m_started = false;        // Flag to check if the servo and rotary encoder are initialized
        Motor m_motor;                 // ESP32Servo instance for controlling the servo
        RotaryEncoder m_rotaryEncoder; // Rotary encoder instance for reading angles
    public:
        // Constructor to initialize the servo on a specific pin
        MotorRotary(uint8_t pin, uint8_t channel, std::string axisName,
                    bool positiveClockwise,
                    Angle initialAngle = Angle::fromDegrees(90),
                    uint16_t minPulseWidth = 500, uint16_t maxPulseWidth = 2500);

        bool start() override;

        bool setRotaryEncoderZero();

        bool move(double speed, bool clockwise);

        bool stop();

        Angle getAngle();

        std::string getAxisName() const;
    };
}

#endif