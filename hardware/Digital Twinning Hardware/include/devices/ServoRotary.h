#ifndef ROTARY_SERVO_H
#define ROTARY_SERVO_H

#include "devices/Servo.h"
#include "devices/RotaryEncoder.h"

#include <stdint.h>
#include <string>

namespace DT
{
    class ServoRotary : public IPositionable
    {
    private:
        bool m_positiveClockwise; // Direction of rotation: true for positive clockwise, false for negative clockwise
        std::string m_axisName;   // Name of the axis for identification
        Angle m_initialAngle;     // Initial angle to set the servo to

        bool m_started = false;        // Flag to check if the servo and rotary encoder are initialized
        Servo m_servo;                 // ESP32Servo instance for controlling the servo
        RotaryEncoder m_rotaryEncoder; // Rotary encoder instance for reading angles
    public:
        // Constructor to initialize the servo on a specific pin
        ServoRotary(uint8_t pin, uint8_t channel, std::string axisName,
                    bool positiveClockwise,
                    Angle initialAngle = Angle::fromDegrees(90),
                    uint16_t minPulseWidth = 500, uint16_t maxPulseWidth = 2500);

        bool start() override;

        bool setToInitialAngle();

        bool setRotaryEncoderZero();

        bool setAngle(Angle angle) override;

        Angle getAngle() override;

        std::string getAxisName() const;
    };
}

#endif
