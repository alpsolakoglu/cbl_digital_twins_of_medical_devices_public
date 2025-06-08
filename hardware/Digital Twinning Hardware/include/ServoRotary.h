#ifndef ROTARY_SERVO_H
#define ROTARY_SERVO_H

#include <Servo.h>
#include <RotaryEncoder.h>

#include <stdint.h>
#include <string>

namespace DT
{
    class ServoRotary : public Servo
    {
    private:
        RotaryEncoder m_rotaryEncoder; // Rotary encoder instance for reading angles
        Angle m_virtualAngle;  // Virtual angle in degrees, which is the angle of the servo with respect to the virtual zero position

        bool m_positiveClockwise;                       // Direction of rotation: true for positive clockwise, false for negative clockwise
        Angle m_servoAngleOffsetFromVirtualZero;        // Offset of the servo angle from the virtual zero position
        Angle m_rotaryEncoderAngleOffsetFromServoAngle; // Offset of the rotary encoder angle from the servo angle
    public:
        // Constructor to initialize the servo on a specific pin
        ServoRotary(uint8_t pin, uint8_t channel, std::string axisName,
                    bool positiveClockwise,
                    Angle servoAngleOffsetFromVirtualZero,
                    Angle rotaryEncoderAngleOffsetFromServoAngle,
                    Angle initialAngle = Angle::fromDegrees(90),
                    uint16_t minPulseWidth = 544, uint16_t maxPulseWidth = 2400);

        bool start() override;
    };
}

#endif
