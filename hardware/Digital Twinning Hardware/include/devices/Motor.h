#ifndef MOTOR_H
#define MOTOR_H

#include "interfaces/IDriveable.h"

#include <ESP32Servo.h>
#include <stdint.h>

namespace DT
{
    class Motor : private IDriveable
    {
    private:
        uint8_t m_pin;          // Pin number for the motor
        std::string m_axisName; // Name of the motor axis for identification
        bool m_positiveClockwise;
        uint16_t m_minPulseWidth;    // Minimum pulse width for the motor
        uint16_t m_maxPulseWidth;    // Maximum pulse width for the motor
        uint16_t m_middlePulseWidth; // Middle pulse width for the motor, used for stopping

        ::Servo m_servo;        // ESP32Servo instance for controlling the motor
        bool m_started = false; // Flag to check if the motor is initialized

        bool drive(uint16_t pulseWidth) override;

    public:
        // Constructor to initialize the motor on a specific pin
        Motor(uint8_t pin, bool positiveClockwise, uint16_t minPulseWidth = 500, uint16_t maxPulseWidth = 2500);

        bool start() override;

        bool move(double speed, bool clockwise);

        bool stop();

        std::string getAxisName() const;

        uint16_t getMinPulseWidth() const;

        uint16_t getMaxPulseWidth() const;
    };
}

#endif