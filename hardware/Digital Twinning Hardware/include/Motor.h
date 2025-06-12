#ifndef MOTOR_H
#define MOTOR_H

#include <IDriveable.h>
#include <ESP32Servo.h>
#include <stdint.h>

namespace DT
{
    class Motor : private IDriveable
    {
    private:
        uint8_t m_pin; // Pin number for the motor
        std::string m_axisName; // Name of the motor axis for identification
        uint16_t m_minPulseWidth; // Minimum pulse width for the motor
        uint16_t m_maxPulseWidth; // Maximum pulse width for the motor

        ::Servo m_motor; // ESP32Servo instance for controlling the motor
        bool m_started = false; // Flag to check if the motor is initialized
    public:
        // Constructor to initialize the motor on a specific pin
        Motor(uint8_t pin, std::string axisName, uint16_t minPulseWidth = 544, uint16_t maxPulseWidth = 2400);

        bool start() override;

        bool drive(uint16_t pulseWidth) override;

        std::string getAxisName() const;

        uint16_t getMinPulseWidth() const;

        uint16_t getMaxPulseWidth() const;
    };
}

#endif