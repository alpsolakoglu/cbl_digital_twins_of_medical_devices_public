#ifndef MOTOR_H
#define MOTOR_H

#include <IAxis.h>

namespace DT
{
    class Motor : public IAxis
    {
    private:
        uint8_t m_pin; // Pin number for the motor
        std::string m_axisName; // Name of the motor axis for identification
        bool m_started = false; // Flag to check if the motor is initialized
    public:

    };
}

#endif