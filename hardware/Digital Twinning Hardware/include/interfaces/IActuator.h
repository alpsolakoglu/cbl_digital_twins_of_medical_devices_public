#ifndef I_ACTUATOR_H
#define I_ACTUATOR_H

#include <string>

namespace DT
{
    class IActuator
    {
    public:
        // Start the actuator
        virtual bool start() = 0;
    };
}

#endif