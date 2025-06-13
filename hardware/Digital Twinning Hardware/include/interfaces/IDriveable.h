#ifndef I_DRIVEABLE_H
#define I_DRIVEABLE_H

#include "interfaces/IActuator.h"

#include <stdint.h>
#include <string>

namespace DT
{
    class IDriveable : public IActuator
    {
    public:
        // Move the axis to a specified position
        virtual bool drive(uint16_t pulseWidth) = 0;
    };
}

#endif