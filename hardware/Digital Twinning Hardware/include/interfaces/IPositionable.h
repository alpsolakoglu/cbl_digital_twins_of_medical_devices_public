#ifndef I_POSITIONABLE_AXIS_H
#define I_POSITIONABLE_AXIS_H

#include "interfaces/IActuator.h"
#include "utils/Angle.h"

#include <stdint.h>
#include <string>

namespace DT
{
    class IPositionable : public IActuator
    {
    public:
        // Move the axis to a specified position
        virtual bool setAngle(Angle angle) = 0;

        // Get the current position of the axis
        virtual Angle getAngle() = 0;
    };
}

#endif