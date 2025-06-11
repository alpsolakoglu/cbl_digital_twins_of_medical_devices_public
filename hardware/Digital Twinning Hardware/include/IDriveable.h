#ifndef I_DRIVEABLE_H
#define I_DRIVEABLE_H

#include <IActuator.h>

#include <stdint.h>
#include <string>

namespace DT
{
    class IDriveable : public IActuator
    {
    public:
        // Move the axis to a specified position
        virtual bool drive(uint16_t pulseWidth) = 0;

        // Get the name of the driveable axis
        virtual std::string getAxisName() const = 0;
    };
}

#endif