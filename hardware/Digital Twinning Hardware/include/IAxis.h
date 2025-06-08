#ifndef IAXIS_H
#define IAXIS_H

#include <Angle.h>

#include <stdint.h>
#include <string>

namespace DT
{
    class IAxis
    {
    public:
        // Attach the axis
        virtual bool start() = 0;

        // Move the axis to a specified position
        virtual bool setAngle(Angle angle) = 0;

        // Get the current position of the axis
        virtual Angle getAngle() = 0;

        // Get the name of the axis
        virtual std::string getAxisName() const = 0;
    };
}

#endif