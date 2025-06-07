#ifndef IAXIS_H
#define IAXIS_H

#include <stdint.h>
#include <string>

class IAxis {
public:
    // Attach the axis
    virtual bool attach() = 0;

    // Move the axis to a specified position
    virtual bool setAngle(double angleDegrees) = 0;

    // Get the current position of the axis
    virtual double getAngle() const = 0;

    // Get the name of the axis
    virtual std::string getName() const = 0;
};

#endif