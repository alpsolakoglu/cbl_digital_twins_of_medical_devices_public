#include "utils/Angle.h"

#include <stdexcept>

namespace DT
{
    // Private constructor
    Angle::Angle(double radians)
    {
        setInRadians(radians);
    }

    // Named constructors
    Angle Angle::fromRadians(double radians) { return Angle(radians); }

    Angle Angle::fromDegrees(double degrees) { return Angle(degreesToRadians(degrees)); }

    // Accessors
    double Angle::getInRadians() const { return m_radians; }

    double Angle::getInDegrees() const
    {
        return radiansToDegrees(m_radians);
    }

    void Angle::setInRadians(double radians)
    {
        m_radians = normalizeRadians(radians);
    }

    void Angle::setInDegrees(double degrees)
    {
        setInRadians(degreesToRadians(normalizeDegrees(degrees)));
    }

    double Angle::degreesToRadians(double degrees)
    {
        return degrees * c_pi / 180.0;
    }

    double Angle::radiansToDegrees(double radians)
    {
        return radians * 180.0 / c_pi;
    }

    double Angle::normalizeRadians(double radians)
    {
        while (radians < 0)
        {
            radians += 2 * c_pi;
        }
        while (radians >= 2 * c_pi)
        {
            radians -= 2 * c_pi;
        }
        return radians;
    }
    double Angle::normalizeDegrees(double degrees)
    {
        while (degrees < 0)
        {
            degrees += 360.0;
        }
        while (degrees >= 360.0)
        {
            degrees -= 360.0;
        }
        return degrees;
    }

    uint16_t Angle::map(Angle in, Angle inMin, Angle inMax, uint16_t outMin, uint16_t outMax)
    {
        double inRad = in.getInRadians();
        double inMinRad = inMin.getInRadians();
        double inMaxRad = inMax.getInRadians();

        double domainSize = inMaxRad - inMinRad;
        uint16_t rangeSize = outMax - outMin;
        uint16_t midpoint = outMin + rangeSize / 2;

        if (inMinRad < 0 || inMaxRad < 0 || outMin < 0 || outMax < 0)
        {
            throw std::runtime_error("Invalid input: Angles and output range must be non-negative.");
        }

        if (inMinRad == inMaxRad)
        {
            throw std::runtime_error("Invalid input range: inMinRad and inMaxRad cannot be equal.");
        }

        if (inMinRad > inMaxRad)
        {
            throw std::runtime_error("Invalid input range: inMinRad cannot be greater than inMaxRad.");
        }

        if (inRad < inMinRad || inRad > inMaxRad)
        {
            throw std::runtime_error("Input angle out of range: inRad must be between inMinRad and inMaxRad.");
        }

        if (outMin > outMax)
        {
            throw std::runtime_error("Invalid output range: outMin cannot be greater than outMax.");
        }

        uint16_t mappedValue = outMin + ((inRad - inMinRad) / domainSize) * rangeSize;
        return mappedValue;
    }

    bool Angle::isWithinDelta(Angle angle1, Angle angle2, Angle delta)
    {
        double deltaDegrees = Angle::fromDegrees(angle2.getInDegrees() - angle1.getInDegrees()).getInDegrees();
        double deltaDegreesShortestArc;

        if (deltaDegrees < 360.0 - deltaDegrees)
        {
            deltaDegreesShortestArc = deltaDegrees;
        }
        else
        {
            deltaDegreesShortestArc = 360.0 - deltaDegrees;
        }

        return deltaDegreesShortestArc <= delta.getInDegrees();
    }
}