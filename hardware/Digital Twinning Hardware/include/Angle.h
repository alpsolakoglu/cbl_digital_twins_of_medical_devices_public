#ifndef ANGLE_DEGREES_H
#define ANGLE_DEGREES_H

#include <stdint.h>

namespace DT
{
    class Angle
    {
    private:
        // Internal representation in radians
        double m_radians;
        explicit Angle(double radians);

    public:
        static constexpr double kPI = 3.14159265358979323846;

        // Named constructors
        static Angle fromRadians(double radians);
        static Angle fromDegrees(double degrees);

        // Accessors
        double getInRadians() const;

        double getInDegrees() const;

        void setInRadians(double radians);

        void setInDegrees(double degrees);

        static double degreesToRadians(double degrees);

        static double radiansToDegrees(double radians);

        static double normalizeRadians(double radians);

        static double normalizeDegrees(double degrees);

        static uint16_t map(Angle in, Angle inMin, Angle inMax, uint16_t outMin, uint16_t outMax);
    };
}

#endif // ANGLE_DEGREES_H