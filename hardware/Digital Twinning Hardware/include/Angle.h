#ifndef ANGLE_DEGREES_H
#define ANGLE_DEGREES_H

#include <cmath>

class Angle {
private:
    static constexpr double PI = 3.14159265358979323846;

    // Internal representation in radians
    double m_radians;
    explicit Angle(double radians);
public:
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
};

#endif // ANGLE_DEGREES_H