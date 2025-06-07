#include <Angle.h>

// Private constructor
Angle::Angle(double radians) {
    setInRadians(radians);
}

// Named constructors
Angle Angle::fromRadians(double radians) { return Angle(radians); }

Angle Angle::fromDegrees(double degrees) { return Angle(degreesToRadians(degrees)); }

// Accessors
double Angle::getInRadians() const { return m_radians; }

double Angle::getInDegrees() const { 
    return radiansToDegrees(m_radians);
}

void Angle::setInRadians(double radians) {
    m_radians = normalizeRadians(radians);
}

void Angle::setInDegrees(double degrees) {
    setInRadians(degreesToRadians(normalizeDegrees(degrees)));
}

double Angle::degreesToRadians(double degrees) {
        return degrees * PI / 180.0;
    }

double Angle::radiansToDegrees(double radians) {
    return radians * 180.0 / PI;
}

double Angle::normalizeRadians(double radians) {
    while (radians < 0) {
        radians += 2 * PI;
    }
    while (radians >= 2 * PI) {
        radians -= 2 * PI;
    }
    return radians;
}
double Angle::normalizeDegrees(double degrees) {
    while (degrees < 0) {
        degrees += 360.0;
    }
    while (degrees >= 360.0) {
        degrees -= 360.0;
    }
    return degrees;
}