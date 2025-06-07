#include <Angle.h>

// Private constructor
Angle::Angle(double radians) {
    setInRadians(radians);
}

// Named constructors
Angle Angle::fromRadians(double radians) { return Angle(radians); }

Angle Angle::fromDegrees(double degrees) { return Angle(degrees * Angle::PI / 180.0); }

// Accessors
double Angle::getInRadians() const { return m_radians; }

double Angle::getInDegrees() const { 
    return m_radians * 180.0 / Angle::PI; 
}

void Angle::setInRadians(double radians) {
    while (radians < 0) {
        radians += 2 * Angle::PI;
    }

    while (radians >= 2 * Angle::PI) {
        radians -= 2 * Angle::PI;
    }

    m_radians = radians;
}

void Angle::setInDegrees(double degrees) {
    double radians = degrees * Angle::PI / 180.0;
    setInRadians(radians);
}