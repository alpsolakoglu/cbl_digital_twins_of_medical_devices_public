#include <Servo.h>
#include <ESP32Servo.h>


// Constructor to initialize the servo on a specific pin
Servo::Servo(uint8_t pin, uint8_t channel, bool positiveClockwise, double servoAngleOffsetFromVirtualZero, double rotaryEncoderAngleOffsetFromServoAngle)
    : m_pin(pin), m_channel(channel), m_angleDegrees(0.0) {

    
}

// Initialize the servo
bool Servo::attach() {
    m_servo.attach(m_pin);
}

// Move the servo to a specified position
bool Servo::setAngle(double angleDegrees) {
    m_servo.read();
    m_servo.write(angleDegrees);
}

// Get the current position of the servo
double Servo::getAngle() const {

}

// Get the name of the servo (for identification)
std::string Servo::getName() const {
    
}
