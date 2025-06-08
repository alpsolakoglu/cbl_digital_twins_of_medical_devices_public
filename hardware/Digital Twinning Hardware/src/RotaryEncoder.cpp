
#include <RotaryEncoder.h>
#include <TCAMultiplexer.h>

namespace DT {
    RotaryEncoder::RotaryEncoder(uint8_t pin, uint8_t channel, std::string axisName)
        : m_pin(pin), m_channel(channel), m_axisName(axisName), m_lastReadAngle(Angle::fromRadians(0)) {}

    bool RotaryEncoder::start() {
        // Select the TCA channel for this encoder
        if (!TCAMultiplexer::getInstance().selectChannel(m_channel)) {
            Serial.println("Failed to select TCA channel " + String(m_channel) + ", for RotaryEncoder on pin " + String(m_pin));
            return false;
        }

        // Initialize the AS5600 sensor
        if (!m_sensor.begin()) {
            Serial.println("AS5600 sensor not found on pin " + String(m_pin));
            return false;
        }

        Serial.println("RotaryEncoder initialized on pin " + String(m_pin) + ", channel " + String(m_channel));
        return true;
    }

    Angle RotaryEncoder::readAngle() {
        TCAMultiplexer::getInstance().selectChannel(m_channel); // Ensure the correct channel is selected

        // Read the angle from the AS5600 sensor
        int rawAngle = m_sensor.readAngle(); // Raw 12-bit angle
        Angle angle = Angle::fromDegrees(rawAngle * 360.0 / 4096.0); // Convert to angle

        m_lastReadAngle = angle; // Store the last read angle
        return angle;
    }

    bool RotaryEncoder::isConnected() {
        // Check if the AS5600 sensor is connected
        return m_sensor.isConnected();
    }

    std::string RotaryEncoder::getAxisName() const {
        // Return the axis name for identification
        return m_axisName;
    }
}