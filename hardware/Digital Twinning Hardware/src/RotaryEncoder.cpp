
#include <RotaryEncoder.h>
#include <TCAMultiplexer.h>

namespace DT
{
    RotaryEncoder::RotaryEncoder(uint8_t channel, std::string axisName)
        : m_channel(channel), m_axisName(axisName), m_lastReadAngle(Angle::fromRadians(0)) {}

    bool RotaryEncoder::start()
    {
        if (m_started)
        {
            Serial.println("AS5600 already started on channel " + String(m_channel));
            return true; // Already started, no need to reinitialize
        }

        // Ensure the multiplexer is started
        if (!TCAMultiplexer::getInstance().start())
        {
            Serial.println("Failed to start TCA Multiplexer");
            return false; // Failed to start the multiplexer
        }

        // Select the TCA channel for this encoder
        if (!TCAMultiplexer::getInstance().selectChannel(m_channel))
        {
            Serial.println("Failed to select TCA channel " + String(m_channel) + ", for AS5600 on axis " + String(m_axisName.c_str()));
            return false;
        }

        // Initialize the AS5600 sensor
        if (!m_sensor.begin())
        {
            Serial.println("AS5600 sensor not found on channel " + String(m_channel));
            return false;
        }

        m_started = true; // Mark the encoder as started
        Serial.println("AS5600 initialized on channel " + String(m_channel));
        return true;
    }

    Angle RotaryEncoder::readAngle()
    {
        TCAMultiplexer::getInstance().selectChannel(m_channel); // Ensure the correct channel is selected

        // Read the angle from the AS5600 sensor
        int rawAngle = m_sensor.readAngle();                         // Raw 12-bit angle
        Angle angle = Angle::fromDegrees(rawAngle * 360.0 / 4096.0); // Convert to angle

        m_lastReadAngle = angle; // Store the last read angle
        return angle;
    }

    bool RotaryEncoder::isConnected()
    {
        // Check if the AS5600 sensor is connected
        return m_sensor.isConnected();
    }

    std::string RotaryEncoder::getAxisName() const
    {
        // Return the axis name for identification
        return m_axisName;
    }
}