
#include "devices/RotaryEncoder.h"
#include "devices/TCAMultiplexer.h"

#include <AS5600.h>

namespace DT
{
    RotaryEncoder::RotaryEncoder(uint8_t channel, bool positiveClockwise)
        : m_channel(channel), m_positiveClockwise(positiveClockwise), m_lastReadAngle(Angle::fromRadians(0)) {}

    bool RotaryEncoder::start()
    {
        if (m_started)
        {
            Serial.println("AS5600 already started on channel " + String(m_channel));
            return true; // Already started, no need to reinitialize
        }

        auto result = TCAMultiplexer::getInstance().withChannel(m_channel, [&]()
        {
            // Initialize the AS5600 sensor
            if (!m_sensor.begin())
            {
                Serial.println("AS5600 sensor not found on channel " + String(m_channel));
                return false;
            }

            m_started = true; // Mark the encoder as started
            Serial.println("AS5600 initialized on channel " + String(m_channel));
            return true;
        });

        if (result.has_value() && result.value() == true) {
            return true; // Successfully started the encoder
        } else {
            return false;
        }
    }

    bool RotaryEncoder::configure()
    {
        Serial.println("Configuring AS5600 on channel " + String(m_channel));
        Serial.println("Axis Name: " + String(m_axisName.c_str()));
        Serial.println("Current Angle: " + String(m_sensor.readAngle()));

        auto result = TCAMultiplexer::getInstance().withChannel(m_channel, [&]()
        {
            m_sensor.setZPosition(m_sensor.readAngle());               // Set the zero position to 0 degrees
            m_sensor.setDirection(m_positiveClockwise ? 0 : 1); // Set the direction towards the esp32 on the wooden plate to be positive (or clockwise from top for R axis)
            delay(1000);                                               // Allow time for the sensor to stabilize
            return true;                                               // Successfully configured the encoder
        });

        if (result.has_value() && result.value() == true) {
            Serial.println("AS5600 for axis " + String(m_axisName.c_str()) + " configured on channel " + String(m_channel));
            return true; // Successfully started the encoder
        } else {
            return false;
        }
    }

    Angle RotaryEncoder::readAngle()
    {
        auto result = TCAMultiplexer::getInstance().withChannel(m_channel, [&]()
        {
            // Read the angle from the AS5600 sensors
            int rawAngle = m_sensor.readAngle();                         // Raw 12-bit angle
            Angle angle = Angle::fromDegrees(rawAngle * 360.0 / 4096.0); // Convert to angle

            m_lastReadAngle = angle; // Store the last read angle
            return angle; 
        });


        if (result.has_value()) {
            return result.value();
        } else {
            throw std::runtime_error("Failed to read angle from AS5600 on channel " + std::to_string(m_channel));
        }
    }

    bool RotaryEncoder::isConnected()
    {
        // Check if the AS5600 sensor is connected
        return m_sensor.isConnected();
    }

    uint8_t RotaryEncoder::getChannel() const
    {
        return m_channel;
    }

    bool RotaryEncoder::getPositiveClockwise() const
    {
        return m_positiveClockwise;
    }
}