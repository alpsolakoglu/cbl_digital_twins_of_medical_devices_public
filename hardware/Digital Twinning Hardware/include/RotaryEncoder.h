#ifndef ROTARY_ENCODER_H
#define ROTARY_ENCODER_H

#include <Angle.h>
#include <AS5600.h>

#include <stdint.h>
#include <string>

namespace DT
{
    class RotaryEncoder
    {
    private:
        uint8_t m_channel;
        std::string m_axisName;
        bool m_defaultPositiveClockwise;
        
        AS5600 m_sensor;
        Angle m_lastReadAngle;
        bool m_started = false;

    public:
        // Constructor to initialize the rotary encoder on a specific pin and channel
        RotaryEncoder(uint8_t channel, std::string axisName, bool defaultPositiveClockwise);

        // Initialize the rotary encoder
        bool start();

        // Configure the rotary encoder
        bool configure();

        // Read sthe angle from the rotary encoder
        Angle readAngle();

        // Check if the sensor is connected
        bool isConnected();

        // Get the last read angle
        uint8_t getChannel() const;

        // Get the axis name for identification
        std::string getAxisName() const;

        // Get default positive clockwise direction
        bool getDefaultPositiveClockwise() const;

    };
}

#endif