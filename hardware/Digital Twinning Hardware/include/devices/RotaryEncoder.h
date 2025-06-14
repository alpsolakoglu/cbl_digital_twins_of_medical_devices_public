#ifndef ROTARY_ENCODER_H
#define ROTARY_ENCODER_H

#include "interfaces/IActuator.h"
#include "utils/Angle.h"

#include <AS5600.h>
#include <stdint.h>
#include <string>

namespace DT
{
    class RotaryEncoder : public IActuator
    {
    private:
        uint8_t m_channel;
        std::string m_axisName;
        bool m_positiveClockwise;
        
        AS5600 m_sensor;
        Angle m_lastReadAngle;
        bool m_started = false;

    public:
        // Constructor to initialize the rotary encoder on a specific pin and channel
        RotaryEncoder(uint8_t channel, bool positiveClockwise);

        // Initialize the rotary encoder
        bool start() override;

        // Configure the rotary encoder
        bool configure();

        // Read sthe angle from the rotary encoder
        Angle readAngle();

        // Check if the sensor is connected
        bool isConnected();

        // Get the last read angle
        uint8_t getChannel() const;

        // Get default positive clockwise direction
        bool getPositiveClockwise() const;

    };
}

#endif