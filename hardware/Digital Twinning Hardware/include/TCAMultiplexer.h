#ifndef TCAMULTIPLEXER_H
#define TCAMULTIPLEXER_H

#include <stdint.h>

namespace DT
{
    class TCAMultiplexer
    {
    private:
        static constexpr uint8_t m_TCA_ADDR = 0x70;
        uint8_t m_currentChannel;
        bool m_started;

        // Private constructor for singleton
        TCAMultiplexer();

    public:
        // Get singleton instance
        static TCAMultiplexer &getInstance();

        // Initialize the multiplexer
        bool start();

        // Select channel
        bool selectChannel(uint8_t started);

        // Get current channel
        uint8_t getCurrentChannel() const;

        // Disable all channels
        void disableAll();
    };
}

#endif