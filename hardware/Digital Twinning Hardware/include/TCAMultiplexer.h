#ifndef TCAMULTIPLEXER_H
#define TCAMULTIPLEXER_H

#include <stdint.h>
#include <mutex>

namespace DT
{
    class TCAMultiplexer
    {
    private:
        static constexpr uint8_t m_TCA_ADDR = 0x70;
        uint8_t m_currentChannel = 255;
        std::mutex m_lock;
        bool m_started = false;

        // Private constructor for singleton
        TCAMultiplexer();

        // Select channel
        bool selectChannel(uint8_t started);

        // Get current channel
        uint8_t getCurrentChannel() const;

    public:
        // Get singleton instance
        static TCAMultiplexer &getInstance();

        // Initialize the multiplexer
        bool start();

        // Ensure channel cannot be changed while in use
        template <typename F>
        auto withChannel(uint8_t channel, F &&fn) -> decltype(fn());

        // Disable all channels
        void disableAll();
    };
}

#endif