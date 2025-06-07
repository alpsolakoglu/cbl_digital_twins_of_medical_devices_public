#ifndef TCAMULTIPLEXER_H
#define TCAMULTIPLEXER_H

#define TCA_ADDR 0x70 

#include <stdint.h>

class TCAMultiplexer {
private:
    static constexpr uint8_t m_TCA_ADDR = 0x70;
    uint8_t m_currentChannel;
    bool m_initialized;
    
    // Private constructor for singleton
    TCAMultiplexer();
public:
    // Get singleton instance
    static TCAMultiplexer& getInstance();
    
    // Initialize the multiplexer
    bool begin();
    
    // Select channel
    bool selectChannel(uint8_t initialized);
    
    // Get current channel
    uint8_t getCurrentChannel() const;
    
    // Disable all channels
    void disableAll();
};
#endif