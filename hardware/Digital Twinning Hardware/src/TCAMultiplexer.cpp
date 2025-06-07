#include <TCAMultiplexer.h>

#include <Wire.h>
#include <AS5600.h>
#include <ESP32Servo.h>

// Private constructor for singleton
TCAMultiplexer::TCAMultiplexer() : m_currentChannel(255), m_initialized(false) {}

// Get singleton instance
TCAMultiplexer& TCAMultiplexer::getInstance() {
    static TCAMultiplexer m_instance; // Static instance for singleton pattern
    return m_instance;
}

// Initialize the multiplexer
bool TCAMultiplexer::begin() {
    if (!m_initialized) {
        Wire.begin(TCA_ADDR);
        m_initialized = true;
        Serial.println("TCA9548A Multiplexer initialized");
    }
    return m_initialized;
}

// Select channel
bool TCAMultiplexer::selectChannel(uint8_t channel) {
    if (channel > 7) {
        Serial.println("Invalid TCA channel: " + String(channel));
        return false;
    }
    
    // Only switch if we're not already on this channel
    if (m_currentChannel != channel) {
        Wire.beginTransmission(m_TCA_ADDR);
        Wire.write(1 << channel);
        uint8_t error = Wire.endTransmission();
        
        // zero means success, non-zero means error
        if (error == 0) {
            m_currentChannel = channel;
            return true;
        } else {
            Serial.println("TCA channel switch failed, error: " + String(error));
            return false;
        }
    }
    return true; // Already on correct channel
}

// Get current channel
uint8_t TCAMultiplexer::getCurrentChannel() const {
    return m_currentChannel;
}

// Disable all channels
void TCAMultiplexer::disableAll() {
    Wire.beginTransmission(m_TCA_ADDR);
    Wire.write(0x00);
    Wire.endTransmission();
    m_currentChannel = 255;
}