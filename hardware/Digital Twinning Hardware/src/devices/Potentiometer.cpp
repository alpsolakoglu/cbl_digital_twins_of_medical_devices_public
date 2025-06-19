#include <devices/Potentiometer.h>
#include <Arduino.h>

namespace DT
{
    Potentiometer::Potentiometer(uint8_t pin, uint16_t minADC, uint16_t maxADC, uint16_t zeroADC, bool invertVirtual, double minVirtual, double maxVirtual, double zeroVirtual)
        : m_pin(pin),
          m_minADC(minADC),
          m_zeroADC(zeroADC),
          m_maxADC(maxADC),
          m_invertVirtual(invertVirtual),
          m_minVirtual(minVirtual),
          m_maxVirtual(maxVirtual),
          m_zeroVirtual(zeroVirtual) {};

    uint16_t Potentiometer::getMinADC() const { return m_minADC; }
    uint16_t Potentiometer::getMaxADC() const { return m_maxADC; }
    uint16_t Potentiometer::getZeroADC() const { return m_zeroADC; }
    bool Potentiometer::getInvertVirtual() const { return m_invertVirtual; }
    double Potentiometer::getMinVirtual() const { return m_minVirtual; }
    double Potentiometer::getMaxVirtual() const { return m_maxVirtual; }
    double Potentiometer::getZeroVirtual() const { return m_zeroVirtual; }

    uint16_t Potentiometer::setZeroADC(uint16_t zeroADC)
    {
        m_zeroADC = zeroADC;
        return m_zeroADC;
    }

    uint16_t Potentiometer::getCurrentADC() 
    {
        return analogRead(m_pin);
    }

    double Potentiometer::getCurrentVirtual() const
    {
        uint16_t valueADC = analogRead(m_pin);

        Serial.println("ADC value read from pin " + String(m_pin) + ": " + String(valueADC));

        if (valueADC < m_minADC)
        {
            valueADC = m_minADC;
        }
        else if (valueADC > m_maxADC)
        {
            valueADC = m_maxADC;
        }

        if (valueADC == m_zeroADC)
        {
            // Avoid division by zero in next step
            return m_zeroVirtual;
        }

        double ratio = (double) (valueADC - m_minADC) / (double) (m_maxADC - m_minADC);

        Serial.println("Calculated ratio: " + String(ratio));

        if (ratio < 0.0 || ratio > 1.0)
        {
            throw std::out_of_range("Ratio out of range: " + std::to_string(ratio));
        }

        if (m_invertVirtual)
        {
            ratio = 1.0 - ratio; // Invert the ratio if m_invertVirtual is true
        }

        Serial.println("Adjusted ratio after inversion: " + String(ratio));
        // Calculate the virtual value based on the ratio
        Serial.println("Calculated virtual value: " + String(m_minVirtual + ratio * (m_maxVirtual - m_minVirtual)));

        return m_minVirtual + ratio * (m_maxVirtual - m_minVirtual);
    }

    double Potentiometer::getCurrentVirtualComplicated() const
    {
        uint16_t valueADC = analogRead(m_pin);

        if (valueADC < m_minADC)
        {
            valueADC = m_minADC;
        }
        else if (valueADC > m_maxADC)
        {
            valueADC = m_maxADC;
        }

        if (valueADC == m_zeroADC)
        {
            // Avoid divison by zero in next step
            return m_zeroVirtual;
        }

        double ratio = std::abs(valueADC - m_zeroADC);
        if (valueADC >= m_zeroADC && !m_invertVirtual || valueADC < m_zeroADC && m_invertVirtual)
        {
            ratio /= (double)std::abs(m_maxADC - m_zeroADC);
        }
        else
        {
            ratio /= (double)(-1 * std::abs(m_minADC - m_zeroADC));
        }

        if (ratio < -1.0 || ratio > 1.0)
        {
            throw std::out_of_range("Ratio out of range: " + std::to_string(ratio));
        }

        if (ratio >= 0)
        {
            return m_zeroVirtual + ratio * std::abs(m_maxVirtual - m_zeroVirtual);
        }
        else
        {
            return m_zeroVirtual + ratio * std::abs(m_minVirtual - m_zeroVirtual);
        }
    }
}