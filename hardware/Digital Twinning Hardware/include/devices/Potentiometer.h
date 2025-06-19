#ifndef POTENTIOMETER_H
#define POTENTIOMETER_H

#include <stdint.h>
#include <cstdlib>

namespace DT
{
    class Potentiometer
    {
    private:
        uint8_t m_pin;

        uint16_t m_minADC;
        uint16_t m_maxADC;
        uint16_t m_zeroADC;

        bool m_invertVirtual;
        double m_minVirtual;
        double m_maxVirtual;
        double m_zeroVirtual;

    public:
        Potentiometer(uint8_t pin, uint16_t minADC, uint16_t maxADC, uint16_t zeroADC, bool invertVirtual, double minVirtual, double maxVirtual, double zeroVirtual);

        // Convert ADC value to virtual value
        double getCurrentVirtualComplicated() const;
        double getCurrentVirtual() const;

        uint8_t getPin() const;
        uint16_t getMinADC() const;
        uint16_t getMaxADC() const;
        uint16_t getZeroADC() const;
        bool getInvertVirtual() const;
        double getMinVirtual() const;
        double getMaxVirtual() const;
        double getZeroVirtual() const;

        uint16_t getCurrentADC();

        uint16_t setZeroADC(uint16_t zeroADC);
    };
}

#endif