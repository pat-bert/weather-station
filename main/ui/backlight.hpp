#ifndef BACKLIGHT_HPP_INCLUDED
#define BACKLIGHT_HPP_INCLUDED

#include <cstdint>

class Backlight
{
public:
    Backlight(int32_t gpioNumber, uint32_t pwmFrequency) : m_gpioNumber{gpioNumber},
                                                           m_pwmFrequency{pwmFrequency}
    {
        m_channel = m_nextChannel;
        ++m_nextChannel;
    }

    void init();
    void power(bool isOn);
    void dim(unsigned int percentage, int fadeTimeMs = 0);
    void stopFade();

private:
    static uint8_t m_nextChannel;

    int32_t m_gpioNumber;
    uint32_t m_pwmFrequency;
    uint8_t m_channel;
};

#endif