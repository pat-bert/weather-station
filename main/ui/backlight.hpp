#ifndef BACKLIGHT_HPP_INCLUDED
#define BACKLIGHT_HPP_INCLUDED

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#include "esp_lcd_types.h"

#include <cstdint>

struct FadeCallbackData
{
    FadeCallbackData(QueueHandle_t queueHandle) : m_queueHandle{queueHandle}
    {
    }

    QueueHandle_t m_queueHandle;
    bool m_isBacklightOn{false};
    bool m_isBacklightInitialized{false};
};

class Backlight
{
public:
    Backlight(int32_t gpioNumber, uint32_t pwmFrequency, QueueHandle_t queueHandle) : m_gpioNumber{gpioNumber},
                                                                                               m_pwmFrequency{pwmFrequency},
                                                                                               m_fadeCallbackData{queueHandle}
    {
        m_channel = m_nextChannel;
        ++m_nextChannel;
    }

    void init();
    void power(bool isOn);
    void dim(unsigned int percentage, int fadeTimeMs = 0);
    void stopFade();
    bool isOn();

private:
    static uint8_t m_nextChannel;

    int32_t m_gpioNumber;
    uint32_t m_pwmFrequency;
    uint8_t m_channel;
    FadeCallbackData m_fadeCallbackData;
};

#endif