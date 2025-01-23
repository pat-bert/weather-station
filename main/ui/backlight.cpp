#include "backlight.hpp"

#include "driver/ledc.h"

constexpr ledc_mode_t speedMode{LEDC_LOW_SPEED_MODE};
constexpr ledc_timer_t timer{LEDC_TIMER_0};

uint8_t Backlight::m_nextChannel = 0U;

void Backlight::init()
{
    // Prepare and then apply the LEDC PWM timer configuration
    ledc_timer_config_t ledc_timer{};
    ledc_timer.speed_mode = speedMode;
    ledc_timer.timer_num = timer;
    ledc_timer.duty_resolution = static_cast<ledc_timer_bit_t>(m_resolutionBits);
    ledc_timer.freq_hz = m_pwmFrequency;
    ledc_timer.clk_cfg = LEDC_AUTO_CLK;
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t ledc_channel{};
    ledc_channel.speed_mode = speedMode;
    ledc_channel.channel = static_cast<ledc_channel_t>(m_channel);
    ledc_channel.timer_sel = timer;
    ledc_channel.intr_type = LEDC_INTR_DISABLE;
    ledc_channel.gpio_num = m_gpioNumber;
    ledc_channel.duty = 0;
    ledc_channel.hpoint = 0;
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));

    ESP_ERROR_CHECK(ledc_fade_func_install(0));
}

void Backlight::power(bool isOn)
{
    dim(isOn ? 100 : 0);
}

void Backlight::dim(int percentage, int fadeTimeMs)
{
    if (fadeTimeMs == 0)
    {
        ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, static_cast<ledc_channel_t>(m_channel), (1ULL << m_resolutionBits) * percentage / 100));
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, static_cast<ledc_channel_t>(m_channel)));
    }
    else
    {
        ESP_ERROR_CHECK(ledc_set_fade_with_time(LEDC_LOW_SPEED_MODE,
                                                static_cast<ledc_channel_t>(m_channel), 0, fadeTimeMs));
        ESP_ERROR_CHECK(ledc_fade_start(LEDC_LOW_SPEED_MODE, static_cast<ledc_channel_t>(m_channel), LEDC_FADE_NO_WAIT));
    }
}

void Backlight::stopFade()
{
    ESP_ERROR_CHECK(ledc_fade_stop(LEDC_LOW_SPEED_MODE, static_cast<ledc_channel_t>(m_channel)));
}