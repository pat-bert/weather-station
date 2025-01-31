#include "backlight.hpp"

#include "driver/ledc.h"

#include "esp_attr.h"

constexpr ledc_mode_t speedMode{LEDC_LOW_SPEED_MODE};
constexpr ledc_timer_t timer{LEDC_TIMER_0};
constexpr ledc_timer_bit_t resolution{LEDC_TIMER_12_BIT};

uint8_t Backlight::m_nextChannel = 0U;

// Brightness 0 - 100% gamma correction look up table (gamma = 2.6)
// Y = B ^ 2.6
// Pre-computed LUT to save some runtime computation
static const float gamma_correction_lut[101] = {
    0.000000,
    0.000006,
    0.000038,
    0.000110,
    0.000232,
    0.000414,
    0.000666,
    0.000994,
    0.001406,
    0.001910,
    0.002512,
    0.003218,
    0.004035,
    0.004969,
    0.006025,
    0.007208,
    0.008525,
    0.009981,
    0.011580,
    0.013328,
    0.015229,
    0.017289,
    0.019512,
    0.021902,
    0.024465,
    0.027205,
    0.030125,
    0.033231,
    0.036527,
    0.040016,
    0.043703,
    0.047593,
    0.051688,
    0.055993,
    0.060513,
    0.065249,
    0.070208,
    0.075392,
    0.080805,
    0.086451,
    0.092333,
    0.098455,
    0.104821,
    0.111434,
    0.118298,
    0.125416,
    0.132792,
    0.140428,
    0.148329,
    0.156498,
    0.164938,
    0.173653,
    0.182645,
    0.191919,
    0.201476,
    0.211321,
    0.221457,
    0.231886,
    0.242612,
    0.253639,
    0.264968,
    0.276603,
    0.288548,
    0.300805,
    0.313378,
    0.326268,
    0.339480,
    0.353016,
    0.366879,
    0.381073,
    0.395599,
    0.410461,
    0.425662,
    0.441204,
    0.457091,
    0.473325,
    0.489909,
    0.506846,
    0.524138,
    0.541789,
    0.559801,
    0.578177,
    0.596920,
    0.616032,
    0.635515,
    0.655374,
    0.675610,
    0.696226,
    0.717224,
    0.738608,
    0.760380,
    0.782542,
    0.805097,
    0.828048,
    0.851398,
    0.875148,
    0.899301,
    0.923861,
    0.948829,
    0.974208,
    1.000000,
};

static IRAM_ATTR bool fadeEndCallback(const ledc_cb_param_t *param, void *user_arg)
{
    if (param->event == LEDC_FADE_END_EVT)
    {
        FadeCallbackData *fadeCallbackData{static_cast<FadeCallbackData *>(user_arg)};
        fadeCallbackData->m_isBacklightOn = param->duty > 0;

        if (!(fadeCallbackData->m_isBacklightOn))
        {
            ledc_fade_func_uninstall();

            ESP_ERROR_CHECK(ledc_stop(static_cast<ledc_mode_t>(param->speed_mode), static_cast<ledc_channel_t>(param->channel), 0));

            ESP_ERROR_CHECK(ledc_timer_pause(static_cast<ledc_mode_t>(param->speed_mode), timer));

            ledc_timer_config_t ledc_timer{};
            ledc_timer.speed_mode = static_cast<ledc_mode_t>(param->speed_mode);
            ledc_timer.timer_num = timer;
            ledc_timer.deconfigure = true;
            ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

            fadeCallbackData->m_isBacklightInitialized = false;
        }
    }

    return false;
}

uint32_t gammaCorrection(uint32_t duty)
{
    return gamma_correction_lut[duty * 100 / (1 << resolution)] * (1 << resolution);
}

void Backlight::init()
{
    if (m_fadeCallbackData.m_isBacklightInitialized)
    {
        return;
    }

    // Prepare and then apply the LEDC PWM timer configuration
    ledc_timer_config_t ledc_timer{};
    ledc_timer.speed_mode = speedMode;
    ledc_timer.timer_num = timer;
    ledc_timer.duty_resolution = resolution;
    ledc_timer.freq_hz = m_pwmFrequency;
    ledc_timer.clk_cfg = LEDC_USE_RC_FAST_CLK;
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
    ledc_channel.sleep_mode = LEDC_SLEEP_MODE_KEEP_ALIVE;
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));

    ESP_ERROR_CHECK(ledc_fade_func_install(0));

    ledc_cbs_t ledcCallbackConfig{};
    ledcCallbackConfig.fade_cb = fadeEndCallback;

    ESP_ERROR_CHECK(ledc_cb_register(speedMode, static_cast<ledc_channel_t>(m_channel), &ledcCallbackConfig, static_cast<void *>(&m_fadeCallbackData)));

    m_fadeCallbackData.m_isBacklightInitialized = true;
}

void Backlight::power(bool isOn)
{
    dim(isOn ? 100 : 0);
}

void Backlight::dim(unsigned int percentage, int fadeTimeMs)
{
    const uint32_t duty{(1UL << resolution) * percentage / 100};
    if (fadeTimeMs == 0)
    {
        ESP_ERROR_CHECK(ledc_set_duty(speedMode, static_cast<ledc_channel_t>(m_channel), duty));
        ESP_ERROR_CHECK(ledc_update_duty(speedMode, static_cast<ledc_channel_t>(m_channel)));
        m_fadeCallbackData.m_isBacklightOn = percentage > 0;
    }
    else
    {
        const uint32_t linear_fade_segments = 12;
        uint32_t actual_fade_ranges;
        ledc_fade_param_config_t fade_params_list[SOC_LEDC_GAMMA_CURVE_FADE_RANGE_MAX] = {};

        uint32_t startDuty{ledc_get_duty(speedMode, static_cast<ledc_channel_t>(m_channel))};

        ESP_ERROR_CHECK(ledc_fill_multi_fade_param_list(speedMode,
                                                        static_cast<ledc_channel_t>(m_channel),
                                                        startDuty,
                                                        duty,
                                                        linear_fade_segments,
                                                        fadeTimeMs,
                                                        gammaCorrection,
                                                        SOC_LEDC_GAMMA_CURVE_FADE_RANGE_MAX,
                                                        fade_params_list,
                                                        &actual_fade_ranges));
        ESP_ERROR_CHECK(ledc_set_multi_fade_and_start(speedMode,
                                                      static_cast<ledc_channel_t>(m_channel),
                                                      gammaCorrection(startDuty),
                                                      fade_params_list,
                                                      actual_fade_ranges,
                                                      LEDC_FADE_NO_WAIT));
    }
}

void Backlight::stopFade()
{
    ESP_ERROR_CHECK(ledc_fade_stop(speedMode, static_cast<ledc_channel_t>(m_channel)));
}

bool Backlight::isOn()
{
    return m_fadeCallbackData.m_isBacklightOn;
}