#ifndef TASK_UI_HPP_INCLUDED
#define TASK_UI_HPP_INCLUDED

#include "ui/ui_task_interface.hpp"

// SPI Settings
#define LCD_SPI_TRADITIONAL_MODE (0) // CPOL = 0, CPHA = 0

// ST7735 Settings
#define LCD_CMD_BITS 8
#define LCD_PARAM_BITS 8

// LVGL Settings
#define LVGL_TASK_MAX_DELAY_MS (1000 * CONFIG_MEASUREMENT_INTERVAL_SECONDS / 2)
#define LVGL_TASK_MIN_DELAY_MS 1
#define LVGL_SCREEN_DIVIDER (10)
#define LVGL_BUFFER_ELEMENTS ((CONFIG_LCD_H_RES * CONFIG_LCD_V_RES) / LVGL_SCREEN_DIVIDER)

// UI Settings
#define MIN_TEMPERATURE_C (15)
#define MAX_TEMPERATURE_C (35)
#define TEMPERATURE_SCALING_FACTOR (10)

#define MIN_PRESSURE_HPA (950)
#define PRESSURE_THRESHOLD_CHANGING (1000)
#define PRESSURE_THRESHOLD_FAIR (1020)
#define MAX_PRESSURE_HPA (1050)

#define PRESSURE_TICKS_HPA (5)
#define PRESSURE_SCALING_DIVISOR (100)

struct SensorAverageData
{
    int32_t m_temperatureAverageLastHourCentigrade{0};
    uint32_t m_humidityAverageLastHour{0U};

    uint8_t m_sensorReadingsLastHour{0U};
    uint8_t m_hoursTracked{0U};
};

struct FlushCallbackData
{
    lv_display_t *m_display;
};

void task_lvgl(void *arg);

#endif