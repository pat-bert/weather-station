#ifndef TASK_UI_HPP_INCLUDED
#define TASK_UI_HPP_INCLUDED

#include "ui/ui_task_interface.hpp"
#include "interface_sensor.hpp"

#include "backlight.hpp"

// SPI Settings
#define LCD_SPI_TRADITIONAL_MODE (0) // CPOL = 0, CPHA = 0

// ST7735 Settings
#define LCD_CMD_BITS 8
#define LCD_PARAM_BITS 8

// LVGL Settings
#define LVGL_TASK_MAX_DELAY_MS (1000 * CONFIG_MEASUREMENT_INTERVAL_SECONDS / 2)
#define LVGL_TASK_MIN_DELAY_MS 1

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

namespace Ui
{
    constexpr uint8_t bufferDisplayFraction{10U};
    constexpr size_t numberOfBufferElements{(CONFIG_LCD_H_RES * CONFIG_LCD_V_RES) / bufferDisplayFraction};
    constexpr size_t bufferSize{static_cast<size_t>(numberOfBufferElements * LV_COLOR_FORMAT_GET_SIZE(LV_COLOR_FORMAT_RGB565))};

    struct FlushCallbackData
    {
        lv_display_t *m_display;
    };

    struct UiHandles
    {
        lv_obj_t *m_pressureMeter;
        lv_obj_t *m_indic;
        lv_obj_t *m_temperatureBar;
        lv_obj_t *m_temperatureLabel;
        lv_obj_t *m_illuminanceLabel;
        lv_obj_t *m_humidityLabel;
        lv_obj_t *m_timeLabel;
        lv_obj_t *m_tabview;
        lv_obj_t *m_temperatureAndHumidityChart;
        lv_chart_series_t *m_temperatureSeries;
        lv_chart_series_t *m_humiditySeries;
        lv_obj_t *m_provisioningQrCode;
    };

    class Task
    {
    public:
        explicit Task(UiTaskInterface *uiTaskInterface)
            : m_uiTaskInterface(uiTaskInterface),
              m_backlight{CONFIG_LCD_BACKLIGHT_GPIO, CONFIG_LCD_BACKLIGHT_HZ, uiTaskInterface->m_queue_in}
        {
            m_displayBuffer1 = heap_caps_malloc(bufferSize, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
            assert(m_displayBuffer1);
            m_displayBuffer2 = heap_caps_malloc(bufferSize, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
            assert(m_displayBuffer2);
        }

        ~Task()
        {
            heap_caps_free(m_displayBuffer1);
            heap_caps_free(m_displayBuffer2);
        }

        void init();
        void run();

    protected:
        void create_ui();

        void handleQueue(const QueueValueType &queueData);

        void handleSensorData(UiHandles *uiHandles, const SensorData &sensorData);

    private:
        UiTaskInterface *m_uiTaskInterface{nullptr};
        UiHandles m_uiHandles;
        lv_display_t *m_display;
        FlushCallbackData m_flushCallbackData;
        Backlight m_backlight;
        esp_lcd_panel_handle_t m_panelHandle;

        void *m_displayBuffer1{nullptr};
        void *m_displayBuffer2{nullptr};

        int32_t m_temperatureBuffer[numberOfSensorReadingsSaved]{};
        int32_t m_humidityBuffer[numberOfSensorReadingsSaved]{};
    };

    static void run_task_wrapper(void *arg)
    {
        Task task{static_cast<UiTaskInterface *>(arg)};
        task.init();
        task.run();
    }
}

#endif