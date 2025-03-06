#ifndef UI_TASK_INTERFACE_HPP_INCLUDED
#define UI_TASK_INTERFACE_HPP_INCLUDED

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "lvgl.h"

struct UiTaskInterface
{
    QueueHandle_t m_queue_in;
    EventGroupHandle_t m_sleepEventGroup;

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

#endif