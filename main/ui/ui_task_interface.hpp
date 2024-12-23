#ifndef UI_TASK_INTERFACE_HPP_INCLUDED
#define UI_TASK_INTERFACE_HPP_INCLUDED

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "lvgl.h"

struct UiTaskInterface
{
    QueueHandle_t m_measurementQueue_in;
    lv_disp_t *m_disp;
    lv_obj_t *m_pressureMeter;
    lv_meter_indicator_t *m_indic;
    lv_obj_t *m_temperatureBar;
    lv_obj_t *m_temperatureLabel;
    lv_obj_t *m_timeLabel;
};

#endif