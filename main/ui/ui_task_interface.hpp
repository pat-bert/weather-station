#ifndef UI_TASK_INTERFACE_HPP_INCLUDED
#define UI_TASK_INTERFACE_HPP_INCLUDED

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "lvgl.h"

namespace Ui
{
    struct UiTaskInterface
    {
        QueueHandle_t m_queue_in;
        EventGroupHandle_t m_sleepEventGroup;
    };
}

#endif