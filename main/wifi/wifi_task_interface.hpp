#ifndef WIFI_TASK_INTERFACE_HPP_INCLUDED
#define WIFI_TASK_INTERFACE_HPP_INCLUDED

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

struct WifiTaskInterface
{
    QueueHandle_t m_measurementQueue_out;
    EventGroupHandle_t m_sleepEventGroup;
};

#endif