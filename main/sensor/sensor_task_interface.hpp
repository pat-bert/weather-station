#ifndef SENSOR_TASK_INTERFACE_HPP_INCLUDED
#define SENSOR_TASK_INTERFACE_HPP_INCLUDED

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

struct SensorTaskInterface
{
    QueueHandle_t m_measurementQueue_out;
};

#endif