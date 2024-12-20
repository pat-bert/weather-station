#ifndef SENSOR_TASK_INTERFACE_HPP_INCLUDED
#define SENSOR_TASK_INTERFACE_HPP_INCLUDED

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#include "driver/i2c.h"

#include <stdint.h>

struct SensorTaskInterface
{
    QueueHandle_t m_measurementQueue_out;
};

#endif