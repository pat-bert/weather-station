#ifndef INTERFACE_SENSOR_HPP_INCLUDED
#define INTERFACE_SENSOR_HPP_INCLUDED

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#include <stdint.h>

struct SensorData
{
    double m_pressure;
    double m_temperature;
    uint16_t m_illuminance;
};

struct SensorTaskInterface
{
    QueueHandle_t m_measurementQueue_out;
};

#endif