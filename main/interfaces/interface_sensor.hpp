#ifndef INTERFACE_SENSOR_HPP_INCLUDED
#define INTERFACE_SENSOR_HPP_INCLUDED

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#include <stdint.h>
#include <variant>

struct ButtonData
{
    bool m_tabviewButtonPressed;
};

struct SensorData
{
    double m_pressure;
    double m_temperature;
    uint16_t m_illuminance;
    uint8_t m_humidity;
};

using QueueValueType = std::variant<SensorData, ButtonData>;

struct SensorTaskInterface
{
    QueueHandle_t m_measurementQueue_out;
};

#endif