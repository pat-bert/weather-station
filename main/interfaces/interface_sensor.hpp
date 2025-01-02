#ifndef INTERFACE_SENSOR_HPP_INCLUDED
#define INTERFACE_SENSOR_HPP_INCLUDED

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#include <stdint.h>
#include <variant>

struct WifiData
{
    char m_provisioningPayload[150];
};

struct ButtonData
{
    bool m_tabviewButtonPressed;
};

struct SensorData
{
    double m_pressure;
    double m_temperature;
    double m_humidity;
    uint16_t m_illuminance;
};

using QueueValueType = std::variant<SensorData, ButtonData, WifiData>;

struct SensorTaskInterface
{
    QueueHandle_t m_measurementQueue_out;
};

#endif