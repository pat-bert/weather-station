#ifndef INTERFACE_SENSOR_HPP_INCLUDED
#define INTERFACE_SENSOR_HPP_INCLUDED

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
#ifdef BME280_32BIT_ENABLE
    uint32_t m_pressure;
    int32_t m_temperature;
    uint32_t m_humidity;
#else
    double m_pressure;
    double m_temperature;
    double m_humidity;
#endif
    uint16_t m_illuminance;
};

using QueueValueType = std::variant<SensorData, ButtonData, WifiData>;

#endif