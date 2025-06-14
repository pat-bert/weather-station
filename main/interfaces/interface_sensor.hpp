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
    bool m_buttonPressed;
};

struct FadeData
{
    bool m_requestLcdControllerOff;
};

constexpr uint16_t numberOfSensorReadingsSaved{25};

struct SensorData
{
#ifdef BME280_DOUBLE_ENABLE
    double m_pressure;
    double m_temperature;
    double m_humidity;
#else
    uint32_t m_pressure;   // Pressure in Pa
    int32_t m_temperature; // Temperature in 0.01 °C
    uint32_t m_humidity;   // Humidity in RH%
#endif
    uint16_t m_illuminance;
};

using QueueValueType = std::variant<SensorData, ButtonData, WifiData, FadeData>;

#define SNTP_READY_FOR_DEEP_SLEEP BIT0
#define UI_READY_FOR_DEEP_SLEEP BIT1

#endif