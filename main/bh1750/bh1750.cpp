#include "bh1750.hpp"

int32_t BH1750::powerOnMode(const bool isPowerOn) const
{
    if (isPowerOn)
    {
        return writeCommand(BH1750_POWER_ON);
    }

    return writeCommand(BH1750_POWER_DOWN);
}

int32_t BH1750::setMeasurementMode(const Resolution resolution, const bool isOneTime) const
{
    uint8_t measurementCommand{0x00};
    measurementCommand |= (1 << (4 + static_cast<uint8_t>(isOneTime)));
    measurementCommand |= static_cast<uint8_t>(resolution);
    return writeCommand(measurementCommand);
}

uint32_t BH1750::getMeasurementTimeMs(const Resolution resolution)
{
    if (resolution == Resolution::low)
    {
        return 16U;
    }

    return 120U;
}

int32_t BH1750::resetMeasurement() const
{
    return writeCommand(BH1750_RESET);
}

int32_t BH1750::readMeasurement(uint16_t &illuminance) const
{
    uint8_t illuminanceBytes[2]{};

    int32_t result{m_readFunctionPtr(illuminanceBytes, 2, m_userData)};
    illuminance = (illuminanceBytes[0] << 8) + illuminanceBytes[1];
    illuminance = (illuminance / 12U) * 10U;

    return result;
}

int32_t BH1750::writeCommand(const uint8_t command) const
{
    return m_writeFunctionPtr(&command, 1, m_userData);
}