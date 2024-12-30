#ifndef BH1750_HPP_INCLUDED
#define BH1750_HPP_INCLUDED

#include <cstdint>

#define BH1750_I2C_ADDR_PRIM (0x5C) // Address pin high
#define BH1750_I2C_ADDR_SEC (0x23)  // Address pin low

#define BH1750_POWER_DOWN (0x00)
#define BH1750_POWER_ON (0x01)
#define BH1750_RESET (0x07)

#define BH1750_CONT_HI_RES_MODE_1 (0x10) // 1 lx resolution, 120 ms measurement time, continuous
#define BH1750_CONT_HI_RES_MODE_2 (0x11) // 0.5 lx resolution, 120 ms measurement time, continuous
#define BH1750_CONT_LO_RES_MODE (0x13)   // 4 lx resolution, 16 ms measurement time, continuous

// Sensor switches to power down mode automatically after measurement
#define BH1750_OT_HI_RES_MODE_1 (0x20) // 1 lx resolution, 120 ms measurement time, single
#define BH1750_OT_HI_RES_MODE_2 (0x21) // 0.5 lx resolution, 120 ms measurement time, single
#define BH1750_OT_LO_RES_MODE (0x23)   // 4 lx resolution, 16 ms measurement time, single

class BH1750
{
public:
    enum class Resolution : uint8_t
    {
        medium = 0U,
        high = 1U,
        low = 3U,
    };

    using readFunctionPointer_t = int32_t (*)(uint8_t *registerData, int32_t length, void *userData);
    using writeFunctionPointer_t = int32_t (*)(const uint8_t *registerData, int32_t length, void *userData);

    BH1750(const readFunctionPointer_t readFunctionPtr, const writeFunctionPointer_t writeFunctionPtr, void *userData)
        : m_readFunctionPtr{readFunctionPtr},
          m_writeFunctionPtr{writeFunctionPtr},
          m_userData{userData}
    {
    }

    int32_t powerOnMode(const bool isPowerOn) const;

    int32_t setMeasurementMode(const Resolution resolution, const bool isOneTime) const;

    static uint32_t getMeasurementTimeMs(const Resolution resolution);

    int32_t resetMeasurement() const;
    int32_t readMeasurement(uint16_t &illuminance) const;

protected:
    int32_t writeCommand(const uint8_t command) const;

private:
    readFunctionPointer_t m_readFunctionPtr;
    writeFunctionPointer_t m_writeFunctionPtr;
    void *m_userData;
};

#endif