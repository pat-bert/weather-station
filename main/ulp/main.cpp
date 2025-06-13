#include "../bh1750/bh1750.hpp"
#include "../bme280/bme280.h"
#include "../interfaces/interface_sensor.hpp"

#include "ulp_lp_core_i2c.h"
#include "ulp_lp_core_utils.h"

#include <cstdint>
#include <cstring>
#include <sdkconfig.h>
#include <queue>

#define ESP_LP_GOTO_ON_ERROR(x, goto_tag, format, ...)                                      \
    do                                                                                      \
    {                                                                                       \
        esp_err_t err_rc_ = (x);                                                            \
        if (unlikely(err_rc_ != ESP_OK))                                                    \
        {                                                                                   \
            /* lp_core_printf("%s(%d): " format, __FUNCTION__, __LINE__, ##__VA_ARGS__); */ \
            ret = err_rc_;                                                                  \
            goto goto_tag;                                                                  \
        }                                                                                   \
    } while (0)

#define BOSCH_LP_GOTO_ON_ERROR(x, goto_tag, format, ...)                                    \
    do                                                                                      \
    {                                                                                       \
        esp_err_t err_rc_ = (x);                                                            \
        if (unlikely(err_rc_ != BME280_OK))                                                 \
        {                                                                                   \
            /* lp_core_printf("%s(%d): " format, __FUNCTION__, __LINE__, ##__VA_ARGS__); */ \
            ret = err_rc_;                                                                  \
            goto goto_tag;                                                                  \
        }                                                                                   \
    } while (0)

volatile uint32_t illuminance = 0;
volatile uint32_t pressure = 0;
volatile int32_t temperature = 0;
volatile uint32_t humidity = 0;

volatile uint32_t averageHumidity[numberOfSensorReadingsSaved];
volatile int32_t averageTemperature[numberOfSensorReadingsSaved];
volatile uint32_t sensorReadingsLastHour = 0;
volatile uint32_t hoursTracked = 0;

static BME280_INTF_RET_TYPE bme280_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
    if ((length == 0) || (length > static_cast<uint32_t>(INT32_MAX)))
    {
        return BME280_INTF_RET_SUCCESS;
    }
    esp_err_t ret = lp_core_i2c_master_write_read_device(LP_I2C_NUM_0, BME280_I2C_ADDR_PRIM, &reg_addr, 1, reg_data, static_cast<int32_t>(length), -1);
    return (ESP_OK == ret) ? BME280_INTF_RET_SUCCESS : -1;
}

static BME280_INTF_RET_TYPE bme280_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
    if ((length > static_cast<uint32_t>(INT32_MAX)) || (length > BME280_MAX_LEN))
    {
        return -1;
    }

    uint8_t writeBuffer[BME280_MAX_LEN + 1];
    writeBuffer[0] = reg_addr;
    std::memcpy(writeBuffer + 1, reg_data, length);

    esp_err_t ret = lp_core_i2c_master_write_to_device(LP_I2C_NUM_0, BME280_I2C_ADDR_PRIM, writeBuffer, static_cast<int32_t>(length + 1), -1);
    return (ESP_OK == ret) ? BME280_INTF_RET_SUCCESS : -1;
}

static void bme280_delay_ms(uint32_t period, void *intf_ptr)
{
    (void)intf_ptr;
    ulp_lp_core_delay_us(1000U * period);
}

static int32_t bh1750_i2c_read(uint8_t *reg_data, int32_t length, void *intf_ptr)
{
    if (length == 0)
    {
        return 0;
    }

    esp_err_t ret = lp_core_i2c_master_read_from_device(LP_I2C_NUM_0, BH1750_I2C_ADDR_SEC, reg_data, length, -1);
    return ret;
}

static int32_t bh1750_i2c_write(const uint8_t *reg_data, int32_t length, void *intf_ptr)
{
    return lp_core_i2c_master_write_to_device(LP_I2C_NUM_0, BH1750_I2C_ADDR_SEC, reg_data, length, -1);
}

extern "C" int main(void)
{
    BH1750 bh1750{bh1750_i2c_read, bh1750_i2c_write, nullptr};
    constexpr BH1750::Resolution illuminanceResolution{BH1750::Resolution::medium};
    const uint32_t illuminanceMeasurementTimeMs{bh1750.getMeasurementTimeMs(illuminanceResolution)};
    uint16_t illuminanceReading{0U};

    bme280_dev bme280{};
    bme280.intf = BME280_I2C_INTF;
    bme280.intf_ptr = nullptr;
    bme280.read = bme280_i2c_read;
    bme280.write = bme280_i2c_write;
    bme280.delay_ms = bme280_delay_ms;
    bme280_data measurement{};
    uint32_t bme280MeasurementTimeUs{0U};

    int32_t ret;
    bme280_settings settings{};

    ret = bme280_init(&bme280);
    BOSCH_LP_GOTO_ON_ERROR(ret, err, "%d", ret);

    // Always read the current settings before writing, especially when all the configuration is not modified
    ret = bme280_get_sensor_settings(&settings, &bme280);
    BOSCH_LP_GOTO_ON_ERROR(ret, err, "%d", ret);

    // Configuring the over-sampling mode, filter coefficient and output data rate
    // Overwrite the desired settings
    settings.filter = BME280_FILTER_COEFF_OFF;
    settings.osr_h = BME280_OVERSAMPLING_1X;
    settings.osr_t = BME280_OVERSAMPLING_1X;
    settings.osr_p = BME280_OVERSAMPLING_1X;
    settings.standby_time = BME280_STANDBY_TIME_0_5_MS;

    ret = bme280_set_sensor_settings(BME280_SEL_ALL_SETTINGS, &settings, &bme280);
    BOSCH_LP_GOTO_ON_ERROR(ret, err, "%d", ret);

    ret = bh1750.powerOnMode(true);
    ESP_LP_GOTO_ON_ERROR(ret, err, "%d", ret);

    ret = bh1750.setMeasurementMode(illuminanceResolution, true);
    ESP_LP_GOTO_ON_ERROR(ret, err, "%d", ret);

    ulp_lp_core_delay_us(1000 * illuminanceMeasurementTimeMs);

    ret = bh1750.readMeasurement(illuminanceReading);
    ESP_LP_GOTO_ON_ERROR(ret, err, "%d", ret);

    ret = bme280_set_sensor_mode(BME280_POWERMODE_FORCED, &bme280);
    BOSCH_LP_GOTO_ON_ERROR(ret, err, "%d", ret);

    ret = bme280_cal_meas_delay(&bme280MeasurementTimeUs, &settings);
    BOSCH_LP_GOTO_ON_ERROR(ret, err, "%d", ret);

    ulp_lp_core_delay_us(bme280MeasurementTimeUs);

    ret = bme280_get_sensor_data(BME280_ALL, &measurement, &bme280);
    BOSCH_LP_GOTO_ON_ERROR(ret, err, "%d", ret);

    // Synchronize sensor data for main core
#ifndef BME280_DOUBLE_ENABLE
    temperature = measurement.temperature;
    pressure = measurement.pressure / 100;
    humidity = measurement.humidity / 1000;
#else
    temperature = measurement.temperature;
    pressure = measurement.pressure;
    humidity = measurement.humidity;
#endif
    illuminance = static_cast<uint32_t>(illuminanceReading);

    sensorReadingsLastHour++;
    if (sensorReadingsLastHour > 60)
    {
        if (hoursTracked < numberOfSensorReadingsSaved)
        {
            hoursTracked++;
        }
        else
        {
            // Shift the average data to the left
            for (uint32_t i = 0; i < numberOfSensorReadingsSaved - 1; ++i)
            {
                averageTemperature[i] = averageTemperature[i + 1];
                averageHumidity[i] = averageHumidity[i + 1];
            }
        }

        averageTemperature[hoursTracked] = 0;
        averageHumidity[hoursTracked] = 0U;
        sensorReadingsLastHour = 1;
    }

    averageTemperature[hoursTracked] = (averageTemperature[hoursTracked] * (sensorReadingsLastHour - 1) + temperature) / sensorReadingsLastHour;
    averageHumidity[hoursTracked] = (averageHumidity[hoursTracked] * (sensorReadingsLastHour - 1) + humidity) / sensorReadingsLastHour;

    // ulp_lp_core_wakeup_main_processor();

    return 0;

err:
    return ret;
}