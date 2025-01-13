#include "task_sensor.hpp"

#include "bme280/bme280.h"
#include "bh1750/bh1750.hpp"
#include "interfaces/interface_sensor.hpp"

#include "driver/i2c_master.h"
#include "esp_log.h"
#include "esp_timer.h"

#if SOC_LP_CORE_SUPPORTED && CONFIG_ULP_COPROC_TYPE_LP_CORE
#include "ulp_lp_sensor.h"
#endif

#include "rom/ets_sys.h"

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#include <cstring>

static const char TAG[] = "sensor";

i2c_master_bus_handle_t initMasterI2C()
{
    ESP_LOGI(TAG, "Creating I2C Master Bus");
    i2c_master_bus_config_t busConfig{};
    busConfig.i2c_port = CONFIG_I2C_PORT;
    busConfig.sda_io_num = static_cast<gpio_num_t>(CONFIG_SDA_GPIO);
    busConfig.scl_io_num = static_cast<gpio_num_t>(CONFIG_SCL_GPIO);
    busConfig.clk_source = I2C_CLK_SRC_DEFAULT;
    busConfig.glitch_ignore_cnt = 7;
    busConfig.intr_priority = 0;
    busConfig.flags.enable_internal_pullup = 0;

    i2c_master_bus_handle_t busHandle{};
    ESP_ERROR_CHECK(i2c_new_master_bus(&busConfig, &busHandle));

    return busHandle;
}

i2c_master_dev_handle_t initTempPressureI2CSlave(const i2c_master_bus_handle_t busHandle)
{
    ESP_LOGI(TAG, "Adding T+P I2C sensor with address %#02x to bus", BME280_I2C_ADDR_PRIM);
    i2c_device_config_t deviceConfig{};
    deviceConfig.scl_speed_hz = CONFIG_BME_I2C_CLOCK_KHZ * 1000U;
    deviceConfig.device_address = BME280_I2C_ADDR_PRIM;
    deviceConfig.dev_addr_length = I2C_ADDR_BIT_LEN_7;

    i2c_master_dev_handle_t deviceHandle{};
    ESP_ERROR_CHECK(i2c_master_bus_add_device(busHandle, &deviceConfig, &deviceHandle));

    return deviceHandle;
}

i2c_master_dev_handle_t initIlluminanceI2CSlave(const i2c_master_bus_handle_t busHandle)
{
    ESP_LOGI(TAG, "Adding illuminance I2C sensor with address %#02x to bus", BH1750_I2C_ADDR_SEC);
    i2c_device_config_t deviceConfig{};
    deviceConfig.scl_speed_hz = CONFIG_BH1750_I2C_CLOCK_KHZ * 1000U;
    deviceConfig.device_address = BH1750_I2C_ADDR_SEC;
    deviceConfig.dev_addr_length = I2C_ADDR_BIT_LEN_7;

    i2c_master_dev_handle_t deviceHandle{};
    ESP_ERROR_CHECK(i2c_master_bus_add_device(busHandle, &deviceConfig, &deviceHandle));

    return deviceHandle;
}

BME280_INTF_RET_TYPE bme280_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
    if ((length == 0) || (length > static_cast<uint32_t>(INT32_MAX)))
    {
        return BME280_INTF_RET_SUCCESS;
    }

    const i2c_master_dev_handle_t *deviceHandle{static_cast<const i2c_master_dev_handle_t *>(intf_ptr)};
    esp_err_t ret = i2c_master_transmit_receive(*deviceHandle, &reg_addr, 1, reg_data, static_cast<int32_t>(length), pdMS_TO_TICKS(10000));
    return (ESP_OK == ret) ? BME280_INTF_RET_SUCCESS : -1;
}

BME280_INTF_RET_TYPE bme280_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
    if ((length > static_cast<uint32_t>(INT32_MAX)) || (length > BME280_MAX_LEN))
    {
        return -1;
    }

    uint8_t writeBuffer[BME280_MAX_LEN + 1];
    writeBuffer[0] = reg_addr;
    std::memcpy(writeBuffer + 1, reg_data, length);

    const i2c_master_dev_handle_t *deviceHandle{static_cast<const i2c_master_dev_handle_t *>(intf_ptr)};
    esp_err_t ret = i2c_master_transmit(*deviceHandle, writeBuffer, static_cast<int32_t>(length + 1), pdMS_TO_TICKS(10000));
    return (ESP_OK == ret) ? BME280_INTF_RET_SUCCESS : -1;
}

void bme280_delay_ms(uint32_t period, void *intf_ptr)
{
    (void)intf_ptr;
    vTaskDelay(pdMS_TO_TICKS(period));
}

void bme280_error_codes_print_result(int8_t rslt)
{
    if (rslt != BME280_OK)
    {
        switch (rslt)
        {
        case BME280_E_NULL_PTR:
            ESP_LOGE(TAG, "Nullptr");
            break;
        case BME280_E_COMM_FAIL:
            ESP_LOGE(TAG, "Communication failure");
            break;
        case BME280_E_INVALID_LEN:
            ESP_LOGE(TAG, "Zero message length");
            break;
        case BME280_E_DEV_NOT_FOUND:
            ESP_LOGE(TAG, "Device not found");
            break;
        case BME280_E_SLEEP_MODE_FAIL:
            ESP_LOGE(TAG, "Sleep mode failed");
            break;
        case BME280_E_NVM_COPY_FAILED:
            ESP_LOGE(TAG, "Nvm copy failed");
            break;
        default:
            ESP_LOGE(TAG, "Unknown error code: %d", rslt);
            break;
        }
    }
}

int32_t bh1750_i2c_read(uint8_t *reg_data, int32_t length, void *intf_ptr)
{
    if (length == 0)
    {
        return 0;
    }

    const i2c_master_dev_handle_t *deviceHandle{static_cast<const i2c_master_dev_handle_t *>(intf_ptr)};
    esp_err_t ret = i2c_master_receive(*deviceHandle, reg_data, length, pdMS_TO_TICKS(10000));
    return ret;
}

int32_t bh1750_i2c_write(const uint8_t *reg_data, int32_t length, void *intf_ptr)
{
    const i2c_master_dev_handle_t *deviceHandle{static_cast<const i2c_master_dev_handle_t *>(intf_ptr)};
    esp_err_t ret = i2c_master_transmit(*deviceHandle, reg_data, length, pdMS_TO_TICKS(10000));
    return ret;
}

void task_sensor(void *arg)
{
    ESP_LOGI(TAG, "Starting sensor task");
    const SensorTaskInterface *sensorTaskInterface{static_cast<SensorTaskInterface *>(arg)};

#if SOC_LP_CORE_SUPPORTED && CONFIG_ULP_COPROC_TYPE_LP_CORE
    while (true)
    {
        if (ulp_synchronisation)
        {
            SensorData sensorData{};

            sensorData.m_illuminance = ulp_illuminance;
            sensorData.m_humidity = ulp_humidity;
            sensorData.m_temperature = ulp_temperature;
            sensorData.m_pressure = ulp_pressure;

            ESP_LOGI(TAG, "%ld.%ld °C %ld%% %ld hPa %u lx", sensorData.m_temperature / 100, sensorData.m_temperature % 100, sensorData.m_humidity, sensorData.m_pressure / 100, sensorData.m_illuminance);

            QueueValueType queueData{sensorData};
            if (xQueueSend(sensorTaskInterface->m_measurementQueue_out, &queueData, portMAX_DELAY) != pdPASS)
            {
                ESP_LOGE(TAG, "Failed to send measurement data to queue");
            }
        }

        vTaskDelay(pdMS_TO_TICKS(1000 * CONFIG_MEASUREMENT_INTERVAL_SECONDS));
    }
#else

    i2c_master_bus_handle_t i2cBusHandle{initMasterI2C()};
    i2c_master_dev_handle_t i2cTempPressSensorHandle{initTempPressureI2CSlave(i2cBusHandle)};
    i2c_master_dev_handle_t i2cIlluminanceSensorHandle(initIlluminanceI2CSlave(i2cBusHandle));

    BH1750 bh1750{bh1750_i2c_read, bh1750_i2c_write, static_cast<void *>(&i2cIlluminanceSensorHandle)};

    bme280_dev bme280{};
    bme280.intf = BME280_I2C_INTF;
    bme280.intf_ptr = static_cast<void *>(&i2cTempPressSensorHandle);
    bme280.read = bme280_i2c_read;
    bme280.write = bme280_i2c_write;
    bme280.delay_ms = bme280_delay_ms;

    int8_t rslt = bme280_init(&bme280);
    bme280_error_codes_print_result(rslt);

    /* Always read the current settings before writing, especially when all the configuration is not modified */
    bme280_settings settings{};
    rslt = bme280_get_sensor_settings(&settings, &bme280);
    bme280_error_codes_print_result(rslt);

    /* Configuring the over-sampling mode, filter coefficient and output data rate */
    /* Overwrite the desired settings */
    settings.filter = BME280_FILTER_COEFF_OFF;
    settings.osr_h = BME280_OVERSAMPLING_1X;
    settings.osr_t = BME280_OVERSAMPLING_1X;
    settings.osr_p = BME280_OVERSAMPLING_1X;
    settings.standby_time = BME280_STANDBY_TIME_0_5_MS;

    rslt = bme280_set_sensor_settings(BME280_SEL_ALL_SETTINGS, &settings, &bme280);
    bme280_error_codes_print_result(rslt);

    constexpr BH1750::Resolution illuminanceResolution{BH1750::Resolution::medium};
    const uint32_t illuminanceMeasurementTimeMs{bh1750.getMeasurementTimeMs(illuminanceResolution)};

    while (true)
    {
        ESP_ERROR_CHECK(bh1750.powerOnMode(true));
        ESP_ERROR_CHECK(bh1750.setMeasurementMode(illuminanceResolution, true));
        int64_t illuminanceStartTimeUs{esp_timer_get_time()};

        /* Set forced power mode */
        rslt = bme280_set_sensor_mode(BME280_POWERMODE_FORCED, &bme280);
        bme280_error_codes_print_result(rslt);

        /* Calculate measurement time in microseconds */
        uint32_t meas_time_us{0U};
        rslt = bme280_cal_meas_delay(&meas_time_us, &settings);
        bme280_error_codes_print_result(rslt);
        ets_delay_us(meas_time_us);

        /* Read compensated data */
        bme280_data measurement{};
        rslt = bme280_get_sensor_data(BME280_ALL, &measurement, &bme280);
        bme280_error_codes_print_result(rslt);

        SensorData sensorData{};

#ifndef BME280_DOUBLE_ENABLE
        sensorData.m_temperature = measurement.temperature;
        sensorData.m_pressure = measurement.pressure / 100;
        sensorData.m_humidity = measurement.humidity / 1000;
#else
        sensorData.m_temperature = measurement.temperature;
        sensorData.m_pressure = measurement.pressure;
        sensorData.m_humidity = measurement.humidity;
#endif

        int64_t illuminanceEndTimeUs{esp_timer_get_time()};
        int64_t illuminancePassedTimeMs{(illuminanceEndTimeUs - illuminanceStartTimeUs) / 1000};
        if (illuminancePassedTimeMs < illuminanceMeasurementTimeMs)
        {
            vTaskDelay(pdMS_TO_TICKS(illuminanceMeasurementTimeMs - illuminancePassedTimeMs));
        }

        ESP_ERROR_CHECK(bh1750.readMeasurement(sensorData.m_illuminance));

        ESP_LOGI(TAG, "%ld.%ld °C %ld%% %ld hPa %u lx", sensorData.m_temperature / 100, sensorData.m_temperature % 100, sensorData.m_humidity, sensorData.m_pressure / 100, sensorData.m_illuminance);

        QueueValueType queueData{sensorData};
        if (xQueueSend(sensorTaskInterface->m_measurementQueue_out, &queueData, portMAX_DELAY) != pdPASS)
        {
            ESP_LOGE(TAG, "Failed to send measurement data to queue");
        }

        ESP_LOGI(TAG, "Free stack: %u", uxTaskGetStackHighWaterMark(nullptr));
        vTaskDelay(pdMS_TO_TICKS(1000 * CONFIG_MEASUREMENT_INTERVAL_SECONDS));
    }
#endif
}