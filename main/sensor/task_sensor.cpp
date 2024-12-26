#include "task_sensor.hpp"

#include "bmp280/bmp2.h"
#include "bh1750/bh1750.hpp"

#include "esp_log.h"
#include "esp_timer.h"

#include "rom/ets_sys.h"

#include <cstring>

static const char TAG[] = "sensor";

i2c_master_bus_handle_t initMasterI2C()
{
    ESP_LOGI(TAG, "Creating I2C Master Bus");
    i2c_master_bus_config_t busConfig{};
    busConfig.i2c_port = CONFIG_BMP_I2C_PORT;
    busConfig.sda_io_num = static_cast<gpio_num_t>(CONFIG_BMP_SDA_GPIO);
    busConfig.scl_io_num = static_cast<gpio_num_t>(CONFIG_BMP_SCL_GPIO);
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
    ESP_LOGI(TAG, "Adding T+P I2C sensor with address %#02x to bus", BMP2_I2C_ADDR_PRIM);
    i2c_device_config_t deviceConfig{};
    deviceConfig.scl_speed_hz = CONFIG_BMP_I2C_CLOCK_KHZ * 1000U;
    deviceConfig.device_address = BMP2_I2C_ADDR_PRIM;
    deviceConfig.dev_addr_length = I2C_ADDR_BIT_LEN_7;

    i2c_master_dev_handle_t deviceHandle{};
    ESP_ERROR_CHECK(i2c_master_bus_add_device(busHandle, &deviceConfig, &deviceHandle));

    return deviceHandle;
}

i2c_master_dev_handle_t initIlluminanceI2CSlave(const i2c_master_bus_handle_t busHandle)
{
    ESP_LOGI(TAG, "Adding illuminance I2C sensor with address %#02x to bus", BH1750_I2C_ADDR_SEC);
    i2c_device_config_t deviceConfig{};
    deviceConfig.scl_speed_hz = CONFIG_BMP_I2C_CLOCK_KHZ * 1000U;
    deviceConfig.device_address = BH1750_I2C_ADDR_SEC;
    deviceConfig.dev_addr_length = I2C_ADDR_BIT_LEN_7;

    i2c_master_dev_handle_t deviceHandle{};
    ESP_ERROR_CHECK(i2c_master_bus_add_device(busHandle, &deviceConfig, &deviceHandle));

    return deviceHandle;
}

BMP2_INTF_RET_TYPE bmp2_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, const void *intf_ptr)
{
    if ((length == 0) || (length > static_cast<uint32_t>(INT32_MAX)))
    {
        return BMP2_INTF_RET_SUCCESS;
    }

    const i2c_master_dev_handle_t *deviceHandle{static_cast<const i2c_master_dev_handle_t *>(intf_ptr)};
    esp_err_t ret = i2c_master_transmit_receive(*deviceHandle, &reg_addr, 1, reg_data, static_cast<int32_t>(length), pdMS_TO_TICKS(10000));
    return (ESP_OK == ret) ? BMP2_INTF_RET_SUCCESS : -1;
}

BMP2_INTF_RET_TYPE bmp2_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, const void *intf_ptr)
{
    if ((length > static_cast<uint32_t>(INT32_MAX)) || (length > BMP2_MAX_LEN))
    {
        return -1;
    }

    uint8_t writeBuffer[BMP2_MAX_LEN + 1];
    writeBuffer[0] = reg_addr;
    std::memcpy(writeBuffer + 1, reg_data, length);

    const i2c_master_dev_handle_t *deviceHandle{static_cast<const i2c_master_dev_handle_t *>(intf_ptr)};
    esp_err_t ret = i2c_master_transmit(*deviceHandle, writeBuffer, static_cast<int32_t>(length + 1), pdMS_TO_TICKS(10000));
    return (ESP_OK == ret) ? BMP2_INTF_RET_SUCCESS : -1;
}

void bmp2_delay_ms(uint32_t period, void *intf_ptr)
{
    (void)intf_ptr;
    vTaskDelay(pdMS_TO_TICKS(period));
}

void bmp2_error_codes_print_result(int8_t rslt)
{
    if (rslt != BMP2_OK)
    {
        switch (rslt)
        {
        case BMP2_E_NULL_PTR:
            ESP_LOGE(TAG, "Nullptr");
            break;
        case BMP2_E_COM_FAIL:
            ESP_LOGE(TAG, "Communication failure");
            break;
        case BMP2_E_INVALID_LEN:
            ESP_LOGE(TAG, "Zero message length");
            break;
        case BMP2_E_DEV_NOT_FOUND:
            ESP_LOGE(TAG, "Device not found");
            break;
        case BMP2_E_UNCOMP_TEMP_RANGE:
            ESP_LOGE(TAG, "Uncompensated temperature data not in valid range");
            break;
        case BMP2_E_UNCOMP_PRESS_RANGE:
            ESP_LOGE(TAG, "Uncompensated pressure data not in valid range");
            break;
        case BMP2_E_UNCOMP_TEMP_AND_PRESS_RANGE:
            ESP_LOGE(TAG, "Uncompensated pressure data and uncompensated temperature data are not in valid range");
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

void task_sensor(void *pvParameters)
{
    ESP_LOGI(TAG, "Starting sensor task");

    const SensorTaskInterface *sensorTaskInterface{static_cast<SensorTaskInterface *>(pvParameters)};

    i2c_master_bus_handle_t i2cBusHandle{initMasterI2C()};
    i2c_master_dev_handle_t i2cTempPressSensorHandle{initTempPressureI2CSlave(i2cBusHandle)};
    i2c_master_dev_handle_t i2cIlluminanceSensorHandle(initIlluminanceI2CSlave(i2cBusHandle));

    BH1750 bh1750{bh1750_i2c_read, bh1750_i2c_write, static_cast<void *>(&i2cIlluminanceSensorHandle)};

    bmp2_dev bmp280{};
    bmp280.intf = BMP2_I2C_INTF;
    bmp280.intf_ptr = static_cast<void *>(&i2cTempPressSensorHandle);
    bmp280.read = bmp2_i2c_read;
    bmp280.write = bmp2_i2c_write;
    bmp280.delay_ms = bmp2_delay_ms;

    int8_t rslt = bmp2_init(&bmp280);
    bmp2_error_codes_print_result(rslt);

    /* Always read the current settings before writing, especially when all the configuration is not modified */
    bmp2_config conf{};
    rslt = bmp2_get_config(&conf, &bmp280);
    bmp2_error_codes_print_result(rslt);

    /* Configuring the over-sampling mode, filter coefficient and output data rate */
    /* Overwrite the desired settings */
    conf.filter = BMP2_FILTER_OFF;
    conf.os_mode = BMP2_OS_MODE_ULTRA_LOW_POWER;

    rslt = bmp2_set_config(&conf, &bmp280);
    bmp2_error_codes_print_result(rslt);

    while (true)
    {
        /* Set forced power mode */
        rslt = bmp2_set_power_mode(BMP2_POWERMODE_FORCED, &conf, &bmp280);
        bmp2_error_codes_print_result(rslt);

        /* Calculate measurement time in microseconds */
        uint32_t meas_time_us;
        rslt = bmp2_compute_meas_time(&meas_time_us, &conf, &bmp280);
        bmp2_error_codes_print_result(rslt);
        ets_delay_us(meas_time_us);

        /* Read compensated data */
        bmp2_data measurementTempAndPressure{};
        rslt = bmp2_get_sensor_data(&measurementTempAndPressure, &bmp280);
        bmp2_error_codes_print_result(rslt);

        SensorData sensorData{};
        sensorData.m_temperature = measurementTempAndPressure.temperature;
        sensorData.m_pressure = measurementTempAndPressure.pressure;

        ESP_ERROR_CHECK(bh1750.powerOnMode(true));
        ESP_ERROR_CHECK(bh1750.setMeasurementMode(BH1750::Resolution::medium, true));
        vTaskDelay(pdMS_TO_TICKS(120U));
        ESP_ERROR_CHECK(bh1750.readMeasurement(sensorData.m_illuminance));

        ESP_LOGI(TAG, "%.2f °C %.2f hPa %u lx", sensorData.m_temperature, sensorData.m_pressure / 100.0, sensorData.m_illuminance);

        if (xQueueSend(sensorTaskInterface->m_measurementQueue_out, &sensorData, portMAX_DELAY) != pdPASS)
        {
            ESP_LOGE(TAG, "Failed to send measurement data to queue");
        }

        ESP_LOGI(TAG, "Free stack: %u", uxTaskGetStackHighWaterMark(nullptr));
        vTaskDelay(pdMS_TO_TICKS(1000 * CONFIG_MEASUREMENT_INTERVAL_SECONDS));
    }
}