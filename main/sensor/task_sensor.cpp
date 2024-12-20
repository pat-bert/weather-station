#include "task_sensor.hpp"

#include "bmp280/bmp2.h"

#include "esp_log.h"
#include "esp_timer.h"

#include "driver/i2c_master.h"

#include "rom/ets_sys.h"

#include <cstring>

static const char TAG[] = "bmp280";

void initI2C(i2c_master_dev_handle_t &deviceHandle)
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

    ESP_LOGI(TAG, "Adding I2C device with address %#07x to bus", BMP2_I2C_ADDR_PRIM);
    i2c_device_config_t deviceConfig{};
    deviceConfig.scl_speed_hz = CONFIG_BMP_I2C_CLOCK_KHZ * 1000U;
    deviceConfig.device_address = BMP2_I2C_ADDR_PRIM;
    deviceConfig.dev_addr_length = I2C_ADDR_BIT_LEN_7;

    ESP_ERROR_CHECK(i2c_master_bus_add_device(busHandle, &deviceConfig, &deviceHandle));
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
    static const char tag[] = "bmp280";
    if (rslt != BMP2_OK)
    {
        switch (rslt)
        {
        case BMP2_E_NULL_PTR:
            ESP_LOGE(tag, "Error [%d] : Null pointer error.", rslt);
            ESP_LOGE(
                tag, "It occurs when the user tries to assign value (not address) to a pointer, which has been initialized to NULL.\r\n");
            break;
        case BMP2_E_COM_FAIL:
            ESP_LOGE(tag, "Error [%d] : Communication failure error.", rslt);
            ESP_LOGE(
                tag, "It occurs due to read/write operation failure and also due to power failure during communication\r\n");
            break;
        case BMP2_E_INVALID_LEN:
            ESP_LOGE(tag, "Error [%d] : Invalid length error.", rslt);
            ESP_LOGE(tag, "Occurs when length of data to be written is zero\n");
            break;
        case BMP2_E_DEV_NOT_FOUND:
            ESP_LOGE(tag, "Error [%d] : Device not found error. It occurs when the device chip id is incorrectly read\r\n",
                     rslt);
            break;
        case BMP2_E_UNCOMP_TEMP_RANGE:
            ESP_LOGE(tag, "Error [%d] : Uncompensated temperature data not in valid range error.", rslt);
            break;
        case BMP2_E_UNCOMP_PRESS_RANGE:
            ESP_LOGE(tag, "Error [%d] : Uncompensated pressure data not in valid range error.", rslt);
            break;
        case BMP2_E_UNCOMP_TEMP_AND_PRESS_RANGE:
            ESP_LOGE(
                tag, "Error [%d] : Uncompensated pressure data and uncompensated temperature data are not in valid range error.",
                rslt);
            break;
        default:
            ESP_LOGE(tag, "Error [%d] : Unknown error code\r\n", rslt);
            break;
        }
    }
}

void task_sensor(void *pvParameters)
{
    ESP_LOGI(TAG, "Starting sensor task");

    i2c_master_dev_handle_t deviceHandle{};
    initI2C(deviceHandle);

    const SensorTaskInterface *sensorTaskInterface{static_cast<SensorTaskInterface *>(pvParameters)};

    bmp2_dev bmp280{};
    bmp280.intf = BMP2_I2C_INTF;
    bmp280.intf_ptr = static_cast<void *>(&deviceHandle);
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
        bmp2_data sensorData{};
        rslt = bmp2_get_sensor_data(&sensorData, &bmp280);
        bmp2_error_codes_print_result(rslt);
        ESP_LOGI(TAG, "%.2f °C %.2f Pa", sensorData.temperature, sensorData.pressure);

        if (xQueueSend(sensorTaskInterface->m_measurementQueue_out, &sensorData, portMAX_DELAY) != pdPASS)
        {
            ESP_LOGE(TAG, "Failed to send measurement data to queue");
        }

        vTaskDelay(pdMS_TO_TICKS(1000 * CONFIG_MEASUREMENT_INTERVAL_SECONDS));
    }
}