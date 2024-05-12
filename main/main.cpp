#include "esp_log.h"

#include "driver/i2c.h"
#include "driver/gpio.h"

#include "rom/ets_sys.h"

#include "bmp280/bmp2.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// I2C common protocol defines
#define ACK_CHECK_EN true   /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS false /*!< I2C master will not check ack from slave */

#define STACK_SIZE 2048
#define BMP2_32BIT_COMPENSATION

struct I2CInterfaceData
{
    uint8_t i2c_addr;
    i2c_port_t i2c_num;
};

void initI2C()
{
    i2c_port_t port{I2C_NUM_0};
    i2c_config_t config{};
    config.mode = I2C_MODE_MASTER;
    config.sda_io_num = GPIO_NUM_21;
    config.scl_io_num = GPIO_NUM_22;
    config.sda_pullup_en = false;
    config.scl_pullup_en = false;
    config.master.clk_speed = 100000U;
    config.clk_flags = I2C_SCLK_SRC_FLAG_FOR_NOMAL;

    ESP_ERROR_CHECK(i2c_param_config(port, &config));

    // Not needed in master mode
    size_t slv_rx_buf_len{0};
    size_t slv_tx_buf_len{0};

    int intr_alloc_flags{0};
    ESP_ERROR_CHECK(i2c_driver_install(port, I2C_MODE_MASTER, slv_rx_buf_len, slv_tx_buf_len, intr_alloc_flags));
}

BMP2_INTF_RET_TYPE bmp2_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, const void *intf_ptr)
{
    if (length == 0)
    {
        return BMP2_INTF_RET_SUCCESS;
    }

    const I2CInterfaceData *interface{(I2CInterfaceData *)intf_ptr};
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    // first, send device address (indicating write) & register to be read
    i2c_master_write_byte(cmd, (interface->i2c_addr << 1), ACK_CHECK_EN);
    // send register we want
    i2c_master_write_byte(cmd, reg_addr, ACK_CHECK_EN);
    // Send repeated start
    i2c_master_start(cmd);
    // now send device address (indicating read) & read data
    i2c_master_write_byte(cmd, (interface->i2c_addr << 1) | I2C_MASTER_READ, ACK_CHECK_EN);
    if (length > 1)
    {
        i2c_master_read(cmd, reg_data, length - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, reg_data + length - 1, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(interface->i2c_num, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    return (ESP_OK == ret) ? BMP2_INTF_RET_SUCCESS : -1;
}

BMP2_INTF_RET_TYPE bmp2_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, const void *intf_ptr)
{
    const I2CInterfaceData *interface{(I2CInterfaceData *)intf_ptr};
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    // first, send device address (indicating write) & register to be written
    i2c_master_write_byte(cmd, (interface->i2c_addr << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
    // send register we want
    i2c_master_write_byte(cmd, reg_addr, ACK_CHECK_EN);
    // write the data
    i2c_master_write(cmd, reg_data, length, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(interface->i2c_num, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    return (ESP_OK == ret) ? BMP2_INTF_RET_SUCCESS : -1;
}

void bmp2_delay_us(uint32_t period, void *intf_ptr)
{
    (void)intf_ptr;
    ets_delay_us(period);
}

void bmp2_error_codes_print_result(int8_t rslt)
{
    static const char tag[] = "BMP280";
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

void task_bmp280(void *pvParameters)
{
    initI2C();

    I2CInterfaceData interface{};
    interface.i2c_addr = BMP2_I2C_ADDR_PRIM;
    interface.i2c_num = I2C_NUM_0;

    bmp2_dev bmp280{};
    bmp280.intf = BMP2_I2C_INTF;
    bmp280.intf_ptr = (void *)&interface;
    bmp280.read = bmp2_i2c_read;
    bmp280.write = bmp2_i2c_write;
    bmp280.delay_us = bmp2_delay_us;

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

    bmp2_data sensorData{};

    while (true)
    {
        /* Set normal power mode */
        rslt = bmp2_set_power_mode(BMP2_POWERMODE_FORCED, &conf, &bmp280);
        bmp2_error_codes_print_result(rslt);

        /* Calculate measurement time in microseconds */
        uint32_t meas_time_us;
        rslt = bmp2_compute_meas_time(&meas_time_us, &conf, &bmp280);
        bmp2_error_codes_print_result(rslt);

        uint32_t meas_time_ms = (meas_time_us / 1000) + (meas_time_us % 1000 > 0);
        vTaskDelay(pdMS_TO_TICKS(meas_time_ms));

        /* Read compensated data */
        rslt = bmp2_get_sensor_data(&sensorData, &bmp280);
        bmp2_error_codes_print_result(rslt);
        ESP_LOGI("BMP280", "%.2f °C %.2f Pa", sensorData.temperature, sensorData.pressure);

        vTaskDelay(pdMS_TO_TICKS(1000 * 60));
    }
}

void task_display(void *pvParameters)
{
    while (true)
    {
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

extern "C" void app_main(void)
{
    TaskHandle_t xHandleBmp280 = nullptr;
    xTaskCreate(task_bmp280, "BMP280", STACK_SIZE, nullptr, tskIDLE_PRIORITY + 1, &xHandleBmp280);
    configASSERT(xHandleBmp280);

    TaskHandle_t xHandleDisplay = nullptr;
    xTaskCreate(task_display, "Display", STACK_SIZE, nullptr, tskIDLE_PRIORITY + 1, &xHandleDisplay);
    configASSERT(xHandleDisplay);

    vTaskSuspend(nullptr);
}