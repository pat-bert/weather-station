#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/spi_master.h"

#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"

#include "esp_log.h"
#include "rom/ets_sys.h"

#include "bmp280/bmp2.h"
#include "st7735/esp_lcd_panel_custom_vendor.h"
// #include "esp_lcd_panel_vendor.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// I2C common protocol defines
#define ACK_CHECK_EN true   /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS false /*!< I2C master will not check ack from slave */

#define I2C_SDA_PIN GPIO_NUM_21
#define I2C_SCL_PIN GPIO_NUM_22
#define BMP_I2C_PORT I2C_NUM_0
#define BMP2_32BIT_COMPENSATION

#define LCD_PIN_NUM_SCLK GPIO_NUM_18
#define LCD_PIN_NUM_MOSI GPIO_NUM_23
#define LCD_PIN_NUM_CS GPIO_NUM_25
#define LCD_PIN_NUM_DC GPIO_NUM_26
#define LCD_PIN_NUM_RST GPIO_NUM_27

#define LCD_V_RES 128
#define LCD_H_RES 160
#define LCD_PARALLEL_LINES 80
#define LCD_PIXEL_CLOCK_HZ (20 * 1000 * 1000)
#define LCD_CMD_BITS 8
#define LCD_PARAM_BITS 8
#define LCD_HOST SPI3_HOST

#define STACK_SIZE_SENSOR_TASK 2048
#define STACK_SIZE_DISPLAY_TASK 4096

struct I2CInterfaceData
{
    uint8_t i2c_addr;
    i2c_port_t i2c_num;
};

uint16_t RGB888ToRGB565(uint8_t r, uint8_t g, uint8_t b)
{
    return (((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b & 0xF8));
}

uint16_t swapBytes(uint16_t bytes)
{
    return (bytes << 8) | (bytes >> 8);
}

void initI2C()
{
    i2c_port_t port{BMP_I2C_PORT};
    i2c_config_t config{};
    config.mode = I2C_MODE_MASTER;
    config.sda_io_num = I2C_SDA_PIN;
    config.scl_io_num = I2C_SCL_PIN;
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

void initSPI()
{
    spi_bus_config_t buscfg{};
    buscfg.sclk_io_num = LCD_PIN_NUM_SCLK;
    buscfg.mosi_io_num = LCD_PIN_NUM_MOSI;
    buscfg.miso_io_num = -1;
    buscfg.quadwp_io_num = -1;                                         // Quad SPI LCD driver is not yet supported
    buscfg.quadhd_io_num = -1;                                         // Quad SPI LCD driver is not yet supported
    buscfg.max_transfer_sz = LCD_H_RES * LCD_V_RES * sizeof(uint16_t); // transfer 80 lines of pixels (assume pixel is RGB565) at most in one SPI transaction

    ESP_ERROR_CHECK(spi_bus_initialize(LCD_HOST, &buscfg, SPI_DMA_CH_AUTO)); // Enable the DMA feature
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

void task_sensor(void *pvParameters)
{
    initI2C();

    I2CInterfaceData interface{};
    interface.i2c_addr = BMP2_I2C_ADDR_PRIM;
    interface.i2c_num = BMP_I2C_PORT;

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
    initSPI();

    esp_lcd_panel_io_handle_t io_handle = nullptr;
    esp_lcd_panel_io_spi_config_t io_config{};
    io_config.dc_gpio_num = LCD_PIN_NUM_DC;
    io_config.cs_gpio_num = LCD_PIN_NUM_CS;
    io_config.pclk_hz = LCD_PIXEL_CLOCK_HZ;
    io_config.lcd_cmd_bits = LCD_CMD_BITS;
    io_config.lcd_param_bits = LCD_PARAM_BITS;
    io_config.spi_mode = 0;       // CPOL = CPHA = 0
    io_config.flags.sio_mode = 1; // only MOSI, no MISO
    io_config.trans_queue_depth = 10;

    // Attach the LCD to the SPI bus
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_HOST, &io_config, &io_handle));

    esp_lcd_panel_handle_t panel_handle = nullptr;
    esp_lcd_panel_dev_config_t panel_config{};
    panel_config.reset_gpio_num = LCD_PIN_NUM_RST;
    panel_config.rgb_endian = LCD_RGB_ENDIAN_RGB;
    panel_config.bits_per_pixel = 16;

    // Create LCD panel handle for ST7735, with the SPI IO device handle
    ESP_ERROR_CHECK(esp_lcd_new_panel_st7735(io_handle, &panel_config, &panel_handle));

    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));
    vTaskDelay(pdMS_TO_TICKS(100));

    uint16_t *colorData = (uint16_t *)heap_caps_malloc(LCD_H_RES * LCD_V_RES * sizeof(uint16_t), MALLOC_CAP_DMA);
    for (int i = 0; i < LCD_H_RES; ++i)
    {
        for (int j = 0; j < LCD_V_RES; ++j)
        {
            colorData[i * LCD_H_RES + j] = swapBytes(RGB888ToRGB565(255, 0, 0));
        }
    }

    esp_lcd_panel_draw_bitmap(panel_handle, 0, 0, LCD_H_RES, LCD_V_RES, (void *)&colorData);

    while (true)
    {
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

extern "C" void app_main(void)
{
    TaskHandle_t xHandleSensor = nullptr;
    xTaskCreate(task_sensor, "Sensor", STACK_SIZE_SENSOR_TASK, nullptr, tskIDLE_PRIORITY + 1, &xHandleSensor);
    configASSERT(xHandleSensor);

    TaskHandle_t xHandleDisplay = nullptr;
    xTaskCreate(task_display, "Display", STACK_SIZE_DISPLAY_TASK, nullptr, tskIDLE_PRIORITY + 1, &xHandleDisplay);
    configASSERT(xHandleDisplay);

    vTaskSuspend(nullptr);
}