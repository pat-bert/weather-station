#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/spi_master.h"

#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "rom/ets_sys.h"

#include "bmp280/bmp2.h"
#include "st7735/esp_lcd_panel_custom_vendor.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "lvgl.h"

// I2C common protocol defines
#define ACK_CHECK_EN true   /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS false /*!< I2C master will not check ack from slave */

// I2C Pins
#define I2C_SDA_PIN GPIO_NUM_21
#define I2C_SCL_PIN GPIO_NUM_22
#define BMP_I2C_PORT I2C_NUM_0

// BMP280 Settings
#define BMP2_32BIT_COMPENSATION // needs to be defined before bmp include !!!
#define MEASUREMENT_REFRESH_S (10)

// SPI Settings
#define LCD_HOST VSPI_HOST
#define LCD_SPI_CLOCK_HZ (60 * 1000 * 1000) // Datasheet specifies 66 ns -> 15 MHz but this is running fine, stops working @80 MHz which
                                            // is maximum of ESP32 via IO_MUX Pins
#define LCD_SPI_TRADITIONAL_MODE (0)        // CPOL = 0, CPHA = 0

#define PIN_NUM_LCD_SCLK GPIO_NUM_18
#define PIN_NUM_LCD_MOSI GPIO_NUM_23
#define PIN_NUM_LCD_CS GPIO_NUM_5
#define PIN_NUM_LCD_DC GPIO_NUM_26
#define PIN_NUM_LCD_RST GPIO_NUM_27

// ST7735 Settings
#define LCD_H_RES (160)
#define LCD_V_RES (128)
#define LCD_CMD_BITS 8
#define LCD_PARAM_BITS 8

// LVGL Settings
#define LVGL_TICK_PERIOD_MS 2
#define LVGL_TASK_MAX_DELAY_MS 500
#define LVGL_TASK_MIN_DELAY_MS 1
#define LVGL_SCREEN_DIVIDER (10)
#define LVGL_BUFFER_ELEMENTS ((LCD_H_RES * LCD_V_RES) / LVGL_SCREEN_DIVIDER)

// UI Settings
#define UI_HORIZONTAL_PADDING (2)
#define UI_VERTICAL_PADDING (2)

#define MIN_TEMPERATURE_C (15)
#define MAX_TEMPERATURE_C (35)
#define TEMPERATURE_SCALING_FACTOR (10)

#define MIN_PRESSURE_HPA (700)
#define MAX_PRESSURE_HPA (1100)
#define PRESSURE_TICKS_HPA (25)
#define PRESSURE_SCALING_DIVISOR (100)

// FreeRTOS
#define STACK_SIZE_SENSOR_TASK 2048
#define LVGL_TASK_PRIORITY 2
#define LVGL_TASK_STACK_SIZE (4 * 1024)

static SemaphoreHandle_t lvgl_mux = NULL;

struct SensorTaskInterface
{
    QueueHandle_t m_measurementQueue_out;
};

struct UiTaskInterface
{
    QueueHandle_t m_measurementQueue_in;
    lv_disp_t *m_disp;
    lv_obj_t *m_pressureMeter;
    lv_meter_indicator_t *m_indic;
    lv_obj_t *m_temperatureBar;
    lv_obj_t *m_temperatureLabel;
};

void lvgl_create_ui(UiTaskInterface *uiTaskInterface)
{
    lv_disp_t *disp = uiTaskInterface->m_disp;
    lv_obj_t *scr = lv_disp_get_scr_act(disp);

    static lv_style_t styleLabel;
    lv_style_init(&styleLabel);
    lv_style_set_text_font(&styleLabel, &lv_font_montserrat_8);

    // Pressure Meter
    uiTaskInterface->m_pressureMeter = lv_meter_create(scr);
    lv_obj_set_align(uiTaskInterface->m_pressureMeter, LV_ALIGN_RIGHT_MID);

    constexpr lv_coord_t pressureMeterSize{100};
    lv_obj_set_size(uiTaskInterface->m_pressureMeter, pressureMeterSize, pressureMeterSize);
    lv_obj_add_style(uiTaskInterface->m_pressureMeter, &styleLabel, LV_PART_TICKS);

    // Add a scale first
    lv_meter_scale_t *scale = lv_meter_add_scale(uiTaskInterface->m_pressureMeter);

    constexpr uint16_t tickCount{((MAX_PRESSURE_HPA - MIN_PRESSURE_HPA) / PRESSURE_TICKS_HPA) + 1};
    constexpr uint16_t lineWidthMinor{2U};
    constexpr uint16_t lineLengthMinor{5U};
    constexpr uint16_t nthMajor{4U};
    constexpr uint16_t lineWidthMajor{4U};
    constexpr uint16_t lineLengthMajor{10U};
    constexpr int16_t labelGap{10};
    constexpr uint32_t angleRange{270};
    constexpr uint32_t rotation{135};

    lv_meter_set_scale_ticks(uiTaskInterface->m_pressureMeter, scale, tickCount, lineWidthMinor, lineLengthMinor, lv_palette_main(LV_PALETTE_GREY));
    lv_meter_set_scale_major_ticks(uiTaskInterface->m_pressureMeter, scale, nthMajor, lineWidthMajor, lineLengthMajor, lv_color_black(), labelGap);
    lv_meter_set_scale_range(uiTaskInterface->m_pressureMeter, scale, MIN_PRESSURE_HPA, MAX_PRESSURE_HPA, angleRange, rotation);

    // Add a blue arc to the start
    uiTaskInterface->m_indic = lv_meter_add_arc(uiTaskInterface->m_pressureMeter, scale, 3, lv_palette_main(LV_PALETTE_BLUE), 0);
    lv_meter_set_indicator_start_value(uiTaskInterface->m_pressureMeter, uiTaskInterface->m_indic, 0);
    lv_meter_set_indicator_end_value(uiTaskInterface->m_pressureMeter, uiTaskInterface->m_indic, 20);

    // Make the tick lines blue at the start of the scale
    uiTaskInterface->m_indic = lv_meter_add_scale_lines(uiTaskInterface->m_pressureMeter, scale, lv_palette_main(LV_PALETTE_BLUE), lv_palette_main(LV_PALETTE_BLUE), false, 0);
    lv_meter_set_indicator_start_value(uiTaskInterface->m_pressureMeter, uiTaskInterface->m_indic, 0);
    lv_meter_set_indicator_end_value(uiTaskInterface->m_pressureMeter, uiTaskInterface->m_indic, 20);

    // Add a red arc to the end
    uiTaskInterface->m_indic = lv_meter_add_arc(uiTaskInterface->m_pressureMeter, scale, 3, lv_palette_main(LV_PALETTE_RED), 0);
    lv_meter_set_indicator_start_value(uiTaskInterface->m_pressureMeter, uiTaskInterface->m_indic, 80);
    lv_meter_set_indicator_end_value(uiTaskInterface->m_pressureMeter, uiTaskInterface->m_indic, 100);

    // Make the tick lines red at the end of the scale
    uiTaskInterface->m_indic = lv_meter_add_scale_lines(uiTaskInterface->m_pressureMeter, scale, lv_palette_main(LV_PALETTE_RED), lv_palette_main(LV_PALETTE_RED), false, 0);
    lv_meter_set_indicator_start_value(uiTaskInterface->m_pressureMeter, uiTaskInterface->m_indic, 80);
    lv_meter_set_indicator_end_value(uiTaskInterface->m_pressureMeter, uiTaskInterface->m_indic, 100);

    // Add a needle line indicator
    uiTaskInterface->m_indic = lv_meter_add_needle_line(uiTaskInterface->m_pressureMeter, scale, 4, lv_palette_main(LV_PALETTE_GREY), -10);

    // Temperature bar
    lv_obj_t *temperatureContainer = lv_obj_create(scr);
    lv_obj_set_align(temperatureContainer, LV_ALIGN_LEFT_MID);
    lv_obj_set_size(temperatureContainer, 40, LCD_V_RES);

    static lv_style_t styleIndic;
    lv_style_init(&styleIndic);
    lv_style_set_bg_opa(&styleIndic, LV_OPA_COVER);
    lv_style_set_bg_color(&styleIndic, lv_palette_main(LV_PALETTE_RED));
    lv_style_set_bg_grad_color(&styleIndic, lv_palette_main(LV_PALETTE_BLUE));
    lv_style_set_bg_grad_dir(&styleIndic, LV_GRAD_DIR_VER);

    uiTaskInterface->m_temperatureBar = lv_bar_create(temperatureContainer);
    lv_obj_add_style(uiTaskInterface->m_temperatureBar, &styleIndic, LV_PART_INDICATOR);
    lv_obj_set_size(uiTaskInterface->m_temperatureBar, 20, LCD_V_RES * (2.0 / 3.0));
    lv_obj_set_align(uiTaskInterface->m_temperatureBar, LV_ALIGN_TOP_MID);
    lv_bar_set_range(uiTaskInterface->m_temperatureBar, TEMPERATURE_SCALING_FACTOR * MIN_TEMPERATURE_C, TEMPERATURE_SCALING_FACTOR * MAX_TEMPERATURE_C);

    uiTaskInterface->m_temperatureLabel = lv_label_create(temperatureContainer);
    lv_obj_add_style(uiTaskInterface->m_temperatureLabel, &styleLabel, LV_PART_MAIN);
    lv_obj_set_align(uiTaskInterface->m_temperatureLabel, LV_ALIGN_BOTTOM_MID);
}

static bool notify_lvgl_flush_ready(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_io_event_data_t *edata, void *user_ctx)
{
    lv_disp_drv_t *disp_driver = (lv_disp_drv_t *)user_ctx;
    lv_disp_flush_ready(disp_driver);
    return false;
}

static void lvgl_flush_cb(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_map)
{
    esp_lcd_panel_handle_t panel_handle = (esp_lcd_panel_handle_t)drv->user_data;
    int offsetx1 = area->x1;
    int offsetx2 = area->x2;
    int offsety1 = area->y1;
    int offsety2 = area->y2;
    // copy a buffer's content to a specific area of the display
    esp_lcd_panel_draw_bitmap(panel_handle, offsetx1, offsety1, offsetx2 + 1, offsety2 + 1, color_map);
}

static void increase_lvgl_tick(void *arg)
{
    /* Tell LVGL how many milliseconds has elapsed */
    lv_tick_inc(LVGL_TICK_PERIOD_MS);
}

bool lvgl_lock(int timeout_ms)
{
    // Convert timeout in milliseconds to FreeRTOS ticks
    // If `timeout_ms` is set to -1, the program will block until the condition is met
    const TickType_t timeout_ticks = (timeout_ms == -1) ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);
    return xSemaphoreTakeRecursive(lvgl_mux, timeout_ticks) == pdTRUE;
}

void lvgl_unlock(void)
{
    xSemaphoreGiveRecursive(lvgl_mux);
}

struct I2CInterfaceData
{
    uint8_t i2c_addr;
    i2c_port_t i2c_num;
};

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
    buscfg.sclk_io_num = PIN_NUM_LCD_SCLK;
    buscfg.mosi_io_num = PIN_NUM_LCD_MOSI;
    buscfg.miso_io_num = -1;
    buscfg.quadwp_io_num = -1;                                        // Quad SPI LCD driver is not yet supported
    buscfg.quadhd_io_num = -1;                                        // Quad SPI LCD driver is not yet supported
    buscfg.max_transfer_sz = LVGL_BUFFER_ELEMENTS * sizeof(uint16_t); // transfer x lines of pixels (assume pixel is RGB565) at most in one SPI transaction

    ESP_ERROR_CHECK(spi_bus_initialize(LCD_HOST, &buscfg, SPI_DMA_CH_AUTO)); // Enable the DMA feature
}

esp_lcd_panel_io_handle_t initPanelIO(lv_disp_drv_t *disp_drv)
{
    initSPI();

    esp_lcd_panel_io_handle_t io_handle = nullptr;
    esp_lcd_panel_io_spi_config_t io_config{};
    io_config.dc_gpio_num = PIN_NUM_LCD_DC;
    io_config.cs_gpio_num = PIN_NUM_LCD_CS;
    io_config.pclk_hz = LCD_SPI_CLOCK_HZ;
    io_config.lcd_cmd_bits = LCD_CMD_BITS;
    io_config.lcd_param_bits = LCD_PARAM_BITS;
    io_config.spi_mode = LCD_SPI_TRADITIONAL_MODE;
    io_config.flags.sio_mode = 1; // only MOSI, no MISO
    io_config.trans_queue_depth = LVGL_SCREEN_DIVIDER;
    io_config.on_color_trans_done = notify_lvgl_flush_ready;
    io_config.user_ctx = disp_drv;

    // Attach the LCD to the SPI bus
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_HOST, &io_config, &io_handle));

    return io_handle;
}

esp_lcd_panel_handle_t initPanel(lv_disp_drv_t *disp_drv)
{
    esp_lcd_panel_io_handle_t io_handle = initPanelIO(disp_drv);
    esp_lcd_panel_handle_t panel_handle = nullptr;
    esp_lcd_panel_dev_config_t panel_config{};
    panel_config.reset_gpio_num = PIN_NUM_LCD_RST;
    panel_config.rgb_endian = LCD_RGB_ENDIAN_RGB;
    panel_config.bits_per_pixel = 16;

    // Create LCD panel handle for ST7735, with the SPI IO device handle
    ESP_ERROR_CHECK(esp_lcd_new_panel_st7735(io_handle, &panel_config, &panel_handle));

    return panel_handle;
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

void task_ui(void *pvParameters)
{
    static const char tag[] = "UI";
    UiTaskInterface *uiTaskInterface{static_cast<UiTaskInterface *>(pvParameters)};

    // Lock the mutex due to the LVGL APIs are not thread-safe
    if (lvgl_lock(-1))
    {
        lvgl_create_ui(uiTaskInterface);
        // Release the mutex
        lvgl_unlock();
    }

    while (true)
    {
        bmp2_data sensorData{};
        if (xQueueReceive(uiTaskInterface->m_measurementQueue_in, &sensorData, portMAX_DELAY) == pdPASS)
        {
            if (lvgl_lock(-1))
            {
                ESP_LOGI(tag, "%.2f °C %.2f Pa", sensorData.temperature, sensorData.pressure);
                lv_label_set_text_fmt(uiTaskInterface->m_temperatureLabel, "%.1f °C", sensorData.temperature);
                lv_bar_set_value(uiTaskInterface->m_temperatureBar, sensorData.temperature * TEMPERATURE_SCALING_FACTOR, LV_ANIM_OFF);
                lv_meter_set_indicator_end_value(uiTaskInterface->m_pressureMeter, uiTaskInterface->m_indic, sensorData.pressure / PRESSURE_SCALING_DIVISOR);
                lvgl_unlock();
            }
        }
    }
}

void task_sensor(void *pvParameters)
{
    const SensorTaskInterface *sensorTaskInterface{static_cast<SensorTaskInterface *>(pvParameters)};

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
        ESP_LOGI("BMP280", "%.2f °C %.2f Pa", sensorData.temperature, sensorData.pressure);

        if (xQueueSend(sensorTaskInterface->m_measurementQueue_out, &sensorData, portMAX_DELAY) != pdPASS)
        {
            ESP_LOGE("BMP280", "Failed to send measurement data to queue");
        }

        vTaskDelay(pdMS_TO_TICKS(1000 * MEASUREMENT_REFRESH_S));
    }
}

static void lvgl_port_task(void *arg)
{
    ESP_LOGI("LVGL", "Starting LVGL task");
    uint32_t task_delay_ms = LVGL_TASK_MAX_DELAY_MS;

    while (true)
    {
        // Lock the mutex due to the LVGL APIs are not thread-safe
        if (lvgl_lock(-1))
        {
            task_delay_ms = lv_timer_handler();

            // Release the mutex
            lvgl_unlock();
        }
        if (task_delay_ms > LVGL_TASK_MAX_DELAY_MS)
        {
            task_delay_ms = LVGL_TASK_MAX_DELAY_MS;
        }
        else if (task_delay_ms < LVGL_TASK_MIN_DELAY_MS)
        {
            task_delay_ms = LVGL_TASK_MIN_DELAY_MS;
        }
        vTaskDelay(pdMS_TO_TICKS(task_delay_ms));
    }
}

extern "C" void app_main(void)
{
    QueueHandle_t measurementQueue{xQueueCreate(5, sizeof(bmp2_data))};
    configASSERT(measurementQueue);

    TaskHandle_t xHandleSensor{nullptr};
    SensorTaskInterface sensorTaskInterface{measurementQueue};
    xTaskCreate(task_sensor, "Sensor", STACK_SIZE_SENSOR_TASK, static_cast<void *>(&sensorTaskInterface), tskIDLE_PRIORITY + 1, &xHandleSensor);
    configASSERT(xHandleSensor);

    static lv_disp_draw_buf_t disp_buf; // contains internal graphic buffer(s) called draw buffer(s)
    static lv_disp_drv_t disp_drv;      // contains callback functions

    ESP_LOGI("MAIN", "Install ST7735 panel driver");
    esp_lcd_panel_handle_t panel_handle = initPanel(&disp_drv);

    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_mirror(panel_handle, true, false));
    ESP_ERROR_CHECK(esp_lcd_panel_swap_xy(panel_handle, true));

    // user can flush pre-defined pattern to the screen before we turn on the screen or backlight
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));

    ESP_LOGI("MAIN", "Initialize LVGL library");
    lv_init();
    // alloc draw buffers used by LVGL
    // it's recommended to choose the size of the draw buffer(s) to be at least 1/10 screen sized
    lv_color_t *buf1 = (lv_color_t *)heap_caps_malloc(LVGL_BUFFER_ELEMENTS * sizeof(lv_color_t), MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    assert(buf1);
    lv_color_t *buf2 = (lv_color_t *)heap_caps_malloc(LVGL_BUFFER_ELEMENTS * sizeof(lv_color_t), MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    assert(buf2);
    // initialize LVGL draw buffers
    lv_disp_draw_buf_init(&disp_buf, buf1, buf2, LVGL_BUFFER_ELEMENTS);

    ESP_LOGI("MAIN", "Register display driver to LVGL");
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = LCD_H_RES;
    disp_drv.ver_res = LCD_V_RES;
    disp_drv.flush_cb = lvgl_flush_cb;
    disp_drv.draw_buf = &disp_buf;
    disp_drv.user_data = panel_handle;
    lv_disp_t *disp = lv_disp_drv_register(&disp_drv);

    ESP_LOGI("MAIN", "Install LVGL tick timer");
    // Tick interface for LVGL (using esp_timer to generate 2ms periodic event)
    esp_timer_create_args_t lvgl_tick_timer_args{};
    lvgl_tick_timer_args.callback = &increase_lvgl_tick;
    lvgl_tick_timer_args.name = "lvgl_tick";

    esp_timer_handle_t lvgl_tick_timer = NULL;
    ESP_ERROR_CHECK(esp_timer_create(&lvgl_tick_timer_args, &lvgl_tick_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(lvgl_tick_timer, LVGL_TICK_PERIOD_MS * 1000));

    lvgl_mux = xSemaphoreCreateRecursiveMutex();
    assert(lvgl_mux);

    ESP_LOGI("MAIN", "Create LVGL task");
    TaskHandle_t xHandleDisplay = nullptr;
    xTaskCreate(lvgl_port_task, "Display", LVGL_TASK_STACK_SIZE, nullptr, LVGL_TASK_PRIORITY, &xHandleDisplay);
    configASSERT(xHandleDisplay);

    UiTaskInterface uiTaskInterface{measurementQueue, disp};
    TaskHandle_t xHandleUi{nullptr};
    xTaskCreate(task_ui, "UI", STACK_SIZE_SENSOR_TASK, static_cast<void *>(&uiTaskInterface), tskIDLE_PRIORITY + 1, &xHandleUi);
    configASSERT(xHandleUi);

    vTaskSuspend(nullptr);
}