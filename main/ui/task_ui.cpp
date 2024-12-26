#include "task_ui.hpp"

#include "interface_sensor.hpp"

#include "st7735/esp_lcd_panel_custom_vendor.h"

#include "driver/spi_master.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "lvgl.h"

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#include <ctime>
#include <iterator>

void lvgl_create_ui(UiTaskInterface *uiTaskInterface)
{
    constexpr lv_coord_t topRibbonHeight{15};
    constexpr lv_coord_t mainAreaHeight{CONFIG_LCD_V_RES - topRibbonHeight - 10};
    constexpr lv_coord_t temperatureContainerWidth{40};
    constexpr lv_coord_t pressureContainerWidth{CONFIG_LCD_H_RES - temperatureContainerWidth - 10};

    static lv_style_t styleLabel;
    lv_style_init(&styleLabel);
    lv_style_set_text_font(&styleLabel, &lv_font_montserrat_8);

    // Grid layout
    static lv_coord_t rowDescriptor[] = {topRibbonHeight, mainAreaHeight, LV_GRID_TEMPLATE_LAST};
    static lv_coord_t columnDescriptor[] = {temperatureContainerWidth, pressureContainerWidth, LV_GRID_TEMPLATE_LAST};

    lv_disp_t *disp = uiTaskInterface->m_disp;
    lv_obj_t *scr = lv_disp_get_scr_act(disp);
    lv_obj_t *grid = lv_obj_create(scr);
    lv_obj_set_style_grid_column_dsc_array(grid, columnDescriptor, 0);
    lv_obj_set_style_grid_row_dsc_array(grid, rowDescriptor, 0);
    lv_obj_set_size(grid, CONFIG_LCD_H_RES, CONFIG_LCD_V_RES);
    lv_obj_center(grid);
    lv_obj_set_layout(grid, LV_LAYOUT_GRID);
    lv_obj_set_style_pad_all(grid, 0, 0);
    lv_obj_set_style_border_width(grid, 0, LV_PART_MAIN);
    lv_obj_set_style_radius(grid, 0, LV_PART_MAIN);

    {
        // Top ribbon
        lv_obj_t *topRibbon = lv_obj_create(grid);
        lv_obj_set_grid_cell(topRibbon, LV_GRID_ALIGN_START, 0, 2, LV_GRID_ALIGN_START, 0, 1);
        lv_obj_set_align(topRibbon, LV_ALIGN_CENTER);
        lv_obj_set_size(topRibbon, CONFIG_LCD_H_RES, topRibbonHeight);
        lv_obj_set_style_pad_all(topRibbon, 0, LV_PART_MAIN);
        lv_obj_set_style_bg_color(topRibbon, lv_palette_darken(LV_PALETTE_BLUE, 3), LV_PART_MAIN);

        // Time label
        lv_obj_t *timeLabel = lv_label_create(topRibbon);
        lv_obj_set_style_text_font(timeLabel, &lv_font_montserrat_8, LV_PART_MAIN);
        lv_obj_set_align(timeLabel, LV_ALIGN_CENTER);
        lv_obj_set_style_text_align(timeLabel, LV_ALIGN_CENTER, LV_PART_MAIN);
        lv_label_set_text(timeLabel, "");
        lv_obj_set_style_pad_all(timeLabel, 0, LV_PART_MAIN);

        uiTaskInterface->m_timeLabel = timeLabel;
    }

    {
        // Temperature container
        lv_obj_t *temperatureContainer = lv_obj_create(grid);
        lv_obj_set_grid_cell(temperatureContainer, LV_GRID_ALIGN_START, 0, 1, LV_GRID_ALIGN_START, 1, 1);
        lv_obj_set_size(temperatureContainer, temperatureContainerWidth, mainAreaHeight);
        lv_obj_set_align(temperatureContainer, LV_ALIGN_CENTER);
        lv_obj_set_style_pad_all(temperatureContainer, 0, LV_PART_MAIN);
        lv_obj_set_style_border_width(temperatureContainer, 0, LV_PART_MAIN);

        // Temperature bar
        constexpr lv_coord_t temperatureBarWidth{20};
        constexpr lv_coord_t temperatureBarHeight{static_cast<lv_coord_t>(mainAreaHeight * 0.8F)};

        static lv_style_t styleIndic;
        lv_style_init(&styleIndic);
        lv_style_set_bg_opa(&styleIndic, LV_OPA_COVER);
        lv_style_set_bg_color(&styleIndic, lv_palette_main(LV_PALETTE_RED));
        lv_style_set_bg_grad_color(&styleIndic, lv_palette_main(LV_PALETTE_BLUE));
        lv_style_set_bg_grad_dir(&styleIndic, LV_GRAD_DIR_VER);

        lv_obj_t *temperatureBar = lv_bar_create(temperatureContainer);
        lv_obj_add_style(temperatureBar, &styleIndic, LV_PART_INDICATOR);
        lv_obj_set_size(temperatureBar, temperatureBarWidth, temperatureBarHeight);
        lv_obj_set_align(temperatureBar, LV_ALIGN_TOP_MID);
        lv_bar_set_range(temperatureBar, TEMPERATURE_SCALING_FACTOR * MIN_TEMPERATURE_C, TEMPERATURE_SCALING_FACTOR * MAX_TEMPERATURE_C);
        lv_obj_set_style_pad_all(temperatureBar, 0, LV_PART_MAIN);
        lv_obj_set_style_border_width(temperatureBar, 1, LV_PART_MAIN);
        lv_obj_set_style_border_color(temperatureBar, lv_palette_main(LV_PALETTE_INDIGO), LV_PART_MAIN);
        
        lv_obj_t *temperatureLabel = lv_label_create(temperatureContainer);
        lv_obj_add_style(temperatureLabel, &styleLabel, LV_PART_MAIN);
        // Align the label horizontally with the bar and position it 5 px above
        lv_label_set_text(temperatureLabel, "     °C");
        lv_obj_align_to(temperatureLabel, temperatureBar, LV_ALIGN_OUT_BOTTOM_MID, 0, 5);

        uiTaskInterface->m_temperatureBar = temperatureBar;
        uiTaskInterface->m_temperatureLabel = temperatureLabel;
    }

    {
        // Pressure container
        lv_obj_t *pressureContainer = lv_obj_create(grid);
        lv_obj_set_grid_cell(pressureContainer, LV_GRID_ALIGN_START, 1, 1, LV_GRID_ALIGN_START, 1, 1);
        lv_obj_set_align(pressureContainer, LV_ALIGN_CENTER);
        lv_obj_set_style_pad_all(pressureContainer, 0, LV_PART_MAIN);
        lv_obj_set_size(pressureContainer, pressureContainerWidth, mainAreaHeight);
        lv_obj_set_style_border_width(pressureContainer, 0, LV_PART_MAIN);

        constexpr lv_coord_t pressureMeterWidth{pressureContainerWidth};
        constexpr lv_coord_t pressureMeterHeight{mainAreaHeight};

        // Pressure Meter
        lv_obj_t *pressureMeter = lv_meter_create(pressureContainer);
        lv_obj_set_size(pressureMeter, pressureMeterWidth, pressureMeterHeight);
        lv_obj_set_align(pressureMeter, LV_ALIGN_CENTER);
        lv_obj_add_style(pressureMeter, &styleLabel, LV_PART_TICKS);
        lv_obj_set_style_pad_all(pressureMeter, 0, LV_PART_MAIN);
        lv_obj_set_style_border_width(pressureMeter, 0, LV_PART_MAIN);

        // Add a scale first
        lv_meter_scale_t *scale = lv_meter_add_scale(pressureMeter);

        constexpr uint16_t tickCount{((MAX_PRESSURE_HPA - MIN_PRESSURE_HPA) / PRESSURE_TICKS_HPA) + 1};
        constexpr uint16_t lineWidthMinor{2U};
        constexpr uint16_t lineLengthMinor{5U};
        constexpr uint16_t nthMajor{4U};
        constexpr uint16_t lineWidthMajor{4U};
        constexpr uint16_t lineLengthMajor{10U};
        constexpr int16_t labelGap{10};
        constexpr uint32_t angleRange{270};
        constexpr uint32_t rotation{135};

        lv_meter_set_scale_ticks(pressureMeter, scale, tickCount, lineWidthMinor, lineLengthMinor, lv_palette_main(LV_PALETTE_GREY));
        lv_meter_set_scale_major_ticks(pressureMeter, scale, nthMajor, lineWidthMajor, lineLengthMajor, lv_color_black(), labelGap);
        lv_meter_set_scale_range(pressureMeter, scale, MIN_PRESSURE_HPA, MAX_PRESSURE_HPA, angleRange, rotation);

        // Add a needle line indicator
        uiTaskInterface->m_indic = lv_meter_add_needle_line(pressureMeter, scale, 4, lv_palette_main(LV_PALETTE_RED), -10);
        uiTaskInterface->m_pressureMeter = pressureMeter;
    }
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

void initSPI()
{
    spi_bus_config_t buscfg{};
    buscfg.sclk_io_num = CONFIG_LCD_SCLK_GPIO;
    buscfg.mosi_io_num = CONFIG_LCD_MOSI_GPIO;
    buscfg.miso_io_num = -1;
    buscfg.quadwp_io_num = -1;                                        // Quad SPI LCD driver is not yet supported
    buscfg.quadhd_io_num = -1;                                        // Quad SPI LCD driver is not yet supported
    buscfg.max_transfer_sz = LVGL_BUFFER_ELEMENTS * sizeof(uint16_t); // transfer x lines of pixels (assume pixel is RGB565) at most in one SPI transaction

    ESP_ERROR_CHECK(spi_bus_initialize(static_cast<spi_host_device_t>(CONFIG_LCD_SPI_HOST), &buscfg, SPI_DMA_CH_AUTO)); // Enable the DMA feature
}

esp_lcd_panel_io_handle_t initPanelIO(lv_disp_drv_t *disp_drv)
{
    initSPI();

    esp_lcd_panel_io_handle_t io_handle = nullptr;
    esp_lcd_panel_io_spi_config_t io_config{};
    io_config.dc_gpio_num = CONFIG_LCD_DC_GPIO;
    io_config.cs_gpio_num = CONFIG_LCD_CS_GPIO;
    io_config.pclk_hz = CONFIG_LCD_CLOCK_MHZ * 1000 * 1000;
    io_config.lcd_cmd_bits = LCD_CMD_BITS;
    io_config.lcd_param_bits = LCD_PARAM_BITS;
    io_config.spi_mode = LCD_SPI_TRADITIONAL_MODE;
    io_config.flags.sio_mode = 1; // only MOSI, no MISO
    io_config.trans_queue_depth = 2;
    io_config.on_color_trans_done = notify_lvgl_flush_ready;
    io_config.user_ctx = disp_drv;

    // Attach the LCD to the SPI bus
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)CONFIG_LCD_SPI_HOST, &io_config, &io_handle));

    return io_handle;
}

esp_lcd_panel_handle_t initPanel(lv_disp_drv_t *disp_drv)
{
    esp_lcd_panel_io_handle_t io_handle = initPanelIO(disp_drv);
    esp_lcd_panel_handle_t panel_handle = nullptr;
    esp_lcd_panel_dev_config_t panel_config{};
    panel_config.reset_gpio_num = CONFIG_LCD_RST_GPIO;
    panel_config.rgb_endian = LCD_RGB_ENDIAN_RGB;
    panel_config.bits_per_pixel = 16;

    // Create LCD panel handle for ST7735, with the SPI IO device handle
    ESP_ERROR_CHECK(esp_lcd_new_panel_st7735(io_handle, &panel_config, &panel_handle));

    return panel_handle;
}

void task_lvgl(void *arg)
{
    const char TAG[] = "lvgl";
    ESP_LOGI(TAG, "Starting LVGL task");

    static lv_disp_draw_buf_t disp_buf; // contains internal graphic buffer(s) called draw buffer(s)
    static lv_disp_drv_t disp_drv;      // contains callback functions

    ESP_LOGI(TAG, "Install ST7735 panel driver");
    esp_lcd_panel_handle_t panel_handle = initPanel(&disp_drv);

    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_mirror(panel_handle, true, false));
    ESP_ERROR_CHECK(esp_lcd_panel_swap_xy(panel_handle, true));

    // user can flush pre-defined pattern to the screen before we turn on the screen or backlight
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));

    ESP_LOGI(TAG, "Initialize LVGL library");
    lv_init();
    // alloc draw buffers used by LVGL
    // it's recommended to choose the size of the draw buffer(s) to be at least 1/10 screen sized
    lv_color_t *buf1 = (lv_color_t *)heap_caps_malloc(LVGL_BUFFER_ELEMENTS * sizeof(lv_color_t), MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    assert(buf1);
    lv_color_t *buf2 = (lv_color_t *)heap_caps_malloc(LVGL_BUFFER_ELEMENTS * sizeof(lv_color_t), MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    assert(buf2);
    // initialize LVGL draw buffers
    lv_disp_draw_buf_init(&disp_buf, buf1, buf2, LVGL_BUFFER_ELEMENTS);

    ESP_LOGI(TAG, "Register display driver to LVGL");
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = CONFIG_LCD_H_RES;
    disp_drv.ver_res = CONFIG_LCD_V_RES;
    disp_drv.flush_cb = lvgl_flush_cb;
    disp_drv.draw_buf = &disp_buf;
    disp_drv.user_data = panel_handle;
    lv_disp_t *disp = lv_disp_drv_register(&disp_drv);

    ESP_LOGI(TAG, "Install LVGL tick timer");
    // Tick interface for LVGL (using esp_timer to generate 2ms periodic event)
    esp_timer_create_args_t lvgl_tick_timer_args{};
    lvgl_tick_timer_args.callback = &increase_lvgl_tick;
    lvgl_tick_timer_args.name = "lvgl_tick";

    esp_timer_handle_t lvgl_tick_timer = NULL;
    ESP_ERROR_CHECK(esp_timer_create(&lvgl_tick_timer_args, &lvgl_tick_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(lvgl_tick_timer, LVGL_TICK_PERIOD_MS * 1000));

    UiTaskInterface *uiTaskInterface{static_cast<UiTaskInterface *>(arg)};
    uiTaskInterface->m_disp = disp;
    lvgl_create_ui(uiTaskInterface);

    uint32_t task_delay_ms = LVGL_TASK_MAX_DELAY_MS;

    while (true)
    {
        task_delay_ms = lv_timer_handler();

        if (task_delay_ms > LVGL_TASK_MAX_DELAY_MS)
        {
            task_delay_ms = LVGL_TASK_MAX_DELAY_MS;
        }
        else if (task_delay_ms < LVGL_TASK_MIN_DELAY_MS)
        {
            task_delay_ms = LVGL_TASK_MIN_DELAY_MS;
        }

        // Since no animations need to run frequently this task can wait for sensor data to update the UI
        // using xQueueReceive instead of yielding to a second task using vTaskDelay
        SensorData sensorData{};
        if (xQueueReceive(uiTaskInterface->m_measurementQueue_in, &sensorData, pdMS_TO_TICKS(task_delay_ms)) == pdPASS)
        {
            // Get time
            std::time_t now{};
            std::time(&now);

            std::tm *timeinfo{};
            timeinfo = std::localtime(&now);

            char timeStringBuffer[std::size("Wednesday dd.mm.yyyy hh:mm")];
            std::strftime(timeStringBuffer, sizeof(timeStringBuffer), "%A %e.%m.%Y %H:%M", timeinfo);

            ESP_LOGI(TAG, "%.2f °C %.2f hPa %u lx @ %s", sensorData.m_temperature, sensorData.m_pressure / 100.0, sensorData.m_illuminance, timeStringBuffer);

            // Update current date and time
            if ((timeinfo->tm_year > (1970 - 1900)) && (uiTaskInterface->m_timeLabel != nullptr))
            {
                lv_label_set_text(uiTaskInterface->m_timeLabel, timeStringBuffer);
            }

            // Update sensor readings
            if (uiTaskInterface->m_temperatureLabel != nullptr)
            {
                lv_label_set_text_fmt(uiTaskInterface->m_temperatureLabel, "%.1f °C", sensorData.m_temperature);
            }
            if (uiTaskInterface->m_temperatureBar != nullptr)
            {
                lv_bar_set_value(uiTaskInterface->m_temperatureBar, sensorData.m_temperature * TEMPERATURE_SCALING_FACTOR, LV_ANIM_OFF);
            }
            if (uiTaskInterface->m_pressureMeter != nullptr)
            {
                lv_meter_set_indicator_end_value(uiTaskInterface->m_pressureMeter, uiTaskInterface->m_indic, sensorData.m_pressure / PRESSURE_SCALING_DIVISOR);
            }
            if (uiTaskInterface->m_illuminanceLabel != nullptr)
            {
                lv_label_set_text_fmt(uiTaskInterface->m_illuminanceLabel, "%u lx", sensorData.m_illuminance);
            }
            ESP_LOGI(TAG, "Free stack: %u", uxTaskGetStackHighWaterMark(nullptr));
        }
    }
}