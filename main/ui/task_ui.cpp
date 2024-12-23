#include "task_ui.hpp"

#include "bmp280/bmp2.h"
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
    static lv_style_t styleLabel;
    lv_style_init(&styleLabel);
    lv_style_set_text_font(&styleLabel, &lv_font_montserrat_8);

    lv_disp_t *disp = uiTaskInterface->m_disp;
    lv_obj_t *scr = lv_disp_get_scr_act(disp);

    constexpr lv_coord_t topRibbonHeight{10};
    constexpr lv_coord_t topRibbonWidth{CONFIG_LCD_H_RES};

    constexpr lv_coord_t mainAreaHeight{CONFIG_LCD_V_RES - topRibbonHeight};
    constexpr lv_coord_t mainAreaWidth{CONFIG_LCD_H_RES};

    constexpr lv_coord_t temperatureContainerHeight{mainAreaHeight};
    constexpr lv_coord_t temperatureContainerWidth{40};
    constexpr lv_coord_t temperatureBarWidth{20};
    constexpr lv_coord_t temperatureBarHeight{static_cast<lv_coord_t>(temperatureContainerHeight * (2.0F / 3.0F))};

    constexpr lv_coord_t pressureContainerHeight{mainAreaHeight};
    constexpr lv_coord_t pressureContainerWidth{mainAreaWidth - temperatureContainerWidth};
    constexpr lv_coord_t pressureMeterWidth{pressureContainerWidth};
    constexpr lv_coord_t pressureMeterHeight{pressureContainerHeight};

    // Top ribbon
    lv_obj_t *topRibbon = lv_obj_create(scr);
    lv_obj_set_size(topRibbon, topRibbonWidth, topRibbonHeight);
    lv_obj_set_pos(topRibbon, 0, 0);

    // Time label
    uiTaskInterface->m_timeLabel = lv_label_create(topRibbon);
    lv_obj_add_style(uiTaskInterface->m_timeLabel, &styleLabel, LV_PART_MAIN);
    lv_obj_set_align(uiTaskInterface->m_timeLabel, LV_ALIGN_CENTER);
    lv_label_set_text(uiTaskInterface->m_timeLabel, "");

    // Main area
    lv_obj_t *mainArea = lv_obj_create(scr);
    lv_obj_set_size(mainArea, mainAreaWidth, mainAreaHeight);
    lv_obj_set_pos(mainArea, 0, topRibbonHeight);

    // Temperature container
    lv_obj_t *temperatureContainer = lv_obj_create(mainArea);
    lv_obj_set_size(temperatureContainer, temperatureContainerWidth, temperatureContainerHeight);
    lv_obj_set_pos(temperatureContainer, 0, 0);

    // Pressure Meter
    uiTaskInterface->m_pressureMeter = lv_meter_create(mainArea);
    lv_obj_set_size(uiTaskInterface->m_pressureMeter, pressureMeterWidth, pressureMeterHeight);
    lv_obj_set_pos(uiTaskInterface->m_pressureMeter, temperatureContainerWidth, 0);
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
    static lv_style_t styleIndic;
    lv_style_init(&styleIndic);
    lv_style_set_bg_opa(&styleIndic, LV_OPA_COVER);
    lv_style_set_bg_color(&styleIndic, lv_palette_main(LV_PALETTE_RED));
    lv_style_set_bg_grad_color(&styleIndic, lv_palette_main(LV_PALETTE_BLUE));
    lv_style_set_bg_grad_dir(&styleIndic, LV_GRAD_DIR_VER);

    uiTaskInterface->m_temperatureBar = lv_bar_create(temperatureContainer);
    lv_obj_add_style(uiTaskInterface->m_temperatureBar, &styleIndic, LV_PART_INDICATOR);
    lv_obj_set_size(uiTaskInterface->m_temperatureBar, temperatureBarWidth, temperatureBarHeight);
    lv_obj_set_align(uiTaskInterface->m_temperatureBar, LV_ALIGN_CENTER);
    lv_bar_set_range(uiTaskInterface->m_temperatureBar, TEMPERATURE_SCALING_FACTOR * MIN_TEMPERATURE_C, TEMPERATURE_SCALING_FACTOR * MAX_TEMPERATURE_C);

    uiTaskInterface->m_temperatureLabel = lv_label_create(temperatureContainer);
    lv_obj_add_style(uiTaskInterface->m_temperatureLabel, &styleLabel, LV_PART_MAIN);
    lv_obj_align_to(uiTaskInterface->m_temperatureLabel, uiTaskInterface->m_temperatureBar, LV_ALIGN_OUT_BOTTOM_MID, 0, -10);
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
    io_config.trans_queue_depth = LVGL_SCREEN_DIVIDER;
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
        bmp2_data sensorData{};
        if (xQueueReceive(uiTaskInterface->m_measurementQueue_in, &sensorData, pdMS_TO_TICKS(task_delay_ms)) == pdPASS)
        {
            // Get time
            std::time_t now{};
            std::time(&now);

            std::tm *timeinfo{};
            timeinfo = std::localtime(&now);

            char timeStringBuffer[std::size("Wednesday dd.mm.yyyy hh:mm")];
            std::strftime(timeStringBuffer, sizeof(timeStringBuffer), "%A %e.%m.%Y %H:%M", timeinfo);

            ESP_LOGI(TAG, "%.2f °C %.2f Pa @ %s", sensorData.temperature, sensorData.pressure, timeStringBuffer);

            // Update current date and time
            if (timeinfo->tm_year > (1970 - 1900))
            {
                lv_label_set_text(uiTaskInterface->m_timeLabel, timeStringBuffer);
            }

            // Update sensor readings
            lv_label_set_text_fmt(uiTaskInterface->m_temperatureLabel, "%.1f °C", sensorData.temperature);
            lv_bar_set_value(uiTaskInterface->m_temperatureBar, sensorData.temperature * TEMPERATURE_SCALING_FACTOR, LV_ANIM_OFF);
            lv_meter_set_indicator_end_value(uiTaskInterface->m_pressureMeter, uiTaskInterface->m_indic, sensorData.pressure / PRESSURE_SCALING_DIVISOR);
        }
    }
}