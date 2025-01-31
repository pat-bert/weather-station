#include "task_ui.hpp"

#include "backlight.hpp"

#include "interface_sensor.hpp"

#include "st7735/esp_lcd_panel_custom_vendor.h"

#include "driver/spi_master.h"

#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_freertos_hooks.h"

#include "lvgl.h"

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#include <cstring>
#include <ctime>
#include <iterator>

LV_FONT_DECLARE(lv_font_montserrat_8_german)

LV_IMG_DECLARE(sun);
LV_IMG_DECLARE(drop);

constexpr uint16_t numberOfSensorReadingsSaved{24};
constexpr uint8_t hoursTickDivider{6};
constexpr uint16_t numberMajorTicksHoursHistory{numberOfSensorReadingsSaved / hoursTickDivider + 1};

void lvgl_create_ui(UiTaskInterface *uiTaskInterface)
{
    constexpr lv_coord_t topRibbonHeight{15};
    constexpr lv_coord_t mainAreaHeight{CONFIG_LCD_V_RES - topRibbonHeight - 10};
    constexpr lv_coord_t temperatureContainerWidth{40};
    constexpr lv_coord_t illuminanceContainerWidth{24};
    constexpr lv_coord_t pressureContainerWidth{CONFIG_LCD_H_RES - temperatureContainerWidth - illuminanceContainerWidth - 17};

    static lv_style_t styleLabel;
    lv_style_init(&styleLabel);
    lv_style_set_text_font(&styleLabel, &lv_font_montserrat_8_german);

    // Grid layout
    static lv_coord_t rowDescriptor[] = {topRibbonHeight, mainAreaHeight, LV_GRID_TEMPLATE_LAST};
    static lv_coord_t columnDescriptor[] = {temperatureContainerWidth, illuminanceContainerWidth, pressureContainerWidth, LV_GRID_TEMPLATE_LAST};

    lv_obj_t *scr = lv_screen_active();

    lv_obj_t *tabview = lv_tabview_create(scr);
    lv_tabview_set_tab_bar_position(tabview, LV_DIR_VER);
    lv_tabview_set_tab_bar_size(tabview, 0);
    lv_obj_set_align(tabview, LV_ALIGN_CENTER);
    lv_obj_set_style_pad_all(tabview, 0, 0);

    uiTaskInterface->m_tabview = tabview;

    lv_obj_t *tabviewContent = lv_tabview_get_content(tabview);
    lv_obj_set_size(tabviewContent, CONFIG_LCD_H_RES, CONFIG_LCD_V_RES);
    lv_obj_set_style_pad_all(tabviewContent, 0, 0);
    lv_obj_center(tabviewContent);

    lv_obj_t *wifiProvisioningTab = lv_tabview_add_tab(tabview, "");
    lv_obj_set_size(wifiProvisioningTab, CONFIG_LCD_H_RES, CONFIG_LCD_V_RES);
    lv_obj_set_style_pad_all(wifiProvisioningTab, 0, 0);
    lv_obj_center(wifiProvisioningTab);

    {
        {
            lv_obj_t *onboardingLabel = lv_label_create(wifiProvisioningTab);
            lv_obj_add_style(onboardingLabel, &styleLabel, LV_PART_MAIN);
            lv_label_set_long_mode(onboardingLabel, LV_LABEL_LONG_WRAP);
            lv_label_set_text_static(onboardingLabel, "Willkommen bei der ESP32 Wetterstation!");
            lv_obj_set_width(onboardingLabel, lv_pct(80));
            lv_obj_align(onboardingLabel, LV_ALIGN_TOP_MID, 0, 5);
            lv_obj_set_style_text_align(onboardingLabel, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
        }

        lv_coord_t qrCodeSize{std::min(CONFIG_LCD_H_RES, CONFIG_LCD_V_RES) / 2};

        {
            lv_obj_t *instructionLabel = lv_label_create(wifiProvisioningTab);
            lv_label_set_text_static(instructionLabel, "1. App \"ESP SoftAP Provisioning\" herunterladen\n2. QR Code in der App scannen");
            lv_obj_set_width(instructionLabel, lv_pct(50));
            lv_obj_set_align(instructionLabel, LV_ALIGN_LEFT_MID);
            lv_obj_set_style_pad_left(instructionLabel, 2, LV_PART_MAIN);
            lv_label_set_long_mode(instructionLabel, LV_LABEL_LONG_WRAP);
            lv_obj_add_style(instructionLabel, &styleLabel, LV_PART_MAIN);
        }

        {
            lv_obj_t *provisioningQR = lv_qrcode_create(wifiProvisioningTab);
            lv_qrcode_set_size(provisioningQR, qrCodeSize);
            lv_qrcode_set_dark_color(provisioningQR, lv_color_black());
            lv_qrcode_set_light_color(provisioningQR, lv_color_white());
            lv_obj_set_align(provisioningQR, LV_ALIGN_RIGHT_MID);
            lv_obj_add_flag(provisioningQR, LV_OBJ_FLAG_HIDDEN);
            lv_obj_set_style_pad_right(provisioningQR, 2, LV_PART_MAIN);
            lv_obj_set_style_border_color(provisioningQR, lv_color_white(), 0);
            lv_obj_set_style_border_width(provisioningQR, 5, 0);
            uiTaskInterface->m_provisioningQrCode = provisioningQR;
        }
    }

    lv_obj_t *dashboardTab = lv_tabview_add_tab(tabview, "");
    lv_obj_set_size(dashboardTab, CONFIG_LCD_H_RES, CONFIG_LCD_V_RES);
    lv_obj_set_style_pad_all(dashboardTab, 0, 0);
    lv_obj_center(dashboardTab);

    lv_obj_t *grid = lv_obj_create(dashboardTab);
    lv_obj_set_size(grid, CONFIG_LCD_H_RES, CONFIG_LCD_V_RES);
    lv_obj_center(grid);
    lv_obj_set_layout(grid, LV_LAYOUT_GRID);
    lv_obj_set_style_grid_column_dsc_array(grid, columnDescriptor, 0);
    lv_obj_set_style_grid_row_dsc_array(grid, rowDescriptor, 0);
    lv_obj_set_style_pad_all(grid, 0, 0);
    lv_obj_set_style_border_width(grid, 0, LV_PART_MAIN);
    lv_obj_set_style_radius(grid, 0, LV_PART_MAIN);

    {
        // Top ribbon
        lv_obj_t *topRibbon = lv_obj_create(grid);
        lv_obj_set_grid_cell(topRibbon, LV_GRID_ALIGN_START, 0, 3, LV_GRID_ALIGN_START, 0, 1);
        lv_obj_set_align(topRibbon, LV_ALIGN_CENTER);
        lv_obj_set_size(topRibbon, CONFIG_LCD_H_RES, topRibbonHeight);
        lv_obj_set_style_pad_all(topRibbon, 0, LV_PART_MAIN);
        lv_obj_set_style_bg_color(topRibbon, lv_palette_darken(LV_PALETTE_BLUE, 3), LV_PART_MAIN);

        // Time label
        lv_obj_t *timeLabel = lv_label_create(topRibbon);
        lv_obj_add_style(timeLabel, &styleLabel, LV_PART_MAIN);
        lv_obj_set_align(timeLabel, LV_ALIGN_CENTER);
        lv_obj_set_style_text_align(timeLabel, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
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
        constexpr lv_coord_t temperatureBarWidth{temperatureContainerWidth / 2};
        constexpr lv_coord_t temperatureBarHeight{8 * mainAreaHeight / 10};

        static lv_style_t styleIndic;
        lv_style_init(&styleIndic);
        lv_style_set_bg_opa(&styleIndic, LV_OPA_COVER);
        lv_style_set_bg_color(&styleIndic, lv_palette_main(LV_PALETTE_RED));
        lv_style_set_bg_grad_color(&styleIndic, lv_palette_main(LV_PALETTE_BLUE));
        lv_style_set_bg_grad_dir(&styleIndic, LV_GRAD_DIR_VER);

        lv_obj_t *temperatureLabel = lv_label_create(temperatureContainer);
        lv_obj_add_style(temperatureLabel, &styleLabel, LV_PART_MAIN);
        // Align the label horizontally with the bar and position it 5 px above
        lv_label_set_text(temperatureLabel, "     °C");
        lv_obj_set_style_text_align(temperatureLabel, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
        lv_obj_set_align(temperatureLabel, LV_ALIGN_BOTTOM_MID);

        lv_obj_t *temperatureBar = lv_bar_create(temperatureContainer);
        lv_obj_add_style(temperatureBar, &styleIndic, LV_PART_INDICATOR);
        lv_obj_set_size(temperatureBar, temperatureBarWidth, temperatureBarHeight);
        lv_bar_set_range(temperatureBar, TEMPERATURE_SCALING_FACTOR * MIN_TEMPERATURE_C, TEMPERATURE_SCALING_FACTOR * MAX_TEMPERATURE_C);
        lv_obj_set_style_pad_all(temperatureBar, 0, LV_PART_MAIN);
        lv_obj_set_style_border_width(temperatureBar, 1, LV_PART_MAIN);
        lv_obj_set_style_border_color(temperatureBar, lv_palette_main(LV_PALETTE_INDIGO), LV_PART_MAIN);
        lv_obj_align_to(temperatureBar, temperatureLabel, LV_ALIGN_OUT_TOP_MID, 0, -5);

        // Register updating widgets
        uiTaskInterface->m_temperatureBar = temperatureBar;
        uiTaskInterface->m_temperatureLabel = temperatureLabel;
    }

    {
        // Container
        lv_obj_t *illuminanceContainer = lv_obj_create(grid);
        lv_obj_set_grid_cell(illuminanceContainer, LV_GRID_ALIGN_START, 1, 1, LV_GRID_ALIGN_START, 1, 1);
        lv_obj_set_align(illuminanceContainer, LV_ALIGN_CENTER);
        lv_obj_set_style_pad_all(illuminanceContainer, 0, LV_PART_MAIN);
        lv_obj_set_size(illuminanceContainer, illuminanceContainerWidth, mainAreaHeight);
        lv_obj_set_style_border_width(illuminanceContainer, 0, LV_PART_MAIN);

        // Illuminance
        lv_obj_t *illumninanceIcon = lv_image_create(illuminanceContainer);
        lv_image_set_src(illumninanceIcon, &sun);
        lv_obj_set_align(illumninanceIcon, LV_ALIGN_TOP_MID);
        lv_obj_set_style_bg_opa(illumninanceIcon, LV_OPA_TRANSP, LV_PART_MAIN);

        lv_obj_t *illuminanceLabel = lv_label_create(illuminanceContainer);
        lv_obj_add_style(illuminanceLabel, &styleLabel, LV_PART_MAIN);
        lv_obj_set_style_pad_all(illuminanceLabel, 0, LV_PART_MAIN);
        lv_obj_align_to(illuminanceLabel, illumninanceIcon, LV_ALIGN_OUT_BOTTOM_MID, 0, 5);

        // Humidity
        lv_obj_t *humidityLabel = lv_label_create(illuminanceContainer);
        lv_obj_add_style(humidityLabel, &styleLabel, LV_PART_MAIN);
        lv_obj_set_align(humidityLabel, LV_ALIGN_BOTTOM_MID);
        lv_obj_set_style_pad_all(humidityLabel, 0, LV_PART_MAIN);

        lv_obj_t *humidityIcon = lv_image_create(illuminanceContainer);
        lv_image_set_src(humidityIcon, &drop);
        lv_obj_set_style_bg_opa(humidityIcon, LV_OPA_TRANSP, LV_PART_MAIN);
        lv_obj_align_to(humidityIcon, humidityLabel, LV_ALIGN_OUT_TOP_MID, 0, -5);

        // Register updating widgets
        uiTaskInterface->m_illuminanceLabel = illuminanceLabel;
        uiTaskInterface->m_humidityLabel = humidityLabel;
    }

    {
        // Pressure container
        lv_obj_t *pressureContainer = lv_obj_create(grid);
        lv_obj_set_grid_cell(pressureContainer, LV_GRID_ALIGN_START, 2, 1, LV_GRID_ALIGN_START, 1, 1);
        lv_obj_set_align(pressureContainer, LV_ALIGN_CENTER);
        lv_obj_set_style_pad_all(pressureContainer, 0, LV_PART_MAIN);
        lv_obj_set_size(pressureContainer, pressureContainerWidth, mainAreaHeight);
        lv_obj_set_style_border_width(pressureContainer, 0, LV_PART_MAIN);

        constexpr lv_coord_t pressureMeterWidth{std::min(pressureContainerWidth, mainAreaHeight)};

        // Pressure Meter
        lv_obj_t *pressureMeter = lv_scale_create(pressureContainer);
        lv_scale_set_mode(pressureMeter, LV_SCALE_MODE_ROUND_INNER);
        lv_obj_set_size(pressureMeter, pressureMeterWidth, pressureMeterWidth);
        lv_obj_set_align(pressureMeter, LV_ALIGN_CENTER);
        lv_obj_add_style(pressureMeter, &styleLabel, LV_PART_INDICATOR);
        lv_obj_set_style_pad_all(pressureMeter, 0, LV_PART_MAIN);
        lv_obj_set_style_border_width(pressureMeter, 0, LV_PART_MAIN);

        constexpr uint16_t tickCount{((MAX_PRESSURE_HPA - MIN_PRESSURE_HPA) / PRESSURE_TICKS_HPA) + 1};
        constexpr uint16_t nthMajor{4U};

        constexpr uint16_t arcWidth{2U};
        constexpr uint16_t lineLengthMinor{4U};
        constexpr uint16_t lineLengthMajor{6U};

        constexpr uint16_t lineWidthMinor{2U};
        constexpr uint16_t lineWidthMajor{3U};

        constexpr uint32_t angleRange{270};
        constexpr uint32_t rotation{135};

        lv_scale_set_total_tick_count(pressureMeter, tickCount);
        lv_scale_set_major_tick_every(pressureMeter, nthMajor);
        lv_obj_set_style_length(pressureMeter, lineLengthMinor, LV_PART_ITEMS);
        lv_obj_set_style_length(pressureMeter, lineLengthMajor, LV_PART_INDICATOR);
        lv_obj_set_style_line_width(pressureMeter, lineWidthMinor, LV_PART_ITEMS);
        lv_obj_set_style_line_width(pressureMeter, lineWidthMajor, LV_PART_INDICATOR);
        lv_obj_set_style_line_color(pressureMeter, lv_palette_main(LV_PALETTE_GREY), LV_PART_ITEMS);
        lv_obj_set_style_line_color(pressureMeter, lv_color_black(), LV_PART_ITEMS);
        lv_scale_set_angle_range(pressureMeter, angleRange);
        lv_scale_set_rotation(pressureMeter, rotation);
        lv_scale_set_range(pressureMeter, MIN_PRESSURE_HPA, MAX_PRESSURE_HPA);

        lv_scale_section_t *rainyArc = lv_scale_add_section(pressureMeter);
        lv_scale_section_t *changingArc = lv_scale_add_section(pressureMeter);
        lv_scale_section_t *fairArc = lv_scale_add_section(pressureMeter);

        lv_scale_section_set_range(rainyArc, MIN_PRESSURE_HPA, PRESSURE_THRESHOLD_CHANGING);
        lv_scale_section_set_range(changingArc, PRESSURE_THRESHOLD_CHANGING, PRESSURE_THRESHOLD_FAIR);
        lv_scale_section_set_range(fairArc, PRESSURE_THRESHOLD_FAIR, MAX_PRESSURE_HPA);

        static lv_style_t styleRainy;
        lv_style_init(&styleRainy);
        lv_style_set_arc_color(&styleRainy, lv_palette_main(LV_PALETTE_BLUE));
        lv_style_set_arc_width(&styleRainy, arcWidth);

        static lv_style_t styleChanging;
        lv_style_init(&styleChanging);
        lv_style_set_arc_color(&styleChanging, lv_palette_main(LV_PALETTE_GREY));
        lv_style_set_arc_width(&styleChanging, arcWidth);

        static lv_style_t styleFair;
        lv_style_init(&styleFair);
        lv_style_set_arc_color(&styleFair, lv_palette_main(LV_PALETTE_YELLOW));
        lv_style_set_arc_width(&styleFair, arcWidth);

        lv_scale_section_set_style(rainyArc, LV_PART_MAIN, &styleRainy);
        lv_scale_section_set_style(changingArc, LV_PART_MAIN, &styleChanging);
        lv_scale_section_set_style(fairArc, LV_PART_MAIN, &styleFair);

        lv_obj_t *unitLabel = lv_label_create(pressureMeter);
        lv_label_set_text_static(unitLabel, "hPa");
        lv_obj_set_style_text_align(unitLabel, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
        lv_obj_align_to(unitLabel, pressureMeter, LV_ALIGN_CENTER, 0, pressureMeterWidth / 3);
        lv_obj_add_style(unitLabel, &styleLabel, LV_PART_MAIN);

        // Add a needle line indicator
        lv_obj_t *needle = lv_line_create(pressureMeter);
        lv_obj_set_style_line_width(needle, 3, LV_PART_MAIN);
        lv_obj_set_style_line_color(needle, lv_palette_main(LV_PALETTE_RED), LV_PART_MAIN);

        uiTaskInterface->m_indic = needle;
        uiTaskInterface->m_pressureMeter = pressureMeter;
    }

    lv_obj_t *historyTab = lv_tabview_add_tab(tabview, "");
    lv_obj_set_size(historyTab, CONFIG_LCD_H_RES, CONFIG_LCD_V_RES);
    lv_obj_set_style_pad_all(historyTab, 0, 0);
    lv_obj_center(historyTab);

    {
        constexpr lv_coord_t labelPaddingTemperature{30};
        constexpr lv_coord_t labelPaddingHumidity{30};
        constexpr lv_coord_t labelPaddingTime{25};
        constexpr lv_coord_t titlePadding{15};
        constexpr lv_coord_t labelTopPadding{3};
        constexpr lv_coord_t chartHeight{CONFIG_LCD_V_RES - labelPaddingTime - titlePadding - labelTopPadding};
        constexpr lv_coord_t chartWidth{CONFIG_LCD_H_RES - labelPaddingTemperature - labelPaddingHumidity};

        lv_obj_t *title = lv_label_create(historyTab);
        lv_obj_set_align(title, LV_ALIGN_TOP_MID);
        lv_obj_set_style_pad_all(title, 0, LV_PART_MAIN);
        lv_obj_add_style(title, &styleLabel, LV_PART_MAIN);
        lv_label_set_text_static(title, "Zeitverlauf");

        lv_obj_t *temperatureAxis = lv_label_create(historyTab);
        lv_obj_set_align(temperatureAxis, LV_ALIGN_TOP_LEFT);
        lv_obj_set_style_pad_all(temperatureAxis, 0, LV_PART_MAIN);
        lv_obj_add_style(temperatureAxis, &styleLabel, LV_PART_MAIN);
        lv_label_set_text_static(temperatureAxis, "°C");

        lv_obj_t *humidityAxis = lv_label_create(historyTab);
        lv_obj_set_align(humidityAxis, LV_ALIGN_TOP_RIGHT);
        lv_obj_set_style_pad_all(humidityAxis, 0, LV_PART_MAIN);
        lv_obj_add_style(humidityAxis, &styleLabel, LV_PART_MAIN);
        lv_label_set_text_static(humidityAxis, "%");

        lv_obj_t *chartWrapper = lv_obj_create(historyTab);
        lv_obj_remove_style_all(chartWrapper);
        lv_obj_set_size(chartWrapper, CONFIG_LCD_H_RES, CONFIG_LCD_V_RES - titlePadding);
        lv_obj_align(chartWrapper, LV_ALIGN_TOP_LEFT, 0, titlePadding);

        lv_obj_t *chart = lv_chart_create(chartWrapper);
        lv_obj_set_size(chart, chartWidth, chartHeight);
        lv_obj_align(chart, LV_ALIGN_TOP_LEFT, labelPaddingTemperature, labelTopPadding);
        lv_obj_set_style_pad_all(chart, 0, LV_PART_MAIN);
        lv_obj_set_style_radius(chart, 0, LV_PART_MAIN);
        lv_chart_set_type(chart, LV_CHART_TYPE_LINE);
        lv_chart_set_update_mode(chart, LV_CHART_UPDATE_MODE_SHIFT);
        lv_chart_set_point_count(chart, numberOfSensorReadingsSaved);

        // Data point size
        lv_obj_set_style_size(chart, 2, 2, LV_PART_INDICATOR);

        // Vertical range
        lv_chart_set_range(chart, LV_CHART_AXIS_PRIMARY_Y, MIN_TEMPERATURE_C, MAX_TEMPERATURE_C);

        constexpr lv_coord_t minimumHumidity{0};
        constexpr lv_coord_t maximumHumidity{100};
        lv_chart_set_range(chart, LV_CHART_AXIS_SECONDARY_Y, minimumHumidity, maximumHumidity);

        // x-Axis
        {
            lv_obj_t *scaleBottom = lv_scale_create(chartWrapper);
            lv_scale_set_mode(scaleBottom, LV_SCALE_MODE_HORIZONTAL_BOTTOM);
            lv_obj_set_size(scaleBottom, chartWidth, labelPaddingTime);
            lv_obj_align_to(scaleBottom, chart, LV_ALIGN_OUT_BOTTOM_MID, 0, 0);
            lv_scale_set_total_tick_count(scaleBottom, numberOfSensorReadingsSaved + 1);
            lv_scale_set_major_tick_every(scaleBottom, hoursTickDivider);
            lv_obj_add_style(scaleBottom, &styleLabel, LV_PART_INDICATOR);
            lv_obj_set_style_length(scaleBottom, 10, LV_PART_INDICATOR);
            lv_obj_set_style_length(scaleBottom, 5, LV_PART_ITEMS);
            lv_obj_set_style_line_width(scaleBottom, 1, LV_PART_INDICATOR);
            lv_obj_set_style_line_width(scaleBottom, 1, LV_PART_ITEMS);

            static const char *hoursPassed[numberMajorTicksHoursHistory + 1]{"-24h", "-18h", "-12h", "-6h", "-0h", nullptr};
            lv_scale_set_text_src(scaleBottom, hoursPassed);
        }

        // Primary y-Axis
        {
            lv_obj_t *scaleLeft = lv_scale_create(chartWrapper);
            lv_scale_set_mode(scaleLeft, LV_SCALE_MODE_VERTICAL_LEFT);
            lv_scale_set_range(scaleLeft, MIN_TEMPERATURE_C, MAX_TEMPERATURE_C);
            lv_obj_set_size(scaleLeft, labelPaddingTemperature, chartHeight);
            lv_obj_align_to(scaleLeft, chart, LV_ALIGN_OUT_LEFT_MID, 0, 0);
            lv_scale_set_total_tick_count(scaleLeft, MAX_TEMPERATURE_C - MIN_TEMPERATURE_C + 1);
            lv_scale_set_major_tick_every(scaleLeft, 2);
            lv_obj_add_style(scaleLeft, &styleLabel, LV_PART_INDICATOR);
            lv_obj_set_style_line_color(scaleLeft, lv_palette_main(LV_PALETTE_RED), LV_PART_INDICATOR);
            lv_obj_set_style_line_color(scaleLeft, lv_palette_main(LV_PALETTE_RED), LV_PART_MAIN);
            lv_obj_set_style_line_color(scaleLeft, lv_palette_main(LV_PALETTE_RED), LV_PART_ITEMS);
            lv_obj_set_style_length(scaleLeft, 10, LV_PART_INDICATOR);
            lv_obj_set_style_length(scaleLeft, 5, LV_PART_ITEMS);
            lv_obj_set_style_line_width(scaleLeft, 1, LV_PART_INDICATOR);
            lv_obj_set_style_line_width(scaleLeft, 1, LV_PART_ITEMS);
        }

        // Secondary y-Axis
        {
            lv_obj_t *scaleRight = lv_scale_create(chartWrapper);
            lv_scale_set_mode(scaleRight, LV_SCALE_MODE_VERTICAL_RIGHT);
            lv_scale_set_range(scaleRight, 0, 100);
            lv_obj_set_size(scaleRight, labelPaddingHumidity, chartHeight);
            lv_obj_align_to(scaleRight, chart, LV_ALIGN_OUT_RIGHT_MID, 0, 0);
            lv_scale_set_total_tick_count(scaleRight, 11);
            lv_scale_set_major_tick_every(scaleRight, 2);
            lv_obj_add_style(scaleRight, &styleLabel, LV_PART_INDICATOR);
            lv_obj_set_style_line_color(scaleRight, lv_palette_main(LV_PALETTE_BLUE), LV_PART_INDICATOR);
            lv_obj_set_style_line_color(scaleRight, lv_palette_main(LV_PALETTE_BLUE), LV_PART_MAIN);
            lv_obj_set_style_line_color(scaleRight, lv_palette_main(LV_PALETTE_BLUE), LV_PART_ITEMS);
            lv_obj_set_style_length(scaleRight, 10, LV_PART_INDICATOR);
            lv_obj_set_style_length(scaleRight, 5, LV_PART_ITEMS);
            lv_obj_set_style_line_width(scaleRight, 1, LV_PART_INDICATOR);
            lv_obj_set_style_line_width(scaleRight, 1, LV_PART_ITEMS);
        }

        // Division lines
        lv_chart_set_div_line_count(chart, 6, numberMajorTicksHoursHistory);

        lv_chart_series_t *temperatureSeries = lv_chart_add_series(chart, lv_palette_main(LV_PALETTE_RED), LV_CHART_AXIS_PRIMARY_Y);
        lv_chart_series_t *humiditySeries = lv_chart_add_series(chart, lv_palette_main(LV_PALETTE_BLUE), LV_CHART_AXIS_SECONDARY_Y);

        lv_chart_set_x_start_point(chart, temperatureSeries, numberOfSensorReadingsSaved - 1);
        lv_chart_set_x_start_point(chart, humiditySeries, numberOfSensorReadingsSaved - 1);

        uiTaskInterface->m_temperatureAndHumidityChart = chart;
        uiTaskInterface->m_temperatureSeries = temperatureSeries;
        uiTaskInterface->m_humiditySeries = humiditySeries;
    }
}

struct FlushCallbackData
{
    QueueHandle_t m_semaphore;
    lv_display_t *m_display;
};

static bool notify_lvgl_flush_ready(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_io_event_data_t *edata, void *user_ctx)
{
    FlushCallbackData *callbackData = static_cast<FlushCallbackData *>(user_ctx);
    lv_display_flush_ready(callbackData->m_display);

    BaseType_t pxHigherPriorityTaskWoken{pdFALSE};
    if (lv_display_flush_is_last(callbackData->m_display))
    {
        xSemaphoreGiveFromISR(callbackData->m_semaphore, &pxHigherPriorityTaskWoken);
    }

    return (pxHigherPriorityTaskWoken == pdPASS);
}

static bool screenModified{false};

static void lvgl_flush_cb(lv_display_t *display, const lv_area_t *area, unsigned char *color_map)
{
    esp_lcd_panel_handle_t panel_handle = static_cast<esp_lcd_panel_handle_t>(lv_display_get_user_data(display));
    int offsetx1 = area->x1;
    int offsetx2 = area->x2;
    int offsety1 = area->y1;
    int offsety2 = area->y2;

    // swap upper and lower bytes of 16 bit color values due to endianness
    lv_draw_sw_rgb565_swap(color_map, lv_area_get_size(area));
    esp_lcd_panel_draw_bitmap(panel_handle, offsetx1, offsety1, offsetx2 + 1, offsety2 + 1, color_map);
    screenModified = true;
}

esp_lcd_panel_handle_t initPanel(void *callbackData)
{
    spi_bus_config_t buscfg{};
    buscfg.sclk_io_num = CONFIG_LCD_SCLK_GPIO;
    buscfg.mosi_io_num = CONFIG_LCD_MOSI_GPIO;
    buscfg.miso_io_num = -1;
    buscfg.quadwp_io_num = -1;                                        // Quad SPI LCD driver is not yet supported
    buscfg.quadhd_io_num = -1;                                        // Quad SPI LCD driver is not yet supported
    buscfg.max_transfer_sz = LVGL_BUFFER_ELEMENTS * sizeof(uint16_t); // transfer x lines of pixels (assume pixel is RGB565) at most in one SPI transaction

    ESP_ERROR_CHECK(spi_bus_initialize(static_cast<spi_host_device_t>(CONFIG_LCD_SPI_HOST), &buscfg, SPI_DMA_CH_AUTO)); // Enable the DMA feature

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
    io_config.user_ctx = callbackData;

    // Attach the LCD to the SPI bus
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi(static_cast<spi_host_device_t>(CONFIG_LCD_SPI_HOST), &io_config, &io_handle));

    esp_lcd_panel_handle_t panel_handle = nullptr;
    esp_lcd_panel_dev_config_t panel_config{};
    panel_config.reset_gpio_num = CONFIG_LCD_RST_GPIO;
    panel_config.rgb_endian = LCD_RGB_ENDIAN_RGB;
    panel_config.bits_per_pixel = 16;

    // Create LCD panel handle for ST7735, with the SPI IO device handle
    ESP_ERROR_CHECK(esp_lcd_new_panel_st7735(io_handle, &panel_config, &panel_handle));

    return panel_handle;
}

void IRAM_ATTR increase_lvgl_tick(void)
{
    lv_tick_inc(1000 / CONFIG_FREERTOS_HZ);
}

void task_lvgl(void *arg)
{
    const char TAG[] = "lvgl";
    ESP_LOGI(TAG, "Starting LVGL task");

    QueueHandle_t xSemaphore = xSemaphoreCreateBinary();

    ESP_LOGI(TAG, "Initialize LVGL library");
    lv_init();

    ESP_LOGI(TAG, "Install ST7735 panel driver");
    lv_display_t *display = lv_display_create(CONFIG_LCD_H_RES, CONFIG_LCD_V_RES);
    lv_display_set_color_format(display, LV_COLOR_FORMAT_RGB565);

    FlushCallbackData callbackData{xSemaphore, display};
    esp_lcd_panel_handle_t panel_handle = initPanel(static_cast<void *>(&callbackData));

    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, false));
    ESP_ERROR_CHECK(esp_lcd_panel_mirror(panel_handle, true, false));
    ESP_ERROR_CHECK(esp_lcd_panel_swap_xy(panel_handle, true));

    ESP_ERROR_CHECK(esp_register_freertos_tick_hook_for_cpu(increase_lvgl_tick, xPortGetCoreID()));
    // alloc draw buffers used by LVGL
    // it's recommended to choose the size of the draw buffer(s) to be at least 1/10 screen sized
    const size_t bufferSize{static_cast<size_t>(LVGL_BUFFER_ELEMENTS * lv_color_format_get_size(lv_display_get_color_format(display)))};
    void *buf1 = heap_caps_malloc(bufferSize, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    assert(buf1);
    void *buf2 = heap_caps_malloc(bufferSize, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    assert(buf2);

    lv_display_set_user_data(display, static_cast<void *>(panel_handle));
    lv_display_set_buffers(display, buf1, buf2, bufferSize, LV_DISPLAY_RENDER_MODE_PARTIAL);
    lv_display_set_flush_cb(display, lvgl_flush_cb);

    UiTaskInterface *uiTaskInterface{static_cast<UiTaskInterface *>(arg)};
    lvgl_create_ui(uiTaskInterface);
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));

    Backlight backlight{CONFIG_LCD_BACKLIGHT_GPIO, CONFIG_LCD_BACKLIGHT_HZ};
    backlight.init();
    backlight.power(true);
    vTaskDelay(pdMS_TO_TICKS(10));
    backlight.dim(0, CONFIG_LCD_FADE_TIME_SECONDS * 1000);

    int32_t temperatureAverageLastHourCentigrade{0};
    uint32_t humidityAverageLastHour{0U};

    uint8_t sensorReadingsLastHour{0U};
    uint8_t hoursTracked{0U};

    while (true)
    {
        TickType_t xTicksToWait{portMAX_DELAY};
        if (backlight.isOn())
        {
            ESP_LOGI(TAG, "Updating UI");
            uint32_t task_delay_ms = lv_timer_handler();
            xTicksToWait = pdMS_TO_TICKS(task_delay_ms);
        }

        QueueValueType queueData{};
        if (xQueueReceive(uiTaskInterface->m_measurementQueue_in, &queueData, xTicksToWait) == pdPASS)
        {
            // Get time
            std::time_t now{};
            std::time(&now);

            std::tm *timeinfo{};
            timeinfo = std::localtime(&now);

            char timeStringBuffer[std::size("Wednesday dd.mm.yyyy hh:mm")];
            std::strftime(timeStringBuffer, sizeof(timeStringBuffer), "%A %d.%m.%Y %H:%M", timeinfo);

            // Update current date and time
            if ((timeinfo->tm_year > (1970 - 1900)) && (uiTaskInterface->m_timeLabel != nullptr))
            {
                lv_label_set_text(uiTaskInterface->m_timeLabel, timeStringBuffer);
            }
            if (std::holds_alternative<ButtonData>(queueData))
            {
                ButtonData &buttonData = std::get<ButtonData>(queueData);

                if (backlight.isOn() && buttonData.m_tabviewButtonPressed && (uiTaskInterface->m_tabview != nullptr))
                {
                    uint16_t current_tab = lv_tabview_get_tab_act(uiTaskInterface->m_tabview);
                    uint16_t next_tab = current_tab + 1;
                    if (next_tab >= lv_tabview_get_tab_count(uiTaskInterface->m_tabview))
                    {
                        next_tab = 0;
                    }
                    lv_tabview_set_active(uiTaskInterface->m_tabview, next_tab, LV_ANIM_OFF);
                }

                esp_lcd_panel_disp_sleep(panel_handle, false);
                backlight.init();
                backlight.stopFade();
                backlight.power(true);
                vTaskDelay(pdMS_TO_TICKS(10));
                backlight.dim(0, CONFIG_LCD_FADE_TIME_SECONDS * 1000);
            }
            else if (std::holds_alternative<WifiData>(queueData))
            {
                WifiData &wifiData = std::get<WifiData>(queueData);

                if (uiTaskInterface->m_provisioningQrCode != nullptr)
                {
                    lv_qrcode_update(uiTaskInterface->m_provisioningQrCode, wifiData.m_provisioningPayload, std::strlen(wifiData.m_provisioningPayload));
                    lv_obj_remove_flag(uiTaskInterface->m_provisioningQrCode, LV_OBJ_FLAG_HIDDEN);
                }
            }
            else if (std::holds_alternative<SensorData>(queueData))
            {
                SensorData &sensorData = std::get<SensorData>(queueData);

                ESP_LOGI(TAG, "%ld.%ld °C %ld%% %ld hPa %u lx @ %s", sensorData.m_temperature / 100, sensorData.m_temperature % 100, sensorData.m_humidity, sensorData.m_pressure / 100, sensorData.m_illuminance, timeStringBuffer);

                sensorReadingsLastHour++;
                if (sensorReadingsLastHour > 60)
                {
                    temperatureAverageLastHourCentigrade = 0;
                    humidityAverageLastHour = 0U;
                    sensorReadingsLastHour = 1;
                    hoursTracked++;
                    if (hoursTracked >= 24U)
                    {
                        hoursTracked = 0U;
                    }
                }
                temperatureAverageLastHourCentigrade = (temperatureAverageLastHourCentigrade * (sensorReadingsLastHour - 1) + sensorData.m_temperature) / sensorReadingsLastHour;
                humidityAverageLastHour = (humidityAverageLastHour * (sensorReadingsLastHour - 1) + sensorData.m_humidity) / sensorReadingsLastHour;

                // Update sensor readings
                if (uiTaskInterface->m_temperatureLabel != nullptr)
                {
                    lv_label_set_text_fmt(uiTaskInterface->m_temperatureLabel, "%ld.%ld °C", sensorData.m_temperature / 100, (sensorData.m_temperature % 100) / 10);
                }
                if (uiTaskInterface->m_temperatureBar != nullptr)
                {
                    lv_bar_set_value(uiTaskInterface->m_temperatureBar, sensorData.m_temperature / 100 * TEMPERATURE_SCALING_FACTOR, LV_ANIM_OFF);
                }
                if ((uiTaskInterface->m_pressureMeter != nullptr) && (uiTaskInterface->m_indic))
                {
                    int32_t w{lv_obj_get_width(uiTaskInterface->m_pressureMeter)};
                    lv_scale_set_line_needle_value(uiTaskInterface->m_pressureMeter, uiTaskInterface->m_indic, w / 2 - 5, sensorData.m_pressure / PRESSURE_SCALING_DIVISOR);
                }
                if (uiTaskInterface->m_illuminanceLabel != nullptr)
                {
                    lv_label_set_text_fmt(uiTaskInterface->m_illuminanceLabel, "%u lx", sensorData.m_illuminance);
                }
                if (uiTaskInterface->m_humidityLabel != nullptr)
                {
                    lv_label_set_text_fmt(uiTaskInterface->m_humidityLabel, "%ld%%", sensorData.m_humidity);
                }
                if (uiTaskInterface->m_temperatureAndHumidityChart != nullptr)
                {
                    if (uiTaskInterface->m_temperatureSeries != nullptr)
                    {
                        if (sensorReadingsLastHour == 1)
                        {
                            lv_chart_set_next_value(uiTaskInterface->m_temperatureAndHumidityChart, uiTaskInterface->m_temperatureSeries, temperatureAverageLastHourCentigrade / 100);
                        }
                        else
                        {
                            lv_chart_set_value_by_id(uiTaskInterface->m_temperatureAndHumidityChart, uiTaskInterface->m_temperatureSeries, (numberOfSensorReadingsSaved - 1 + hoursTracked) % numberOfSensorReadingsSaved, temperatureAverageLastHourCentigrade / 100);
                        }
                    }

                    if (uiTaskInterface->m_humiditySeries != nullptr)
                    {
                        if (sensorReadingsLastHour == 1)
                        {
                            lv_chart_set_next_value(uiTaskInterface->m_temperatureAndHumidityChart, uiTaskInterface->m_humiditySeries, humidityAverageLastHour);
                        }
                        else
                        {
                            lv_chart_set_value_by_id(uiTaskInterface->m_temperatureAndHumidityChart, uiTaskInterface->m_humiditySeries, (numberOfSensorReadingsSaved - 1 + hoursTracked) % numberOfSensorReadingsSaved, humidityAverageLastHour);
                        }
                    }
                }
            }

            ESP_LOGI(TAG, "Free stack: %u", uxTaskGetStackHighWaterMark(nullptr));
        }
    }
}