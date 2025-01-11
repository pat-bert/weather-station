#include "esp_log.h"
#include "esp_pm.h"
#include "esp_sleep.h"
#include "nvs_flash.h"

#if SOC_LP_CORE_SUPPORTED && CONFIG_ULP_COPROC_TYPE_LP_CORE
#include "ulp_lp_core.h"
#include "ulp_lp_sensor.h"
#include "lp_core_i2c.h"
#endif

#include "interfaces/interface_sensor.hpp"

#include "sensor/task_sensor.hpp"
#include "ui/task_ui.hpp"
#include "wifi/task_sntp.hpp"
#include "wifi/wifi_task_interface.hpp"

#include "iot_button.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include <algorithm>

// FreeRTOS
#define STACK_SIZE_SENSOR_TASK (3000)
#define LVGL_TASK_STACK_SIZE (5500)
#define SNTP_TASK_STACK_SIZE (4000)

extern const uint8_t ulp_lp_sensor_bin_start[] asm("_binary_ulp_lp_sensor_bin_start");
extern const uint8_t ulp_lp_sensor_bin_end[] asm("_binary_ulp_lp_sensor_bin_end");

static const char *TAG{"main"};

static void IRAM_ATTR factoryResetCallback(void *button_handle, void *usr_data)
{
    nvs_flash_erase();
    esp_restart();
}

static void IRAM_ATTR changeActiveTabCallback(void *button_handle, void *usr_data)
{
    UiTaskInterface *uiTaskInterface = static_cast<UiTaskInterface *>(usr_data);

    ButtonData buttonData{};
    buttonData.m_tabviewButtonPressed = true;

    QueueValueType queueData{buttonData};

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xQueueSendFromISR(uiTaskInterface->m_measurementQueue_in, &queueData, &xHigherPriorityTaskWoken);

    if (xHigherPriorityTaskWoken)
    {
        portYIELD_FROM_ISR();
    }
}

static void init_button(UiTaskInterface &uiTaskInterface)
{
    button_config_t gpio_btn_cfg{};
    gpio_btn_cfg.type = BUTTON_TYPE_GPIO;
    gpio_btn_cfg.long_press_time = 7000;
    gpio_btn_cfg.short_press_time = CONFIG_BUTTON_SHORT_PRESS_TIME_MS;
    gpio_btn_cfg.gpio_button_config.gpio_num = CONFIG_BUTTON_GPIO;
    gpio_btn_cfg.gpio_button_config.active_level = 0;
    gpio_btn_cfg.gpio_button_config.enable_power_save = true;
    button_handle_t gpio_btn = iot_button_create(&gpio_btn_cfg);
    if (nullptr == gpio_btn)
    {
        ESP_LOGE(TAG, "Button create failed");
    }

    ESP_ERROR_CHECK(iot_button_register_cb(gpio_btn, BUTTON_SINGLE_CLICK, changeActiveTabCallback, static_cast<void *>(&uiTaskInterface)));
    ESP_ERROR_CHECK(iot_button_register_cb(gpio_btn, BUTTON_LONG_PRESS_HOLD, factoryResetCallback, nullptr));
}

#if SOC_LP_CORE_SUPPORTED && CONFIG_ULP_COPROC_TYPE_LP_CORE
esp_err_t initLpCoreI2C()
{
    lp_core_i2c_cfg_t busConfigLpCore{};
    busConfigLpCore.i2c_pin_cfg.scl_io_num = GPIO_NUM_7;
    busConfigLpCore.i2c_pin_cfg.scl_pullup_en = false;
    busConfigLpCore.i2c_pin_cfg.sda_io_num = GPIO_NUM_6;
    busConfigLpCore.i2c_pin_cfg.sda_pullup_en = false;
    busConfigLpCore.i2c_src_clk = LP_I2C_SCLK_DEFAULT;
    busConfigLpCore.i2c_timing_cfg.clk_speed_hz = std::min(CONFIG_BME_I2C_CLOCK_KHZ, CONFIG_BH1750_I2C_CLOCK_KHZ) * 1000U;
    return lp_core_i2c_master_init(LP_I2C_NUM_0, &busConfigLpCore);
}

static void init_ulp_program(void)
{
    initLpCoreI2C();

    esp_err_t err = ulp_lp_core_load_binary(ulp_lp_sensor_bin_start, (ulp_lp_sensor_bin_end - ulp_lp_sensor_bin_start));
    ESP_ERROR_CHECK(err);

    /* Start the program */
    ulp_lp_core_cfg_t cfg{};
    cfg.wakeup_source = ULP_LP_CORE_WAKEUP_SOURCE_LP_TIMER;
    cfg.lp_timer_sleep_duration_us = 1000 * 1000 * CONFIG_MEASUREMENT_INTERVAL_SECONDS;

    err = ulp_lp_core_run(&cfg);
    ESP_ERROR_CHECK(err);
}
#endif

extern "C" void app_main(void)
{
    esp_pm_config_t pm_config{};
    ESP_ERROR_CHECK(esp_pm_get_configuration(&pm_config));
#if CONFIG_FREERTOS_USE_TICKLESS_IDLE
    pm_config.light_sleep_enable = true;
#endif
    ESP_ERROR_CHECK(esp_pm_configure(&pm_config));
    ESP_ERROR_CHECK(esp_sleep_pd_config(ESP_PD_DOMAIN_MODEM, ESP_PD_OPTION_ON));
    ESP_ERROR_CHECK(esp_sleep_pd_config(ESP_PD_DOMAIN_MODEM, ESP_PD_OPTION_OFF));

    esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
    if (cause != ESP_SLEEP_WAKEUP_ULP)
    {
#if SOC_LP_CORE_SUPPORTED && CONFIG_ULP_COPROC_TYPE_LP_CORE
        ESP_LOGI(TAG, "Not an LP core wakeup. Cause = %d\n", cause);
        ESP_LOGI(TAG, "Initializing...\n");

        ESP_ERROR_CHECK(esp_sleep_enable_ulp_wakeup());

        /* Load LP Core binary and start the coprocessor */
        init_ulp_program();
#endif
    }

    QueueHandle_t measurementQueue{xQueueCreate(5, sizeof(QueueValueType))};
    configASSERT(measurementQueue);

    TaskHandle_t xHandleSensor{nullptr};
    SensorTaskInterface sensorTaskInterface{measurementQueue};
    xTaskCreate(task_sensor, "sensor", STACK_SIZE_SENSOR_TASK, static_cast<void *>(&sensorTaskInterface), tskIDLE_PRIORITY + 1, &xHandleSensor);
    configASSERT(xHandleSensor);

    UiTaskInterface uiTaskInterface{};
    uiTaskInterface.m_measurementQueue_in = sensorTaskInterface.m_measurementQueue_out;
    TaskHandle_t xHandleDisplay{nullptr};
    xTaskCreate(task_lvgl, "lvgl", LVGL_TASK_STACK_SIZE, static_cast<void *>(&uiTaskInterface), tskIDLE_PRIORITY + 1, &xHandleDisplay);
    configASSERT(xHandleDisplay);

    WifiTaskInterface wifiTaskInterface{measurementQueue};
    TaskHandle_t xHandleSntp{nullptr};
    xTaskCreate(task_sntp, "sntp", SNTP_TASK_STACK_SIZE, static_cast<void *>(&wifiTaskInterface), tskIDLE_PRIORITY + 1, &xHandleSntp);
    configASSERT(xHandleSntp);

    init_button(uiTaskInterface);

    vTaskSuspend(nullptr);
}