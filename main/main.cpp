#include "esp_log.h"
#include "esp_pm.h"
#include "esp_sleep.h"
#include "nvs_flash.h"

#if SOC_LP_CORE_SUPPORTED && CONFIG_ULP_COPROC_TYPE_LP_CORE
#include "ulp_lp_core.h"
#include "ulp_lp_sensor.h"
#include "lp_core_i2c.h"
#include "hal/pmu_ll.h"
#include "esp_intr_alloc.h"
#include "soc/interrupts.h"
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

static void IRAM_ATTR singleClickCallback(void *button_handle, void *args)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    Ui::UiTaskInterface *uiTaskInterface = static_cast<Ui::UiTaskInterface *>(args);

    if (uiTaskInterface != nullptr)
    {
        // Only handle in case the UI task interface is initialized, e.g. not when exiting deep sleep
        ButtonData buttonData{};
        buttonData.m_buttonPressed = true;

        QueueValueType queueData{buttonData};
        xQueueSendFromISR(uiTaskInterface->m_queue_in, &queueData, &xHigherPriorityTaskWoken);
    }

    if (xHigherPriorityTaskWoken)
    {
        portYIELD_FROM_ISR();
    }
}

static void initButton(Ui::UiTaskInterface &uiTaskInterface)
{
    button_config_t gpio_btn_cfg{};
    gpio_btn_cfg.type = BUTTON_TYPE_GPIO;
    gpio_btn_cfg.long_press_time = 7000;
    gpio_btn_cfg.short_press_time = CONFIG_BUTTON_SHORT_PRESS_TIME_MS;
    gpio_btn_cfg.gpio_button_config.gpio_num = CONFIG_BUTTON_GPIO;
    gpio_btn_cfg.gpio_button_config.active_level = 0;
    gpio_btn_cfg.gpio_button_config.enable_power_save = true;
    button_handle_t gpio_btn = iot_button_create(&gpio_btn_cfg);
    ESP_ERROR_CHECK(gpio_sleep_sel_dis(static_cast<gpio_num_t>(CONFIG_BUTTON_GPIO)));
    if (nullptr == gpio_btn)
    {
        ESP_LOGE(TAG, "Button create failed");
    }

    ESP_ERROR_CHECK(iot_button_register_cb(gpio_btn, BUTTON_SINGLE_CLICK, singleClickCallback, static_cast<void *>(&uiTaskInterface)));
    ESP_ERROR_CHECK(iot_button_register_cb(gpio_btn, BUTTON_LONG_PRESS_HOLD, factoryResetCallback, nullptr));
}

#if SOC_LP_CORE_SUPPORTED && CONFIG_ULP_COPROC_TYPE_LP_CORE
static void ulpSoftwareInterruptCallback(void *arg)
{
    // Clear the interrupt bit
    REG_SET_BIT(PMU_HP_INT_CLR_REG, PMU_SW_INT_CLR);
}

static void initLpCore(SensorTaskInterface *sensorTaskInterface)
{
    // Allocate and enable interrupt on signal from LP-core
    intr_handle_t interruptHandle{};
    ESP_ERROR_CHECK(esp_intr_alloc_intrstatus(ETS_PMU_INTR_SOURCE, 0, PMU_HP_INT_ST_REG, PMU_SW_INT_ST, ulpSoftwareInterruptCallback, static_cast<void *>(sensorTaskInterface), &interruptHandle));
    ESP_ERROR_CHECK(esp_intr_enable(interruptHandle));
    REG_SET_BIT(PMU_HP_INT_ENA_REG, PMU_SW_INT_ENA);

    ESP_ERROR_CHECK(esp_sleep_enable_ulp_wakeup());

    lp_core_i2c_cfg_t busConfigLpCore{};
    busConfigLpCore.i2c_pin_cfg.scl_io_num = static_cast<gpio_num_t>(CONFIG_SCL_LP_GPIO);
    busConfigLpCore.i2c_pin_cfg.scl_pullup_en = false;
    busConfigLpCore.i2c_pin_cfg.sda_io_num = static_cast<gpio_num_t>(CONFIG_SDA_LP_GPIO);
    busConfigLpCore.i2c_pin_cfg.sda_pullup_en = false;
    busConfigLpCore.i2c_src_clk = LP_I2C_SCLK_DEFAULT;
    busConfigLpCore.i2c_timing_cfg.clk_speed_hz = std::min(CONFIG_BME_I2C_CLOCK_KHZ, CONFIG_BH1750_I2C_CLOCK_KHZ) * 1000U;
    ESP_ERROR_CHECK(lp_core_i2c_master_init(LP_I2C_NUM_0, &busConfigLpCore));

    ESP_ERROR_CHECK(ulp_lp_core_load_binary(ulp_lp_sensor_bin_start, (ulp_lp_sensor_bin_end - ulp_lp_sensor_bin_start)));

    // When using LP timer wake-up the LP-core starts with the specified delay instead of immediately
    // which means that the ulp_* variables are still uninitialized and accessing them from the HP core results in an exception
    ulp_lp_core_cfg_t cfg{};
    cfg.wakeup_source = ULP_LP_CORE_WAKEUP_SOURCE_LP_TIMER | ULP_LP_CORE_WAKEUP_SOURCE_HP_CPU;
    cfg.lp_timer_sleep_duration_us = 1000 * 1000 * CONFIG_MEASUREMENT_INTERVAL_SECONDS;

    // Start the coprocessor
    ESP_ERROR_CHECK(ulp_lp_core_run(&cfg));
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

    QueueHandle_t uiInputQueue{xQueueCreate(5, sizeof(QueueValueType))};
    configASSERT(uiInputQueue);

    EventGroupHandle_t sleepEventGroup{xEventGroupCreate()};
    configASSERT(sleepEventGroup);

    SensorTaskInterface sensorTaskInterface{uiInputQueue};
    WifiTaskInterface wifiTaskInterface{uiInputQueue, sleepEventGroup};

    EventBits_t uxBitsToWaitFor{};

    constexpr uint64_t sntpSyncTimeUs{1000000LL * 3600LL * CONFIG_SNTP_INTERVAL_HOURS};

    esp_sleep_wakeup_cause_t cause{esp_sleep_get_wakeup_cause()};
    switch (cause)
    {
    case ESP_SLEEP_WAKEUP_UNDEFINED:
    {
        ESP_LOGI(TAG, "Initial boot");

#if SOC_LP_CORE_SUPPORTED && CONFIG_ULP_COPROC_TYPE_LP_CORE
        initLpCore(&sensorTaskInterface);

        TaskHandle_t xHandleSntp{nullptr};
        xTaskCreate(task_sntp, "sntp", SNTP_TASK_STACK_SIZE, static_cast<void *>(&wifiTaskInterface), tskIDLE_PRIORITY + 1, &xHandleSntp);
        configASSERT(xHandleSntp);

        uxBitsToWaitFor |= SNTP_READY_FOR_DEEP_SLEEP;

        break;
#endif
    }
    case ESP_SLEEP_WAKEUP_ULP:
    {
        ESP_LOGI(TAG, "LP core wakeup");
        // TODO Start UI without display reinit, with backlight off, in case user presses button while we are awake
        break;
    }
    case ESP_SLEEP_WAKEUP_GPIO:
    {
        ESP_LOGI(TAG, "GPIO wakeup");
        SensorData sensorData{};
        sensorData.m_illuminance = ulp_illuminance;
        sensorData.m_humidity = ulp_humidity;
        sensorData.m_temperature = static_cast<int32_t>(ulp_temperature);
        sensorData.m_pressure = ulp_pressure;

        for (size_t i = 0; i < numberOfSensorReadingsSaved; ++i)
        {
            sensorData.m_averageHumidity[i] = static_cast<uint32_t>((&ulp_averageHumidity)[i]);
            sensorData.m_averageTemperatureCentrigrade[i] = static_cast<int32_t>((&ulp_averageTemperature)[i]);
        }

        sensorData.m_hoursTracked = ulp_hoursTracked;

        QueueValueType queueData{sensorData};
        xQueueSend(uiInputQueue, &queueData, portMAX_DELAY);
        break;
    }
    case ESP_SLEEP_WAKEUP_TIMER:
    {
        ESP_LOGI(TAG, "Timer wakeup");
        TaskHandle_t xHandleSntp{nullptr};
        xTaskCreate(task_sntp, "sntp", SNTP_TASK_STACK_SIZE, static_cast<void *>(&wifiTaskInterface), tskIDLE_PRIORITY + 1, &xHandleSntp);
        configASSERT(xHandleSntp);

        uxBitsToWaitFor |= SNTP_READY_FOR_DEEP_SLEEP;

        break;
    }
    default:
    {
        ESP_LOGI(TAG, "Wakeup cause = %d", cause);
        break;
    }
    }

#if !(SOC_LP_CORE_SUPPORTED && CONFIG_ULP_COPROC_TYPE_LP_CORE)
    TaskHandle_t xHandleSensor{nullptr};
    xTaskCreate(task_sensor, "sensor", STACK_SIZE_SENSOR_TASK, static_cast<void *>(&sensorTaskInterface), tskIDLE_PRIORITY + 1, &xHandleSensor);
    configASSERT(xHandleSensor);
#endif

    Ui::UiTaskInterface uiTaskInterface{};
    uiTaskInterface.m_queue_in = sensorTaskInterface.m_uiInputQueue;
    uiTaskInterface.m_sleepEventGroup = sleepEventGroup;
    TaskHandle_t xHandleDisplay{nullptr};
    xTaskCreate(Ui::run_task_wrapper, "lvgl", LVGL_TASK_STACK_SIZE, static_cast<void *>(&uiTaskInterface), tskIDLE_PRIORITY + 1, &xHandleDisplay);
    configASSERT(xHandleDisplay);
    uxBitsToWaitFor |= UI_READY_FOR_DEEP_SLEEP;

    initButton(uiTaskInterface);

    // Wait for all tasks to signal that they are ready for deep sleep
    if (uxBitsToWaitFor != 0)
    {
        // Skip wait if no tasks need to be waited on, asserts otherwise
        xEventGroupWaitBits(sleepEventGroup,
                            uxBitsToWaitFor,
                            pdFALSE,
                            pdTRUE,
                            portMAX_DELAY);
    }

    // Disable timer wake-up enabled for FreeRTOS tickless
    pm_config.light_sleep_enable = false;
    ESP_ERROR_CHECK(esp_pm_configure(&pm_config));

    // Enable button wake-up also from deep sleep and hold pin
    ESP_ERROR_CHECK(esp_deep_sleep_enable_gpio_wakeup(1ULL << CONFIG_BUTTON_GPIO, ESP_GPIO_WAKEUP_GPIO_LOW));

    if ((ESP_SLEEP_WAKEUP_UNDEFINED == cause) || (ESP_SLEEP_WAKEUP_TIMER == cause))
    {
        // Sync SNTP after certain period w.r.t. first boot or last sync attempt
        ESP_ERROR_CHECK(esp_sleep_enable_timer_wakeup(sntpSyncTimeUs));
    }

    ESP_LOGI(TAG, "Entering deep sleep");
    esp_deep_sleep_start();
}