#include "esp_log.h"
#include "esp_pm.h"
#include "esp_sleep.h"

#include "sensor/task_sensor.hpp"
#include "ui/task_ui.hpp"
#include "wifi/task_sntp.hpp"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

// FreeRTOS
#define STACK_SIZE_SENSOR_TASK (3000)
#define LVGL_TASK_STACK_SIZE (3000)

extern "C" void app_main(void)
{
    const char *TAG{"main"};

    esp_pm_config_t pm_config{};
    ESP_ERROR_CHECK(esp_pm_get_configuration(&pm_config));
#if CONFIG_FREERTOS_USE_TICKLESS_IDLE
    pm_config.light_sleep_enable = true;
#endif
    ESP_ERROR_CHECK(esp_pm_configure(&pm_config));
    ESP_ERROR_CHECK(esp_sleep_pd_config(ESP_PD_DOMAIN_MODEM, ESP_PD_OPTION_ON));
    ESP_ERROR_CHECK(esp_sleep_pd_config(ESP_PD_DOMAIN_MODEM, ESP_PD_OPTION_OFF));

    TaskHandle_t xHandleSntp{nullptr};
    xTaskCreate(task_sntp, "sntp", 3000, nullptr, tskIDLE_PRIORITY + 1, &xHandleSntp);
    configASSERT(xHandleSntp);

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

    vTaskSuspend(nullptr);
}