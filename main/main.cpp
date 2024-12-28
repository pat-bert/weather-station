
#include "esp_log.h"
#include "esp_pm.h"
#include "esp_sntp.h"

#include "sensor/task_sensor.hpp"
#include "ui/task_ui.hpp"
#include "wifi/wifi_client.hpp"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

// FreeRTOS
#define STACK_SIZE_SENSOR_TASK (3000)
#define LVGL_TASK_STACK_SIZE (2800)

extern "C" void app_main(void)
{
    const char *TAG{"main"};

    WifiClient wifi_client{};
    ESP_ERROR_CHECK(wifi_client.init());

    esp_err_t ret{wifi_client.connect(CONFIG_WIFI_SSID, CONFIG_WIFI_PASSWORD)};
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to connect to Wi-Fi network");
    }

    // Set timezone
    setenv("TZ", CONFIG_TIMEZONE, 1);
    tzset();

    esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
    esp_sntp_setservername(0, CONFIG_SNTP_SERVER_URL);
    esp_sntp_set_sync_interval(CONFIG_SNTP_INTERVAL_HOURS * 3600 * 1000);
    esp_sntp_set_sync_mode(SNTP_SYNC_MODE_SMOOTH);
    esp_sntp_init();

    QueueHandle_t measurementQueue{xQueueCreate(5, sizeof(SensorData))};
    configASSERT(measurementQueue);

    TaskHandle_t xHandleSensor{nullptr};
    SensorTaskInterface sensorTaskInterface{measurementQueue};
    xTaskCreatePinnedToCore(task_sensor, "sensor", STACK_SIZE_SENSOR_TASK, static_cast<void *>(&sensorTaskInterface), tskIDLE_PRIORITY + 1, &xHandleSensor, 0);
    configASSERT(xHandleSensor);

    UiTaskInterface uiTaskInterface{};
    uiTaskInterface.m_measurementQueue_in = sensorTaskInterface.m_measurementQueue_out;
    TaskHandle_t xHandleDisplay{nullptr};
    xTaskCreatePinnedToCore(task_lvgl, "lvgl", LVGL_TASK_STACK_SIZE, static_cast<void *>(&uiTaskInterface), tskIDLE_PRIORITY + 1, &xHandleDisplay, 1);
    configASSERT(xHandleDisplay);

    esp_pm_config_t pm_config{};
    ESP_ERROR_CHECK(esp_pm_get_configuration(&pm_config));
#if CONFIG_FREERTOS_USE_TICKLESS_IDLE
    pm_config.light_sleep_enable = true;
#endif
    ESP_ERROR_CHECK(esp_pm_configure(&pm_config));

    vTaskSuspend(nullptr);
}