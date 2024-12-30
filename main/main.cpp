#include "esp_log.h"
#include "esp_pm.h"
#include "esp_sntp.h"
#include "esp_sleep.h"

#include "sensor/task_sensor.hpp"
#include "ui/task_ui.hpp"
#include "wifi/wifi_client.hpp"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

// FreeRTOS
#define STACK_SIZE_SENSOR_TASK (3000)
#define LVGL_TASK_STACK_SIZE (3000)

static void task_sntp(void *arg)
{
    const char *TAG{"sntp"};

    // Set timezone
    setenv("TZ", CONFIG_TIMEZONE, 1);
    tzset();

    WifiClient wifiClient{};

    while (true)
    {
        ESP_ERROR_CHECK(wifiClient.init());
        ESP_ERROR_CHECK(wifiClient.connect(CONFIG_WIFI_SSID, CONFIG_WIFI_PASSWORD));

        esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
        esp_sntp_setservername(0, CONFIG_SNTP_SERVER_URL);
        // SNTP enforces a minimum of 15s
        esp_sntp_set_sync_interval(15000);
        esp_sntp_init();

        // Wait for time to be synced
        int retry = 0;
        const int retry_count = 15;
        while (esp_sntp_get_sync_status() == SNTP_SYNC_STATUS_RESET && ++retry < retry_count)
        {
            ESP_LOGI(TAG, "Waiting for time synchronisation, attempt #%d", retry);
            vTaskDelay(pdMS_TO_TICKS(2000));
        }

        ESP_LOGI(TAG, "Time synchronisation successful, stopping SNTP service");
        esp_sntp_stop();

        ESP_ERROR_CHECK(wifiClient.disconnect());
        ESP_ERROR_CHECK(wifiClient.deinit());

        ESP_LOGI(TAG, "Free stack: %u", uxTaskGetStackHighWaterMark(nullptr));
        vTaskDelay(pdMS_TO_TICKS(CONFIG_SNTP_INTERVAL_HOURS * 60 * 60 * 1000));
    }
}

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
    xTaskCreatePinnedToCore(task_sensor, "sensor", STACK_SIZE_SENSOR_TASK, static_cast<void *>(&sensorTaskInterface), tskIDLE_PRIORITY + 1, &xHandleSensor, 0);
    configASSERT(xHandleSensor);

    UiTaskInterface uiTaskInterface{};
    uiTaskInterface.m_measurementQueue_in = sensorTaskInterface.m_measurementQueue_out;
    TaskHandle_t xHandleDisplay{nullptr};
    xTaskCreatePinnedToCore(task_lvgl, "lvgl", LVGL_TASK_STACK_SIZE, static_cast<void *>(&uiTaskInterface), tskIDLE_PRIORITY + 1, &xHandleDisplay, 1);
    configASSERT(xHandleDisplay);

    vTaskSuspend(nullptr);
}