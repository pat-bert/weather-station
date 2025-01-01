#include "task_sntp.hpp"

#include "wifi_client.hpp"

#include "esp_log.h"
#include "esp_netif_sntp.h"
#include "esp_sntp.h"

const char *TAG{"sntp"};

void sntp_time_sync_callback(struct timeval *tv)
{
    settimeofday(tv, NULL);
    ESP_LOGI(TAG, "Time synchronized");
    sntp_set_sync_status(SNTP_SYNC_STATUS_COMPLETED);
}

void task_sntp(void *arg)
{
    // Set timezone
    setenv("TZ", CONFIG_TIMEZONE, 1);
    tzset();

    WifiClient wifiClient{};

    while (true)
    {
        ESP_ERROR_CHECK(wifiClient.init());
        ESP_ERROR_CHECK(wifiClient.connect(CONFIG_WIFI_SSID, CONFIG_WIFI_PASSWORD));

        esp_sntp_config_t config = ESP_NETIF_SNTP_DEFAULT_CONFIG(CONFIG_SNTP_SERVER_URL);
        config.start = true;
        config.sync_cb = sntp_time_sync_callback;
        ESP_ERROR_CHECK(esp_netif_sntp_init(&config));

        // Wait for time to be synced
        int retry = 0;
        const int retry_count = 15;
        while ((esp_netif_sntp_sync_wait(pdMS_TO_TICKS(2000)) == ESP_ERR_TIMEOUT) && (++retry < retry_count))
        {
            ESP_LOGI(TAG, "Waiting for system time to be set... (%d/%d)", retry, retry_count);
        }

        esp_netif_sntp_deinit();

        ESP_ERROR_CHECK(wifiClient.disconnect());
        ESP_ERROR_CHECK(wifiClient.deinit());

        ESP_LOGI(TAG, "Free stack: %u", uxTaskGetStackHighWaterMark(nullptr));

        // The intermediate value becomes too large when 1000 is inside the macro
        vTaskDelay(pdMS_TO_TICKS(CONFIG_SNTP_INTERVAL_HOURS * 60 * 60) * 1000);
    }
}
