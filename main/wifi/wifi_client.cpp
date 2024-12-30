#include "wifi_client.hpp"

#include "nvs_flash.h"
#include "esp_log.h"
#include "esp_wifi.h"

#include "freertos/FreeRTOS.h"

#include <inttypes.h>
#include <string.h>

#define TAG "wifi-client"

#define WIFI_AUTHMODE WIFI_AUTH_WPA2_PSK

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT BIT1

static void ip_event_cb(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    ESP_LOGI(TAG, "Handling IP event, event code 0x%" PRIx32, event_id);

    EventState *eventArguments{static_cast<EventState *>(arg)};

    switch (event_id)
    {
    case (IP_EVENT_STA_GOT_IP):
    {
        ip_event_got_ip_t *event_ip = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event_ip->ip_info.ip));
        eventArguments->m_wifi_retry_count = 0;
        xEventGroupSetBits(eventArguments->m_wifi_event_group, WIFI_CONNECTED_BIT);
        break;
    }
    case (IP_EVENT_STA_LOST_IP):
    {
        ESP_LOGI(TAG, "Lost IP");
        break;
    }
    case (IP_EVENT_GOT_IP6):
    {
        ip_event_got_ip6_t *event_ip6 = (ip_event_got_ip6_t *)event_data;
        ESP_LOGI(TAG, "Got IPv6: " IPV6STR, IPV62STR(event_ip6->ip6_info.ip));
        eventArguments->m_wifi_retry_count = 0;
        xEventGroupSetBits(eventArguments->m_wifi_event_group, WIFI_CONNECTED_BIT);
        break;
    }
    default:
    {
        ESP_LOGI(TAG, "IP event not handled");
        break;
    }
    }
}

static void wifi_event_cb(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    ESP_LOGI(TAG, "Handling Wi-Fi event, event code 0x%" PRIx32, event_id);

    EventState *eventArguments{static_cast<EventState *>(arg)};

    switch (event_id)
    {
    case (WIFI_EVENT_WIFI_READY):
    {
        ESP_LOGI(TAG, "Wi-Fi ready");
        break;
    }
    case (WIFI_EVENT_SCAN_DONE):
    {
        ESP_LOGI(TAG, "Wi-Fi scan done");
        break;
    }
    case (WIFI_EVENT_STA_START):
    {
        ESP_LOGI(TAG, "Wi-Fi started, connecting to AP...");
        esp_wifi_connect();
        break;
    }
    case (WIFI_EVENT_STA_STOP):
    {
        ESP_LOGI(TAG, "Wi-Fi stopped");
        break;
    }
    case (WIFI_EVENT_STA_CONNECTED):
    {
        ESP_LOGI(TAG, "Wi-Fi connected");
        break;
    }
    case (WIFI_EVENT_STA_DISCONNECTED):
    {
        ESP_LOGI(TAG, "Wi-Fi disconnected");
        if (eventArguments->m_allowReconnection && (eventArguments->m_wifi_retry_count < eventArguments->m_wifi_retry_attempts))
        {
            ESP_LOGI(TAG, "Retrying to connect to Wi-Fi network...");
            esp_wifi_connect();
            eventArguments->m_wifi_retry_count++;
        }
        else
        {
            xEventGroupSetBits(eventArguments->m_wifi_event_group, WIFI_FAIL_BIT);
        }
        break;
    }
    case (WIFI_EVENT_STA_AUTHMODE_CHANGE):
    {
        ESP_LOGI(TAG, "Wi-Fi authmode changed");
        break;
    }
    default:
    {
        ESP_LOGI(TAG, "Wi-Fi event not handled");
        break;
    }
    }
}

esp_err_t WifiClient::init(void)
{
    // Initialize Non-Volatile Storage (NVS)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }

    ret = esp_netif_init();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to initialize TCP/IP network stack");
        return ret;
    }

    ret = esp_event_loop_create_default();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to create default event loop");
        return ret;
    }

    ret = esp_wifi_set_default_wifi_sta_handlers();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to set default handlers");
        return ret;
    }

    m_netif = esp_netif_create_default_wifi_sta();
    if (m_netif == NULL)
    {
        ESP_LOGE(TAG, "Failed to create default WiFi STA interface");
        return ESP_FAIL;
    }

    // Wi-Fi stack configuration parameters
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        wifi_event_cb,
                                                        &m_event_state,
                                                        &m_wifi_event_handler));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        ip_event_cb,
                                                        &m_event_state,
                                                        &m_ip_event_handler));
    return ret;
}

esp_err_t WifiClient::connect(const char *wifi_ssid, const char *wifi_password)
{
    wifi_config_t wifi_config = {};
    // this sets the weakest authmode accepted in fast scan mode (default)
    wifi_config.sta.threshold.authmode = WIFI_AUTHMODE;

    strncpy((char *)wifi_config.sta.ssid, wifi_ssid, sizeof(wifi_config.sta.ssid));
    strncpy((char *)wifi_config.sta.password, wifi_password, sizeof(wifi_config.sta.password));

    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_MIN_MODEM));     // default is WIFI_PS_MIN_MODEM
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM)); // default is WIFI_STORAGE_FLASH

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));

    ESP_LOGI(TAG, "Connecting to Wi-Fi network: %s", wifi_config.sta.ssid);
    m_event_state.m_allowReconnection = true;
    m_event_state.m_wifi_retry_count = 0;
    ESP_ERROR_CHECK(esp_wifi_start());

    EventBits_t bits = xEventGroupWaitBits(m_event_state.m_wifi_event_group, WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                           pdFALSE, pdFALSE, portMAX_DELAY);

    if (bits & WIFI_CONNECTED_BIT)
    {
        ESP_LOGI(TAG, "Connected to Wi-Fi network: %s", wifi_config.sta.ssid);
        return ESP_OK;
    }
    else if (bits & WIFI_FAIL_BIT)
    {
        ESP_LOGE(TAG, "Failed to connect to Wi-Fi network: %s", wifi_config.sta.ssid);
        return ESP_FAIL;
    }

    ESP_LOGE(TAG, "Unexpected Wi-Fi error");
    return ESP_FAIL;
}

esp_err_t WifiClient::disconnect(void)
{
    m_event_state.m_allowReconnection = false;
    return esp_wifi_disconnect();
}

esp_err_t WifiClient::deinit(void)
{
    esp_err_t ret = esp_wifi_stop();
    if (ret == ESP_ERR_WIFI_NOT_INIT)
    {
        ESP_LOGE(TAG, "Wi-Fi stack not initialized");
        return ret;
    }

    ESP_ERROR_CHECK(esp_wifi_deinit());
    ESP_ERROR_CHECK(esp_wifi_clear_default_wifi_driver_and_handlers(m_netif));
    esp_netif_destroy(m_netif);

    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(IP_EVENT, ESP_EVENT_ANY_ID, m_ip_event_handler));
    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, m_wifi_event_handler));

    return ESP_OK;
}
