#ifndef WIFI_CLIENT_HPP_INCLUDED
#define WIFI_CLIENT_HPP_INCLUDED

#include "esp_err.h"
#include "esp_event.h"
#include "esp_wifi.h"

struct EventState
{
    EventState(const int retry_attempts = 3) : m_wifi_retry_attempts{retry_attempts}
    {
        m_wifi_event_group = xEventGroupCreate();
    }

    ~EventState()
    {
        if (m_wifi_event_group)
        {
            vEventGroupDelete(m_wifi_event_group);
        };
    }

    EventGroupHandle_t m_wifi_event_group;
    int m_wifi_retry_attempts;
    int m_wifi_retry_count;
    bool m_allowReconnection{true};
};

class WifiClient
{
public:
    esp_err_t init(void);

    esp_err_t connect(const char *wifi_ssid, const char *wifi_password);

    esp_err_t disconnect(void);

    esp_err_t deinit(void);

private:
    EventState m_event_state;

    esp_netif_t *m_netif = nullptr;
    esp_event_handler_instance_t m_ip_event_handler;
    esp_event_handler_instance_t m_wifi_event_handler;
};

#endif
