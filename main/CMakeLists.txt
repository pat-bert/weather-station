idf_component_register(
    SRCS 
        "main.cpp"

        "bmp280/bmp2_defs.h"
        "bmp280/bmp2.c"
        "bmp280/bmp2.h"

        "sensor/sensor_task_interface.hpp"
        "sensor/task_sensor.cpp"
        "sensor/task_sensor.hpp"

        "st7735/esp_lcd_panel_st7735.c"
        "st7735/esp_lcd_panel_custom_vendor.h"

        "ui/ui_task_interface.hpp"

        "wifi/wifi_client.cpp"
        "wifi/wifi_client.hpp"
    INCLUDE_DIRS 
        "."
        "bmp280"
        "sensor"
        "st7735"
        "ui"
        "wifi"
)