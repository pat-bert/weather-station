idf_component_register(
    SRCS 
        "main.cpp"

        "bh1750/bh1750_defs.hpp"

        "bmp280/bmp2_defs.h"
        "bmp280/bmp2.c"
        "bmp280/bmp2.h"

        "sensor/sensor_task_interface.hpp"
        "sensor/task_sensor.cpp"
        "sensor/task_sensor.hpp"

        "st7735/esp_lcd_panel_st7735.c"
        "st7735/esp_lcd_panel_custom_vendor.h"

        "ui/ui_task_interface.hpp"
        "ui/task_ui.cpp"
        "ui/task_ui.hpp"

        "wifi/wifi_client.cpp"
        "wifi/wifi_client.hpp"
    INCLUDE_DIRS 
        "."
        "bh1750"
        "bmp280"
        "sensor"
        "st7735"
        "ui"
        "wifi"
)