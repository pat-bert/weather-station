idf_component_register(
    SRCS 
        "main.cpp"

        "bh1750/bh1750.cpp"
        "bh1750/bh1750.hpp"

        "bmp280/bmp2_defs.h"
        "bmp280/bmp2.c"
        "bmp280/bmp2.h"

        "interfaces/interface_sensor.hpp"

        "sensor/task_sensor.cpp"
        "sensor/task_sensor.hpp"

        "st7735/esp_lcd_panel_st7735.c"
        "st7735/esp_lcd_panel_custom_vendor.h"

        "ui/drop.c"
        "ui/sun.c"
        "ui/task_ui.cpp"
        "ui/task_ui.hpp"
        "ui/ui_task_interface.hpp"

        "wifi/wifi_client.cpp"
        "wifi/wifi_client.hpp"
    INCLUDE_DIRS 
        "."
        "bh1750"
        "bmp280"
        "interfaces"
        "sensor"
        "st7735"
        "ui"
        "wifi"
)