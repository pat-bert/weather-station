idf_component_register(
    SRCS 
        "main.cpp"

        "bh1750/bh1750.cpp"
        "bh1750/bh1750.hpp"

        "bme280/bme280_defs.h"
        "bme280/bme280.c"
        "bme280/bme280.h"

        "interfaces/interface_sensor.hpp"

        "sensor/sensor_task_interface.hpp"
        "sensor/task_sensor.cpp"
        "sensor/task_sensor.hpp"

        "st7735/esp_lcd_panel_st7735.c"
        "st7735/esp_lcd_panel_custom_vendor.h"

        "ui/fonts/lv_font_montserrat_8_german.c"
        "ui/icons/drop.c"
        "ui/icons/sun.c"
        "ui/backlight.cpp"
        "ui/backlight.hpp"
        "ui/task_ui.cpp"
        "ui/task_ui.hpp"
        "ui/ui_task_interface.hpp"

        "wifi/task_sntp.cpp"
        "wifi/task_sntp.hpp"
        "wifi/wifi_client.cpp"
        "wifi/wifi_client.hpp"
        "wifi/wifi_task_interface.hpp"
    INCLUDE_DIRS 
        "."
        "bh1750"
        "bme280"
        "interfaces"
        "sensor"
        "st7735"
        "ui"
        "wifi"
    REQUIRES 
        "driver"
        "esp_event"
        "esp_lcd"
        "esp_netif"
        "esp_wifi"
        "nvs_flash"
        "ulp"
        "wifi_provisioning"
    WHOLE_ARCHIVE
)

add_compile_definitions(BME280_64BIT_ENABLE)

if(CONFIG_ULP_COPROC_TYPE_LP_CORE)
ulp_add_project("ulp_lp_sensor" "${CMAKE_SOURCE_DIR}/main/ulp/")
endif()