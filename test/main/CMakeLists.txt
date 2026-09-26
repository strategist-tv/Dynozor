set(PROJECT_VER "0.1.0.1")

idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    INCLUDE_DIRS ${CMAKE_CURRENT_LIST_DIR}
    REQUIRES 
        esp_driver_ledc
        esp_driver_uart
        esp_driver_gpio
        esp_event
        nvs_flash 
        esp_timer
)
