set(req esp_wifi nvs_flash)

idf_component_register(
    SRCS JRC_WiFi.c
    INCLUDE_DIRS .
    REQUIRES ${req}
)