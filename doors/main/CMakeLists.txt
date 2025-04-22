   idf_component_register(SRCS "led.c" "doors.c"
           INCLUDE_DIRS "."
           REQUIRES esp_ws28xx mqtt nvs_flash esp_wifi)