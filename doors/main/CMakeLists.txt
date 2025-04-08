   idf_component_register(SRCS "doors.c"
           INCLUDE_DIRS "."
           REQUIRES esp_ws28xx mqtt nvs_flash esp_wifi)