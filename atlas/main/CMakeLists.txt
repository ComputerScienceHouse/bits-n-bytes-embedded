idf_component_register(SRCS "atlas.c"
                    INCLUDE_DIRS "."
                    REQUIRES nvs_flash esp_timer json fans esp_wifi esp_driver_uart esp_driver_gpio shelf_manager

)
