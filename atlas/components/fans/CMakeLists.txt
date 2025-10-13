idf_component_register(SRCS "fans.c"
                    INCLUDE_DIRS "include"
                    REQUIRES esp_driver_ledc esp_driver_gpio
)
