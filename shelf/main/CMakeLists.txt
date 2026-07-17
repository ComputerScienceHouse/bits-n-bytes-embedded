idf_component_register(SRCS "shelf.c"
                    INCLUDE_DIRS "."
                    REQUIRES bnb_protocol bnb_ack
                    PRIV_REQUIRES mqtt driver nvs_flash esp_netif esp_wifi hx711 esp_timer json)
