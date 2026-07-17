idf_component_register(SRCS "atlas.c"
                    INCLUDE_DIRS "."
                    REQUIRES nvs_flash esp_timer json fans esp_wifi esp_driver_uart esp_driver_gpio shelf_manager bnb_protocol

)

# cJSON_ArrayForEach is a false positive under -Werror=maybe-uninitialized on some
# xtensa-esp-elf gcc versions: the loop variable is genuinely initialized by the
# macro's own for-loop clause, gcc just can't always prove it through the expansion.
target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-error=maybe-uninitialized)
