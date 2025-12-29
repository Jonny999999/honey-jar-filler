idf_component_register(SRCS "main.c" "scale_hx711.c"
                       INCLUDE_DIRS "."
                       REQUIRES nvs_flash hx711 esp_timer
                       )

