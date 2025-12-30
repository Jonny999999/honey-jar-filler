idf_component_register(SRCS "main.c" "iotest.c" "scale_hx711.c"
                       INCLUDE_DIRS "."
                       REQUIRES 
                            nvs_flash 
                            app gate motor filler ui buzzer led_strip
                       )
