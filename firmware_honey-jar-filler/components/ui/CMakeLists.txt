idf_component_register(SRCS "ui_task.c"
                       INCLUDE_DIRS "."
                       PRIV_INCLUDE_DIRS "../../main"
                       REQUIRES ssd1306 app filler driver encoder hx711 buzzer)
