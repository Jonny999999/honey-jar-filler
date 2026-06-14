idf_component_register(SRCS "filler_fsm.c" "filler_strategy.c" "filler_strategy_heuristic.c"
                       INCLUDE_DIRS "."
                       PRIV_INCLUDE_DIRS "../../main"
                       REQUIRES app gate motor driver esp_timer hx711 buzzer telemetry)
