idf_component_register(SRCS "filler_fsm.c" "filler_strategy.c" "filler_strategy_heuristic.c" "filler_strategy_adaptive_heuristic.c" "filler_strategy_flow_control.c" "filler_strategy_flow_cascade.c" "filler_strategy_manual.c"
                       INCLUDE_DIRS "."
                       PRIV_INCLUDE_DIRS "../../main"
                       REQUIRES app gate motor driver esp_timer hx711 buzzer telemetry nvs_flash)
