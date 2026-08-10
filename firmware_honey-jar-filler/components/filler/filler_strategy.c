#include "filler_strategy.h"

extern const filler_strategy_ops_t g_filler_strategy_heuristic_ops;
extern const filler_strategy_ops_t g_filler_strategy_adaptive_heuristic_ops;
extern const filler_strategy_ops_t g_filler_strategy_flow_control_ops;
extern const filler_strategy_ops_t g_filler_strategy_flow_cascade_ops;
extern const filler_strategy_ops_t g_filler_strategy_sequence_ops;
extern const filler_strategy_ops_t g_filler_strategy_manual_ops;

const filler_strategy_ops_t *filler_strategy_ops_for_app(app_fill_strategy_t strategy)
{
    switch (strategy) {
    case APP_FILL_STRATEGY_HEURISTIC:
        return &g_filler_strategy_heuristic_ops;
    case APP_FILL_STRATEGY_ADAPTIVE_HEURISTIC:
        return &g_filler_strategy_adaptive_heuristic_ops;
    case APP_FILL_STRATEGY_FLOW_CONTROL:
        return &g_filler_strategy_flow_control_ops;
    case APP_FILL_STRATEGY_FLOW_CASCADE:
        return &g_filler_strategy_flow_cascade_ops;
    case APP_FILL_STRATEGY_SEQUENCE:
        return &g_filler_strategy_sequence_ops;
    case APP_FILL_STRATEGY_MANUAL:
        return &g_filler_strategy_manual_ops;
    default:
        return &g_filler_strategy_heuristic_ops;
    }
}

const filler_strategy_ops_t *filler_strategy_active_ops(void)
{
    return filler_strategy_ops_for_app(app_fill_strategy_get_active());
}
