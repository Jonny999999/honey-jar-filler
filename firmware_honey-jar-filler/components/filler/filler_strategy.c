#include "filler_strategy.h"

extern const filler_strategy_ops_t g_filler_strategy_heuristic_ops;
extern const filler_strategy_ops_t g_filler_strategy_adaptive_heuristic_ops;

const filler_strategy_ops_t *filler_strategy_ops_for_app(app_fill_strategy_t strategy)
{
    switch (strategy) {
    case APP_FILL_STRATEGY_HEURISTIC:
        return &g_filler_strategy_heuristic_ops;
    case APP_FILL_STRATEGY_ADAPTIVE_HEURISTIC:
        return &g_filler_strategy_adaptive_heuristic_ops;
    default:
        return &g_filler_strategy_heuristic_ops;
    }
}

const filler_strategy_ops_t *filler_strategy_active_ops(void)
{
    return filler_strategy_ops_for_app(app_fill_strategy_get_active());
}
