// Parameter definitions for defaults + UI metadata.
// Columns: name, label, unit, default, min, max, step, brief, detail, group.
// Use DEFAULT/MIN/MAX/STEP to make numeric fields easier to read.

#ifndef APP_PARAMS_DEF_H
#define APP_PARAMS_DEF_H

// Bump APP_PARAMS_VERSION to force defaults reload.
#define APP_PARAMS_VERSION 9

#define VAR(x)     x
#define LABEL(x)   x
#define UNIT(x)    x
#define DEFAULT(x) (x)
#define MIN(x)  (x)
#define MAX(x)  (x)
#define STEP(x) (x)
#define BRIEF(x)  x
#define DETAIL(x) x
#define GROUP(x)  x

#define APP_PARAMS_DEF_LIST(APP_PARAM_FLOAT, APP_PARAM_U32, APP_PARAM_U8) \
    /*=== Target + verification ===*/                                    \
    APP_PARAM_FLOAT(VAR(target_grams), LABEL("Target"), UNIT("g"),          \
                    DEFAULT(150.0f), MIN(0.0f), MAX(2000.0f), STEP(5.0f),    \
                    BRIEF("Target filled mass per jar"),                    \
                    DETAIL("Final filled weight target for each jar"),      \
                    GROUP("Target + verification"))                         \
    APP_PARAM_FLOAT(VAR(target_tol_low_pct), LABEL("Tol -"), UNIT("%"),     \
                    DEFAULT(3.0f), MIN(0.0f), MAX(50.0f), STEP(0.5f),        \
                    BRIEF("Below target -> refill"),                        \
                    DETAIL("If under target by this %, reopen gate to add more"), \
                    GROUP("Target + verification"))                         \
    APP_PARAM_FLOAT(VAR(target_tol_high_pct), LABEL("Tol +"), UNIT("%"),    \
                    DEFAULT(20.0f), MIN(0.0f), MAX(100.0f), STEP(1.0f),      \
                    BRIEF("Above target -> fault"),                         \
                    DETAIL("If over target by this %, stop and fault"),     \
                    GROUP("Target + verification"))                         \
    APP_PARAM_U32(VAR(fill_timeout_ms), LABEL("Fill timeout"), UNIT("ms"),  \
                  DEFAULT(180000), MIN(1000), MAX(900000), STEP(1000),       \
                  BRIEF("Max time in FILL before fault"),                   \
                  DETAIL("Safety timeout to prevent endless filling"),      \
                  GROUP("Target + verification"))                           \
                                                                          \
    /*=== Honey flow tuning ===*/                                         \
    APP_PARAM_FLOAT(VAR(near_close_delta_g), LABEL("Near close"), UNIT("g"), \
                    DEFAULT(60.0f), MIN(0.0f), MAX(500.0f), STEP(1.0f),     \
                    BRIEF("Within delta, partially close gate"),           \
                    DETAIL("Start slowing flow when this close to target"), \
                    GROUP("Honey flow tuning"))                             \
    APP_PARAM_FLOAT(VAR(near_close_gate_pct), LABEL("Near close %"), UNIT("%"), \
                    DEFAULT(16.0f), MIN(0.0f), MAX(100.0f), STEP(1.0f),     \
                    BRIEF("Partial opening near target"),                  \
                    DETAIL("Gate opening used after near-close delta reached"), \
                    GROUP("Honey flow tuning"))                             \
    APP_PARAM_FLOAT(VAR(max_gate_pct), LABEL("Max gate %"), UNIT("%"),      \
                    DEFAULT(30.0f), MIN(0.0f), MAX(100.0f), STEP(1.0f),     \
                    BRIEF("Max opening during bulk fill"),                 \
                    DETAIL("Caps full-open position to reduce flow"),      \
                    GROUP("Honey flow tuning"))                             \
    APP_PARAM_FLOAT(VAR(close_early_pct), LABEL("Close early %"), UNIT("%"), \
                    DEFAULT(10.0f), MIN(0.0f), MAX(100.0f), STEP(1.0f),     \
                    BRIEF("Close before target to compensate drips"),      \
                    DETAIL("Thick honey usually needs a larger value; thin honey needs less"), \
                    GROUP("Honey flow tuning"))                             \
    APP_PARAM_U32(VAR(drip_delay_ms), LABEL("Drip delay"), UNIT("ms"),      \
                  DEFAULT(4000), MIN(0), MAX(20000), STEP(100),             \
                  BRIEF("Wait after closing gate for drips"),              \
                  DETAIL("Let residual honey fall into jar"),              \
                  GROUP("Honey flow tuning"))                               \
                                                                          \
    /*=== Glass detection ===*/                                           \
    APP_PARAM_FLOAT(VAR(empty_glass_min_g), LABEL("Empty min"), UNIT("g"), \
                    DEFAULT(100.0f), MIN(0.0f), MAX(1000.0f), STEP(1.0f),   \
                    BRIEF("Below this -> no jar"),                         \
                    DETAIL("Empty jar weight window; outside range skips slot"), \
                    GROUP("Glass detection"))                               \
    APP_PARAM_FLOAT(VAR(empty_glass_max_g), LABEL("Empty max"), UNIT("g"), \
                    DEFAULT(200.0f), MIN(0.0f), MAX(2000.0f), STEP(1.0f),   \
                    BRIEF("Above this -> jar not empty"),                  \
                    DETAIL("Empty jar weight window; outside range skips slot"), \
                    GROUP("Glass detection"))                               \
                                                                          \
    /*=== Mechanics / motion ===*/                                        \
    APP_PARAM_U32(VAR(advance_timeout_ms), LABEL("Advance timeout"), UNIT("ms"), \
                  DEFAULT(4000), MIN(100), MAX(20000), STEP(100),          \
                  BRIEF("Max time to find position switch"),              \
                  DETAIL("Motor stop timeout while searching for slot"),  \
                  GROUP("Mechanics / motion"))                             \
    APP_PARAM_U32(VAR(find_ignore_ms), LABEL("Find ignore"), UNIT("ms"),   \
                  DEFAULT(500), MIN(0), MAX(5000), STEP(50),               \
                  BRIEF("Ignore POS switch after motor start"),           \
                  DETAIL("Prevents immediate stop when switch already low"), \
                  GROUP("Mechanics / motion"))                             \
    APP_PARAM_U32(VAR(slot_settle_ms), LABEL("Slot settle"), UNIT("ms"),   \
                  DEFAULT(1500), MIN(0), MAX(10000), STEP(100),            \
                  BRIEF("Wait after slot found before weighing"),         \
                  DETAIL("Allows motor and scale to settle"),             \
                  GROUP("Mechanics / motion"))                             \
    APP_PARAM_U8(VAR(slots_total), LABEL("Slots total"), UNIT(""),         \
                 DEFAULT(3), MIN(1), MAX(20), STEP(1),                     \
                 BRIEF("Number of jars per run"),                          \
                 DETAIL("Stops after this many positions"),               \
                 GROUP("Mechanics / motion"))

#endif // APP_PARAMS_DEF_H
