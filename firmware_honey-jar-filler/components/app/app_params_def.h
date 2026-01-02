// Parameter definitions for defaults + UI metadata.
// Columns: name, label, unit, default, min, max, step, brief, detail, group.
// Use DEFAULT/MIN/MAX/STEP to make numeric fields easier to read.

#ifndef APP_PARAMS_DEF_H
#define APP_PARAMS_DEF_H

// Bump APP_PARAMS_VERSION to force defaults reload.
#define APP_PARAMS_VERSION 10

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
    APP_PARAM_U32(VAR(target_grams), LABEL("Target"), UNIT("g"),            \
                    DEFAULT(150), MIN(10), MAX(2000), STEP(10),               \
                    BRIEF("Target filled mass per jar"),                    \
                    DETAIL("Final filled weight target for each jar (actual content, not including jar)"),      \
                    GROUP("Target + verification"))                         \
    APP_PARAM_U32(VAR(target_tol_low_g), LABEL("Tol below acceptable"), UNIT("g"),          \
                    DEFAULT(10), MIN(0), MAX(200), STEP(1),                  \
                    BRIEF("ok Below target, else -> refill"),                        \
                    DETAIL("Acceptable missing grams. If under target by this many grams, reopens gate"), \
                    GROUP("Target + verification"))                         \
    APP_PARAM_U32(VAR(target_tol_high_g), LABEL("Tol above acceptable"), UNIT("g"),         \
                    DEFAULT(10), MIN(0), MAX(200), STEP(1),                 \
                    BRIEF("ok Above target, else -> fault"),                         \
                    DETAIL("If over target by this many grams, stop and fault"), \
                    GROUP("Target + verification"))                         \
    APP_PARAM_U32(VAR(fill_timeout_ms), LABEL("Fill timeout"), UNIT("ms"),  \
                  DEFAULT(180e3), MIN(10e3), MAX(600e3), STEP(10e3),       \
                  BRIEF("Max time FILLING before fault"),                   \
                  DETAIL("Safety timeout to prevent endless filling. e.g. empty bucket / hardware issue"),      \
                  GROUP("Target + verification"))                           \
                                                                          \
    /*=== Honey flow tuning ===*/                                         \
    APP_PARAM_U32(VAR(near_close_delta_g), LABEL("Near-close threshold g"), UNIT("g"),   \
                    DEFAULT(60), MIN(0), MAX(500), STEP(5),                 \
                    BRIEF("Grams missing to target, gate partially closes"),           \
                    DETAIL("Start slowing flow when this close to target"), \
                    GROUP("Honey flow tuning"))                             \
    APP_PARAM_U32(VAR(near_close_gate_pct), LABEL("Near-close gate %"), UNIT("%"), \
                    DEFAULT(20), MIN(1), MAX(100), STEP(1),                 \
                    BRIEF("Partial opening near target"),                  \
                    DETAIL("Gate opening used after near-close delta reached"), \
                    GROUP("Honey flow tuning"))                             \
    APP_PARAM_U32(VAR(max_gate_pct), LABEL("Max gate %"), UNIT("%"),        \
                    DEFAULT(80), MIN(5), MAX(100), STEP(1),                 \
                    BRIEF("Max opening during initial bulk fill"),                 \
                    DETAIL("Caps full-open position to reduce max flow (e.g. small glass or water)"),      \
                    GROUP("Honey flow tuning"))                             \
    APP_PARAM_U32(VAR(close_early_g), LABEL("Close early threshold"), UNIT("g"),       \
                    DEFAULT(150), MIN(0), MAX(500), STEP(5),                 \
                    BRIEF("Close before target to compensate drips"),      \
                    DETAIL("Thick honey usually needs a larger value; thin honey needs less - equals estimated in-flight mass"), \
                    GROUP("Honey flow tuning"))                             \
    APP_PARAM_U32(VAR(drip_delay_ms), LABEL("Drip delay"), UNIT("ms"),      \
                  DEFAULT(4e3), MIN(0), MAX(30e3), STEP(100),             \
                  BRIEF("Wait after closing gate for drips before verifying"),              \
                  DETAIL("Let residual honey fall into jar before verifying the weight"),              \
                  GROUP("Honey flow tuning"))                               \
                                                                          \
    /*=== Glass detection ===*/                                           \
    APP_PARAM_U32(VAR(empty_glass_min_g), LABEL("Empty glass min"), UNIT("g"),     \
                    DEFAULT(100), MIN(0), MAX(1000), STEP(10),               \
                    BRIEF("Below this -> no jar"),                         \
                    DETAIL("Empty jar weight window; outside range skips slot"), \
                    GROUP("Glass detection"))                               \
    APP_PARAM_U32(VAR(empty_glass_max_g), LABEL("Empty glass max"), UNIT("g"),     \
                    DEFAULT(200), MIN(0), MAX(2000), STEP(10),               \
                    BRIEF("Above this -> jar not empty"),                  \
                    DETAIL("Empty jar weight window; outside range skips slot"), \
                    GROUP("Glass detection"))                               \
                                                                          \
    /*=== Mechanics / motion ===*/                                        \
    APP_PARAM_U32(VAR(advance_timeout_ms), LABEL("Advance timeout"), UNIT("ms"), \
                  DEFAULT(2000), MIN(200), MAX(10000), STEP(100),          \
                  BRIEF("Max time to find position switch"),              \
                  DETAIL("Motor stop timeout while searching for slot (dont spin forever, catch motor fault / sensor fault)"),  \
                  GROUP("Mechanics / motion"))                             \
    APP_PARAM_U32(VAR(find_ignore_ms), LABEL("Find ignore"), UNIT("ms"),   \
                  DEFAULT(500), MIN(0), MAX(2000), STEP(100),               \
                  BRIEF("Ignore POS switch after motor start"),           \
                  DETAIL("Prevents immediate stop when switch already low"), \
                  GROUP("Mechanics / motion"))                             \
    APP_PARAM_U32(VAR(slot_settle_ms), LABEL("Slot settle"), UNIT("ms"),   \
                  DEFAULT(1000), MIN(0), MAX(5000), STEP(100),            \
                  BRIEF("Wait after slot found before verifying empty"),         \
                  DETAIL("Allows motor, jar and scale to settle before starting next step (verify + fill)"),             \
                  GROUP("Mechanics / motion"))                             \
    APP_PARAM_U8(VAR(slots_total), LABEL("Slots total"), UNIT(""),         \
                 DEFAULT(6), MIN(1), MAX(20), STEP(1),                     \
                 BRIEF("Number of jars per run"),                          \
                 DETAIL("Stops after this many positions filled/tried - number of jar slots in the magazine disk"),               \
                 GROUP("Mechanics / motion"))

#endif // APP_PARAMS_DEF_H
