// Host simulation harness for the fill strategies.
//
// Compiles the REAL firmware strategy sources (components/filler/
// filler_strategy_*.c) against the shims in tools/sim/shim and exposes a tiny C
// API that a Python plant model drives through ctypes. It reproduces the part
// of filler_fsm.c that talks to a strategy: on_enter() on every transition into
// FILL / DRIP_WAIT / VERIFY_TARGET (with the correct prev_state, so a refill is
// FILL entered from VERIFY_TARGET), step() every FSM tick, and the env
// callbacks (gate, fault, telemetry, summary). Everything outside a fill (slot
// search, jar check, carousel) is the Python side's job: when step() returns
// FIND_SLOT the jar is done, the runner swaps in an empty jar and calls
// sim_begin_fill() again.
//
// Static state inside the strategy files (learned stores) persists for the
// lifetime of the loaded library. A simulated reboot = load a fresh copy of
// the .so; the file-backed NVS shim survives that, like real flash.

#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_err.h"
#include "esp_timer.h"
#include "filler_strategy.h"
#include "nvs.h"

extern const filler_strategy_ops_t g_filler_strategy_heuristic_ops;
extern const filler_strategy_ops_t g_filler_strategy_adaptive_heuristic_ops;
extern const filler_strategy_ops_t g_filler_strategy_flow_control_ops;
extern const filler_strategy_ops_t g_filler_strategy_flow_cascade_ops;
#ifdef SIM_HAVE_ADAPTIVE_GATE
extern const filler_strategy_ops_t g_filler_strategy_adaptive_gate_ops;
#endif

static filler_strategy_runtime_t s_rt;
static app_params_t s_params;
static const filler_strategy_ops_t *s_ops;
static filler_state_t s_state = FILLER_IDLE;
static int64_t s_now_us;
static int64_t s_state_enter_us;
static scale_latest_t s_latest;
static float s_gate_pct;
static char s_gate_label[32];
static int s_fault;
static uint32_t s_run_id = 1;
static uint8_t s_slot;
static uint8_t s_preset_index = 1;
static char s_preset_name[24] = "Sim";
static filler_strategy_sample_telemetry_t s_sample;
static char s_summary_json[8192];
static int s_summary_ready;
static FILE *s_log;
static char s_nvs_path[512];

// ---------------------------------------------------------------- shims ----

int64_t esp_timer_get_time(void) { return s_now_us; }

const char *esp_err_to_name(esp_err_t err) { return err == ESP_OK ? "ESP_OK" : "ESP_FAIL"; }

void sim_log(char level, const char *tag, const char *fmt, ...)
{
    if (!s_log) return;
    fprintf(s_log, "%c (%.3f) %s: ", level, (double)s_now_us / 1e6, tag);
    va_list ap;
    va_start(ap, fmt);
    vfprintf(s_log, fmt, ap);
    va_end(ap);
    fputc('\n', s_log);
}

// NVS: one blob per (namespace,key) stored as <nvs_path>.<ns>.<key>
static char s_nvs_ns[32];
static void nvs_file(char *out, size_t n, const char *key)
{
    snprintf(out, n, "%s.%s.%s", s_nvs_path, s_nvs_ns, key);
}
esp_err_t nvs_open(const char *ns, nvs_open_mode_t mode, nvs_handle_t *out)
{
    (void)mode;
    if (!s_nvs_path[0]) return ESP_FAIL;
    snprintf(s_nvs_ns, sizeof(s_nvs_ns), "%s", ns);
    if (out) *out = 1;
    return ESP_OK;
}
esp_err_t nvs_get_blob(nvs_handle_t h, const char *key, void *out, size_t *len)
{
    (void)h;
    char p[600];
    nvs_file(p, sizeof(p), key);
    FILE *f = fopen(p, "rb");
    if (!f) return ESP_ERR_NVS_NOT_FOUND;
    size_t got = fread(out, 1, *len, f);
    fclose(f);
    *len = got;
    return ESP_OK;
}
esp_err_t nvs_set_blob(nvs_handle_t h, const char *key, const void *val, size_t len)
{
    (void)h;
    char p[600];
    nvs_file(p, sizeof(p), key);
    FILE *f = fopen(p, "wb");
    if (!f) return ESP_FAIL;
    fwrite(val, 1, len, f);
    fclose(f);
    return ESP_OK;
}
esp_err_t nvs_commit(nvs_handle_t h) { (void)h; return ESP_OK; }
void nvs_close(nvs_handle_t h) { (void)h; }

// ------------------------------------------------------------------ env ----

static bool env_require_fresh(const char *ctx, const scale_latest_t *s, filler_state_t *state)
{
    (void)ctx; (void)s; (void)state;
    return true;   // the simulated scale never goes stale
}
static void env_set_fault(filler_fault_t flt) { s_fault = (int)flt; }
static bool env_jar_tare_get(float *out) { (void)out; return false; }
static bool env_take_start(void) { return false; }
static int32_t env_take_manual_delta(void) { return 0; }
static void env_gate_close(const char *label)
{
    s_gate_pct = 0.0f;
    snprintf(s_gate_label, sizeof(s_gate_label), "%s", label ? label : "close");
}
static void env_gate_set(float pct, const char *label)
{
    if (pct < 0.0f) pct = 0.0f;
    if (pct > 100.0f) pct = 100.0f;
    s_gate_pct = pct;
    snprintf(s_gate_label, sizeof(s_gate_label), "%s", label ? label : "set");
}
static void env_set_slot(uint8_t idx) { s_slot = idx; }
static uint8_t env_slot_next(uint8_t idx, uint8_t n) { return (uint8_t)((idx + 1u) % (n ? n : 1u)); }
static void env_fill_start(uint32_t run_id, uint8_t slot, const char *strategy, const app_params_t *p, float base)
{
    (void)run_id; (void)slot; (void)strategy; (void)p; (void)base;
}
static void env_set_sample(const filler_strategy_sample_telemetry_t *s) { if (s) s_sample = *s; }
static void env_clear_sample(void) { memset(&s_sample, 0, sizeof(s_sample)); }

#define SUMMARY_FIELDS(X) \
    X(final_mass_g) X(target_g) X(fill_error_g) X(measured_dead_time_s) \
    X(measured_post_close_gain_g) X(measured_near_close_gain_g) X(measured_fast_rate_gps) \
    X(measured_slow_rate_gps) X(rate_at_close_gps) X(fill_duration_s) X(drip_wait_used_ms) \
    X(used_dead_time_s) X(used_post_close_gain_g) X(used_fast_rate_gps) X(used_slow_rate_gps) \
    X(used_near_close_g) X(used_close_early_g) X(next_dead_time_s) X(next_post_close_gain_g) \
    X(next_fast_rate_gps) X(next_slow_rate_gps) X(next_near_close_g) X(next_close_early_g) \
    X(next_drip_wait_ms) X(used_gate_gain_gps_per_pct) X(used_gain_b_gps) X(used_onset_gate_pct) \
    X(used_model_tau_s) X(next_gate_gain_gps_per_pct) X(next_gain_b_gps) X(next_onset_gate_pct) \
    X(next_model_tau_s) X(used_fast_gate_pct) X(used_slow_gate_pct) X(next_fast_gate_pct) \
    X(next_slow_gate_pct) X(target_fast_rate_gps) X(target_slow_rate_gps) X(obs_fast_rate_gps) \
    X(obs_slow_rate_gps) X(gate_corrections) X(regime_change)

static void env_fill_summary(uint32_t run_id, uint8_t slot, const char *preset, const char *strategy,
                             const app_params_t *p, const filler_strategy_fill_summary_t *s)
{
    (void)preset; (void)p;
    int n = snprintf(s_summary_json, sizeof(s_summary_json),
                     "{\"kind\":\"fill_summary\",\"ts_us\":%lld,\"run_id\":%u,\"slot_idx\":%u,"
                     "\"strategy_name\":\"%s\",\"text\":\"%s\",\"refill_count\":%u",
                     (long long)s_now_us, (unsigned)run_id, (unsigned)slot,
                     strategy ? strategy : "?", s->reason, (unsigned)s->refill_count);
#define X(f) n += snprintf(s_summary_json + n, sizeof(s_summary_json) - (size_t)n, ",\"" #f "\":%.4f", (double)s->f);
    SUMMARY_FIELDS(X)
#undef X
    snprintf(s_summary_json + n, sizeof(s_summary_json) - (size_t)n, "}");
    s_summary_ready = 1;
}

static const filler_strategy_env_t s_env = {
    .require_fresh_or_fault = env_require_fresh,
    .set_fault = env_set_fault,
    .jar_tare_get = env_jar_tare_get,
    .take_start_request = env_take_start,
    .take_manual_gate_delta = env_take_manual_delta,
    .gate_close_label = env_gate_close,
    .gate_set_percent_label = env_gate_set,
    .set_slot = env_set_slot,
    .slot_next_idx = env_slot_next,
    .publish_fill_start = env_fill_start,
    .set_sample_telemetry = env_set_sample,
    .clear_sample_telemetry = env_clear_sample,
    .publish_fill_summary = env_fill_summary,
};

// ------------------------------------------------------------- params -----

static void params_defaults(app_params_t *p)
{
    memset(p, 0, sizeof(*p));
    p->version = APP_PARAMS_VERSION;
#define APP_PARAM_FLOAT(field, label, unit, def, mn, mx, st, b, d, g, sc) p->field = (float)(def);
#define APP_PARAM_U32(field, label, unit, def, mn, mx, st, b, d, g, sc) p->field = (uint32_t)(def);
#define APP_PARAM_U8(field, label, unit, def, mn, mx, st, b, d, g, sc) p->field = (uint8_t)(def);
    APP_PARAMS_DEF_LIST(APP_PARAM_FLOAT, APP_PARAM_U32, APP_PARAM_U8)
#undef APP_PARAM_FLOAT
#undef APP_PARAM_U32
#undef APP_PARAM_U8
}

#define SIM_STR_(x) #x
#define SIM_STR(x) SIM_STR_(x)

int sim_param_set(const char *name, double v)
{
#define APP_PARAM_FLOAT(field, ...) if (!strcmp(name, SIM_STR(field))) { s_params.field = (float)v; return 1; }
#define APP_PARAM_U32(field, ...) if (!strcmp(name, SIM_STR(field))) { s_params.field = (uint32_t)(v + 0.5); return 1; }
#define APP_PARAM_U8(field, ...) if (!strcmp(name, SIM_STR(field))) { s_params.field = (uint8_t)(v + 0.5); return 1; }
    APP_PARAMS_DEF_LIST(APP_PARAM_FLOAT, APP_PARAM_U32, APP_PARAM_U8)
#undef APP_PARAM_FLOAT
#undef APP_PARAM_U32
#undef APP_PARAM_U8
    return 0;
}

double sim_param_get(const char *name)
{
#define APP_PARAM_FLOAT(field, ...) if (!strcmp(name, SIM_STR(field))) return (double)s_params.field;
#define APP_PARAM_U32(field, ...) if (!strcmp(name, SIM_STR(field))) return (double)s_params.field;
#define APP_PARAM_U8(field, ...) if (!strcmp(name, SIM_STR(field))) return (double)s_params.field;
    APP_PARAMS_DEF_LIST(APP_PARAM_FLOAT, APP_PARAM_U32, APP_PARAM_U8)
#undef APP_PARAM_FLOAT
#undef APP_PARAM_U32
#undef APP_PARAM_U8
    return -1.0;
}

// ---------------------------------------------------------------- API -----

int sim_init(const char *strategy, int preset_index, const char *log_path, const char *nvs_path)
{
    params_defaults(&s_params);
    s_preset_index = (uint8_t)preset_index;
    snprintf(s_preset_name, sizeof(s_preset_name), "preset%d", preset_index);
    if (s_log && s_log != stderr) fclose(s_log);
    s_log = NULL;
    if (log_path && log_path[0]) s_log = (!strcmp(log_path, "-")) ? stderr : fopen(log_path, "w");
    snprintf(s_nvs_path, sizeof(s_nvs_path), "%s", nvs_path ? nvs_path : "");

    s_ops = NULL;
    if (!strcmp(strategy, "heuristic")) s_ops = &g_filler_strategy_heuristic_ops;
    else if (!strcmp(strategy, "adaptive-heuristic")) s_ops = &g_filler_strategy_adaptive_heuristic_ops;
    else if (!strcmp(strategy, "flow-control")) s_ops = &g_filler_strategy_flow_control_ops;
    else if (!strcmp(strategy, "flow-cascade")) s_ops = &g_filler_strategy_flow_cascade_ops;
#ifdef SIM_HAVE_ADAPTIVE_GATE
    else if (!strcmp(strategy, "adaptive-gate")) s_ops = &g_filler_strategy_adaptive_gate_ops;
#endif
    if (!s_ops) return 0;
    memset(&s_rt, 0, sizeof(s_rt));
    s_state = FILLER_IDLE;
    return 1;
}

static void enter_state(filler_state_t next, filler_state_t prev)
{
    s_state = next;
    s_state_enter_us = s_now_us;
    if (next == FILLER_FILL || next == FILLER_DRIP_WAIT || next == FILLER_VERIFY_TARGET) {
        filler_strategy_tick_t t = {
            .now_us = s_now_us,
            .state_enter_us = s_state_enter_us,
            .run_id = s_run_id,
            .slot_idx = s_slot,
            .new_sample = false,
            .latest = &s_latest,
            .params = &s_params,
            .preset_index = s_preset_index,
            .preset_name = s_preset_name,
            .strategy_name = s_ops->name,
        };
        if (s_ops->on_enter) s_ops->on_enter(&s_rt, next, prev, &s_env, &t);
    } else {
        // FIND_SLOT / FAULT / DONE: the real FSM closes the gate here.
        s_gate_pct = 0.0f;
    }
}

// Start a fill on a freshly verified empty jar (FSM: VERIFY_EMPTY -> FILL).
void sim_begin_fill(long long now_us, float grams)
{
    s_now_us = now_us;
    s_latest.grams = grams;
    s_latest.valid = true;
    s_latest.ts_us = now_us;
    s_fault = 0;
    enter_state(FILLER_FILL, FILLER_VERIFY_EMPTY);
}

// One FSM tick. Returns the state after the tick.
int sim_tick(long long now_us, int new_sample, float grams)
{
    s_now_us = now_us;
    if (new_sample) {
        s_latest.grams = grams;
        s_latest.valid = true;
        s_latest.ts_us = now_us;
    }
    if (s_state != FILLER_FILL && s_state != FILLER_DRIP_WAIT && s_state != FILLER_VERIFY_TARGET) {
        return (int)s_state;
    }
    filler_strategy_tick_t t = {
        .now_us = s_now_us,
        .state_enter_us = s_state_enter_us,
        .run_id = s_run_id,
        .slot_idx = s_slot,
        .new_sample = new_sample != 0,
        .latest = &s_latest,
        .params = &s_params,
        .preset_index = s_preset_index,
        .preset_name = s_preset_name,
        .strategy_name = s_ops->name,
    };
    filler_state_t next = s_ops->step(&s_rt, s_state, &s_env, &t);
    if (next != s_state) enter_state(next, s_state);
    return (int)s_state;
}

float sim_gate(void) { return s_gate_pct; }
int sim_fault(void) { return s_fault; }
const char *sim_gate_label(void) { return s_gate_label; }

int sim_take_summary(char *buf, int n)
{
    if (!s_summary_ready) return 0;
    snprintf(buf, (size_t)n, "%s", s_summary_json);
    s_summary_ready = 0;
    return 1;
}

int sim_sample_json(char *buf, int n)
{
    const filler_strategy_sample_telemetry_t *s = &s_sample;
    return snprintf(buf, (size_t)n,
                    "{\"valid\":%d,\"gate_phase\":\"%s\",\"raw_rate_gps\":%.3f,\"filtered_rate_gps\":%.3f,"
                    "\"target_rate_gps\":%.3f,\"control_rate_gps\":%.3f,\"learned_fast_rate_gps\":%.3f,"
                    "\"learned_slow_rate_gps\":%.3f,\"adapted_near_close_g\":%.3f,\"adapted_close_early_g\":%.3f,"
                    "\"learned_post_close_gain_g\":%.3f,\"learned_dead_time_s\":%.3f,\"gate_gain_gps_per_pct\":%.4f,"
                    "\"flow_onset_gate_pct\":%.2f,\"gain_obs_gate_pct\":%.2f,\"gain_obs_rate_gps\":%.3f,"
                    "\"gain_obs_count\":%.0f,\"refill_count\":%u}",
                    s->valid ? 1 : 0, s->gate_phase, (double)s->raw_rate_gps, (double)s->filtered_rate_gps,
                    (double)s->target_rate_gps, (double)s->control_rate_gps, (double)s->learned_fast_rate_gps,
                    (double)s->learned_slow_rate_gps, (double)s->adapted_near_close_g,
                    (double)s->adapted_close_early_g, (double)s->learned_post_close_gain_g,
                    (double)s->learned_dead_time_s, (double)s->gate_gain_gps_per_pct,
                    (double)s->flow_onset_gate_pct, (double)s->gain_obs_gate_pct, (double)s->gain_obs_rate_gps,
                    (double)s->gain_obs_count, (unsigned)s->refill_count);
}

int sim_runtime_size(void) { return (int)sizeof(filler_strategy_runtime_t); }
