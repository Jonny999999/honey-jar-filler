# Thesis Firmware Roadmap

## Purpose

This document captures the current firmware architecture and a practical roadmap for the next three major features:

1. Presets / parameter profiles
2. Web-based configuration and monitoring
3. Detailed process logging for later scientific comparison

The goal is to make future implementation work incremental and reproducible, without having to re-explain the project context.

## Current Firmware Snapshot

The firmware is already structured well enough to extend without a rewrite.

- `firmware_honey-jar-filler/components/app/app.c`
  Stores one versioned `app_params_t` blob in NVS and exposes `app_params_get()` / `app_params_set()`.
- `firmware_honey-jar-filler/components/app/app_params_def.h`
  Central source of truth for parameter defaults, limits, labels, units, descriptions, and grouping.
- `firmware_honey-jar-filler/components/app/app_params_meta.c`
  Builds a metadata table from the parameter definition macros.
- `firmware_honey-jar-filler/components/filler/filler_fsm.c`
  Owns the process state machine and pulls a fresh parameter snapshot every loop.
- `firmware_honey-jar-filler/components/ui/ui_task.c`
  Provides the runtime OLED status screen and direct encoder-based target editing.
- `firmware_honey-jar-filler/components/ui/ui_menu.c`
  Provides a metadata-driven settings menu with explicit save behavior.
- `firmware_honey-jar-filler/main/scale_hx711.c`
  Already has both a polling queue and a thread-safe "latest sample" snapshot.
- `firmware_honey-jar-filler/main/main.c`
  Initializes all peripherals and starts the FSM/UI tasks. There is no networking stack yet.

## Important Observations Before Adding Features

### 1. The current parameter model is centralized

This is a major advantage. Presets and a web UI do not need a second config model if they reuse `app_params_t` plus the existing metadata table.

### 2. Not all parameters should behave like honey presets

Some parameters are "recipe" values, others are "machine calibration" values.

Examples of likely recipe values:

- `target_grams`
- `target_tol_low_g`
- `target_tol_high_g`
- `slow_remaining_g`
- `slow_gate_pct`
- `max_gate_pct`
- `close_remaining_g`
- `drip_delay_ms`
- maybe `empty_glass_min_g` / `empty_glass_max_g` if jar type changes

Examples of machine-level values that should usually not change per honey preset:

- `gate_open_deg`
- `gate_close_deg`
- scale calibration data in `scale_hx711.c`
- maybe `slots_total`

Important detail:

- `gate_open_deg` and `gate_close_deg` are only applied during `gate_init()` in `main/main.c`.
- That means changing them through a preset or web UI will not fully apply live unless gate re-init is added.

Recommendation:

- Keep the first preset implementation simple, but treat calibration values carefully.
- Do not let daily recipe switching accidentally change hardware calibration.

### 3. Current NVS write behavior is fine for OLED use, but not ideal for a web UI

Today, some edits are explicit saves from the menu, but the encoder target adjustment writes to NVS on each change. A web UI with sliders or repeated edits would write far too often.

Recommendation:

- Introduce a RAM working copy plus explicit commit/save semantics before building the web editor.

### 4. The scale update rate is modest

Current default settings are:

- `CONFIG_HX711_POLL_INTERVAL_MS = 280`
- `CONFIG_HX711_AVG_SAMPLE_COUNT = 3`

That is acceptable for slow honey filling and threshold-based control, but may become limiting for more advanced algorithms based on trend or prediction.

Recommendation:

- Keep the current timing initially.
- Re-evaluate sampling rate only after logging exists and actual plots show whether the control signal is too coarse.

## Overall Evaluation

All three ideas are good and realistic.

### Presets

Very realistic and low risk. This is the best next step because it builds directly on the current parameter architecture and immediately helps experimentation.

### Web app

Realistic if kept lightweight. The ESP32 can host a small operator UI in AP mode, but the backend API and config semantics should be stabilized first. A minimal static app plus JSON API is a good fit.

### Logging

Essential for the thesis. This is not optional if the goal is scientific comparison. It is realistic, but it should be implemented as a dedicated telemetry/logging layer rather than scattered `ESP_LOGI()` calls.

## Recommended Order

1. Implement presets and improve parameter persistence semantics.
2. Implement a dedicated telemetry/logging backbone.
3. Add Wi-Fi AP + HTTP API.
4. Build the web UI on top of the API and telemetry stream.
5. Only then start comparing alternative control algorithms.

Why this order:

- Presets solve the immediate operational need.
- Logging should exist before algorithm experiments.
- The web app depends on stable preset/config and telemetry backends.

## Cross-Cutting Design Rule

Add new features through dedicated modules/tasks, not by spreading logic across UI and FSM code.

Recommended new modules:

- `components/presets/`
- `components/telemetry/`
- `components/web/`

The filler FSM should remain responsible for process control, not for storage, HTTP handling, or log transport.

## Task 1: Presets / Parameter Profiles

## Goal

Allow storing multiple named parameter sets and switching between them safely, for example:

- `Summer thin honey`
- `Cold thick honey`
- `500 g jar`
- `Archive 2026-04-22 trial B`

This is useful both operationally and scientifically.

## Why it is a good idea

- It removes the need to remember "good" values manually.
- It supports repeatable experiments.
- It creates a natural unit for later logging: every run can reference a preset name/id/revision.

## Refined decision: split preset settings from machine settings

Yes, the cleaner design is to exclude machine calibration and machine-specific operating limits from per-honey presets.

Recommended split:

- `Preset / recipe settings`
  Values that depend on honey, jar target, or fill strategy
- `Global / machine settings`
  Values that describe the specific hardware or installation

Recommended `global / machine settings` examples:

- `empty_glass_min_g`
- `empty_glass_max_g`
- `advance_timeout_ms`
- `find_ignore_ms`
- `slot_settle_ms`
- `slots_total`
- `gate_open_deg`
- `gate_close_deg`
- `scale_cal_ref_g`

Recommended `preset / recipe settings` examples:

- `target_grams`
- `target_tol_low_g`
- `target_tol_high_g`
- `fill_timeout_ms`
- `slow_remaining_g`
- `slow_gate_pct`
- `max_gate_pct`
- `close_remaining_g`
- `drip_delay_ms`

Why this split is better:

- changing honey should not accidentally change servo calibration
- the same machine can try many honey recipes safely
- logging becomes more meaningful because each run can reference both a `machine config revision` and a `recipe preset revision`
- the web UI becomes easier to understand because the user sees two clear categories

Recommended runtime model:

- keep one unified runtime `app_params_t`
- mark each field as either `PRESET` or `MACHINE`
- at boot, construct runtime params by merging:
  1. firmware base defaults
  2. selected built-in preset defaults
  3. persisted machine settings from NVS
  4. persisted user preset settings from NVS

This keeps the current control code simple because the FSM still reads one parameter struct.

## Recommended implementation

### Phase 1 design

Keep `app_params_t` as the active runtime parameter structure, but add:

- one machine-settings store
- one preset store
- a merge step into the active runtime struct

Suggested data model:

```c
typedef struct {
    char     name[24];
    uint32_t revision;
    int64_t  created_us;
    int64_t  updated_us;
    app_params_t params;
} app_preset_t;

typedef struct {
    uint16_t version;
    uint8_t  active_index;
    uint8_t  preset_count;
    app_preset_t presets[8];
} app_preset_store_t;
```

Suggested NVS layout:

- Namespace: `app`
- Key for machine settings: `machine_v1`
- Key for preset store: `presets_v1`
- Optional key for active working state later: `working_v1`

Suggested machine-settings model:

```c
typedef struct {
    uint16_t version;
    uint32_t revision;
    app_params_t params;
} app_machine_store_t;
```

Important detail:

- only the `MACHINE`-scoped fields inside `params` are meaningful for `machine_v1`
- only the `PRESET`-scoped fields inside `params` are meaningful for each preset

That means the same `app_params_t` and the same metadata can still be reused without duplicate struct definitions.

### Phase 1 API

Create a preset service instead of touching NVS directly from UI or web code.

Suggested API:

```c
void      app_presets_init(void);
esp_err_t app_presets_list(app_preset_t *out, size_t max_items, size_t *out_count);
esp_err_t app_presets_get_active(app_preset_t *out, uint8_t *out_index);
esp_err_t app_presets_select(uint8_t index, bool persist_now);
esp_err_t app_presets_save_active_to_slot(uint8_t index);
esp_err_t app_presets_create_from_active(const char *name, uint8_t *out_index);
esp_err_t app_presets_rename(uint8_t index, const char *name);
esp_err_t app_presets_delete(uint8_t index);
```

Suggested machine-settings API:

```c
void      app_machine_settings_init(void);
esp_err_t app_machine_settings_get(app_params_t *out_machine_only);
esp_err_t app_machine_settings_set(const app_params_t *in_machine_only);
```

### Parameter persistence change

Before presets or web editing, refactor `app_params` into:

- `app_params_get()` to read the current RAM copy
- `app_params_set_ram()` or equivalent public API for immediate runtime updates
- `app_params_commit()` to persist the active working copy to NVS

Reason:

- Web edits should not commit to flash on every input event.
- Preset editing should feel transactional: edit, test, save intentionally.

Recommended future semantics:

- runtime changes can happen immediately in RAM
- explicit save commits only the relevant scope:
  - save preset -> persist only `PRESET` fields into the active preset
  - save machine settings -> persist only `MACHINE` fields into `machine_v1`

### Safety rules

- Preset switching allowed only in `FILLER_IDLE` or `FILLER_FAULT`
- Never switch preset mid-fill
- Show the active preset name on OLED and later on the web UI
- Add a dirty flag when current RAM settings differ from the saved preset

### Migration path from the current firmware

On first boot with preset support:

1. Read the old single `app_params_t` blob if present.
2. Create a default preset from it, for example `Migrated default`.
3. Split the migrated values by scope:
   - `MACHINE` fields go to `machine_v1`
   - `PRESET` fields go to the new default preset
4. Mark that preset active.
5. Save the new store formats.

## OLED / menu behavior before the web app

The OLED should stay useful, but it should not become a full preset editor.

### Main screen

Recommended change:

- always show the active preset name on the normal runtime screen

Because the display is small, use one of these approaches:

- line 3 shows `Preset:<name>` and alternates every 1-2 s with `Btn:START Tgt:150g`
- or line 2 shows compact state info and line 3 always shows preset name

If the name is too long:

- store both a long name and a short label, or
- truncate to fit, for example 12-16 characters

This is worth doing early because it removes ambiguity during testing.

### Menu behavior

Recommended top-level menu flow before the web UI exists:

1. `Preset`
2. `Save to preset`
3. `Machine settings`
4. `Recipe settings`
5. existing actions like tare/calibrate

Minimal first OLED preset behavior:

- selecting `Preset` opens a list of existing presets
- rotating scrolls presets
- clicking applies the selected preset immediately
- application is only allowed in `IDLE` or `FAULT`
- after applying, all recipe-scoped runtime values update together
- machine-scoped values stay unchanged

Recommended save behavior:

- `Save to preset` writes the current recipe-scoped runtime values back to the active preset
- machine settings are not touched

This means the expected operator flow is:

1. choose preset first
2. all recipe values adjust together
3. optionally fine-tune a few recipe settings
4. save back to the active preset if the new tuning is good

That is the simplest usable pre-web workflow.

### Recommended UI scope for the first implementation

OLED support should stay minimal:

- Show active preset name
- Allow preset selection
- Allow "save current recipe to active preset"
- Keep advanced actions like rename/create/delete for the web UI

Do not try to build full preset management on the OLED first. That becomes tedious quickly. Advanced management belongs in the web UI.

### Calibration policy

Recommended policy for phase 1:

- Allow storing all values in the preset payload for simplicity
- But treat machine-calibration fields as advanced settings
- Document clearly that gate calibration may require restart to apply

Optional future refinement:

- Split settings into `machine_config` and `fill_recipe`
- Then presets contain only `fill_recipe`

With your current requirements, this is no longer just optional refinement. It is the recommended direction.

## Built-in preset defaults in the repo

You want named default presets to stay documented in the repo, while NVS overrides them at runtime if the user has saved values. That is a good idea.

Recommended approach:

### Keep one central parameter definition file

Continue using `app_params_def.h` as the single source of truth for:

- field list
- default baseline values
- type
- min/max/step
- labels and descriptions
- new field scope: `PRESET` or `MACHINE`

Do not duplicate full parameter structs per preset in code.

### Add built-in preset definitions as override tables

Recommended pattern:

1. Start from the normal firmware defaults
2. Apply only the preset-specific overrides

Conceptually:

```c
typedef struct {
    const char *name;
    void (*apply_overrides)(app_params_t *p);
} builtin_preset_def_t;
```

Example:

```c
static void preset_thick_honey(app_params_t *p)
{
    p->slow_remaining_g = 80;
    p->close_remaining_g = 180;
    p->drip_delay_ms = 14000;
}
```

Load flow:

1. call `app_params_apply_defaults()`
2. apply built-in preset overrides
3. apply persisted NVS values for that preset if present

Why this avoids chaos:

- adding a new parameter only requires updating the central macro file once
- existing built-in presets automatically inherit the new default unless they specifically override it
- no huge duplicate preset structs need maintenance

### Even cleaner future variant

If you want less handwritten override code later, add a tiny typed override table based on field metadata/offsets. Then a preset can be expressed as "only these fields differ". That keeps built-in presets compact even when many exist.

For the first implementation, simple override functions are sufficient and much easier to debug.

## Implementation steps

1. Add field-scope metadata: `PRESET` vs `MACHINE`.
2. Refactor `app_params` to support RAM-only update, explicit commit, and scope-based copy/merge helpers.
3. Add machine settings store (`machine_v1`).
4. Add preset store (`presets_v1`).
5. Add migration from the current single `params_v1` blob.
6. Add active preset tracking and preset selection API.
7. Expose active preset name/index to UI.
8. Add basic OLED preset selection and save flow.
9. Later expose the same API through HTTP.

## Acceptance criteria

- Multiple named presets can be created, selected, renamed, and deleted.
- Active preset survives reboot.
- Switching presets updates the runtime configuration without corrupting NVS.
- Preset switching is blocked while filling.
- Current preset information is visible locally.

## Risks / caveats

- NVS wear if edits still commit too often
- Confusion if calibration and recipe values are mixed without good naming
- Gate calibration values still being restart-sensitive

## Task 2: Web App for Configuration and Monitoring

## Goal

Provide an easier operator interface than the OLED/encoder for:

- monitoring state live
- viewing current weight and process state
- selecting or editing presets
- changing settings
- later seeing live plots and logs

## Why it is a good idea

- The OLED is good for local fallback, but not for managing many parameters and presets.
- A web UI naturally complements presets and logging.
- It can become the main thesis demo interface.

## Realism

This is realistic on ESP32 if the design stays small and deliberate.

Recommended scope:

- ESP32 in SoftAP mode
- lightweight static frontend
- JSON API for commands and configuration
- SSE for live monitoring stream

Avoid for the first version:

- heavy frontend frameworks unless already justified
- complex authentication systems
- trying to do full database-style storage on the ESP32

## Recommended backend architecture

### Network mode

Start with:

- ESP32 SoftAP mode
- fixed local IP, for example `192.168.4.1`
- WPA2 password configured in code or NVS

Later optional extension:

- AP+STA mode if you want laptop access through existing lab Wi-Fi

### HTTP server

Use ESP-IDF `esp_http_server`.

Recommended endpoints:

- `GET /api/status`
  Returns current machine snapshot
- `POST /api/control/start`
- `POST /api/control/abort`
- `GET /api/params/meta`
  Returns the existing parameter metadata table
- `GET /api/params/current`
- `PUT /api/params/current`
  Updates current working copy only
- `POST /api/params/commit`
  Explicit save
- `GET /api/presets`
- `POST /api/presets`
- `PUT /api/presets/{id}`
- `DELETE /api/presets/{id}`
- `POST /api/presets/{id}/activate`
- `GET /api/stream`
  Server-Sent Events for live data and events

### Why SSE is a good first choice

Use Server-Sent Events before WebSocket.

Reasons:

- simpler implementation
- good fit for one-way live telemetry
- browser support is straightforward
- commands can still use normal REST endpoints

WebSocket is only necessary if SSE becomes limiting later.

### Threading rule

HTTP handlers must not block the FSM or scale task.

Recommended pattern:

- read snapshots from dedicated service functions
- write control requests into queues/flags
- keep web task priority lower than scale/FSM tasks

## Recommended frontend architecture

The frontend should be intentionally small.

Recommended structure:

- single page
- plain HTML/CSS/JS or a very small build setup
- build once, serve static assets from flash

Suggested UI sections:

- machine overview
- current state, fault, slot, active preset
- large live weight readout
- preset selector and preset management
- parameter editor generated from metadata
- start/abort controls
- live plot area
- log export area

### Strong recommendation: metadata-driven parameter forms

Do not hardcode every setting twice.

Reuse:

- labels
- units
- min/max/step
- group names
- descriptions

from the existing parameter metadata. That keeps OLED, firmware, and web UI aligned.

## Suggested machine status payload

Expose a compact snapshot structure like:

```json
{
  "ts_us": 0,
  "state": "FILL",
  "fault": "NONE",
  "slot_index": 2,
  "slots_total": 6,
  "active_preset": "Cold thick honey",
  "weight_g": 132.4,
  "raw_weight": 123456,
  "jar_tare_g": 84.1,
  "relative_fill_g": 48.3,
  "gate_pct": 20,
  "motor_on": false,
  "params_dirty": false
}
```

This same snapshot can later feed the web monitor and telemetry logger.

## Implementation steps

1. Add Wi-Fi AP startup module.
2. Add HTTP server with a minimal `/api/status`.
3. Expose parameter metadata and current parameters.
4. Expose preset APIs.
5. Add SSE stream for status updates and events.
6. Add a minimal static frontend.
7. Add live plots after telemetry backend exists.

## Acceptance criteria

- A laptop/phone can connect directly to the ESP32 AP.
- The browser shows live machine status without serial monitor.
- Parameters can be edited and explicitly saved.
- Presets can be managed from the browser.
- Local OLED control still works as a fallback.

## Risks / caveats

- Flash/RAM limits if the frontend becomes too heavy
- Network handlers accidentally coupling too tightly to control logic
- Frequent browser edits causing too many flash writes if commit semantics are not fixed first

## Task 3: Detailed Logging for Scientific Comparison

## Goal

Create a reliable and clean logging system for later thesis analysis, including:

- time series of process values
- state transitions
- faults
- preset and parameter context
- per-run summaries

## Why it is a good idea

This is the most important thesis-oriented feature because it turns "it feels better" into measurable evidence.

Without structured logs, comparing parameter sets or control strategies will be weak.

## Core recommendation

Do not rely on scattered `ESP_LOGI()` output as the scientific log.

Instead, create a dedicated telemetry/logging module with a stable record format and separate output transports.

## Recommended architecture

### Logging layers

1. Producers
   The scale task, filler FSM, UI, preset service, and later web/API layer produce telemetry records.
2. Telemetry queue
   A single queue receives structured records.
3. Telemetry task
   One task timestamps, enriches, buffers, and forwards records.
4. Transports
   UART, Wi-Fi SSE, and optional on-device ring buffer use the same record source.

### Record types

Suggested record classes:

- `RUN_START`
- `RUN_END`
- `STATE_CHANGE`
- `FAULT`
- `SAMPLE`
- `GATE_COMMAND`
- `PARAM_CHANGE`
- `PRESET_CHANGE`
- `USER_ACTION`
- `NOTE`

### Suggested telemetry record

```c
typedef enum {
    TEL_RUN_START,
    TEL_RUN_END,
    TEL_STATE_CHANGE,
    TEL_FAULT,
    TEL_SAMPLE,
    TEL_GATE_COMMAND,
    TEL_PARAM_CHANGE,
    TEL_PRESET_CHANGE,
    TEL_USER_ACTION,
} telemetry_kind_t;

typedef struct {
    int64_t ts_us;
    uint32_t run_id;
    telemetry_kind_t kind;
    filler_state_t state;
    filler_fault_t fault;
    uint8_t slot_idx;
    float weight_g;
    float jar_tare_g;
    float relative_fill_g;
    float gate_pct;
    uint32_t target_g;
    char preset_name[24];
    uint32_t preset_revision;
    uint16_t algorithm_id;
    uint16_t algorithm_revision;
} telemetry_record_t;
```

The exact struct can change, but the design should include:

- timestamp
- run id
- preset identity
- algorithm identity
- main process values

## What should be logged

### Per-sample time series

At each published sample, capture at least:

- timestamp
- raw scale value
- calibrated weight
- relative fill weight
- FSM state
- slot index
- gate command percent
- target grams
- active preset id/name
- run id

### Events

Log immediately:

- start requested
- abort requested
- state changes
- near-close transition
- close-early transition
- refill decision
- overweight fault
- motor timeout
- scale stale fault
- preset switched
- parameters committed

### Per-run summary

At the end of each jar or run, produce a summary containing:

- run id
- preset id/name/revision
- algorithm id/revision
- start/end timestamps
- initial jar tare
- final net fill weight
- absolute error from target
- refill count
- time spent filling
- time spent dripping
- final result: ok / skipped / fault

This summary is extremely useful for thesis tables even without plotting the full time series.

## Transport recommendation: separate data model from transport

### UART

Best for first implementation.

Advantages:

- simple
- reliable
- easy to capture on a laptop
- good for debugging before Wi-Fi exists

Recommended output format:

- NDJSON or CSV lines

NDJSON is especially attractive because the same record format can later be reused over HTTP/SSE.

### Wi-Fi

Best for integration with the web UI once the API exists.

Recommended use:

- live telemetry stream to browser
- optional host-side logger tool over HTTP/SSE

### On-device persistence

Do not try to persist high-rate full time series in NVS.

Reasons:

- too much flash wear
- too little capacity
- unnecessary complexity

Recommended instead:

- keep a RAM ring buffer for recent records
- optionally persist only per-run summaries
- store full data on a connected host via UART or Wi-Fi

## Recommended first implementation strategy

### Phase 1

Add telemetry module and UART export only.

This is enough to:

- collect real experiment data
- inspect plots on a laptop
- validate the record format

### Phase 2

Expose the same telemetry stream via Wi-Fi SSE and show it in the web app.

### Phase 3

Add host-side tooling in the repo, for example:

- `tools/log_capture.py`
- `tools/plot_run.py`
- `tools/notebooks/`

The thesis analysis will almost certainly be easier in Python/pandas/matplotlib than inside the ESP32 itself.

## Clean integration points in the current code

Recommended producers:

- `main/scale_hx711.c`
  Publish `SAMPLE`
- `components/filler/filler_fsm.c`
  Publish state transitions, faults, run start/end, gate decisions
- `components/app/` or future preset service
  Publish param commit and preset change events
- `components/ui/ui_task.c`
  Publish user actions like start/abort or target edits if useful

## Acceptance criteria

- Each experimental run has a unique run id.
- Logs can be captured on a laptop without modifying the control loop.
- Logs include preset identity and enough state to reconstruct the fill process.
- Per-run summaries can be exported for thesis tables.
- Live monitoring later reuses the same telemetry source.

## Risks / caveats

- Logging too much directly from high-priority tasks
- Mixing debug logs with scientific logs
- Depending only on browser-side capture for important data

## Recommendation on Algorithm Experiments

Do not start with multiple new control algorithms before presets and logging exist.

First make experiments repeatable and measurable.

After that, the firmware should expose an explicit algorithm identity, for example:

- `ALGO_THRESHOLD_V1`
- `ALGO_THRESHOLD_V2`
- `ALGO_PREDICTIVE_CLOSE_V1`

Recommended future refactor:

- keep the FSM states
- isolate the fill decision logic behind a controller function or small strategy interface

This will make comparisons much cleaner in the thesis because only the control strategy changes while the surrounding machine logic stays constant.

## Final Recommendation

The three ideas are worthwhile, but they should not be started in parallel.

Best next action:

1. Presets plus better config persistence semantics
2. Structured telemetry/logging
3. Wi-Fi API
4. Web UI

If only one feature is implemented next, it should be presets.

If the goal is specifically thesis quality, the second feature after presets should be logging, not the web UI.
