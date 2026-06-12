# honey-jar-filler

Semi-automatic **weight-based honey jar filler** with a **jar carousel/magazine** and a **servo-actuated honey gate**.  
This repo contains the **KiCad PCB-Project**, **FreeCAD** models, and firmware (ESP-IDF).

  <img src="cad/exports/screenshots/assembly.png" alt="Assembly Preview" width="70%">

*3d model of the planned system*

<p align="center">
  <img src="doc/images/pcb_populated.jpg" alt="Custom board fully populated" width="48%">
  <img src="doc/images/pcb_IO-test.jpg" alt="I/O test with peripherals" width="48%">
</p>

*Left: custom pcb fully populated. Right: I/O test with all peripherals connected.*

## Repo contents
- `pcb_honey-jar-filler/` - KiCad project (schematic & PCB, exports)
- `cad/` - FreeCAD parts, 3D models, full assembly
- `firmware_honey-jar-filler/` - firmware for esp32 (ESP-IDF)

## Features (PCB / system)
- Load cell input (HX711) for weight-based dosing
- UI: rotary encoder, 2× buttons, LEDs, OLED display (I²C)
- Actuation: servo interface for honey gate; motor output for carousel
- Optional outputs: heater control, tank pressurization valve
- Power: 12–40 V input, 5 V buck (logic), adjustable 4–9 V rail (servo)

## Ideas
- Rotary carousel automation (indexing, run/stop with braking)
- Outlet warming (silicone band / water jacket)
- Jar presence detection
- Web UI / profiles / batch counter (ESP-IDF)

## Schematic + layout preview
<p align="center">
  <a href="pcb_honey-jar-filler/export/pcb_honey-jar-filler.pdf">
    <img src="pcb_honey-jar-filler/export/pcb_honey-jar-filler.svg" width="50%" alt="Schematic"/>
  </a>
  <img src="pcb_honey-jar-filler/export/layout.png" width="40%" alt="PCB Layout"/>
</p>

## Firmware build (ESP-IDF)
- Install ESP-IDF **v5.5.2** and export its environment (`. ./export.sh`).
- Enter the firmware project:
```bash
cd firmware_honey-jar-filler
```
- Set the target and build:
```bash
idf.py set-target esp32
idf.py build
```
- Flash and monitor (adjust serial port as needed):
```bash
idf.py -p /dev/ttyUSB0 flash monitor
```

## Firmware presets
- Runtime parameters are still defined centrally in `firmware_honey-jar-filler/components/app/app_params_def.h`.
- Each parameter there has one baseline default plus metadata such as min/max/step, UI text, and a scope:
  `APP_PARAM_SCOPE_PRESET` for recipe-specific values and `APP_PARAM_SCOPE_MACHINE` for shared machine settings.
- Built-in preset names and their default overrides live in `firmware_honey-jar-filler/components/app/app.c`.
  These preset defaults only override recipe-scoped fields; they do not duplicate the full parameter list.
- NVS stores shared machine settings separately from editable preset values:
  `machine_v1` for shared settings and `presets_v1` for preset-specific values.
- At boot, the active runtime config is rebuilt from baseline defaults + selected preset defaults + persisted machine settings + persisted preset values.

## Telemetry tools
- Structured telemetry is emitted on the normal UART console with a `TEL ` prefix and JSON payload.
- Normal ESP logs and machine telemetry therefore share one serial stream, but the telemetry lines can be split cleanly on the host.
- The intended workflow is:
  1. capture a session from UART
  2. keep the full console log for debugging
  3. store telemetry separately as NDJSON
  4. split runs and generate thesis figures from those files

Current script layout:
- `tools/telemetry/capture.py`
  Fully usable serial capture tool. Replaces the basic `idf.py monitor` workflow for experiment sessions and shows a live status line with telemetry counters.
- `tools/telemetry/split_runs.py`
  Offline splitter that converts one captured session into per-fill NDJSON files.
  It detects jar filling windows from `state=FILL`, adds configurable pre/post context,
  and writes `fills/index.json` plus `fill_0001.ndjson`, `fill_0002.ndjson`, ...
- `tools/telemetry/plot_runs.py`
  Offline chart exporter for thesis figures. It writes both a plain LaTeX-ready
  chart and a debug variant with metadata, based on fill mass, gate percentage,
  and FSM state overlays.

Session capture output:
- `data/telemetry/<session>/session.log`
  Full serial console output, including normal ESP logs and raw `TEL` lines.
- `data/telemetry/<session>/telemetry.ndjson`
  Only telemetry payload lines, one JSON object per line.
- `data/telemetry/<session>/runs/run_0001.ndjson`
  Per-run telemetry files created live from `run_start` / `run_end` events, including configurable pre/post context.
- `data/telemetry/<session>/fills/index.json`
  Offline manifest of detected jar-fill windows with timing, preset, and parameter metadata.
- `data/telemetry/<session>/fills/fill_0001.ndjson`
  One offline per-fill telemetry slice intended for plotting and later comparison.
- `data/telemetry/<session>/session_meta.json`
  Port, baud rate, and capture timing settings.

Example capture usage:
```bash
python3 tools/telemetry/capture.py --port /dev/ttyUSB0 --baud 115200
```

Example offline workflow:
```bash
python3 tools/telemetry/split_runs.py data/telemetry/2026.06.12_11:39-x-test --force
python3 tools/telemetry/plot_runs.py data/telemetry/2026.06.12_11:39-x-test/fills --list
python3 tools/telemetry/plot_runs.py data/telemetry/2026.06.12_11:39-x-test/fills --fills 1,3 --format pdf --format svg
```

Useful options:
- `--show-tel`
  Also print the `TEL` lines in the terminal instead of only saving them to files.
- `--pre-run-ms 2000 --post-run-ms 2000`
  Keep more telemetry context around each detected run.
- `--session-name my-test`
  Add a readable suffix after the timestamp, for example
  `2026.06.07_22:30-x-my-test`.

Goal of this tooling:
- keep experiment data self-describing and reproducible
- avoid manually filtering UART logs after each test
- support later batch plotting to vector PDF/SVG figures for the thesis
- preserve enough event context to compare different presets and control strategies cleanly
- make it obvious during capture that the custom telemetry tool is running, not plain `idf.py monitor`

## Wiring (servo mount DIN cable)
The cable with 15-pin DIN connector from the base assembly to the servo mount carries OLED, encoder, and servo signals + power.

OLED:
- green: GND
- red: 3V3
- orange: SDA
- yellow: SCL

Encoder:
- gray: SW
- blue: B
- light-green: A

Servo:
- brown, gray-brown, pink: 8V
- gray-black, black, white: GND
- purple: PWM
