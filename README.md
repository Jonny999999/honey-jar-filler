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
