# honey-jar-filler

Semi-automatic **weight-based honey jar filler** with a **jar carousel/magazine** and a **servo-actuated honey gate**.  
This repo currently contains the **KiCad PCB** and **FreeCAD** models and firmware (ESP-IDF).

  <img src="cad/exports/screenshots/assembly.png" alt="Assembly Preview" width="70%">

*3d model of the planned system*

## Repo contents
- `pcb_honey-jar-filler/` - KiCad project (schematic & PCB, exports)
- `cad/` - FreeCAD parts, 3D models, full assembly
- `firmware/` - firmware for esp32 (ESP-IDF)

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