# Yummy FOC ESP32

An ESP32-S3 FOC project derived from a minimal ESP-IDF template and reorganized into clear runtime modules.

## Project Layout

```text
.
├─ components
│  ├─ foc
│  │  ├─ include
│  │  │  ├─ foc_types.h
│  │  │  ├─ foc_control.h
│  │  │  ├─ foc_output.h
│  │  │  ├─ foc_loop.h
│  │  │  └─ foc_driver_esp32.h
│  │  └─ src
│  │     ├─ foc_control.c
│  │     ├─ foc_output.c
│  │     ├─ foc_loop.c
│  │     └─ foc_driver_esp32.c
│  └─ sensors
│     ├─ include
│     │  └─ mt6701.h
│     └─ src
│        └─ mt6701.c
├─ docs
├─ main
├─ reference
│  └─ STM32_Example
└─ sdkconfig
```

## Module Responsibilities

- `components/foc/src/foc_control.c`
  Maintains FOC state, LUT generation, angle conversion, and generic control-side helpers.
- `components/foc/src/foc_output.c`
  Converts `Ud/Uq` into three-phase PWM through the q15 inverse Park plus centered SVPWM path.
- `components/foc/src/foc_loop.c`
  Contains the fast loop logic, shaft angle acquisition entry, and electrical angle derivation.
- `components/foc/src/foc_driver_esp32.c`
  Owns ESP32-specific startup, PWM, timer ISR binding, core placement, NVS calibration storage, and startup calibration flow.
- `components/sensors/src/mt6701.c`
  Owns the MT6701 SPI interface and mechanical angle readout.
- `reference/STM32_Example`
  Keeps the original STM32 reference project for porting comparison only. It is not part of the ESP-IDF build.

## Hardware Mapping

- DRV8313 `IN_A` -> GPIO17
- DRV8313 `IN_B` -> GPIO18
- DRV8313 `IN_C` -> GPIO8
- MT6701 `MOSI` -> GPIO42
- MT6701 `MISO` -> GPIO41
- MT6701 `SCK` -> GPIO40
- MT6701 `NSS` -> GPIO39

## Build Notes

- Project name: `yummy_foc_esp32`
- Main application entry: `main/main.c`
- ESP-IDF auto-discovers components under `components/`

## Suggested Next Cleanup

- Normalize legacy comment encoding in older source blocks.
- Split calibration strategy out of `foc_driver_esp32.c` if startup logic grows further.
- Add a dedicated current-sense component once ADC sampling is introduced.
