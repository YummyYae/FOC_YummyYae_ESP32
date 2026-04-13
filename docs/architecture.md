# Architecture Notes

## Why the project was restructured

The original ESP-IDF template had already grown into a real FOC firmware project, but its file names still reflected template history and mixed responsibilities.

The new structure separates:

- algorithm/state
- output modulation
- fast loop runtime
- ESP32 hardware startup
- external sensor drivers
- reference material

## Naming Rules

- `foc_*` files under `components/foc/src` should describe one responsibility per file.
- sensor drivers live under `components/sensors`.
- reference code never lives beside runtime source files.
- `main/` stays thin and only wires together components.
