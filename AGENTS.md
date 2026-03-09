# Repository Guidelines

## Project Structure & Module Organization
This repository is an ESPHome external component for SMA inverters over classic Bluetooth. Core component code lives in `esphome/components/smabluetooth_solar/`: C++ runtime logic is in `smabluetooth_solar.cpp`, `smabluetooth_solar.h`, `SMA_Inverter.cpp`, and `SMA_Inverter.h`; the ESPHome config schema is in `sensor.py`. Sample configuration and helper scripts live in `esphome/sample/`, including `smabluesolar.yaml` and `generate-partitions.py`. Keep new component files under the existing `smabluetooth_solar` namespace.

## Build, Test, and Development Commands
Use ESPHome from the repository root.

- `esphome config esphome/sample/smabluesolar.yaml` validates YAML and Python schema wiring.
- `esphome compile esphome/sample/smabluesolar.yaml` builds the sample node and catches C++ integration errors.
- `esphome run esphome/sample/smabluesolar.yaml` flashes a test device when hardware is available.
- `python3 esphome/sample/generate-partitions.py` prints a partition table for ESP32 sizing checks.

For local development, point `external_components` in the sample YAML at a local path instead of the GitHub URL.

## Coding Style & Naming Conventions
Follow the existing style in each language. Python uses 4-space indentation, `snake_case` keys, and `UPPER_SNAKE_CASE` constants for config names. C++ uses the ESPHome pattern: `PascalCase` classes, `snake_case_` member fields with trailing underscores, and concise logging via `ESP_LOG*`. Keep YAML option names lowercase with underscores, for example `sma_inverter_bluetooth_mac`.

## Testing Guidelines
There is no dedicated `tests/` directory yet, so validation is compile-first. Run `esphome config` and `esphome compile` for every change. When modifying inverter communication or sensor mapping, test against real ESP32 hardware and note the inverter model used, such as `SB3000TL-20` or `SB1600TL-10`. Treat the sample YAML as the regression fixture and keep it updated when options change.

## Commit & Pull Request Guidelines
Recent history uses short, imperative subjects such as `fix compile`, `buffer`, and `review type label`. Keep commit titles brief, present tense, and focused on one change. Pull requests should describe the user-visible impact, list the ESPHome commands run, and mention any hardware verification performed. Include updated YAML snippets or logs when changing configuration shape or Bluetooth behavior.
