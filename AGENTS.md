# Repository Guidelines

An ESPHome external component for SMA inverters over classic Bluetooth (SMANET2/SPP). See `CLAUDE.md` for architecture detail: data flow (ESPHome main loop vs FreeRTOS BT task on core 0), SMANET2 packet format, and key design points (`ESP32_SMA_Inverter` is a singleton; all blocking I/O stays in the BT task).

## Project Structure
Component code lives in `esphome/components/smabluetooth_solar/`: C++ runtime in `smabluetooth_solar.cpp/.h` (PollingComponent, state machine) and `SMA_Inverter.cpp/.h` (protocol handler); the ESPHome config schema and code generation are in `sensor.py`. Keep new component files under the existing `smabluetooth_solar` namespace.

Test fixtures live in `tests/`: `compile.yaml` (full config), `minimal.yaml`, and `tests/invalid/*.yaml` (configs that must be REJECTED with specific messages). `esphome/sample/smabluesolar.yaml` is the user-facing example, not a CI fixture.

## Canonical Install URL
`.repo-url` is the single source of truth for the install URL (`keerekeerweere/esphome_smabluetooth`). A url-guard CI check fails any PR that adds a `github.com` or `github://` URL for this repo pointing at a different slug — never rewrite the install URL in `README.md` or the sample YAML (e.g., to a fork).

## Hardware Constraints

- Targets **ESP-IDF** framework (not Arduino). Agents may default to Arduino; always verify `framework:` in YAML matches `type: esp-idf`.
- Only standard **ESP32** supports classic Bluetooth (SPP). ESP32-S2/S3/C2/C3/C5/C6/C61 are incompatible. WROVER/U/D/S flash variants all work.
- BLE is disabled (`CONFIG_BT_BLE_ENABLED: n`) because classic BT SPP shares Bluedroid resources — enabling both causes runtime crashes.

## Build, Test, and Development Commands
CI runs ESPHome inside Docker; replicate it from the repository root:

- `docker run --rm -u "$(id -u):$(id -g)" -e HOME=/tmp -v "$PWD:/config" -w /config ghcr.io/esphome/esphome:stable compile tests/compile.yaml` — full compile fixture (CI also runs `beta` and `dev` tags).
- `docker run --rm -u "$(id -u):$(id -g)" -e HOME=/tmp -v "$PWD:/config" -w /config ghcr.io/esphome/esphome:stable /bin/bash tests/check-invalid-configs.sh` — asserts each `tests/invalid/*.yaml` is rejected with its expected error message.
- If ESPHome is installed locally, `esphome config tests/compile.yaml` validates YAML + Python schema quickly without a full build; `esphome compile tests/minimal.yaml` reuses the full-build tree if run after `compile.yaml`.
- Lint locally with pinned tools: `pip install -r requirements-ci.txt`, then `ruff check esphome/components/smabluetooth_solar` and `yamllint --strict .` (CI also runs actionlint and zizmor on workflows).
- `pytest tests/test_config_validation.py -k <name>` runs schema-validation tests locally; this file is NOT run in CI.

For local development, the sample YAML already points `external_components` at a local `path:`; CI fixtures use `tests/compile.yaml` with a local path to `esphome/components`.

## Coding Style & Naming Conventions
Python uses 4-space indentation, `snake_case` keys, and `UPPER_SNAKE_CASE` constants for config names (ruff: line-length 120, rules E4/E7/E9/F). C++ uses the ESPHome pattern: `PascalCase` classes, `snake_case_` member fields with trailing underscores, and concise logging via `ESP_LOG*`. Keep YAML option names lowercase with underscores, for example `sma_inverter_bluetooth_mac`.

## Testing Guidelines
CI verification is the Docker compile + invalid-config checks above; run them after every change. Three CI workflows run on every PR: `compile.yml`, `quality.yml`, and `url-guard.yml`. When adding a config option or validation rule, add a matching fixture: extend `tests/compile.yaml` and, for rejections, add a file under `tests/invalid/` plus a `check_invalid` line in `tests/check-invalid-configs.sh`. When modifying inverter communication or sensor mapping, test against real ESP32 hardware and note the inverter model (e.g., `SB3000TL-20`, `SB1600TL-10`). Keep `esphome/sample/smabluesolar.yaml` up-to-date when config options change — it doubles as documentation.

## Commit & Pull Request Guidelines
Recent history uses short, imperative subjects such as `fix compile`, `buffer`, and `review type label`. Keep commit titles brief, present tense, and focused on one change. Work happens via branches + pull requests; all three CI workflows run on every PR. PRs should describe the user-visible impact, list the ESPHome commands run, and mention any hardware verification performed. Include updated YAML snippets or logs when changing configuration shape or Bluetooth behavior.
