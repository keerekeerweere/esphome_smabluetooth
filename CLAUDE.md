# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What This Project Is

An ESPHome external component enabling ESP32 devices to read data from SMA solar inverters via classic Bluetooth (Serial Port Profile / SMANET2 protocol). Only standard ESP32 variants support classic Bluetooth — ESP32-S2, S3, C2/C3/C5/C6/C61 are incompatible (enforced in the `sensor.py` schema).

## Commands

CI runs ESPHome in Docker; replicate from the repo root (stable/beta/dev image tags are all built in CI):

```bash
# Full compile fixture
docker run --rm -u "$(id -u):$(id -g)" -e HOME=/tmp -v "$PWD:/config" -w /config ghcr.io/esphome/esphome:stable compile tests/compile.yaml

# Assert each tests/invalid/*.yaml is rejected with its expected error
docker run --rm -u "$(id -u):$(id -g)" -e HOME=/tmp -v "$PWD:/config" -w /config ghcr.io/esphome/esphome:stable /bin/bash tests/check-invalid-configs.sh

# With ESPHome installed locally: fast schema validation / compile
esphome config tests/compile.yaml
esphome compile tests/minimal.yaml   # reuses the full-build tree if run after compile.yaml

# Lint (pinned in requirements-ci.txt; CI also runs actionlint + zizmor on workflows)
pip install -r requirements-ci.txt
ruff check esphome/components/smabluetooth_solar    # line-length 120, rules E4/E7/E9/F
yamllint --strict .

# Flash the user-facing sample (uses a local external_components path)
esphome run esphome/sample/smabluesolar.yaml
```

`tests/test_config_validation.py` is a pytest file for schema validation (`pytest tests/test_config_validation.py -k <name>`); it is not run in CI. CI verification is the compile + invalid-config checks plus lint.

When adding a config option or validation rule: extend `tests/compile.yaml`; for rejections add `tests/invalid/<name>.yaml` plus a `check_invalid` line in `tests/check-invalid-configs.sh`. `esphome/sample/smabluesolar.yaml` is the user-facing example (not a CI fixture) — keep it updated when options change. Work goes through branches + PRs; both CI workflows (`compile.yml`, `quality.yml`) run on every PR.

## Framework

The component targets **ESP-IDF** (not Arduino). YAML only needs `framework: type: esp-idf`; `to_code()` in `sensor.py` pulls in the IDF `bt` component and sets the sdkconfig options for classic BT SPP (`CONFIG_BT_ENABLED`, `CONFIG_BT_BLUEDROID_ENABLED`, `CONFIG_BT_CLASSIC_ENABLED`, `CONFIG_BT_SPP_ENABLED`, `CONFIG_BTDM_CTRL_MODE_BR_EDR_ONLY`) and disables BLE (`CONFIG_BT_BLE_ENABLED`).

## Architecture

### Component Files (`esphome/components/smabluetooth_solar/`)

| File | Role |
|------|------|
| `smabluetooth_solar.h/.cpp` | Main ESPHome `PollingComponent` — state machine + sensor publishing |
| `SMA_Inverter.h/.cpp` | Singleton protocol handler — Bluetooth serial, SMANET2 encoding/decoding |
| `sensor.py` | ESPHome config schema (cv.Schema), code generation (`cg.add()`) |
| `__init__.py` | Defines `DOMAIN` only |

### Data Flow

```
ESPHome Main Loop (core 1)
  SmaBluetoothSolar::loop()   ← thin: just manages task lifecycle (Off/Running/Error)
  SmaBluetoothSolar::update() ← takeDataSnapshot(), publishes sensors on update_interval

FreeRTOS BT Task (core 0)  "bt_sma_proto"  12 KB stack
  wait BT_EVT_SPP_INIT → esp_spp_start_discovery() → wait BT_EVT_DISC_DONE
  esp_spp_connect() → wait BT_EVT_CONNECTED
  initialiseSMAConnection() + logonSMAInverter() + setInverterTime()
  once per connection: getInverterData(TypeLabel, SoftwareVersion)
  free-running loop (not tied to update_interval):
    8 fast queries every cycle; 3 slow queries (status/relay/temp) every 6th cycle;
    getBT_SignalStrength every 30th cycle → snapshot copied under data_mutex_, data_ready_ = true
  night (is_nighttime()): disconnect, re-check every 30 min

SPP callback (Bluedroid task)
  ESP_SPP_DATA_IND_EVT → xStreamBufferSend(rx_stream_buf_)
  ESP_SPP_OPEN/CLOSE_EVT  → xEventGroupSetBits(bt_event_group_)
```

### State Machine (`SmaInverterState`)

Simplified — the BT task handles the full protocol internally:
```
Off → (begin + startBtTask) → Running ↔ Error
```

### Key Design Points

- `ESP32_SMA_Inverter` is a **singleton** — only one inverter connection per device.
- All blocking I/O (`BTgetByte` via `xStreamBufferReceive`) runs inside the **FreeRTOS BT task**. The ESPHome main loop is never blocked.
- `handleMissingValues()` calculates `Pdc`/`Pac` from `U × I` when inverters report zero power but non-zero voltage and current. It runs on the snapshot copy in `update()`, so the `needsMissingValues` flag does not persist between cycles.
- RX data path: SPP callback → 4096-byte `StreamBuffer` → `BTgetByte()` in the BT task.
- Error codes: `E_OK`, `E_NODATA`, `E_CHKSUM`, `E_INVPASSW`, etc.

### SMANET2 Protocol Essentials

Packet header: `[0x7E][Length 2B][Checksum][Src MAC 6B][Dst MAC 6B][Cmd 2B]`

Data queries use LRI (Logical Register Index) identifiers, e.g.:
- `0x263F` = Total AC power (`GridMsTotW`)
- `0x251E` = DC power (`DcMsWatt`)
- `0x2622` = Daily yield (`MeteringDyWhOut`)

Authentication adds `0x88` (user group) or `0xBB` (installer) to each password byte, padded to 12 bytes.

## Coding Conventions

- **Python**: 4-space indent, `snake_case` keys, `UPPER_SNAKE_CASE` constants for config names.
- **C++**: `PascalCase` classes, `snake_case_` member fields (trailing underscore), `ESP_LOG*` macros for logging.
- **YAML options**: lowercase underscore, e.g. `sma_inverter_bluetooth_mac`.

## Commit Style

Short imperative present-tense subjects (see recent history: `fix compile`, `buffer`, `review type label`). PRs should list ESPHome commands run and note inverter model tested (e.g. `SB3000TL-20`, `SB1600TL-10`).
