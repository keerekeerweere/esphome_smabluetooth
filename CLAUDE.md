# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What This Project Is

An ESPHome external component enabling ESP32 devices to read data from SMA solar inverters via classic Bluetooth (Serial Port Profile / SMANET2 protocol). Only standard ESP32 variants support classic Bluetooth — ESP32-S2, S3, C2/C3/C5/C6 are incompatible.

## Commands

```bash
# Validate YAML + Python schema wiring
esphome config esphome/sample/smabluesolar.yaml

# Compile and catch C++ integration errors
esphome compile esphome/sample/smabluesolar.yaml

# Flash to connected device
esphome run esphome/sample/smabluesolar.yaml

# Print ESP32 partition table sizing
python3 esphome/sample/generate-partitions.py
```

For local development, replace the `external_components` GitHub URL in the sample YAML with a `path:` pointing to this repo's root.

There is no test suite. `esphome config` + `esphome compile` is the validation loop. The sample YAML is the regression fixture — keep it updated when options change.

## Framework

The component targets **ESP-IDF** (not Arduino). `smabluesolar.yaml` uses `framework: type: esp-idf` with sdkconfig options to enable classic BT SPP (`CONFIG_BT_ENABLED`, `CONFIG_BT_CLASSIC_ENABLED`, `CONFIG_BT_SPP_ENABLED`, `CONFIG_BTDM_CTRL_MODE_BR_EDR_ONLY`). BLE is disabled to save RAM.

## Architecture

### Component Files (`esphome/components/smabluetooth_solar/`)

| File | Role |
|------|------|
| `smabluetooth_solar.h/.cpp` | Main ESPHome `PollingComponent` — state machine + sensor publishing |
| `SMA_Inverter.h/.cpp` | Singleton protocol handler — Bluetooth serial, SMANET2 encoding/decoding |
| `sensor.py` | ESPHome config schema (cv.Schema), code generation (`cg.add()`) |
| `__init__.py` | Component registration (empty) |

### Data Flow

```
ESPHome Main Loop (core 1)
  SmaBluetoothSolar::loop()   ← thin: just manages task lifecycle (Off/Running/Error)
  SmaBluetoothSolar::update() ← reads shared data, publishes sensors on update_interval

FreeRTOS BT Task (core 0)  "bt_sma_proto"  12 KB stack
  wait BT_EVT_SPP_INIT → esp_spp_start_discovery() → wait BT_EVT_DISC_DONE
  esp_spp_connect() → wait BT_EVT_CONNECTED
  initialiseSMAConnection() + logonSMAInverter()
  loop: getBT_SignalStrength + 13× getInverterData() → data_ready_ = true

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
- `handleMissingValues()` calculates `Pdc`/`Pac` from `U × I` when inverters don't return power directly; controlled by `needsMissingValues` flag.
- RX data path: SPP callback → 4096-byte `StreamBuffer` → `BTgetByte()` in the BT task.
- Error codes: `E_OK`, `E_NODATA`, `E_CHKSUM`, `E_INVPASSW`, etc.

### SMANET2 Protocol Essentials

Packet header: `[0x7E][Length 2B][Checksum][Src MAC 6B][Dst MAC 6B][Cmd 2B]`

Data queries use LRI (Logical Register Index) identifiers, e.g.:
- `0x263F` = Total AC power (`GridMsTotW`)
- `0x251E` = DC power (`DcMsWatt`)
- `0x2622` = Daily yield (`MeteringDyWhOut`)

Authentication XORs password bytes with `0x88` per user group.

## Coding Conventions

- **Python**: 4-space indent, `snake_case` keys, `UPPER_SNAKE_CASE` constants for config names.
- **C++**: `PascalCase` classes, `snake_case_` member fields (trailing underscore), `ESP_LOG*` macros for logging.
- **YAML options**: lowercase underscore, e.g. `sma_inverter_bluetooth_mac`.

## Commit Style

Short imperative present-tense subjects (see recent history: `fix compile`, `buffer`, `review type label`). PRs should list ESPHome commands run and note inverter model tested (e.g. `SB3000TL-20`, `SB1600TL-10`).
