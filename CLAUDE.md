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
loop() state machine → ESP32_SMA_Inverter singleton
  ↓ BluetoothSerial (Arduino built-in)
  ↓ SMANET2 packet encode → send → receive → decode
  ↓ InverterData {raw int32/int64} → DisplayData {float}
update() callback → ESPHome sensors → Home Assistant
```

### State Machine (`SmaInverterState`)

```
Off → Begin → Connect → Initialize → Logon → SignalStrength
→ ReadValues → DoneReadingValues → (loop: SignalStrength)
```

Night mode optimization: scan interval drops to 15 minutes when no generation is detected.

### Key Design Points

- `ESP32_SMA_Inverter` is a **singleton** — only one inverter connection per device.
- `handleMissingValues()` calculates `Pdc`/`Pac` from `U × I` when inverters don't return power directly; controlled by `needsMissingValues` flag and `ignoreQueryErrorTypes[]`.
- Watchdog feeding (`App.feed_wdt()`) is called frequently to prevent resets during blocking Bluetooth operations.
- Fixed 2048-byte packet buffer. Max usage is tracked for debugging.
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
