# ESPHome SMA Bluetooth

ESPHome external component for reading SMA solar inverters over classic Bluetooth using the SMANET2/SPP protocol.

> **Beta quality:** Extended multi-day and multi-week production testing is limited. Use in production at your own discretion.

## Status and requirements

- Requires the **ESP-IDF** framework. Arduino is not supported.
- Requires an original **ESP32** with Bluetooth Classic. ESP32-S2, S3, C2, C3, C5, C6, C61 and other BLE-only variants are not supported.
- Uses a non-blocking Bluetooth protocol task on core 0.
- Supports one inverter per ESP32.
- Automatically disconnects after sunset and reconnects at sunrise.
- Uses the latitude and longitude attributes of Home Assistant's `zone.home` by default. Explicit coordinates can override them.

All original ESP32 module variants, including WROOM, WROVER, U, D and S, are supported.

### Tested hardware

#### SMA inverters

- **SB3000TL-20** — tested and working.
- **SB4000TL-20** — tested and working.
- **SB1600TL-10 with Bluetooth module** — tested with known limitations.

#### ESP32 hardware

- [LilyGO T-ETH-Lite ESP32](https://wiki.lilygo.cc/products/t-eth-series/t-eth-lite/) with the [T-ETH-Lite PoE Shield](https://wiki.lilygo.cc/datasheet/ETH-POE-DP9900M-5V.pdf) — tested and working. This is the original ESP32/RTL8201 model, not the incompatible [T-ETH-Lite ESP32-S3](https://wiki.lilygo.cc/products/t-eth-series/t-eth-lite-s3/).

## ESPHome Device Builder

This external component is not part of Device Builder's generated component catalog. It does not appear in **Add Component**, and Device Builder does not generate a visual form for it. Add and edit the `external_components` and `sensor` sections in the YAML editor.

The schema declares field types, defaults, choices, ranges, advanced fields and the sensitive password. ESPHome uses these declarations to validate the configuration.

## Installation

Add the GitHub source to your ESPHome configuration:

```yaml
external_components:
  - source:
      type: git
      url: https://github.com/keerekeerweere/esphome_smabluetooth
      ref: main
    components: [smabluetooth_solar]
```

Pin `ref` to a release tag or commit for a reproducible installation. For local development, use:

```yaml
external_components:
  - source:
      type: local
      path: esphome/components
    components: [smabluetooth_solar]
```

A minimal component configuration is:

```yaml
sensor:
  - platform: smabluetooth_solar
    id: sma_inverter
    sma_inverter_bluetooth_mac: !secret sma_inverter_bluetooth_mac
    sma_inverter_password: !secret sma_inverter_password

    total_ac_power:
      name: "Total AC Output Power"
    energy_production_day:
      name: "Today's Generation"
    total_energy_production:
      name: "Total Energy Production"
```

The ESPHome `api` component must also be configured. See [`esphome/sample/smabluesolar.yaml`](esphome/sample/smabluesolar.yaml) for a complete example.

## Configuration

### Connection and polling

| Option | Required | Default | Description |
|---|---:|---:|---|
| `sma_inverter_bluetooth_mac` | Yes | — | Bluetooth MAC address of the inverter, such as `00:80:25:12:34:56`. |
| `sma_inverter_password` | Yes | — | SMA user password. It is treated as sensitive configuration and may contain at most 12 characters. |
| `update_interval` | No | `60s` | Interval at which ESPHome publishes a completed inverter snapshot. Standard ESPHome polling-component time syntax is accepted. |
| `sma_inverter_delay_values` | No | `200ms` | Delay between inverter protocol queries. Increase this if the inverter or Bluetooth connection is unreliable. This is an advanced setting and must not be negative. |
| `protocol_version` | No | `SMANET2` | SMA protocol version. The supported value is `SMANET2`; matching is case-insensitive. |

### Night mode and location

| Option | Required | Default | Description |
|---|---:|---:|---|
| `sma_inverter_night_margin` | No | `30min` | Time after calculated sunset before polling stops. Reconnection begins at calculated sunrise. This is an advanced setting and must not be negative. |
| `latitude` | No | `zone.home` | Latitude used for sunrise and sunset calculations, from `-90` to `90`. Accepts ESPHome latitude notation. Overrides the Home Assistant value. |
| `longitude` | No | `zone.home` | Longitude used for sunrise and sunset calculations, from `-180` to `180`. Accepts ESPHome longitude notation. Overrides the Home Assistant value. |

When either coordinate is omitted, the component subscribes to the corresponding attribute of Home Assistant's `zone.home` entity through the native API. Until missing coordinates arrive, the component cannot calculate accurate solar times. Configure both values explicitly when Home Assistant is unavailable or `zone.home` should not be used.

### AC phase and PV input entities

`phase_a`, `phase_b` and `phase_c` are optional groups. Each group accepts these standard ESPHome sensor configurations:

| Child option | Unit | Description |
|---|---:|---|
| `voltage` | V | AC phase voltage. |
| `current` | A | AC phase current. |
| `active_power` | kW | Active output power for the phase. |

`pv1` and `pv2` are optional groups with the same child names:

| Child option | Unit | Description |
|---|---:|---|
| `voltage` | V | DC voltage for the PV input/MPPT. |
| `current` | A | DC current for the PV input/MPPT. |
| `active_power` | kW | DC power for the PV input/MPPT. |

Example:

```yaml
sensor:
  - platform: smabluetooth_solar
    sma_inverter_bluetooth_mac: !secret sma_inverter_bluetooth_mac
    sma_inverter_password: !secret sma_inverter_password

    phase_a:
      voltage:
        name: "Phase A Voltage"
      current:
        name: "Phase A Current"
      active_power:
        name: "Phase A Power"

    pv1:
      voltage:
        name: "PV1 Voltage"
      current:
        name: "PV1 Current"
      active_power:
        name: "PV1 Power"
```

### Measurement entities

Every entity is optional and accepts the normal ESPHome configuration for its entity type, including `name`, `id`, filters and entity-category overrides where applicable.

| Option | Type | Unit | Description |
|---|---|---:|---|
| `total_ac_power` | Sensor | kW | Total instantaneous AC output power. |
| `frequency` | Sensor | Hz | Grid frequency reported by the inverter. |
| `energy_production_day` | Sensor | kWh | Energy generated today. |
| `total_energy_production` | Sensor | kWh | Lifetime energy production. |
| `inverter_module_temp` | Sensor | °C | Inverter module temperature. |
| `today_generation_time` | Sensor | h | Feed-in/generation time for the current day. |
| `total_generation_time` | Sensor | h | Total inverter operating/generation time. |
| `wakeup_time` | Sensor | timestamp | Inverter wake-up timestamp. |
| `sma_inverter_bluetooth_signal_strength` | Sensor | % | Bluetooth signal strength. Defaults to the Diagnostics category. |

### Status and identity entities

| Option | Type | Description |
|---|---|---|
| `inverter_status` | Text sensor | Human-readable operating status reported by the inverter. |
| `bluetooth_connection_status` | Text sensor | Bluetooth lifecycle/connection status. Defaults to the Diagnostics category. |
| `grid_relay` | Binary sensor | State of the inverter's grid relay. |
| `serial_number` | Text sensor | Inverter serial number. Defaults to the Diagnostics category. |
| `software_version` | Text sensor | Inverter firmware/software version. Defaults to the Diagnostics category. |
| `device_type` | Text sensor | Inverter model/type. Defaults to the Diagnostics category. |
| `device_class` | Text sensor | SMA device class. Defaults to the Diagnostics category. |
| `device_name` | Text sensor | Name reported by the inverter. Defaults to the Diagnostics category. |
| `inverter_time` | Text sensor | Timestamp attached to the latest inverter measurement, formatted as ISO 8601 UTC, for example `2026-11-07T09:11:02Z`. Defaults to the Diagnostics category. |

`inverter_time` is a measurement timestamp rather than a live copy of the ESPHome system clock. It can lag local time while the inverter samples or buffers data.

## Full example

The maintained full configuration is [`esphome/sample/smabluesolar.yaml`](esphome/sample/smabluesolar.yaml). It demonstrates all available entities and local external-component loading.

## Validation tests

The schema tests invoke the real ESPHome CLI and cover defaults, complete nested configuration, ranges, invalid values, framework restrictions and unsupported ESP32 variants:

```bash
pytest -q tests/test_config_validation.py
```

Validate or compile the complete sample after creating `esphome/sample/secrets.yaml` with the referenced secrets:

```bash
esphome config esphome/sample/smabluesolar.yaml
esphome compile esphome/sample/smabluesolar.yaml
```
