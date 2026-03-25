# esphome_smabluetooth
ESPHome external component to read SMA solar inverters via classic Bluetooth (SMANET2 / SPP protocol).

**Beta quality** — the component is significantly more stable than earlier releases but has not yet been validated over extended multi-day/multi-week runs. Use in production at your own discretion.

## Status
- Tested and working on **SB3000TL-20**; known limitations on **SB1600TL-10** (with BT module)
- Runs on ESP-IDF (not Arduino); BLE disabled to preserve RAM
- Non-blocking BT protocol task on core 0; ESPHome main loop never stalled
- Automatic night mode: disconnects from inverter at sunset, reconnects at sunrise — no polling during darkness

## Hardware compatibility
Only the original ESP32 supports classic Bluetooth (SPP). Other variants do **not** work:

| Supported | Not supported |
|-----------|---------------|
| ESP32 (all flash/antenna variants: WROOM, WROVER, U, D, S) | ESP32-S2, S3 |
| | ESP32-C2, C3, C5, C6, C61 |

## Known limitations
- Single inverter per ESP32 (singleton BT connection)
- Not yet validated over weeks of continuous operation

## Usage
See `esphome/sample/smabluesolar.yaml` for a full configuration example.

For local development, replace the `external_components` GitHub URL with a `path:` pointing to the repository root.
