from __future__ import annotations

import subprocess
from pathlib import Path

import pytest

REPOSITORY_ROOT = Path(__file__).parents[1]
COMPONENTS_PATH = REPOSITORY_ROOT / "esphome" / "components"

BASE_CONFIG = """
esphome:
  name: schema-test

esp32:
  board: {board}
  framework:
    type: {framework}

external_components:
  - source:
      type: local
      path: {components_path}
    components: [smabluetooth_solar]

api:

wifi:
  ssid: test-network
  password: test-password

sensor:
  - platform: smabluetooth_solar
{component_config}
"""

MINIMAL_COMPONENT_CONFIG = """    sma_inverter_bluetooth_mac: 00:80:25:12:34:56
    sma_inverter_password: test
"""


def validate_config(
    tmp_path: Path,
    component_config: str = MINIMAL_COMPONENT_CONFIG,
    *,
    board: str = "esp32dev",
    framework: str = "esp-idf",
) -> subprocess.CompletedProcess[str]:
    config = BASE_CONFIG.format(
        components_path=COMPONENTS_PATH,
        component_config=component_config,
        board=board,
        framework=framework,
    )
    config_path = tmp_path / "schema-test.yaml"
    config_path.write_text(config)
    return subprocess.run(
        ["esphome", "config", str(config_path)],
        check=False,
        capture_output=True,
        text=True,
    )


def assert_invalid(result: subprocess.CompletedProcess[str], message: str) -> None:
    assert result.returncode != 0
    assert message in result.stdout + result.stderr


def test_minimal_config_uses_documented_defaults(tmp_path: Path) -> None:
    result = validate_config(tmp_path)

    assert result.returncode == 0, result.stdout + result.stderr
    assert "sma_inverter_delay_values: 200ms" in result.stdout
    assert "sma_inverter_night_margin: 30min" in result.stdout
    assert "protocol_version: SMANET2" in result.stdout
    assert "update_interval: 60s" in result.stdout


@pytest.mark.parametrize(
    ("component_config", "missing_option"),
    [
        ("    sma_inverter_password: test\n", "sma_inverter_bluetooth_mac"),
        (
            "    sma_inverter_bluetooth_mac: 00:80:25:12:34:56\n",
            "sma_inverter_password",
        ),
    ],
)
def test_required_connection_values_are_enforced(
    tmp_path: Path, component_config: str, missing_option: str
) -> None:
    result = validate_config(tmp_path, component_config)

    assert_invalid(result, f"'{missing_option}' is a required option")


def test_full_config_accepts_ranges_entities_and_tuning(tmp_path: Path) -> None:
    result = validate_config(
        tmp_path,
        MINIMAL_COMPONENT_CONFIG + """    sma_inverter_delay_values: 800ms
    sma_inverter_night_margin: 45min
    protocol_version: smanet2
    latitude: -90
    longitude: 180
    phase_a:
      voltage:
        name: Phase A Voltage
      current:
        name: Phase A Current
      active_power:
        name: Phase A Power
    pv1:
      voltage:
        name: PV1 Voltage
      current:
        name: PV1 Current
      active_power:
        name: PV1 Power
    inverter_status:
      name: Inverter Status
    grid_relay:
      name: Grid Relay
    total_ac_power:
      name: Total AC Power
    inverter_time:
      name: Inverter Time
""",
    )

    assert result.returncode == 0, result.stdout + result.stderr


@pytest.mark.parametrize(
    ("component_config", "message"),
    [
        (
            "    sma_inverter_bluetooth_mac: not-a-mac\n"
            "    sma_inverter_password: test\n",
            "not-a-mac",
        ),
        (
            "    sma_inverter_bluetooth_mac: 00:80:25:12:34:56\n"
            "    sma_inverter_password: thirteenchars!\n",
            "length of value must be at most 12",
        ),
        (
            MINIMAL_COMPONENT_CONFIG + "    protocol_version: SMANET1\n",
            "Unknown value 'SMANET1'",
        ),
        (
            MINIMAL_COMPONENT_CONFIG + "    sma_inverter_delay_values: -1ms\n",
            "value must be at least 0s",
        ),
        (
            MINIMAL_COMPONENT_CONFIG + "    sma_inverter_night_margin: -1min\n",
            "value must be at least 0s",
        ),
        (
            MINIMAL_COMPONENT_CONFIG + "    latitude: 91\n",
            "value must be at most 90",
        ),
        (
            MINIMAL_COMPONENT_CONFIG + "    longitude: -181\n",
            "value must be at least -180",
        ),
    ],
)
def test_invalid_component_values_are_rejected(
    tmp_path: Path, component_config: str, message: str
) -> None:
    result = validate_config(tmp_path, component_config)

    assert_invalid(result, message)


@pytest.mark.parametrize(
    ("board", "framework", "message"),
    [
        ("esp32dev", "arduino", "only available with framework(s) esp-idf"),
        ("esp32-s3-devkitc-1", "esp-idf", "only available on ESP32"),
    ],
)
def test_unsupported_targets_are_rejected(
    tmp_path: Path, board: str, framework: str, message: str
) -> None:
    result = validate_config(tmp_path, board=board, framework=framework)

    assert_invalid(result, message)
