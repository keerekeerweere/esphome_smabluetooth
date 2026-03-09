import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor, text_sensor, binary_sensor
from esphome.const import (
    CONF_ACTIVE_POWER,
    CONF_CURRENT,
    CONF_FREQUENCY,
    CONF_ID,
    CONF_VOLTAGE,
    DEVICE_CLASS_CURRENT,
    DEVICE_CLASS_ENERGY,
    DEVICE_CLASS_POWER,
    DEVICE_CLASS_VOLTAGE,
    DEVICE_CLASS_FREQUENCY,
    DEVICE_CLASS_TEMPERATURE,
    DEVICE_CLASS_SIGNAL_STRENGTH,
    DEVICE_CLASS_DURATION,
    DEVICE_CLASS_TIMESTAMP,
    ICON_CURRENT_AC,
    ICON_SIGNAL_DISTANCE_VARIANT,
    ICON_FLASH,
    ICON_THERMOMETER,
    ICON_TIMER,
    STATE_CLASS_MEASUREMENT,
    STATE_CLASS_TOTAL_INCREASING,
    UNIT_AMPERE,
    UNIT_CELSIUS,
    UNIT_HERTZ,
    UNIT_VOLT,
    UNIT_WATT,
    UNIT_KILOWATT,
    UNIT_PERCENT,
    UNIT_KILOWATT_HOURS,
    UNIT_HOUR
)


from esphome.core import CORE

ICON_CURRENT_DC = "mdi:current-dc"
ICON_SINE_WAVE = "mdi:sine-wave"
ICON_SOLAR_POWER = "mdi:solar-power"
ICON_LIGHTNING_BOLT = "mdi:lightning-bolt"
ICON_TRANSMISSION_TOWER = "mdi:transmission-tower"
ICON_CLOCK = "mdi:clock"

CONF_PHASE_A = "phase_a"
CONF_PHASE_B = "phase_b"
CONF_PHASE_C = "phase_c"

CONF_ENERGY_PRODUCTION_DAY = "energy_production_day"
CONF_TOTAL_ENERGY_PRODUCTION = "total_energy_production"
CONF_TOTAL_GENERATION_TIME = "total_generation_time"
CONF_TODAY_GENERATION_TIME = "today_generation_time"
CONF_WAKEUP_TIME = "wakeup_time"
CONF_SERIAL_NUMBER = "serial_number"
CONF_SOFTWARE_VERSION = "software_version"
CONF_DEVICE_TYPE = "device_type"
CONF_DEVICE_CLASS = "device_class"
CONF_PV1 = "pv1"
CONF_PV2 = "pv2"

CONF_INVERTER_STATUS = "inverter_status"
CONF_INVERTER_MODULE_TEMP = "inverter_module_temp"
CONF_PROTOCOL_VERSION = "protocol_version"

CONF_SMA_INVERTER_BLUETOOTH_MAC = "sma_inverter_bluetooth_mac"
CONF_SMA_INVERTER_PASSWORD = "sma_inverter_password"
CONF_SMA_INVERTER_DELAY_VALUES = "sma_inverter_delay_values"

CONF_SMA_INVERTER_BLUETOOTH_SIGNAL_STRENGTH = "sma_inverter_bluetooth_signal_strength"

CONF_GRID_RELAY = "grid_relay"

AUTO_LOAD = ["text_sensor", "binary_sensor"]
DEPENDENCIES = ["esp32", "sensor", "network"]
CODEOWNERS = ["@keerekeerweere"]


smabluetooth_solar_ns = cg.esphome_ns.namespace("smabluetooth_solar")
SmaBluetoothSolar = smabluetooth_solar_ns.class_(
    "SmaBluetoothSolar", cg.PollingComponent
)

PHASE_SENSORS = {
    CONF_VOLTAGE: sensor.sensor_schema(
        unit_of_measurement=UNIT_VOLT,
        icon=ICON_SINE_WAVE,
        accuracy_decimals=1,
        device_class=DEVICE_CLASS_VOLTAGE,
        state_class=STATE_CLASS_MEASUREMENT
    ),
    CONF_CURRENT: sensor.sensor_schema(
        unit_of_measurement=UNIT_AMPERE,
        icon=ICON_CURRENT_AC,
        accuracy_decimals=2,
        device_class=DEVICE_CLASS_CURRENT,
        state_class=STATE_CLASS_MEASUREMENT
    ),
    CONF_ACTIVE_POWER: sensor.sensor_schema(
        unit_of_measurement=UNIT_KILOWATT,
        icon=ICON_LIGHTNING_BOLT,
        accuracy_decimals=0,
        device_class=DEVICE_CLASS_POWER,
        state_class=STATE_CLASS_MEASUREMENT,
    ),
}
PV_SENSORS = {
    CONF_VOLTAGE: sensor.sensor_schema(
        unit_of_measurement=UNIT_VOLT,
        icon=ICON_SINE_WAVE,
        accuracy_decimals=1,
        device_class=DEVICE_CLASS_VOLTAGE,
        state_class=STATE_CLASS_MEASUREMENT
    ),
    CONF_CURRENT: sensor.sensor_schema(
        unit_of_measurement=UNIT_AMPERE,
        icon=ICON_CURRENT_DC,
        accuracy_decimals=2,
        device_class=DEVICE_CLASS_CURRENT,
        state_class=STATE_CLASS_MEASUREMENT,
    ),
    CONF_ACTIVE_POWER: sensor.sensor_schema(
        unit_of_measurement=UNIT_KILOWATT,
        icon=ICON_SOLAR_POWER,
        accuracy_decimals=0,
        device_class=DEVICE_CLASS_POWER,
        state_class=STATE_CLASS_MEASUREMENT,
    ),
}

PHASE_SCHEMA = cv.Schema(
    {cv.Optional(sensor): schema for sensor, schema in PHASE_SENSORS.items()}
)
PV_SCHEMA = cv.Schema(
    {cv.Optional(sensor): schema for sensor, schema in PV_SENSORS.items()}
)

SmaBluetoothProtocolVersion = smabluetooth_solar_ns.enum("SmaBluetoothProtocolVersion")
PROTOCOL_VERSIONS = {
    "SMANET2": SmaBluetoothProtocolVersion.SMANET2
}


CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(SmaBluetoothSolar),
            cv.Required(CONF_SMA_INVERTER_BLUETOOTH_MAC): cv.string,
            cv.Required(CONF_SMA_INVERTER_PASSWORD): cv.string,
            cv.Optional(CONF_SMA_INVERTER_DELAY_VALUES, default="200ms"): cv.positive_time_period_milliseconds,

            cv.Optional(CONF_SMA_INVERTER_BLUETOOTH_SIGNAL_STRENGTH): sensor.sensor_schema(
                unit_of_measurement=UNIT_PERCENT,
                icon=ICON_SIGNAL_DISTANCE_VARIANT,
                accuracy_decimals=1,
                device_class=DEVICE_CLASS_SIGNAL_STRENGTH,
                state_class=STATE_CLASS_MEASUREMENT
            ),
            cv.Optional(CONF_PROTOCOL_VERSION, default="SMANET2"): cv.enum(
                PROTOCOL_VERSIONS, upper=True
            ),
            cv.Optional(CONF_PHASE_A): PHASE_SCHEMA,
            cv.Optional(CONF_PHASE_B): PHASE_SCHEMA,
            cv.Optional(CONF_PHASE_C): PHASE_SCHEMA,
            cv.Optional(CONF_PV1): PV_SCHEMA,
            cv.Optional(CONF_PV2): PV_SCHEMA,
            cv.Optional(CONF_INVERTER_STATUS): text_sensor.text_sensor_schema(),
            cv.Optional(CONF_GRID_RELAY): binary_sensor.binary_sensor_schema(
                icon=ICON_TRANSMISSION_TOWER,
            ),
            cv.Optional(CONF_FREQUENCY): sensor.sensor_schema(
                unit_of_measurement=UNIT_HERTZ,
                icon=ICON_SINE_WAVE,
                accuracy_decimals=2,
                device_class=DEVICE_CLASS_FREQUENCY,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_ENERGY_PRODUCTION_DAY): sensor.sensor_schema(
                unit_of_measurement=UNIT_KILOWATT_HOURS,
                icon=ICON_FLASH,
                accuracy_decimals=3,
                device_class=DEVICE_CLASS_ENERGY,
                state_class=STATE_CLASS_TOTAL_INCREASING,
            ),
            cv.Optional(CONF_TOTAL_ENERGY_PRODUCTION): sensor.sensor_schema(
                unit_of_measurement=UNIT_KILOWATT_HOURS,
                icon=ICON_FLASH,
                accuracy_decimals=3,
                device_class=DEVICE_CLASS_ENERGY,
                state_class=STATE_CLASS_TOTAL_INCREASING,
            ),
            cv.Optional(CONF_INVERTER_MODULE_TEMP): sensor.sensor_schema(
                unit_of_measurement=UNIT_CELSIUS,
                icon=ICON_THERMOMETER,
                accuracy_decimals=1,
                device_class=DEVICE_CLASS_TEMPERATURE,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_TODAY_GENERATION_TIME): sensor.sensor_schema(
                unit_of_measurement=UNIT_HOUR,
                icon=ICON_TIMER,
                accuracy_decimals=3,
                device_class=DEVICE_CLASS_DURATION,
                state_class=STATE_CLASS_TOTAL_INCREASING,
            ),
            cv.Optional(CONF_TOTAL_GENERATION_TIME): sensor.sensor_schema(
                unit_of_measurement=UNIT_HOUR,
                icon=ICON_TIMER,
                accuracy_decimals=3,
                device_class=DEVICE_CLASS_DURATION,
                state_class=STATE_CLASS_TOTAL_INCREASING,
            ),
            cv.Optional(CONF_WAKEUP_TIME): sensor.sensor_schema(
                icon=ICON_CLOCK,
                device_class=DEVICE_CLASS_TIMESTAMP,
            ),
            cv.Optional(CONF_SERIAL_NUMBER): text_sensor.text_sensor_schema(),
            cv.Optional(CONF_SOFTWARE_VERSION): text_sensor.text_sensor_schema(),
            cv.Optional(CONF_DEVICE_TYPE): text_sensor.text_sensor_schema(),
            cv.Optional(CONF_DEVICE_CLASS): text_sensor.text_sensor_schema(),
        }
    )
    .extend(cv.polling_component_schema("60s"))
#    .extend(modbus.modbus_device_schema(0x01))
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
#    await modbus.register_modbus_device(var, config)

    cg.add(var.set_sma_inverter_bluetooth_mac(config[CONF_SMA_INVERTER_BLUETOOTH_MAC]))
    cg.add(var.set_sma_inverter_password(config[CONF_SMA_INVERTER_PASSWORD]))

    if CONF_SMA_INVERTER_DELAY_VALUES in config:
        cg.add(var.set_sma_inverter_delay_values(config[CONF_SMA_INVERTER_DELAY_VALUES].total_milliseconds))

    cg.add(var.set_protocol_version(config[CONF_PROTOCOL_VERSION]))

    if CONF_INVERTER_STATUS in config:
        sens = await text_sensor.new_text_sensor(config[CONF_INVERTER_STATUS])
        cg.add(var.set_inverter_status_sensor(sens))

    if CONF_GRID_RELAY in config:
        sens = await binary_sensor.new_binary_sensor(config[CONF_GRID_RELAY])
        cg.add(var.set_grid_relay_sensor(sens))

    if CONF_FREQUENCY in config:
        sens = await sensor.new_sensor(config[CONF_FREQUENCY])
        cg.add(var.set_grid_frequency_sensor(sens))

    if CONF_ENERGY_PRODUCTION_DAY in config:
        sens = await sensor.new_sensor(config[CONF_ENERGY_PRODUCTION_DAY])
        cg.add(var.set_today_production_sensor(sens))

    if CONF_TOTAL_ENERGY_PRODUCTION in config:
        sens = await sensor.new_sensor(config[CONF_TOTAL_ENERGY_PRODUCTION])
        cg.add(var.set_total_energy_production_sensor(sens))

    if CONF_INVERTER_MODULE_TEMP in config:
        sens = await sensor.new_sensor(config[CONF_INVERTER_MODULE_TEMP])
        cg.add(var.set_inverter_module_temp_sensor(sens))

    if CONF_SMA_INVERTER_BLUETOOTH_SIGNAL_STRENGTH in config:
        sens = await sensor.new_sensor(config[CONF_SMA_INVERTER_BLUETOOTH_SIGNAL_STRENGTH])
        cg.add(var.set_inverter_bluetooth_signal_strength(sens))

    if CONF_TODAY_GENERATION_TIME in config:
        sens = await sensor.new_sensor(config[CONF_TODAY_GENERATION_TIME])
        cg.add(var.set_today_generation_time(sens))

    if CONF_TOTAL_GENERATION_TIME in config:
        sens = await sensor.new_sensor(config[CONF_TOTAL_GENERATION_TIME])
        cg.add(var.set_total_generation_time(sens))

    if CONF_WAKEUP_TIME in config:
        sens = await sensor.new_sensor(config[CONF_WAKEUP_TIME])
        cg.add(var.set_wakeup_time(sens))

    if CONF_SERIAL_NUMBER in config:
        sens = await text_sensor.new_text_sensor(config[CONF_SERIAL_NUMBER])
        cg.add(var.set_serial_number(sens))

    if CONF_SOFTWARE_VERSION in config:
        sens = await text_sensor.new_text_sensor(config[CONF_SOFTWARE_VERSION])
        cg.add(var.set_software_version(sens))

    if CONF_DEVICE_TYPE in config:
        sens = await text_sensor.new_text_sensor(config[CONF_DEVICE_TYPE])
        cg.add(var.set_device_type(sens))

    if CONF_DEVICE_CLASS in config:
        sens = await text_sensor.new_text_sensor(config[CONF_DEVICE_CLASS])
        cg.add(var.set_device_class(sens))

    for i, phase in enumerate([CONF_PHASE_A, CONF_PHASE_B, CONF_PHASE_C]):
        if phase not in config:
            continue

        phase_config = config[phase]
        for sensor_type in PHASE_SENSORS:
            if sensor_type in phase_config:
                sens = await sensor.new_sensor(phase_config[sensor_type])
                cg.add(getattr(var, f"set_{sensor_type}_sensor")(i, sens))

    for i, pv in enumerate([CONF_PV1, CONF_PV2]):
        if pv not in config:
            continue

        pv_config = config[pv]
        for sensor_type in pv_config:
            if sensor_type in pv_config:
                sens = await sensor.new_sensor(pv_config[sensor_type])
                cg.add(getattr(var, f"set_{sensor_type}_sensor_pv")(i, sens))


    if CORE.using_arduino:
        if CORE.is_esp32 | CORE.is_esp8266:
            cg.add_library("BluetoothSerial", None)

