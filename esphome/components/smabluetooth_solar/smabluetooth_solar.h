#pragma once

#ifndef SMABLUETOOTH_SOLAR_H
#define SMABLUETOOTH_SOLAR_H

/* MIT License

Copyright (c) 2023 keerekeerweere

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.


Disclaimer
A user of the esphome component software acknowledges that he or she is
receiving this software on an "as is" basis and the user is not relying on
the accuracy or functionality of the software for any purpose. The user further
acknowledges that any use of this software will be at his own risk and the
copyright owner accepts no responsibility whatsoever arising from the use or
application of the software.

SMA, Speedwire are registered trademarks of SMA Solar Technology AG

*/

#define PHASES 1  // Type of inverter, one or 3 phases
#define HAVE_MODULE_TEMP false

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "SMA_Inverter.h"

#define SB     "SB"
#define GRID   "GRID"
#define ISLAND "Island"
#define MODE   "mode"
#define TL20   "TL-20"
#define TL10   "TL-10"

namespace esphome {
namespace smabluetooth_solar {

static const float TWO_DEC_UNIT = 0.01;
static const float ONE_DEC_UNIT = 0.1;

enum SmaBluetoothProtocolVersion {
    SMANET2 = 0
};

// Simplified state machine — the BT task handles the full protocol lifecycle
enum class SmaInverterState {
    Off,      // Initial state; BT not started
    Running,  // BT task active, protocol loop running
    Error,    // Task reported an error; waiting before restart
};

struct StatusCode {
    uint16_t    code;
    const char *message;
};

class SmaBluetoothSolar : public PollingComponent {
  public:
    SmaBluetoothSolar() {}

    const char *lookup_code(uint16_t code);

    float get_setup_priority() const override { return setup_priority::LATE; }

    void loop()   override;
    void setup()  override;
    void update() override;
    void handleMissingValues();
    void dump_config() override;

    void updateSensor(text_sensor::TextSensor *sensor, const char *name, std::string v);
    void updateSensor(sensor::Sensor *sensor, const char *name, int32_t v);
    void updateSensor(sensor::Sensor *sensor, const char *name, uint64_t v);
    void updateSensor(sensor::Sensor *sensor, const char *name, float v);
    void updateSensor(binary_sensor::BinarySensor *sensor, const char *name, bool v);

    void set_protocol_version(SmaBluetoothProtocolVersion v) { protocol_version_ = v; }
    void set_sma_inverter_bluetooth_mac(std::string v)       { sma_inverter_bluetooth_mac_ = v; }
    void set_sma_inverter_password(std::string v)            { sma_inverter_password_ = v; }
    void set_sma_inverter_delay_values(uint32_t v)           { sma_inverter_delay_values_ = v; }

    void set_inverter_status_code_sensor(sensor::Sensor *s) { inverter_status_sensor_ = s; }
    void set_grid_relay_code_sensor(sensor::Sensor *s)      { grid_relay_sensor_ = s; }

    void set_inverter_status_sensor(text_sensor::TextSensor *s)   { status_text_sensor_ = s; }
    void set_grid_relay_sensor(binary_sensor::BinarySensor *s)    { grid_relay_binary_sensor_ = s; }

    void set_grid_frequency_sensor(sensor::Sensor *s)             { grid_frequency_sensor_ = s; }
    void set_today_production_sensor(sensor::Sensor *s)           { today_production_ = s; }
    void set_total_energy_production_sensor(sensor::Sensor *s)    { total_energy_production_ = s; }
#ifdef HAVE_MODULE_TEMP
    void set_inverter_module_temp_sensor(sensor::Sensor *s)       { inverter_module_temp_ = s; }
#endif
    void set_inverter_bluetooth_signal_strength(sensor::Sensor *s){ inverter_bluetooth_signal_strength_ = s; }
    void set_today_generation_time(sensor::Sensor *s)             { today_generation_time_ = s; }
    void set_total_generation_time(sensor::Sensor *s)             { total_generation_time_ = s; }
    void set_wakeup_time(sensor::Sensor *s)                       { wakeup_time_ = s; }
    void set_serial_number(text_sensor::TextSensor *s)            { serial_number_ = s; }
    void set_software_version(text_sensor::TextSensor *s)         { software_version_ = s; }
    void set_device_type(text_sensor::TextSensor *s)              { device_type_ = s; }
    void set_device_class(text_sensor::TextSensor *s)             { device_class_ = s; }

    void set_voltage_sensor(uint8_t phase, sensor::Sensor *s) {
        if (phase < PHASES) phases_[phase].voltage_sensor_ = s;
    }
    void set_current_sensor(uint8_t phase, sensor::Sensor *s) {
        if (phase < PHASES) phases_[phase].current_sensor_ = s;
    }
    void set_active_power_sensor(uint8_t phase, sensor::Sensor *s) {
        if (phase < PHASES) phases_[phase].active_power_sensor_ = s;
    }
    void set_voltage_sensor_pv(uint8_t pv, sensor::Sensor *s)      { pvs_[pv].voltage_sensor_ = s; }
    void set_current_sensor_pv(uint8_t pv, sensor::Sensor *s)      { pvs_[pv].current_sensor_ = s; }
    void set_active_power_sensor_pv(uint8_t pv, sensor::Sensor *s) { pvs_[pv].active_power_sensor_ = s; }

  protected:
    struct SmaPhase {
        sensor::Sensor *voltage_sensor_{nullptr};
        sensor::Sensor *current_sensor_{nullptr};
        sensor::Sensor *active_power_sensor_{nullptr};
    } phases_[PHASES];

    struct SmaPV {
        sensor::Sensor *voltage_sensor_{nullptr};
        sensor::Sensor *current_sensor_{nullptr};
        sensor::Sensor *active_power_sensor_{nullptr};
    } pvs_[2];

    sensor::Sensor *inverter_status_sensor_{nullptr};
    sensor::Sensor *grid_relay_sensor_{nullptr};

    text_sensor::TextSensor  *status_text_sensor_{nullptr};
    binary_sensor::BinarySensor *grid_relay_binary_sensor_{nullptr};

    sensor::Sensor *grid_frequency_sensor_{nullptr};
    sensor::Sensor *today_production_{nullptr};
    sensor::Sensor *total_energy_production_{nullptr};
#ifdef HAVE_MODULE_TEMP
    sensor::Sensor *inverter_module_temp_{nullptr};
#endif
    sensor::Sensor *inverter_bluetooth_signal_strength_{nullptr};
    sensor::Sensor *today_generation_time_{nullptr};
    sensor::Sensor *total_generation_time_{nullptr};
    sensor::Sensor *wakeup_time_{nullptr};
    text_sensor::TextSensor *serial_number_{nullptr};
    text_sensor::TextSensor *software_version_{nullptr};
    text_sensor::TextSensor *device_type_{nullptr};
    text_sensor::TextSensor *device_class_{nullptr};

    SmaBluetoothProtocolVersion protocol_version_ = SMANET2;
    std::string sma_inverter_bluetooth_mac_;
    std::string sma_inverter_password_;
    uint32_t    sma_inverter_delay_values_ = 500;

    static const StatusCode status_codes[];

  private:
    ESP32_SMA_Inverter *smaInverter{nullptr};
    SmaInverterState    inverterState{SmaInverterState::Off};
    uint32_t            errorRetryTime_{0};
    bool                hasSetup_{false};

    const float EPSILON = 0.0001f;
};

} // namespace smabluetooth_solar
} // namespace esphome

#endif // SMABLUETOOTH_SOLAR_H
