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

#include "smabluetooth_solar.h"
#include "esphome/core/application.h"
#include "esphome/core/hal.h"
#include "esphome/core/log.h"

namespace esphome {
namespace smabluetooth_solar {

static const char *const TAG = "smabluetooth_solar";

// ============================================================
//  Status code lookup table
// ============================================================

const StatusCode SmaBluetoothSolar::status_codes[] = {
    {50,   "Status"},
    {51,   "Closed"},
    {300,  "Nat"},
    {301,  "Grid failure"},
    {302,  "-------"},
    {303,  "Off"},
    {304,  ISLAND " " MODE},
    {305,  ISLAND " " MODE},
    {306,  "SMA Island " MODE " 60 Hz"},
    {307,  "OK"},
    {308,  "On"},
    {309,  "Operation"},
    {310,  "General operating " MODE},
    {311,  "Open"},
    {312,  "Phase assignment"},
    {313,  "SMA Island mode 50 Hz"},
    {358,  SB " " "4000" TL20},
    {359,  SB " " "5000" TL20},
    {558,  SB " " "3000" TL20},
    {6109, SB " " "1600" TL10},
    {9109, SB " " "1600" TL10},
    {8001, "Solar Inverters"},
    {71,   "Interference device"},
    {73,   "Diffuse insolation"},
    {74,   "Direct insolation"},
    {76,   "Fault correction measure"},
    {77,   "Check AC circuit breaker"},
    {78,   "Check generator"},
    {79,   "Disconnect generator"},
    {80,   "Check parameter"},
    {84,   "Overcurrent " GRID " hw"},
    {85,   "Overcurrent " GRID " sw"},
    {87,   GRID " frequency disturbance"},
    {88,   GRID " frequency not permitted"},
    {89,   GRID " disconnection point"},
};

// ============================================================
//  ESPHome lifecycle
// ============================================================

void SmaBluetoothSolar::setup() {
    ESP_LOGW(TAG, "Starting setup...");
    smaInverter = ESP32_SMA_Inverter::getInstance();
    smaInverter->setup(sma_inverter_bluetooth_mac_,
                       sma_inverter_password_,
                       sma_inverter_delay_values_);
    hasSetup_ = true;
    ESP_LOGW(TAG, "Setup done, inverter mac=%s", sma_inverter_bluetooth_mac_.c_str());
}

void SmaBluetoothSolar::loop() {
    App.feed_wdt();

    if (!hasSetup_) return;

    uint32_t now = millis();

    switch (inverterState) {
    case SmaInverterState::Off:
        // Initialise BT stack and launch the protocol task
        if (smaInverter->begin("ESP32toSMA")) {
            smaInverter->startBtTask();
            inverterState = SmaInverterState::Running;
            ESP_LOGI(TAG, "BT task started, state -> Running");
        } else {
            ESP_LOGE(TAG, "BT begin failed, retrying in 10 s");
            errorRetryTime_ = now + 10000;
            inverterState   = SmaInverterState::Error;
        }
        break;

    case SmaInverterState::Running:
        // Task runs autonomously. Just watch for errors.
        if (smaInverter->hasTaskError()) {
            ESP_LOGE(TAG, "BT task error detected, restarting in 10 s");
            smaInverter->clearTaskError();
            smaInverter->stopBtTask();
            errorRetryTime_ = now + 10000;
            inverterState   = SmaInverterState::Error;
        }
        break;

    case SmaInverterState::Error:
        if (now >= errorRetryTime_) {
            ESP_LOGI(TAG, "Restarting BT task");
            smaInverter->startBtTask();
            inverterState = SmaInverterState::Running;
        }
        break;
    }
}

// ============================================================
//  Polling update — publishes sensor values to Home Assistant
//  Called by ESPHome on the configured update_interval.
// ============================================================

void SmaBluetoothSolar::update() {
    // Compute any derived values that weren't returned by the inverter
    handleMissingValues();

    updateSensor(today_production_,         "EToday",  smaInverter->dispData.EToday);
    updateSensor(total_energy_production_,  "ETotal",  smaInverter->dispData.ETotal);
    updateSensor(grid_frequency_sensor_,    "Freq",    smaInverter->dispData.GridFreq);

    updateSensor(pvs_[0].voltage_sensor_,      "UdcA",  smaInverter->dispData.Udc1);
    updateSensor(pvs_[0].current_sensor_,      "IdcA",  smaInverter->dispData.Idc1);
    updateSensor(pvs_[0].active_power_sensor_, "PDC1",  smaInverter->dispData.Pdc1);

    updateSensor(pvs_[1].voltage_sensor_,      "UdcB",  smaInverter->dispData.Udc2);
    updateSensor(pvs_[1].current_sensor_,      "IdcB",  smaInverter->dispData.Idc2);
    updateSensor(pvs_[1].active_power_sensor_, "PDC2",  smaInverter->dispData.Pdc2);

    updateSensor(phases_[0].voltage_sensor_,      "UacA",  smaInverter->dispData.Uac1);
    updateSensor(phases_[0].current_sensor_,      "IacA",  smaInverter->dispData.Iac1);
    updateSensor(phases_[0].active_power_sensor_, "PacA",  smaInverter->dispData.Pac1);

#if PHASES > 1
    updateSensor(phases_[1].voltage_sensor_,      "UacB",  smaInverter->dispData.Uac2);
    updateSensor(phases_[1].current_sensor_,      "IacB",  smaInverter->dispData.Iac2);
    updateSensor(phases_[1].active_power_sensor_, "PacB",  smaInverter->dispData.Pac2);
#endif

#if PHASES > 2
    updateSensor(phases_[2].voltage_sensor_,      "UacC",  smaInverter->dispData.Uac3);
    updateSensor(phases_[2].current_sensor_,      "IacC",  smaInverter->dispData.Iac3);
    updateSensor(phases_[2].active_power_sensor_, "PacC",  smaInverter->dispData.Pac3);
#endif

    updateSensor(status_text_sensor_,        "InvStatus",
                 std::string(lookup_code(smaInverter->invData.DevStatus)));
    updateSensor(grid_relay_binary_sensor_,  "GridRelay",
                 smaInverter->invData.GridRelay == 51);  // 51 = "Closed"

#if HAVE_MODULE_TEMP
    updateSensor(inverter_module_temp_,      "InvTemp",   smaInverter->dispData.InvTemp);
#endif

    updateSensor(inverter_bluetooth_signal_strength_, "BTSignal",
                 smaInverter->dispData.BTSigStrength);
    updateSensor(today_generation_time_,  "TToday",
                 (float)smaInverter->invData.OperationTime / 3600.0f);
    updateSensor(total_generation_time_,  "TTotal",
                 (float)smaInverter->invData.FeedInTime / 3600.0f);
    updateSensor(wakeup_time_,            "TWakeup",
                 (uint64_t)smaInverter->invData.WakeupTime);
    updateSensor(serial_number_,          "SerialNum",  smaInverter->invData.DeviceName);
    updateSensor(software_version_,       "SWVersion",  smaInverter->invData.SWVersion);
    updateSensor(device_type_,            "DevType",
                 std::string(lookup_code(smaInverter->invData.DeviceType)));
    updateSensor(device_class_,           "DevClass",
                 std::string(lookup_code(smaInverter->invData.DeviceClass)));

    smaInverter->clearDataReady();
}

// ============================================================
//  Missing-value calculation
//  Some older inverters with retrofitted BT don't return power
//  directly; derive it from U * I.
// ============================================================

void SmaBluetoothSolar::handleMissingValues() {
    auto &d = smaInverter->dispData;

    if (d.Pdc1 == 0.0f || d.needsMissingValues) {
        if (d.Udc1 != 0.0f && d.Idc1 != 0.0f) {
            d.Pdc1 = d.Udc1 * d.Idc1 / 1000.0f;
            d.needsMissingValues = true;
        }
    }
    if (d.Pdc2 == 0.0f || d.needsMissingValues) {
        if (d.Udc2 != 0.0f && d.Idc2 != 0.0f) {
            d.Pdc2 = d.Udc2 * d.Idc2 / 1000.0f;
            d.needsMissingValues = true;
        }
    }
    if (d.Pac1 == 0.0f || d.needsMissingValues) {
        if (d.Uac1 != 0.0f && d.Iac1 != 0.0f) {
            d.Pac1 = d.Uac1 * d.Iac1 / 1000.0f;
            d.needsMissingValues = true;
        }
    }
    if (d.Pac2 == 0.0f || d.needsMissingValues) {
        if (d.Uac2 != 0.0f && d.Iac2 != 0.0f) {
            d.Pac2 = d.Uac2 * d.Iac2 / 1000.0f;
            d.needsMissingValues = true;
        }
    }
    if (d.Pac3 == 0.0f || d.needsMissingValues) {
        if (d.Uac3 != 0.0f && d.Iac3 != 0.0f) {
            d.Pac3 = d.Uac3 * d.Iac3 / 1000.0f;
            d.needsMissingValues = true;
        }
    }
}

// ============================================================
//  Generic sensor publish helpers
// ============================================================

void SmaBluetoothSolar::updateSensor(text_sensor::TextSensor *sensor, const char *name,
                                      std::string v) {
    if (!v.empty() && sensor != nullptr) sensor->publish_state(v);
}

void SmaBluetoothSolar::updateSensor(sensor::Sensor *sensor, const char *name, int32_t v) {
    if (v >= 0 && sensor != nullptr) sensor->publish_state(v);
}

void SmaBluetoothSolar::updateSensor(sensor::Sensor *sensor, const char *name, uint64_t v) {
    if (sensor != nullptr) sensor->publish_state((float)v);
}

void SmaBluetoothSolar::updateSensor(sensor::Sensor *sensor, const char *name, float v) {
    if (v >= 0.0f && sensor != nullptr) sensor->publish_state(v);
}

void SmaBluetoothSolar::updateSensor(binary_sensor::BinarySensor *sensor, const char *name,
                                      bool v) {
    if (sensor != nullptr) sensor->publish_state(v);
}

// ============================================================
//  Diagnostics
// ============================================================

void SmaBluetoothSolar::dump_config() {
    ESP_LOGCONFIG(TAG, "SMABluetooth Solar:");
    ESP_LOGCONFIG(TAG, "  MAC: %s", sma_inverter_bluetooth_mac_.c_str());
    ESP_LOGCONFIG(TAG, "  Delay between queries: %u ms", sma_inverter_delay_values_);
}

const char *SmaBluetoothSolar::lookup_code(uint16_t code) {
    for (const auto &entry : status_codes) {
        if (entry.code == code) return entry.message;
    }
    return "Unknown";
}

} // namespace smabluetooth_solar
} // namespace esphome
