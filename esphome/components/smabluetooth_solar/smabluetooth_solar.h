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

#define PHASES 3  // Type of inverter, one or 3 phases
#define HAVE_MODULE_TEMP true

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "SMA_Inverter.h"

#include <vector>
#include <map>
#include <unordered_set>
#include <set>


#define SIZE_INVETER_DATA_TYPE_QUERY 13
#define SB "SB"

namespace esphome {
namespace smabluetooth_solar {

static const float TWO_DEC_UNIT = 0.01;
static const float ONE_DEC_UNIT = 0.1;

enum SmaBluetoothProtocolVersion {
  SMANET2 = 0
};

//states to cycle through
enum class SmaInverterState {
  Off, //when starting up ESP
  Begin, //begin the bluetooth stack
  Connect, //connect to the inverter
  Initialize, //init SMA connection
  SignalStrength,//get signal quality
  Logon,//logon to inverter
  ReadValues,//read values (piece by piece)
  DoneReadingValues,
  Disconnect,//disconnect again from the inverter
  NightTime//nighttime, nothing to do here, wait for next sunlight
};

class SmaBluetoothSolar;

class SmaBluetoothSolar : public PollingComponent {
 public:
    SmaBluetoothSolar() {
      initMap();
    };

    void initMap() {

      codeMap[50]="Status";
      codeMap[51]="Closed";

      codeMap[300]="Nat";
      codeMap[301]="Grid failure";
      codeMap[302]="-------";
      codeMap[303]="Off";
      codeMap[304]="Island mode";
      codeMap[305]="Island mode";
      codeMap[306]="SMA Island mode 60 Hz";
      codeMap[307]="OK";
      codeMap[308]="On";
      codeMap[309]="Operation";
      codeMap[310]="General operating mode";
      codeMap[311]="Open";
      codeMap[312]="Phase assignment";
      codeMap[313]="SMA Island mode 50 Hz";

      codeMap[358]=  SB " " "4000TL-20";
      codeMap[359]=  SB " " "5000TL-20";
      codeMap[558]=  SB " " "3000TL-20";
      codeMap[6109]= SB " " "1600TL-10";
      codeMap[9109]= SB " " "1600TL-10";

      codeMap[8001]="Solar Inverters";

      codeMap[16777213]="Information not available";

      codeMap[71]="Interference device";
      codeMap[73]="Diffuse insolation";
      codeMap[74]="Direct insolation";
      codeMap[76]="Fault correction measure";
      codeMap[77]="Check AC circuit breaker";
      codeMap[78]="Check generator";
      codeMap[79]="Disconnect generator";
      codeMap[80]="Check parameter";
      codeMap[84]="Overcurrent grid hw";
      codeMap[85]="Overcurrent grid sw";

      codeMap[87]="Grid frequency disturbance";
      codeMap[88]="Grid frequency not permitted";
      codeMap[89]="Grid disconnection point";


    }


    std::string getInverterCode(int invCode) {
      std::map<int, std::string>::iterator it = codeMap.find(invCode);
      if (it != codeMap.end())
        return it->second;
      else
        return std::to_string(invCode);
    }

  float get_setup_priority() const override { return setup_priority::LATE; }

  void loop() override;
  void setup() override;

  void update() override;
  void handleMissingValues();
  void updateSensor( text_sensor::TextSensor *sensor,  String sensorName,  std::string publishValue);
  void updateSensor( sensor::Sensor *sensor,  String sensorName,  int32_t publishValue);
  void updateSensor( sensor::Sensor *sensor,  String sensorName,  uint64_t publishValue);
  void updateSensor( sensor::Sensor *sensor,  String sensorName,  float publishValue);
  void updateSensor( binary_sensor::BinarySensor *sensor,  String sensorName,  bool publishValue);
  void on_inverter_data(const std::vector<uint8_t> &data) ;
  void dump_config() override;

  void set_protocol_version(SmaBluetoothProtocolVersion protocol_version) { this->protocol_version_ = protocol_version; }

  void set_sma_inverter_bluetooth_mac(std::string sma_inverter_bluetooth_mac) { this->sma_inverter_bluetooth_mac_ = sma_inverter_bluetooth_mac; }
  void set_sma_inverter_password(std::string sma_inverter_password) {this->sma_inverter_password_ = sma_inverter_password; }

  void set_sma_inverter_delay_values(uint32_t sma_inverter_delay_values) {this->sma_inverter_delay_values_ = sma_inverter_delay_values; }


  void set_inverter_status_code_sensor(sensor::Sensor *sensor) { this->inverter_status_sensor_ = sensor; }
  void set_grid_relay_code_sensor(sensor::Sensor *sensor) { this->grid_relay_sensor_ = sensor; }

  // TEXT_SENSORS
  void set_inverter_status_sensor(text_sensor::TextSensor *text_sensor) { status_text_sensor_ = text_sensor; }
  void set_grid_relay_sensor(binary_sensor::BinarySensor *binary_sensor) { grid_relay_binary_sensor_ = binary_sensor; }
  // END_TEXT_SENSORS

  void set_grid_frequency_sensor(sensor::Sensor *sensor) { this->grid_frequency_sensor_ = sensor; }

  void set_today_production_sensor(sensor::Sensor *sensor) { this->today_production_ = sensor; }
  void set_total_energy_production_sensor(sensor::Sensor *sensor) { this->total_energy_production_ = sensor; }
#ifdef HAVE_MODULE_TEMP
  void set_inverter_module_temp_sensor(sensor::Sensor *sensor) { this->inverter_module_temp_ = sensor; }
#endif
  void set_inverter_bluetooth_signal_strength(sensor::Sensor *sensor) { this->inverter_bluetooth_signal_strength_ = sensor; }
  void set_today_generation_time(sensor::Sensor *sensor) { this->today_generation_time_ = sensor; }
  void set_total_generation_time(sensor::Sensor *sensor) { this->total_generation_time_ = sensor; }
  void set_wakeup_time(sensor::Sensor *sensor) { this->wakeup_time_ = sensor; }
  void set_serial_number(text_sensor::TextSensor *text_sensor) { this->serial_number_ = text_sensor; }
  void set_software_version(text_sensor::TextSensor *text_sensor) { this->software_version_ = text_sensor; }
  void set_device_type(text_sensor::TextSensor *text_sensor) { this->device_type_ = text_sensor; }
  void set_device_class(text_sensor::TextSensor *text_sensor) { this->device_class_ = text_sensor; }
  void set_voltage_sensor(uint8_t phase, sensor::Sensor *voltage_sensor) {
    this->phases_[phase].voltage_sensor_ = voltage_sensor;
  }
  void set_current_sensor(uint8_t phase, sensor::Sensor *current_sensor) {
    this->phases_[phase].current_sensor_ = current_sensor;
  }
  void set_active_power_sensor(uint8_t phase, sensor::Sensor *active_power_sensor) {
    this->phases_[phase].active_power_sensor_ = active_power_sensor;
  }
  void set_voltage_sensor_pv(uint8_t pv, sensor::Sensor *voltage_sensor) {
    this->pvs_[pv].voltage_sensor_ = voltage_sensor;
  }
  void set_current_sensor_pv(uint8_t pv, sensor::Sensor *current_sensor) {
    this->pvs_[pv].current_sensor_ = current_sensor;
  }
  void set_active_power_sensor_pv(uint8_t pv, sensor::Sensor *active_power_sensor) {
    this->pvs_[pv].active_power_sensor_ = active_power_sensor;
  }

  void loopNotification();

 protected:
  bool waiting_to_update_;
  uint32_t last_send_;

  uint32_t nextTime = 0;
  bool nightTime = false;
  bool firstTime = true;
  bool hasBegun = false;
  bool hasSetup = false;

  bool running_update_ = false;

  struct SmaPhase {
    sensor::Sensor *voltage_sensor_{nullptr};
    sensor::Sensor *current_sensor_{nullptr};
    sensor::Sensor *active_power_sensor_{nullptr};
  } phases_[3];
  struct SmaPV {
    sensor::Sensor *voltage_sensor_{nullptr};
    sensor::Sensor *current_sensor_{nullptr};
    sensor::Sensor *active_power_sensor_{nullptr};
  } pvs_[2];

  sensor::Sensor *inverter_status_sensor_{nullptr};
  sensor::Sensor *grid_relay_sensor_{nullptr};

  text_sensor::TextSensor *status_text_sensor_{nullptr};
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
  SmaBluetoothProtocolVersion protocol_version_;

  std::string sma_inverter_bluetooth_mac_ ;
  std::string sma_inverter_password_ ;
  uint32_t sma_inverter_delay_values_ = 500; //ms

  std::map<int, std::string> codeMap;

  private:
    ESP32_SMA_Inverter *smaInverter;
    SmaInverterState inverterState = SmaInverterState::Off;
 static const getInverterDataType invDataTypes[SIZE_INVETER_DATA_TYPE_QUERY];
  static const std::unordered_set<getInverterDataType> ignoreQueryErrorTypes;
  /*    
    static const getInverterDataType invDataTypes[SIZE_INVETER_DATA_TYPE_QUERY] =
       {EnergyProduction, SpotGridFrequency, SpotDCPower, SpotDCVoltage, SpotACPower, SpotACTotalPower, SpotACVoltage, DeviceStatus, GridRelayStatus, InverterTemp, OperationTime, TypeLabel, SoftwareVersion};
    static const std::unordered_set<getInverterDataType> ignoreQueryErrorTypes = {
        DeviceStatus,
        GridRelayStatus
    };*/
    int indexOfInverterDataType = 0;

    const float EPSILON = 0.0001f; //ingore small values, avoid equals for float

  };


}  // namespace smabluetooth_solar
}  // namespace esphome


#endif