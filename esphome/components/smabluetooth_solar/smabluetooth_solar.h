#pragma once
#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "SMA_Inverter.h"

#include <vector>

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


class SmaBluetoothSolar : public PollingComponent {
 public:
  void loop() override;
  void setup() override;

  void update() override;  
  void updateSensor( sensor::Sensor *sensor,  String sensorName,  float publishValue);
  void on_inverter_data(const std::vector<uint8_t> &data) ;
  void dump_config() override;

  void set_protocol_version(SmaBluetoothProtocolVersion protocol_version) { this->protocol_version_ = protocol_version; }

  void set_sma_inverter_bluetooth_mac(std::string sma_inverter_bluetooth_mac) { this->sma_inverter_bluetooth_mac_ = sma_inverter_bluetooth_mac; }
  void set_sma_inverter_password(std::string sma_inverter_password) {this->sma_inverter_password_ = sma_inverter_password; }

  void set_inverter_status_sensor(sensor::Sensor *sensor) { this->inverter_status_ = sensor; }

  void set_grid_frequency_sensor(sensor::Sensor *sensor) { this->grid_frequency_sensor_ = sensor; }
  void set_grid_active_power_sensor(sensor::Sensor *sensor) { this->grid_active_power_sensor_ = sensor; }
  void set_pv_active_power_sensor(sensor::Sensor *sensor) { this->pv_active_power_sensor_ = sensor; }

  void set_today_production_sensor(sensor::Sensor *sensor) { this->today_production_ = sensor; }
  void set_total_energy_production_sensor(sensor::Sensor *sensor) { this->total_energy_production_ = sensor; }
  void set_inverter_module_temp_sensor(sensor::Sensor *sensor) { this->inverter_module_temp_ = sensor; }

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

  sensor::Sensor *inverter_status_{nullptr};

  sensor::Sensor *grid_frequency_sensor_{nullptr};
  sensor::Sensor *grid_active_power_sensor_{nullptr};

  sensor::Sensor *pv_active_power_sensor_{nullptr};

  sensor::Sensor *today_production_{nullptr};
  sensor::Sensor *total_energy_production_{nullptr};
  sensor::Sensor *inverter_module_temp_{nullptr};
  SmaBluetoothProtocolVersion protocol_version_;

  std::string sma_inverter_bluetooth_mac_ ;
  std::string sma_inverter_password_ ;


  private:
    ESP32_SMA_Inverter *smaInverter;
    SmaInverterState inverterState = SmaInverterState::Off;    
    getInverterDataType invDataTypes[6] = {EnergyProduction, SpotGridFrequency, SpotDCPower, SpotDCVoltage, SpotACPower, SpotACVoltage};
    int indexOfInverterDataType = 0;
};

}  // namespace smabluetooth_solar
}  // namespace esphome
