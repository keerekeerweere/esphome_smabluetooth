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
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"


namespace esphome {
namespace smabluetooth_solar {

static const char *const TAG = "smabluetooth_solar";

//{EnergyProduction, SpotGridFrequency, SpotDCPower, SpotDCVoltage, SpotACPower, 
//SpotACTotalPower, SpotACVoltage, DeviceStatus, GridRelayStatus, 
//InverterTemp, OperationTime, TypeLabel, SoftwareVersion};
const getInverterDataType SmaBluetoothSolar::invDataTypes[SIZE_INVETER_DATA_TYPE_QUERY] = {
  SpotDCPower, SpotDCVoltage, SpotACPower,
  SpotACTotalPower, SpotACVoltage, EnergyProduction, SpotGridFrequency, DeviceStatus, 
  GridRelayStatus, InverterTemp, OperationTime, TypeLabel, SoftwareVersion
};

const getInverterDataType SmaBluetoothSolar::ignoreQueryErrorTypes[5] = {
  DeviceStatus,
  GridRelayStatus,
  InverterTemp,
  SpotDCPower,
  SpotACPower
};


const StatusCode SmaBluetoothSolar::status_codes[36] PROGMEM = {
      {50,"Status"},
      {51,"Closed"},
      {300,"Nat"},
      {301,"Grid failure"},
      {302,"-------"},
      {303,"Off"},
      {304, ISLAND " " MODE},
      {305, ISLAND " " MODE},
      {306,"SMA Island " MODE " 60 Hz"},
      {307,"OK"},
      {308,"On"},
      {309,"Operation"},
      {310,"General operating " MODE},
      {311,"Open"},
      {312,"Phase assignment"},
      {313,"SMA Island mode 50 Hz"},
      {358,  SB " " "4000" TL20},
      {359,  SB " " "5000" TL20},
      {558,  SB " " "3000" TL20},
      {6109, SB " " "1600" TL10},
      {9109, SB " " "1600" TL10},
      {8001,"Solar Inverters"},
//      {16777213U,"Information not available"},
      {71,"Interference device"},
      {73,"Diffuse insolation"},
      {74,"Direct insolation"},
      {76,"Fault correction measure"},
      {77,"Check AC circuit breaker"},
      {78,"Check generator"},
      {79,"Disconnect generator"},
      {80,"Check parameter"},
      {84,"Overcurrent " GRID " hw"},
      {85,"Overcurrent " GRID " sw"},
      {87, GRID " frequency disturbance"},
      {88, GRID " frequency not permitted"},
      {89, GRID " disconnection point"},
};

void SmaBluetoothSolar::setup() {
  ESP_LOGW(TAG, "Starting setup...");
  //begin
   smaInverter = ESP32_SMA_Inverter::getInstance();
  ESP_LOGW(TAG, "Inverter/pw to setup... %s ", sma_inverter_bluetooth_mac_.c_str());
    smaInverter->setup(sma_inverter_bluetooth_mac_, sma_inverter_password_);
    nextTime = millis();

  hasSetup = true;
}

 void SmaBluetoothSolar::loopNotification() {
  App.feed_wdt();
 }

void SmaBluetoothSolar::loop() {
  uint32_t thisTime = millis();
  loopNotification();

  if (!hasSetup) {
    return ;
  }
  int adjustedScanRate;
  if (nightTime)  // Scan every 15min
    adjustedScanRate = 900000;
  else
    adjustedScanRate = (60 * 1000); //todo adjust, take 60s for now

  if (nextTime > thisTime) {
    //sleeping
    App.feed_wdt();
    delay(10);
    return ;
  }

  uint32_t waitMillis = 1000; //default wait this time before jumping to next step

  ESP_LOGV(TAG, "loop inverter launching state : %d ", inverterState);

  switch (inverterState) {
    case SmaInverterState::Off: { //do off thing
      
        //next state
        //simply begin here
        inverterState = SmaInverterState::Begin;
    }
    break;

    case SmaInverterState::Begin: {//do Begin
        //lets dobegin
        // "true" creates this device as a BT Master.
        if (smaInverter->begin("ESP32toSMA", true)) {
        //next state
          inverterState = SmaInverterState::Connect;
        } else {
          //try again in 10
          waitMillis *= 10;
        } 
    }
    break;

    case SmaInverterState::Connect:{ // do Connect
        //lets do connect
        if (!smaInverter->isBtConnected()) {
          //reset PcktID
          ESP_LOGD(TAG, "do connect : initPcktID ");
          smaInverter->initPcktID();

          ESP_LOGI(TAG, "Connecting SMA inverter ..");
          if (smaInverter->connect()) {
			      ESP_LOGD(TAG, "connected to inverter");
            inverterState = SmaInverterState::Initialize;
          } else {
            ESP_LOGE(TAG, "Connecting SMA inverter failed");
            smaInverter->disconnect();
            inverterState = SmaInverterState::Begin;
            waitMillis *= 50;  // was 10 bevore
          }
        }
    }
    break;
    
    case SmaInverterState::Initialize:{ //do init
      E_RC rc = smaInverter->initialiseSMAConnection();
      ESP_LOGI(TAG, "SMA initialise SMA connection RC %d ", rc);
      inverterState = SmaInverterState::Logon; // optional, but keep for now
    } 
    break;

    case SmaInverterState::Logon:{ //do LogonSmaInverter
      E_RC rc = smaInverter->logonSMAInverter();
      ESP_LOGI(TAG, "SMA logon RC %d ", rc);
      if (rc == E_OK) {
        inverterState = SmaInverterState::SignalStrength;
      } else {
        //sleep and restart
        ESP_LOGE(TAG, "SMA logonff RC %d ", rc); // we see rc -5
        smaInverter->disconnect();
        inverterState = SmaInverterState::Connect;
        waitMillis = 500;
      }
    } 
    break;

    case SmaInverterState::SignalStrength: {//do SignalStrength
      smaInverter->getBT_SignalStrength();
      inverterState = SmaInverterState::ReadValues;

    } 
    break;

    case SmaInverterState::ReadValues: { //do ReadValues (one by one)
      //cycle through values
      uint32_t tBeforeRead = millis();

      if (indexOfInverterDataType<SIZE_INVETER_DATA_TYPE_QUERY) {
        getInverterDataType dataType = invDataTypes[indexOfInverterDataType++];
        ESP_LOGI(TAG, "Get Data (%d)", dataType);
        E_RC rc = smaInverter->getInverterData(dataType);
        ESP_LOGI(TAG, "Get Data RC %d (%d)", rc, dataType);
        waitMillis = sma_inverter_delay_values_;
        if (rc != E_OK) {
          //we shouldn't need to disconnect here, some values cannot be read on specific inverters, e.g. SB1600TL-10
          //if (dataType == SpotDCPower || dataType == SpotACPower) {
          //findIgnoredTypes(dataType)
          if (findIgnoredTypes(dataType)) {
              ESP_LOGI(TAG, "Get Data RC %d (ignored) for %d", rc, dataType);
            //ignore
          } else {
            ESP_LOGE(TAG, "Get Data RC %d (failed) for %d ", rc, dataType);
            // disconnect and try again
            smaInverter->disconnect(); //moved btConnected to inverter class
            inverterState = SmaInverterState::Connect;
            waitMillis = 2 * 1000; //wait at least 2 seconds before next round
          }
        }
      } else {
        //done for reading values, 
        //handle missing values
        ESP_LOGI(TAG, "Done reading values");
        handleMissingValues();
        //move on
        indexOfInverterDataType = 0;
        inverterState = SmaInverterState::DoneReadingValues;
      }

      uint32_t tAfterRead = millis();
      ESP_LOGD(TAG, "reading took: %u ms", (tAfterRead - tBeforeRead));
    }
    break;

    case SmaInverterState::DoneReadingValues: {
      // continuously read values
      inverterState = SmaInverterState::SignalStrength;
      waitMillis = 500;
    }
    break;
  }

  //don't wait too long
  waitMillis =  (waitMillis > 1800 * 1000) ? 1000*1000 : waitMillis;

  nextTime = thisTime + waitMillis; //wait a bit after beginning

  App.feed_wdt();
  delay(10);
  //delay(100);
}

/**
 * generic publish methods
*/
void SmaBluetoothSolar::updateSensor( text_sensor::TextSensor *sensor,  String sensorName,  std::string publishValue) {
  ESP_LOGV(TAG, "update sensor %s ", sensorName.c_str());
  loopNotification();
  if (!publishValue.empty()) {
    if (sensor!=nullptr) sensor->publish_state(publishValue);
      else ESP_LOGV(TAG, "No %s sensor ", sensorName.c_str());
  } else ESP_LOGV(TAG, "No %s value ", sensorName.c_str());
}

void SmaBluetoothSolar::updateSensor( sensor::Sensor *sensor,  String sensorName,  int32_t publishValue) {
  ESP_LOGV(TAG, "update sensor %s ", sensorName.c_str());
  loopNotification();
  if (publishValue >= 0) {
    if (sensor!=nullptr) sensor->publish_state(publishValue);
      else ESP_LOGV(TAG, "No %s sensor ", sensorName.c_str());
  } else ESP_LOGV(TAG, "No %s value ", sensorName.c_str());
}

void SmaBluetoothSolar::updateSensor( sensor::Sensor *sensor,  String sensorName,  uint64_t publishValue) {
  ESP_LOGV(TAG, "update sensor %s ", sensorName.c_str());
  loopNotification();
  if (publishValue >= 0) {
    if (sensor!=nullptr) sensor->publish_state(publishValue);
      else ESP_LOGV(TAG, "No %s sensor ", sensorName.c_str());
  } else ESP_LOGV(TAG, "No %s value ", sensorName.c_str());
}

void SmaBluetoothSolar::updateSensor( sensor::Sensor *sensor,  String sensorName,  float publishValue) {
  ESP_LOGV(TAG, "update sensor %s ", sensorName.c_str());
  loopNotification();
  if (publishValue >= 0.0) {
    if (sensor!=nullptr) sensor->publish_state(publishValue);
      else ESP_LOGV(TAG, "No %s sensor ", sensorName.c_str());
  } else ESP_LOGV(TAG, "No %s value ", sensorName.c_str());
}

void SmaBluetoothSolar::updateSensor( binary_sensor::BinarySensor *sensor,  String sensorName,  bool publishValue) {
  ESP_LOGV(TAG, "update sensor %s ", sensorName.c_str());
  loopNotification();
  if (sensor!=nullptr) sensor->publish_state(publishValue);
    else ESP_LOGV(TAG, "No %s sensor ", sensorName.c_str());
}


/*
* some (older) inverters with retrofitted bluetoothmodules don't handle all values, especially pdc and pac 
*/
void SmaBluetoothSolar::handleMissingValues(){
  //DC (mptt)
  //once it's triggered, keep on updating the value (stalls the values after first calculation)
  //there should be a better way for this
  if (smaInverter->dispData.Pdc1 == 0.0 || smaInverter->dispData.needsMissingValues) {
    if (smaInverter->dispData.Udc1 != 0.0 && smaInverter->dispData.Idc1 != 0.0){
      smaInverter->dispData.Pdc1 = smaInverter->dispData.Udc1 * smaInverter->dispData.Idc1 / 1000.0;
      ESP_LOGD(TAG, "updated Pdc1 %15.2f ", smaInverter->dispData.Pdc1);
      smaInverter->dispData.needsMissingValues = true;
    }
  }
  if (smaInverter->dispData.Pdc2 == 0.0 || smaInverter->dispData.needsMissingValues) {
    if (smaInverter->dispData.Udc2 != 0.0 && smaInverter->dispData.Idc2 != 0.0){
      smaInverter->dispData.Pdc2 = smaInverter->dispData.Udc2 * smaInverter->dispData.Idc2 / 1000.0;
      ESP_LOGD(TAG, "updated Pdc2 %15.2f ", smaInverter->dispData.Pdc2);
      smaInverter->dispData.needsMissingValues = true;
    }
  }
  //AC (3 fases)
  if (smaInverter->dispData.Pac1 == 0.0 || smaInverter->dispData.needsMissingValues) {
    if (smaInverter->dispData.Uac1 != 0.0 && smaInverter->dispData.Iac1 != 0.0){
      smaInverter->dispData.Pac1 = smaInverter->dispData.Uac1 * smaInverter->dispData.Iac1 / 1000.0;
      ESP_LOGD(TAG, "updated Pac1 %15.2f ", smaInverter->dispData.Pac1);
      smaInverter->dispData.needsMissingValues = true;
    }
  }
  if (smaInverter->dispData.Pac2 ==0.0 || smaInverter->dispData.needsMissingValues) {
    if (smaInverter->dispData.Uac2 != 0.0 && smaInverter->dispData.Iac2 != 0.0){
      smaInverter->dispData.Pac2 = smaInverter->dispData.Uac2 * smaInverter->dispData.Iac2 / 1000.0;
      ESP_LOGD(TAG, "updated Pac2 %15.2f ", smaInverter->dispData.Pac2);
      smaInverter->dispData.needsMissingValues = true;
    }
  }
  if (smaInverter->dispData.Pac3 ==0.0 || smaInverter->dispData.needsMissingValues) {
    if (smaInverter->dispData.Uac3 != 0.0 && smaInverter->dispData.Iac3 != 0.0){
      smaInverter->dispData.Pac3 = smaInverter->dispData.Uac3 * smaInverter->dispData.Iac3 / 1000.0;
      ESP_LOGD(TAG, "updated Pac3 %15.2f ", smaInverter->dispData.Pac3);
      smaInverter->dispData.needsMissingValues = true;
    }
  }
}


void SmaBluetoothSolar::update() {
  // If our last send has had no reply yet, and it wasn't that long ago, do nothing.
  uint32_t now = millis();

  this->running_update_ = true;

  ESP_LOGV(TAG, "update sensors ");

  updateSensor(today_production_, String("EToday"), smaInverter->dispData.EToday);
  updateSensor(total_energy_production_, String("ETotal"), smaInverter->dispData.ETotal);
  updateSensor(grid_frequency_sensor_, String("Freq"), smaInverter->dispData.GridFreq);
  updateSensor(pvs_[0].voltage_sensor_, String("UdcA"), smaInverter->dispData.Udc1);
  updateSensor(pvs_[0].current_sensor_, String("IdcA"), smaInverter->dispData.Idc1);
  updateSensor(pvs_[0].active_power_sensor_, String("PDC"), smaInverter->dispData.Pdc1);

  updateSensor(pvs_[1].voltage_sensor_, String("UdcA"), smaInverter->dispData.Udc2);
  updateSensor(pvs_[1].current_sensor_, String("IdcA"), smaInverter->dispData.Idc2);
  updateSensor(pvs_[1].active_power_sensor_, String("PDC"), smaInverter->dispData.Pdc2);

  updateSensor(phases_[0].voltage_sensor_, String("UacA"), smaInverter->dispData.Uac1);
  updateSensor(phases_[0].current_sensor_, String("IacA"), smaInverter->dispData.Iac1);
  updateSensor(phases_[0].active_power_sensor_, String("IacA"), smaInverter->dispData.Pac1);
#if PHASES > 1
  updateSensor(phases_[1].voltage_sensor_, String("UacA"), smaInverter->dispData.Uac2);
  updateSensor(phases_[1].current_sensor_, String("IacA"), smaInverter->dispData.Iac2);
  updateSensor(phases_[1].active_power_sensor_, String("IacA"), smaInverter->dispData.Pac2);
#endif

#if PHASES > 2
  updateSensor(phases_[2].voltage_sensor_, String("UacA"), smaInverter->dispData.Uac3);
  updateSensor(phases_[2].current_sensor_, String("IacA"), smaInverter->dispData.Iac3);
  updateSensor(phases_[2].active_power_sensor_, String("IacA"), smaInverter->dispData.Pac3);
#endif

  updateSensor(status_text_sensor_, String("InverterStatus"), lookup_code(smaInverter->invData.DevStatus));
  updateSensor(grid_relay_binary_sensor_, String("GridRelay"), smaInverter->invData.GridRelay == 51); // 51 is "Closed"

#if HAVE_MODULE_TEMP
  updateSensor(inverter_module_temp_, String("InvTemp"), smaInverter->dispData.InvTemp);
#endif

  updateSensor(inverter_bluetooth_signal_strength_, String("InvSignal"), smaInverter->dispData.BTSigStrength);
  updateSensor(today_generation_time_, String("TToday"), (float)smaInverter->invData.OperationTime / 3600);
  updateSensor(total_generation_time_, String("TTotal"), (float)smaInverter->invData.FeedInTime / 3600);
  updateSensor(wakeup_time_, String("TWakeup"), (uint64_t)smaInverter->invData.WakeupTime);
  updateSensor(serial_number_, String("SerialNumber"), smaInverter->invData.DeviceName);
  updateSensor(software_version_, String("SoftwareVersion"), smaInverter->invData.SWVersion);
  updateSensor(device_type_, String("DeviceType"), lookup_code(smaInverter->invData.DeviceType));
  updateSensor(device_class_, String("DeviceClass"), lookup_code(smaInverter->invData.DeviceClass));

	this->running_update_ = false;

  this->last_send_ = millis();
}

void SmaBluetoothSolar::dump_config() {
  ESP_LOGCONFIG(TAG, "SMABluetooth Solar:");
  ESP_LOGCONFIG(TAG, "  Address: %s", sma_inverter_bluetooth_mac_.c_str());
}

const char* SmaBluetoothSolar::lookup_code(uint16_t code) {
  for (auto &entry : SmaBluetoothSolar::status_codes) {
    if (entry.code == code) return entry.message;
  }
  return "Unknown";
}

boolean SmaBluetoothSolar::findIgnoredTypes(getInverterDataType dataType) {
  for (int i=0;i<SIZE_INVETER_DATA_TYPE_IGNORE;i++) {
    if (dataType == ignoreQueryErrorTypes[i]) return true;
  }
  return false;
}


}  // namespace smabluetooth_solar
}  // namespace esphome
