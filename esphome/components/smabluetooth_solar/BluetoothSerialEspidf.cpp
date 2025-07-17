// BluetoothSerialWrapper.cpp

#include "BluetoothSerialEspidf.h"
#ifdef ESP_PLATFORM

// BluetoothSerialWrapper.cpp

#include "BluetoothSerialWrapper.h"
#include "esp_log.h"
#include "esp_bt.h"
#include "esp_bt_main.h"

BluetoothSerialWrapper* BluetoothSerialWrapper::instance = nullptr;

BluetoothSerialWrapper::BluetoothSerialWrapper() {
    rxQueue = xQueueCreate(1024, sizeof(uint8_t)); // Simple RX FIFO buffer
    connected = false;
    connectionHandle = 0;
    instance = this; // for static callbacks
}

BluetoothSerialWrapper::~BluetoothSerialWrapper() {
    vQueueDelete(rxQueue);
}

esp_err_t BluetoothSerialWrapper::begin(const char* deviceName) {
    esp_err_t ret;

    ret = esp_bt_controller_mem_release(ESP_BT_MODE_BLE);
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT));
    ESP_ERROR_CHECK(esp_bluedroid_init());
    ESP_ERROR_CHECK(esp_bluedroid_enable());

    ESP_ERROR_CHECK(esp_bt_gap_register_callback(gapCallback));
    ESP_ERROR_CHECK(esp_spp_register_callback(sppCallback));
    ESP_ERROR_CHECK(esp_spp_init(ESP_SPP_MODE_CB));

    ESP_ERROR_CHECK(esp_bt_dev_set_device_name(deviceName));

    return ESP_OK;
}

esp_err_t BluetoothSerialWrapper::write(const uint8_t* data, size_t len) {
    if (!connected) return ESP_FAIL;
    return esp_spp_write(connectionHandle, len, (uint8_t*)data);
}

int BluetoothSerialWrapper::available() {
    return uxQueueMessagesWaiting(rxQueue);
}

int BluetoothSerialWrapper::read(uint8_t* buf, size_t maxLen) {
    int count = 0;
    while (count < maxLen) {
        if (xQueueReceive(rxQueue, &buf[count], 0) != pdTRUE) break;
        count++;
    }
    return count;
}

esp_err_t BluetoothSerialWrapper::disconnect() {
    if (!connected) return ESP_FAIL;
    return esp_spp_disconnect(connectionHandle);
}

bool BluetoothSerialWrapper::isConnected() {
    return connected;
}

// ----------------------------
// Static callbacks
// ----------------------------
void BluetoothSerialWrapper::sppCallback(esp_spp_cb_event_t event, esp_spp_cb_param_t *param) {
    switch (event) {
        case ESP_SPP_INIT_EVT:
            esp_spp_start_srv(ESP_SPP_SEC_NONE, ESP_SPP_ROLE_SLAVE, 0, "SPP_SERVER");
            break;

        case ESP_SPP_SRV_OPEN_EVT:
            instance->connected = true;
            instance->connectionHandle = param->srv_open.handle;
            break;

        case ESP_SPP_CLOSE_EVT:
            instance->connected = false;
            instance->connectionHandle = 0;
            break;

        case ESP_SPP_DATA_IND_EVT:
            for (int i = 0; i < param->data_ind.len; i++) {
                uint8_t byte = param->data_ind.data[i];
                xQueueSend(instance->rxQueue, &byte, 0);
            }
            break;

        default:
            break;
    }
}

void BluetoothSerialWrapper::gapCallback(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param) {
    // For PIN pairing if needed
}



#endif