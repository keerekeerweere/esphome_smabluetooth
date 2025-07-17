// BluetoothSerialWrapper.cpp

#include "BluetoothSerialEspidf.h"
#ifdef USE_ESP_IDF

#include "BluetoothSerialWrapper.h"
#include "esp_log.h"
#include <cstring>
#include <cstdio>

#define BT_TAG "BT_SERIAL"

BluetoothSerialWrapper* BluetoothSerialWrapper::instance = nullptr;

BluetoothSerialWrapper::BluetoothSerialWrapper() {
    rxQueue = xQueueCreate(1024, sizeof(uint8_t));
    connect_semaphore = xSemaphoreCreateBinary();
    connectionHandle = 0;
    connected = false;
    instance = this;

    // Default tick is a no-op
    connect_tick = []() {};    
}

BluetoothSerialWrapper::~BluetoothSerialWrapper() {
    vQueueDelete(rxQueue);
    vSemaphoreDelete(connect_semaphore);
}

bool BluetoothSerialWrapper::begin(const char* name) {
    esp_err_t ret;

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_BLE));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT));
    ESP_ERROR_CHECK(esp_bluedroid_init());
    ESP_ERROR_CHECK(esp_bluedroid_enable());

    ESP_ERROR_CHECK(esp_bt_dev_set_device_name(name));
    ESP_ERROR_CHECK(esp_bt_gap_register_callback(gapCallback));
    ESP_ERROR_CHECK(esp_spp_register_callback(sppCallback));
    ESP_ERROR_CHECK(esp_spp_init(ESP_SPP_MODE_CB));

    // Declare this before calling connect
    esp_bt_pin_code_t pin_code = {'0', '0', '0', '0'};
    esp_bt_pin_type_t pin_type = ESP_BT_PIN_TYPE_FIXED;

    // Apply pairing config
    ESP_ERROR_CHECK(esp_bt_gap_set_pin(pin_type, 4, pin_code));

    return true;
}

bool BluetoothSerialWrapper::connect(const esp_bd_addr_t mac) {
    const TickType_t step = pdMS_TO_TICKS(200);
    const TickType_t timeout = pdMS_TO_TICKS(10000);
    TickType_t waited = 0;

    connected = false; // reset state

    esp_err_t err = esp_spp_connect(ESP_SPP_SEC_NONE, ESP_SPP_ROLE_MASTER, 1, mac);
    if (err != ESP_OK) {
        ESP_LOGE(BT_TAG, "Failed to initiate connect: %s", esp_err_to_name(err));
        return false;
    }

    ESP_LOGI(BT_TAG, "Connecting...");

    while (waited < timeout) {
        if (xSemaphoreTake(connect_semaphore, step) == pdTRUE) {
            return connected; // true if OPEN, false if CLOSE
        }

        waited += step;
        connect_tick();
    }

    ESP_LOGE(BT_TAG, "Connection timed out");
    return false;
}

bool BluetoothSerialWrapper::connect(const char* mac_str) {
    esp_bd_addr_t mac = {0};
    if (sscanf(mac_str, "%hhx:%hhx:%hhx:%hhx:%hhx:%hhx",
               &mac[0], &mac[1], &mac[2], &mac[3], &mac[4], &mac[5]) != 6) {
        ESP_LOGE(BT_TAG, "Invalid MAC format");
        return false;
    }
    return connect(mac);
}

size_t BluetoothSerialWrapper::write(const uint8_t* data, size_t len) {
    if (!connected) return 0;
    if (esp_spp_write(connectionHandle, len, (uint8_t*)data) == ESP_OK) {
        return len;
    }
    return 0;
}

size_t BluetoothSerialWrapper::write(uint8_t c) override {
    return write(&c, 1);  // Reuse buffer version
}

int BluetoothSerialWrapper::read(uint8_t* buf, size_t maxLen) {
    int count = 0;
    while (count < maxLen) {
        if (xQueueReceive(rxQueue, &buf[count], 0) != pdTRUE) break;
        count++;
    }
    return count;
}

int BluetoothSerialWrapper::read() {
    uint8_t byte;
    if (xQueueReceive(rxQueue, &byte, 0) == pdTRUE) {
        return byte;
    } else {
        return -1;  // Arduino-style: no data available
    }
}

int BluetoothSerialWrapper::available() {
    return uxQueueMessagesWaiting(rxQueue);
}

void BluetoothSerialWrapper::disconnect() {
    if (connected) {
        esp_spp_disconnect(connectionHandle);
    }
}

bool BluetoothSerialWrapper::isConnected() {
    return connected;
}

void BluetoothSerialWrapper::sppCallback(esp_spp_cb_event_t event, esp_spp_cb_param_t* param) {
    switch (event) {
        case ESP_SPP_INIT_EVT:
            ESP_LOGI(BT_TAG, "SPP initialized");
            break;

        case ESP_SPP_OPEN_EVT:
            ESP_LOGI(BT_TAG, "Connected to server");
            instance->connected = true;
            instance->connectionHandle = param->open.handle;
            xSemaphoreGive(instance->connect_semaphore);
            break;

        case ESP_SPP_CLOSE_EVT:
            ESP_LOGW(BT_TAG, "SPP connection closed");
            if (!instance->connected) {
                // this was a failed connection attempt
                xSemaphoreGive(instance->connect_semaphore);
            }
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

void BluetoothSerialWrapper::gapCallback(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t* param) {
    switch (event) {
        case ESP_BT_GAP_AUTH_CMPL_EVT:
            if (param->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS) {
                ESP_LOGI(BT_TAG, "Authentication succeeded with %s", param->auth_cmpl.device_name);
            } else {
                ESP_LOGE(BT_TAG, "Authentication failed, status: %d", param->auth_cmpl.stat);
            }
            break;

        case ESP_BT_GAP_PIN_REQ_EVT:
            ESP_LOGI(BT_TAG, "PIN requested by inverter");
            esp_bt_pin_code_t pin = {'0', '0', '0', '0'};
            esp_bt_gap_pin_reply(param->pin_req.bda, true, 4, pin);
            break;

#if CONFIG_BT_SSP_ENABLED
        case ESP_BT_GAP_CFM_REQ_EVT:
            ESP_LOGI(BT_TAG, "SSP confirmation request: %d", param->cfm_req.num_val);
            esp_bt_gap_ssp_confirm_reply(param->cfm_req.bda, true);
            break;
#endif

        default:
            break;
    }
}


#endif