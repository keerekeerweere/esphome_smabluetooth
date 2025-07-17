// BluetoothSerialWrapper.h

#pragma once

#ifdef ESP_PLATFORM

#include "esp_spp_api.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

class BluetoothSerialWrapper {
public:
    BluetoothSerialWrapper();
    ~BluetoothSerialWrapper();

    esp_err_t begin(const char* deviceName);
    esp_err_t write(const uint8_t* data, size_t len);
    int available();
    int read(uint8_t* buf, size_t maxLen);
    esp_err_t disconnect();

    bool isConnected();

private:
    static void sppCallback(esp_spp_cb_event_t event, esp_spp_cb_param_t *param);
    static void gapCallback(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param);

    static BluetoothSerialWrapper* instance;

    QueueHandle_t rxQueue;     // incoming bytes
    uint32_t connectionHandle; // active connection handle
    bool connected;
};


#endif