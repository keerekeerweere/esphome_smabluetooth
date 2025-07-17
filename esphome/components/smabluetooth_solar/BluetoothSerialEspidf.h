// BluetoothSerialWrapper.h

#pragma once

#ifdef USE_ESP_IDF


#include "esp_spp_api.h"
#include "esp_gap_bt_api.h"
#include "esp_bt_device.h"
#include "esp_bt_main.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

class BluetoothSerialWrapper {
public:
    BluetoothSerialWrapper();
    ~BluetoothSerialWrapper();

    bool begin(const char* name);
    bool connect(const esp_bd_addr_t mac);
    bool connect(const char* mac_str);  // "00:11:22:33:44:55"
    size_t write(const uint8_t* data, size_t len);
    size_t write(uint8_t c);
    int read(uint8_t* buf, size_t maxLen = 1);
    int read(); 
    int available();
    void disconnect();
    bool isConnected();

private:
    static void sppCallback(esp_spp_cb_event_t event, esp_spp_cb_param_t *param);
    static void gapCallback(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param);

    static BluetoothSerialWrapper* instance;

    QueueHandle_t rxQueue;
    uint32_t connectionHandle;
    bool connected;
};


#endif