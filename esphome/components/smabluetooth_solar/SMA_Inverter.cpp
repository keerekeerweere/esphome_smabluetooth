/* MIT License

Copyright (c) 2022 Lupo135
Copyright (c) 2023 darrylb123
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
*/

#include "SMA_Inverter.h"
#include "esphome/core/log.h"
#include "esp_idf_version.h"

namespace esphome {
namespace smabluetooth_solar {

static const char *const TAG = "smabluetooth_solar";

// ============================================================
//  setup / begin
// ============================================================

void ESP32_SMA_Inverter::setup(std::string mac, std::string pw, uint32_t delay_values_ms) {
    ESP_LOGW(TAG, "setup inverter mac=%s", mac.c_str());
    sscanf(mac.c_str(), "%2hhx:%2hhx:%2hhx:%2hhx:%2hhx:%2hhx",
           &smaBTAddress[0], &smaBTAddress[1], &smaBTAddress[2],
           &smaBTAddress[3], &smaBTAddress[4], &smaBTAddress[5]);

    for (int i = 0; i < (int)sizeof(smaInvPass); i++) smaInvPass[i] = '\0';
    strlcpy(smaInvPass, pw.c_str(), sizeof(smaInvPass));

    delay_values_ms_ = delay_values_ms;

    invData.SUSyID = 0x7d;
    invData.Serial = 0;
    for (uint8_t i = 0; i < 6; i++) invData.btAddress[i] = smaBTAddress[5 - i];

    ESP_LOGD(TAG, "invData.btAddress: %02X:%02X:%02X:%02X:%02X:%02X",
             invData.btAddress[5], invData.btAddress[4], invData.btAddress[3],
             invData.btAddress[2], invData.btAddress[1], invData.btAddress[0]);
}

bool ESP32_SMA_Inverter::begin(const char *localName) {
    ESP_LOGD(TAG, "begin BT stack");

    // Create FreeRTOS primitives
    rx_stream_buf_ = xStreamBufferCreate(4096, 1);
    bt_event_group_ = xEventGroupCreate();
    if (!rx_stream_buf_ || !bt_event_group_) {
        ESP_LOGE(TAG, "Failed to create FreeRTOS primitives");
        return false;
    }

    // Release BLE memory (we only need classic BT)
    esp_err_t ret = esp_bt_controller_mem_release(ESP_BT_MODE_BLE);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGW(TAG, "mem_release BLE: %s", esp_err_to_name(ret));
    }

    // BT controller
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret != ESP_OK) { ESP_LOGE(TAG, "controller_init: %s", esp_err_to_name(ret)); return false; }

    ret = esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT);
    if (ret != ESP_OK) { ESP_LOGE(TAG, "controller_enable: %s", esp_err_to_name(ret)); return false; }

    // Bluedroid stack
    ret = esp_bluedroid_init();
    if (ret != ESP_OK) { ESP_LOGE(TAG, "bluedroid_init: %s", esp_err_to_name(ret)); return false; }

    ret = esp_bluedroid_enable();
    if (ret != ESP_OK) { ESP_LOGE(TAG, "bluedroid_enable: %s", esp_err_to_name(ret)); return false; }

    // Register callbacks
    ret = esp_bt_gap_register_callback(gap_callback);
    if (ret != ESP_OK) { ESP_LOGE(TAG, "gap_register_callback: %s", esp_err_to_name(ret)); return false; }

    ret = esp_spp_register_callback(spp_callback);
    if (ret != ESP_OK) { ESP_LOGE(TAG, "spp_register_callback: %s", esp_err_to_name(ret)); return false; }

    // SPP init (callback mode — async RX via ESP_SPP_DATA_IND_EVT)
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
    esp_spp_cfg_t spp_cfg = {
        .mode             = ESP_SPP_MODE_CB,
        .enable_l2cap_ertm = true,
        .tx_buffer_size   = 0,
    };
    ret = esp_spp_enhanced_init(&spp_cfg);
#else
    ret = esp_spp_init(ESP_SPP_MODE_CB);
#endif
    if (ret != ESP_OK) { ESP_LOGE(TAG, "spp_init: %s", esp_err_to_name(ret)); return false; }

    // Set device name and PIN
    esp_bt_gap_set_device_name(localName);
    esp_bt_gap_set_pin(ESP_BT_PIN_TYPE_FIXED, 4, (uint8_t *)btPin);

    // Make device non-discoverable / non-connectable (we initiate; don't want to be found)
    esp_bt_gap_set_scan_mode(ESP_BT_NON_CONNECTABLE, ESP_BT_NON_DISCOVERABLE);

    ESP_LOGD(TAG, "BT stack init complete, waiting for SPP_INIT event");
    return true;
}

// ============================================================
//  FreeRTOS task management
// ============================================================

void ESP32_SMA_Inverter::startBtTask() {
    if (bt_task_handle_ != nullptr) {
        ESP_LOGW(TAG, "startBtTask: task already running");
        return;
    }
    stop_task_  = false;
    task_error_ = false;
    ESP_LOGI(TAG, "Starting BT protocol task");
    TaskHandle_t h = nullptr;
    xTaskCreatePinnedToCore(
        btTask,       // task function
        "bt_sma_proto", // name
        12288,        // stack bytes
        this,         // parameter
        2,            // priority (above idle, below WiFi)
        &h,           // handle out
        0             // core 0 (BT stack also runs on core 0)
    );
    bt_task_handle_ = h;
}

void ESP32_SMA_Inverter::stopBtTask() {
    stop_task_ = true;
    // Give the task up to 3 s to notice the flag and self-delete
    for (int i = 0; i < 30 && bt_task_handle_ != nullptr; i++) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    TaskHandle_t h = bt_task_handle_;
    if (h != nullptr) {
        bt_task_handle_ = nullptr;
        vTaskDelete(h);
    }
}

// ============================================================
//  Static SPP callback — runs in BT stack task context
// ============================================================

void ESP32_SMA_Inverter::spp_callback(esp_spp_cb_event_t event, esp_spp_cb_param_t *param) {
    ESP32_SMA_Inverter *self = getInstance();

    switch (event) {
    case ESP_SPP_INIT_EVT:
        ESP_LOGI(TAG, "SPP_INIT status=%d", param->init.status);
        if (param->init.status == ESP_SPP_SUCCESS) {
            xEventGroupSetBits(self->bt_event_group_, BT_EVT_SPP_INIT);
        }
        break;

    case ESP_SPP_DISCOVERY_COMP_EVT:
        ESP_LOGI(TAG, "SPP_DISCOVERY_COMP status=%d scn_num=%d",
                 param->disc_comp.status, param->disc_comp.scn_num);
        if (param->disc_comp.status == ESP_SPP_SUCCESS && param->disc_comp.scn_num > 0) {
            self->discovered_scn_ = param->disc_comp.scn[0];
            ESP_LOGI(TAG, "Discovered SPP SCN=%d", self->discovered_scn_);
        } else {
            // Fall back to default SCN 1
            self->discovered_scn_ = 1;
            ESP_LOGW(TAG, "Discovery failed or empty, using SCN=1");
        }
        xEventGroupSetBits(self->bt_event_group_, BT_EVT_DISC_DONE);
        break;

    case ESP_SPP_OPEN_EVT:
        ESP_LOGI(TAG, "SPP_OPEN status=%d handle=%lu", param->open.status, param->open.handle);
        if (param->open.status == ESP_SPP_SUCCESS) {
            self->spp_handle_   = param->open.handle;
            self->btConnected_  = true;
            xEventGroupSetBits(self->bt_event_group_, BT_EVT_CONNECTED);
        } else {
            xEventGroupSetBits(self->bt_event_group_, BT_EVT_DISCONNECTED);
        }
        break;

    case ESP_SPP_CLOSE_EVT:
        ESP_LOGI(TAG, "SPP_CLOSE handle=%lu", param->close.handle);
        self->spp_handle_  = 0;
        self->btConnected_ = false;
        xEventGroupSetBits(self->bt_event_group_, BT_EVT_DISCONNECTED);
        break;

    case ESP_SPP_DATA_IND_EVT: {
        // SPP callback runs in Bluedroid stack task context (not an ISR),
        // so use the regular (non-ISR) stream buffer send with zero timeout
        // to avoid blocking the BT stack.
        size_t written = xStreamBufferSend(
            self->rx_stream_buf_,
            param->data_ind.data,
            param->data_ind.len,
            0);  // non-blocking
        if (written != param->data_ind.len) {
            ESP_LOGW(TAG, "RX stream buffer full! dropped %u bytes",
                     (unsigned)(param->data_ind.len - written));
        }
        break;
    }

    case ESP_SPP_WRITE_EVT:
        // Could track congestion here if needed
        break;

    case ESP_SPP_CONG_EVT:
        ESP_LOGD(TAG, "SPP_CONG cong=%d", param->cong.cong);
        break;

    default:
        break;
    }
}

// ============================================================
//  Static GAP callback — handles PIN / auth
// ============================================================

void ESP32_SMA_Inverter::gap_callback(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param) {
    ESP32_SMA_Inverter *self = getInstance();
    switch (event) {
    case ESP_BT_GAP_AUTH_CMPL_EVT:
        if (param->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS) {
            ESP_LOGI(TAG, "GAP auth OK, peer: %s", param->auth_cmpl.device_name);
        } else {
            ESP_LOGE(TAG, "GAP auth FAILED stat=%d", param->auth_cmpl.stat);
        }
        break;

    case ESP_BT_GAP_PIN_REQ_EVT:
        ESP_LOGI(TAG, "GAP PIN request min_16_digit=%d", param->pin_req.min_16_digit);
        // Reply with our fixed PIN "0000"
        esp_bt_gap_pin_reply(param->pin_req.bda, true, 4,
                             (uint8_t *)self->btPin);
        break;

    default:
        break;
    }
}

// ============================================================
//  FreeRTOS BT protocol task
//  Sequence: wait_init → discover → connect → init → logon → read loop
// ============================================================

void ESP32_SMA_Inverter::btTask(void *pvParameters) {
    ESP32_SMA_Inverter *self = static_cast<ESP32_SMA_Inverter *>(pvParameters);
    static const char *TTAG = "bt_task";

    ESP_LOGI(TTAG, "BT task started");

    // --- Phase 1: wait for SPP stack to be ready ---
    EventBits_t bits = xEventGroupWaitBits(
        self->bt_event_group_, BT_EVT_SPP_INIT, pdFALSE, pdFALSE,
        pdMS_TO_TICKS(15000));
    if (!(bits & BT_EVT_SPP_INIT)) {
        ESP_LOGE(TTAG, "Timeout waiting for SPP init");
        self->task_error_    = true;
        self->bt_task_handle_ = nullptr;
        vTaskDelete(nullptr);
        return;
    }

    while (!self->stop_task_) {
        // --- Phase 2: service discovery ---
        xEventGroupClearBits(self->bt_event_group_, BT_EVT_DISC_DONE | BT_EVT_CONNECTED | BT_EVT_DISCONNECTED);
        self->flushRxBuffer();

        ESP_LOGI(TTAG, "Starting SPP discovery");
        esp_err_t err = esp_spp_start_discovery(self->smaBTAddress);
        if (err != ESP_OK) {
            ESP_LOGE(TTAG, "spp_start_discovery: %s", esp_err_to_name(err));
            vTaskDelay(pdMS_TO_TICKS(5000));
            continue;
        }

        bits = xEventGroupWaitBits(
            self->bt_event_group_, BT_EVT_DISC_DONE, pdFALSE, pdFALSE,
            pdMS_TO_TICKS(10000));
        if (!(bits & BT_EVT_DISC_DONE)) {
            ESP_LOGE(TTAG, "SPP discovery timeout");
            vTaskDelay(pdMS_TO_TICKS(5000));
            continue;
        }

        // --- Phase 3: connect ---
        ESP_LOGI(TTAG, "Connecting, SCN=%d", self->discovered_scn_);
        xEventGroupClearBits(self->bt_event_group_, BT_EVT_CONNECTED | BT_EVT_DISCONNECTED);
        err = esp_spp_connect(ESP_SPP_SEC_AUTHENTICATE, ESP_SPP_ROLE_MASTER,
                              self->discovered_scn_, self->smaBTAddress);
        if (err != ESP_OK) {
            ESP_LOGE(TTAG, "spp_connect: %s", esp_err_to_name(err));
            vTaskDelay(pdMS_TO_TICKS(5000));
            continue;
        }

        bits = xEventGroupWaitBits(
            self->bt_event_group_, BT_EVT_CONNECTED | BT_EVT_DISCONNECTED, pdFALSE, pdFALSE,
            pdMS_TO_TICKS(15000));
        if (!(bits & BT_EVT_CONNECTED)) {
            ESP_LOGE(TTAG, "SPP connect timeout or failed");
            vTaskDelay(pdMS_TO_TICKS(5000));
            continue;
        }

        ESP_LOGI(TTAG, "Connected to SMA inverter");
        self->initPcktID();
        self->flushRxBuffer();

        // --- Phase 4: SMA protocol init ---
        E_RC rc = self->initialiseSMAConnection();
        if (rc != E_OK) {
            ESP_LOGE(TTAG, "initialiseSMAConnection RC=%d", rc);
            esp_spp_disconnect((uint32_t)self->spp_handle_);
            vTaskDelay(pdMS_TO_TICKS(3000));
            continue;
        }

        // --- Phase 5: logon ---
        rc = self->logonSMAInverter();
        if (rc != E_OK) {
            ESP_LOGE(TTAG, "logon RC=%d", rc);
            esp_spp_disconnect((uint32_t)self->spp_handle_);
            vTaskDelay(pdMS_TO_TICKS(3000));
            continue;
        }

        // --- Phase 5b: sync inverter clock from ESP32 NTP time ---
        self->setInverterTime();

        ESP_LOGI(TTAG, "Logged on to inverter, starting read loop");

        // --- Phase 6: continuous read loop ---
        static const getInverterDataType dataTypes[] = {
            SpotDCPower, SpotDCVoltage, SpotACPower,
            SpotACTotalPower, SpotACVoltage, EnergyProduction, SpotGridFrequency,
            DeviceStatus, GridRelayStatus, InverterTemp, OperationTime,
            TypeLabel, SoftwareVersion
        };
        static const int NUM_DATA_TYPES = sizeof(dataTypes) / sizeof(dataTypes[0]);

        // Types where errors are silently ignored (not all inverters support them)
        static const getInverterDataType ignoreErrors[] = {
            DeviceStatus, GridRelayStatus, InverterTemp, SpotDCPower, SpotACPower
        };
        static const int NUM_IGNORE = sizeof(ignoreErrors) / sizeof(ignoreErrors[0]);

        while (self->btConnected_ && !self->stop_task_) {
            self->getBT_SignalStrength();

            bool cycle_ok = true;
            for (int i = 0; i < NUM_DATA_TYPES && self->btConnected_ && !self->stop_task_; i++) {
                getInverterDataType dt = dataTypes[i];
                rc = self->getInverterData(dt);
                if (rc != E_OK) {
                    bool ignored = false;
                    for (int j = 0; j < NUM_IGNORE; j++) {
                        if (ignoreErrors[j] == dt) { ignored = true; break; }
                    }
                    if (ignored) {
                        ESP_LOGI(TTAG, "getInverterData %d RC=%d (ignored)", dt, rc);
                    } else {
                        ESP_LOGE(TTAG, "getInverterData %d RC=%d (fatal)", dt, rc);
                        cycle_ok = false;
                        break;
                    }
                }
                vTaskDelay(pdMS_TO_TICKS(self->delay_values_ms_));
            }

            if (!cycle_ok || !self->btConnected_) break;

            // On-demand time sync (triggered from main loop via requestTimeSync())
            if (self->sync_time_requested_) {
                self->setInverterTime();
                // flag cleared inside setInverterTime()
            }

            // Signal ESPHome that fresh data is available
            self->data_ready_ = true;

            vTaskDelay(pdMS_TO_TICKS(500));
        }

        // Disconnected or error — disconnect cleanly and retry
        if (self->spp_handle_ != 0) {
            esp_spp_disconnect((uint32_t)self->spp_handle_);
        }
        self->btConnected_ = false;
        ESP_LOGI(TTAG, "Disconnected, retrying in 5 s");
        vTaskDelay(pdMS_TO_TICKS(5000));
    }

    ESP_LOGI(TTAG, "BT task stopping");
    self->bt_task_handle_ = nullptr;
    vTaskDelete(nullptr);
}

// ============================================================
//  Low-level BT I/O
// ============================================================

uint8_t ESP32_SMA_Inverter::BTgetByte() {
    readTimeout = false;
    uint8_t rec = 0;
    // Block until one byte arrives or timeout
    size_t n = xStreamBufferReceive(rx_stream_buf_, &rec, 1,
                                    pdMS_TO_TICKS(btgetByteTimeout));
    if (n == 0) {
        ESP_LOGD(TAG, "BTgetByte timeout");
        readTimeout = true;
    }
    return rec;
}

void ESP32_SMA_Inverter::BTsendPacket(uint8_t *btbuffer) {
    if (spp_handle_ == 0) {
        ESP_LOGW(TAG, "BTsendPacket: not connected");
        return;
    }
    ESP_LOGV(TAG, "BTsendPacket: %u bytes", pcktBufPos);
    esp_err_t err = esp_spp_write((uint32_t)spp_handle_, pcktBufPos, btbuffer);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "esp_spp_write: %s", esp_err_to_name(err));
    }
}

void ESP32_SMA_Inverter::flushRxBuffer() {
    if (rx_stream_buf_ == nullptr) return;
    uint8_t tmp[64];
    while (xStreamBufferReceive(rx_stream_buf_, tmp, sizeof(tmp),
                                pdMS_TO_TICKS(10)) > 0) {
        // discard
    }
}

// ============================================================
//  Packet receive
// ============================================================

bool ESP32_SMA_Inverter::isValidSender(const uint8_t expAddr[6], const uint8_t isAddr[6]) {
    for (int i = 0; i < 6; i++) {
        if ((isAddr[i] != expAddr[i]) && (expAddr[i] != 0xFF)) {
            ESP_LOGV(TAG, "Wrong sender addr");
            return false;
        }
    }
    return true;
}

E_RC ESP32_SMA_Inverter::getPacket(const uint8_t expAddr[6], int wait4Command) {
    ESP_LOGV(TAG, "getPacket cmd=0x%04x", wait4Command);

    int index = 0;
    bool hasL2pckt = false;
    E_RC rc = E_OK;
    L1Hdr *pL1Hdr = (L1Hdr *)&btrdBuf[0];

    do {
        // Read L1 header (18 bytes)
        uint8_t rdCnt = 0;
        for (rdCnt = 0; rdCnt < 18; rdCnt++) {
            btrdBuf[rdCnt] = BTgetByte();
            if (readTimeout) break;
        }
        ESP_LOGD(TAG, "L1 Rec=%d bytes pkL=0x%04x Cmd=0x%04x",
                 rdCnt, pL1Hdr->pkLength, pL1Hdr->command);

        if (rdCnt < 17) {
            ESP_LOGV(TAG, "L1 < 18 bytes");
            return E_NODATA;
        }

        if (!((btrdBuf[0] ^ btrdBuf[1] ^ btrdBuf[2]) == btrdBuf[3])) {
            ESP_LOGW(TAG, "Wrong L1 CRC");
        }

        if (pL1Hdr->pkLength > sizeof(L1Hdr)) {
            // Read L2 payload
            for (rdCnt = 18; rdCnt < pL1Hdr->pkLength; rdCnt++) {
                btrdBuf[rdCnt] = BTgetByte();
                if (readTimeout) break;
            }
            ESP_LOGD(TAG, "L2 Rec=%d bytes", rdCnt - 18);

            if (isValidSender(expAddr, pL1Hdr->SourceAddr)) {
                rc = E_OK;

                ESP_LOGD(TAG, "HasL2pckt: 0x7E?=0x%02X 0x656003FF?=0x%08X",
                         btrdBuf[18], get_u32(btrdBuf + 19));

                if (!hasL2pckt && btrdBuf[18] == 0x7E &&
                    get_u32(btrdBuf + 19) == 0x656003FF) {
                    hasL2pckt = true;
                }

                if (hasL2pckt) {
                    bool escNext = false;
                    for (int i = sizeof(L1Hdr); i < pL1Hdr->pkLength; i++) {
                        pcktBuf[index] = btrdBuf[i];
                        if (escNext) {
                            pcktBuf[index] ^= 0x20;
                            escNext = false;
                            index++;
                        } else {
                            if (pcktBuf[index] == 0x7D)
                                escNext = true;
                            else
                                index++;
                        }
                        if (index >= MAX_PCKT_BUF_SIZE) {
                            ESP_LOGE(TAG, "pcktBuf overflow! (%d)", index);
                        }
                    }
                    pcktBufPos = index;
                } else {
                    memcpy(pcktBuf, btrdBuf, rdCnt);
                    pcktBufPos = rdCnt;
                }
            } else {
                rc = E_RETRY;
            }
        } else {
            // L1-only packet
            if (isValidSender(expAddr, pL1Hdr->SourceAddr)) {
                rc = E_OK;
                memcpy(pcktBuf, btrdBuf, sizeof(L1Hdr));
                pcktBufPos = sizeof(L1Hdr);
            } else {
                rc = E_RETRY;
            }
        }

        // Flush if garbage start byte
        if (btrdBuf[0] != 0x7e) {
            flushRxBuffer();
            ESP_LOGD(TAG, "CommBuf[0]!=0x7e -> flush");
        }

    } while (((pL1Hdr->command != wait4Command) || (rc == E_RETRY)) &&
             (0xFF != wait4Command));

    if (pcktBufPos > pcktBufMax) {
        pcktBufMax = pcktBufPos;
        ESP_LOGD(TAG, "pcktBufMax now %d", pcktBufMax);
    }

    return rc;
}

// ============================================================
//  Packet building
// ============================================================

void ESP32_SMA_Inverter::writePacketHeader(uint8_t *buf, const uint16_t control,
                                            const uint8_t *destaddress) {
    pcktBufPos   = 0;
    fcsChecksum  = 0xFFFF;
    buf[pcktBufPos++] = 0x7E;
    buf[pcktBufPos++] = 0;
    buf[pcktBufPos++] = 0;
    buf[pcktBufPos++] = 0;
    for (int i = 0; i < 6; i++) buf[pcktBufPos++] = espBTAddress[i];
    for (int i = 0; i < 6; i++) buf[pcktBufPos++] = destaddress[i];
    buf[pcktBufPos++] = (uint8_t)(control & 0xFF);
    buf[pcktBufPos++] = (uint8_t)(control >> 8);
}

bool ESP32_SMA_Inverter::isCrcValid(uint8_t lb, uint8_t hb) {
    if (lb == 0x7E || hb == 0x7E || lb == 0x7D || hb == 0x7D) return false;
    return true;
}

uint32_t ESP32_SMA_Inverter::getattribute(uint8_t *pcktbuf) {
    const int recordsize = 40;
    uint32_t tag = 0, attribute = 0, prevTag = 0;
    for (int idx = 8; idx < recordsize; idx += 4) {
        attribute = get_u32(pcktbuf + idx);
        tag = attribute & 0x00FFFFFF;
        if (tag == 0xFFFFFE) break;
        if ((attribute >> 24) == 1)
            if (prevTag == 0) prevTag = tag;
    }
    return prevTag;
}

// ============================================================
//  Data query dispatcher
// ============================================================

E_RC ESP32_SMA_Inverter::getInverterDataCfl(uint32_t command, uint32_t first, uint32_t last) {
    ESP_LOGV(TAG, "getInverterDataCfl cmd=%u first=%u last=%u", command, first, last);

    do {
        pcktID++;
        writePacketHeader(pcktBuf, 0x01, sixff);
        writePacket(pcktBuf, 0x09, 0xA0, 0, invData.SUSyID, invData.Serial);
        write32(pcktBuf, command);
        write32(pcktBuf, first);
        write32(pcktBuf, last);
        writePacketTrailer(pcktBuf);
        writePacketLength(pcktBuf);
    } while (!isCrcValid(pcktBuf[pcktBufPos - 3], pcktBuf[pcktBufPos - 2]));

    BTsendPacket(pcktBuf);

    bool validPcktID = false;
    do {
        uint8_t pcktcount = 0;
        do {
            invData.status = getPacket(invData.btAddress, 0x0001);
            if (invData.status != E_OK) return invData.status;

            if (validateChecksum()) {
                if ((invData.status = (E_RC)get_u16(pcktBuf + 23)) != E_OK) {
                    ESP_LOGD(TAG, "Packet status 0x%02X", invData.status);
                    return invData.status;
                }
                uint8_t iSPOT_PDC = 0, iSPOT_UDC = 0, iSPOT_IDC = 0;
                pcktcount = get_u16(pcktBuf + 25);
                uint16_t rcvpcktID = get_u16(pcktBuf + 27) & 0x7FFF;

                if (pcktID == rcvpcktID) {
                    if (get_u16(pcktBuf + 15) == invData.SUSyID &&
                        get_u32(pcktBuf + 17) == invData.Serial) {

                        validPcktID = true;
                        value32 = 0;
                        value64 = 0;
                        uint16_t recordsize = 4 * ((uint32_t)pcktBuf[5] - 9) /
                                              (get_u32(pcktBuf + 37) - get_u32(pcktBuf + 33) + 1);
                        ESP_LOGD(TAG, "pcktID=0x%04x recsize=%d BufPos=%d pcktCnt=%04x",
                                 rcvpcktID, recordsize, pcktBufPos, pcktcount);

                        for (uint16_t ii = 41; ii < pcktBufPos - 3; ii += recordsize) {
                            uint8_t *recptr  = pcktBuf + ii;
                            uint32_t code    = get_u32(recptr);
                            uint16_t lri     = (code & 0x00FFFF00) >> 8;
                            uint8_t  dataType = code >> 24;
                            time_t   datetime = (time_t)get_u32(recptr + 4);

                            if (recordsize == 16) {
                                value64 = get_u64(recptr + 8);
                                if (is_NaN(value64) || is_NaN((uint64_t)value64)) value64 = 0;
                            } else if (dataType != DT_STRING && dataType != DT_STATUS) {
                                value32 = get_u32(recptr + 16);
                                if (is_NaN(value32) || is_NaN((uint32_t)value32)) value32 = 0;
                            }

                            switch (lri) {
                            case GridMsTotW:
                                invData.LastTime = datetime;
                                invData.TotalPac = toW(value32);
                                dispData.TotalPac = tokW(value32);
                                printUnixTime(timeBuf, datetime);
                                ESP_LOGI(TAG, "SPOT_PACTOT %15.3f kW  GMT:%s", tokW(value32), timeBuf);
                                break;
                            case GridMsWphsA:
                                invData.Pac1 = toW(value32);
                                dispData.Pac1 = tokW(value32);
                                ESP_LOGI(TAG, "SPOT_PAC1 %14.2f kW", tokW(value32));
                                break;
                            case GridMsWphsB:
                                invData.Pac2 = toW(value32);
                                dispData.Pac2 = tokW(value32);
                                ESP_LOGI(TAG, "SPOT_PAC2 %14.2f kW", tokW(value32));
                                break;
                            case GridMsWphsC:
                                invData.Pac3 = toW(value32);
                                dispData.Pac3 = tokW(value32);
                                ESP_LOGI(TAG, "SPOT_PAC3 %14.2f kW", tokW(value32));
                                break;
                            case GridMsPhVphsA:
                                invData.Uac1 = value32;
                                dispData.Uac1 = toVolt(value32);
                                ESP_LOGI(TAG, "SPOT_UAC1 %15.2f V", toVolt(value32));
                                break;
                            case GridMsPhVphsB:
                                invData.Uac2 = value32;
                                dispData.Uac2 = toVolt(value32);
                                ESP_LOGI(TAG, "SPOT_UAC2 %15.2f V", toVolt(value32));
                                break;
                            case GridMsPhVphsC:
                                invData.Uac3 = value32;
                                dispData.Uac3 = toVolt(value32);
                                ESP_LOGI(TAG, "SPOT_UAC3 %15.2f V", toVolt(value32));
                                break;
                            case GridMsAphsA_1:
                            case GridMsAphsA:
                                invData.Iac1 = value32;
                                dispData.Iac1 = toAmp(value32);
                                ESP_LOGI(TAG, "SPOT_IAC1 %15.2f A", toAmp(value32));
                                break;
                            case GridMsAphsB_1:
                            case GridMsAphsB:
                                invData.Iac2 = value32;
                                dispData.Iac2 = toAmp(value32);
                                ESP_LOGI(TAG, "SPOT_IAC2 %15.2f A", toAmp(value32));
                                break;
                            case GridMsAphsC_1:
                            case GridMsAphsC:
                                invData.Iac3 = value32;
                                dispData.Iac3 = toAmp(value32);
                                ESP_LOGI(TAG, "SPOT_IAC3 %15.2f A", toAmp(value32));
                                break;
                            case GridMsHz:
                                invData.GridFreq = value32;
                                dispData.GridFreq = toHz(value32);
                                ESP_LOGI(TAG, "Freq %14.2f Hz", toHz(value32));
                                break;
                            case DcMsWatt:
                                if      (iSPOT_PDC == 0) { invData.Pdc1 = toW(value32); dispData.Pdc1 = tokW(value32); }
                                else if (iSPOT_PDC == 1) { invData.Pdc2 = toW(value32); dispData.Pdc2 = tokW(value32); }
                                ESP_LOGI(TAG, "SPOT_PDC%d %15.2f kW", iSPOT_PDC + 1, tokW(value32));
                                iSPOT_PDC++;
                                break;
                            case DcMsVol:
                                if      (iSPOT_UDC == 0) { invData.Udc1 = value32; dispData.Udc1 = toVolt(value32); }
                                else if (iSPOT_UDC == 1) { invData.Udc2 = value32; dispData.Udc2 = toVolt(value32); }
                                ESP_LOGI(TAG, "SPOT_UDC%d %15.2f V", iSPOT_UDC + 1, toVolt(value32));
                                iSPOT_UDC++;
                                break;
                            case DcMsAmp:
                                if      (iSPOT_IDC == 0) { invData.Idc1 = value32; dispData.Idc1 = toAmp(value32); }
                                else if (iSPOT_IDC == 1) { invData.Idc2 = value32; dispData.Idc2 = toAmp(value32); }
                                ESP_LOGI(TAG, "SPOT_IDC%d %15.2f A", iSPOT_IDC + 1, toAmp(value32));
                                iSPOT_IDC++;
                                break;
                            case MeteringDyWhOut:
                                invData.EToday = value64;
                                dispData.EToday = tokWh(value64);
                                ESP_LOGI(TAG, "SPOT_ETODAY %11.3f kWh", tokWh(value64));
                                break;
                            case MeteringTotWhOut:
                                invData.ETotal = value64;
                                dispData.ETotal = tokWh(value64);
                                ESP_LOGI(TAG, "SPOT_ETOTAL %11.3f kWh", tokWh(value64));
                                break;
                            case MeteringTotOpTms:
                                invData.OperationTime = value64;
                                ESP_LOGI(TAG, "SPOT_OPERTM %7.3f h", toHour(value64));
                                break;
                            case MeteringTotFeedTms:
                                invData.FeedInTime = value64;
                                ESP_LOGI(TAG, "SPOT_FEEDTM %7.3f h", toHour(value64));
                                break;
                            case NameplateLocation:
                                invData.WakeupTime = datetime;
                                if (recordsize > 8) {
                                    const char *nameptr = (const char *)recptr + 8;
                                    size_t max_copy = recordsize - 8;
                                    if (max_copy >= max_buf_size) max_copy = max_buf_size - 1;
                                    strncpy(charBuf, nameptr, max_copy);
                                    charBuf[max_copy] = '\0';
                                    invData.DeviceName = std::string(charBuf);
                                }
                                ESP_LOGI(TAG, "INV_NAME %s", invData.DeviceName.c_str());
                                break;
                            case NameplatePkgRev:
                                get_version(get_u32(recptr + 24), inverter_version);
                                invData.SWVersion = std::string(inverter_version);
                                ESP_LOGI(TAG, "INV_SWVER %s", invData.SWVersion.c_str());
                                break;
                            case NameplateModel:
                                value32 = getattribute(recptr);
                                invData.DeviceType = value32;
                                ESP_LOGI(TAG, "INV_TYPE %d", value32);
                                break;
                            case NameplateMainModel:
                                value32 = getattribute(recptr);
                                invData.DeviceClass = value32;
                                ESP_LOGI(TAG, "INV_CLASS %d", value32);
                                break;
                            case CoolsysTmpNom:
                                invData.InvTemp = value32;
                                dispData.InvTemp = toTemp(value32);
                                ESP_LOGI(TAG, "Temp %7.3f C", toTemp(value32));
                                break;
                            case OperationHealth:
                                value32 = getattribute(recptr);
                                invData.DevStatus = value32;
                                ESP_LOGI(TAG, "DevStatus %d", value32);
                                break;
                            case OperationGriSwStt:
                                value32 = getattribute(recptr);
                                invData.GridRelay = value32;
                                ESP_LOGI(TAG, "GridRelay %d", value32);
                                break;
                            case MeteringGridMsTotWOut:
                                invData.MeteringGridMsTotWOut = value32;
                                break;
                            case MeteringGridMsTotWIn:
                                invData.MeteringGridMsTotWIn = value32;
                                break;
                            default:
                                ESP_LOGV(TAG, "Unknown LRI 0x%04x val=%d", lri, value32);
                                break;
                            }
                        } // for records
                    } else {
                        ESP_LOGW(TAG, "Wrong SUSyID/Serial");
                    }
                } else {
                    ESP_LOGW(TAG, "PacketID mismatch: exp=0x%04X is=0x%04X", pcktID, rcvpcktID);
                    validPcktID = false;
                    pcktcount = 0;
                }
            } else {
                invData.status = E_CHKSUM;
                return invData.status;
            }
        } while (pcktcount > 0);
    } while (!validPcktID);

    return invData.status;
}

E_RC ESP32_SMA_Inverter::getInverterData(enum getInverterDataType type) {
    uint32_t command, first, last;

    switch (type) {
    case EnergyProduction:
        command = 0x54000200; first = 0x00260100; last = 0x002622FF; break;
    case SpotDCPower:
        command = 0x53800200; first = 0x00251E00; last = 0x00251EFF; break;
    case SpotDCVoltage:
        command = 0x53800200; first = 0x00451F00; last = 0x004521FF; break;
    case SpotACPower:
        command = 0x51000200; first = 0x00464000; last = 0x004642FF; break;
    case SpotACVoltage:
        command = 0x51000200; first = 0x00464800; last = 0x004655FF; break;
    case SpotGridFrequency:
        command = 0x51000200; first = 0x00465700; last = 0x004657FF; break;
    case SpotACTotalPower:
        command = 0x51000200; first = 0x00263F00; last = 0x00263FFF; break;
    case TypeLabel:
        command = 0x58000200; first = 0x00821E00; last = 0x008220FF; break;
    case SoftwareVersion:
        command = 0x58000200; first = 0x00823400; last = 0x008234FF; break;
    case DeviceStatus:
        command = 0x51800200; first = 0x00214800; last = 0x002148FF; break;
    case GridRelayStatus:
        command = 0x51800200; first = 0x00416400; last = 0x004164FF; break;
    case OperationTime:
        command = 0x54000200; first = 0x00462E00; last = 0x00462FFF; break;
    case InverterTemp:
        command = 0x52000200; first = 0x00237700; last = 0x002377FF; break;
    case MeteringGridMsTotW:
        command = 0x51000200; first = 0x00463600; last = 0x004637FF; break;
    default:
        ESP_LOGW(TAG, "Invalid getInverterDataType");
        return E_BADARG;
    }

    for (uint8_t retries = 1; ; retries++) {
        E_RC rc = getInverterDataCfl(command, first, last);
        if (rc != E_OK) {
            if (retries > 1) return rc;
            ESP_LOGI(TAG, "Retry %d", retries);
        } else {
            return rc;
        }
    }
}

bool ESP32_SMA_Inverter::getBT_SignalStrength() {
    ESP_LOGI(TAG, "*** SignalStrength ***");
    writePacketHeader(pcktBuf, 0x03, invData.btAddress);
    writeByte(pcktBuf, 0x05);
    writeByte(pcktBuf, 0x00);
    writePacketLength(pcktBuf);
    BTsendPacket(pcktBuf);
    getPacket(invData.btAddress, 4);
    dispData.BTSigStrength = ((float)btrdBuf[22] * 100.0f / 255.0f);
    ESP_LOGI(TAG, "BT-Signal %9.1f %%", dispData.BTSigStrength);
    return true;
}

E_RC ESP32_SMA_Inverter::initialiseSMAConnection() {
    ESP_LOGI(TAG, "-> Initialize");
    getPacket(invData.btAddress, 2);
    invData.NetID = pcktBuf[22];
    ESP_LOGI(TAG, "SMA netID=0x%02X", invData.NetID);

    writePacketHeader(pcktBuf, 0x02, invData.btAddress);
    write32(pcktBuf, 0x00700400);
    writeByte(pcktBuf, invData.NetID);
    write32(pcktBuf, 0);
    write32(pcktBuf, 1);
    writePacketLength(pcktBuf);
    BTsendPacket(pcktBuf);

    getPacket(invData.btAddress, 5);
    memcpy(espBTAddress, pcktBuf + 26, 6);
    ESP_LOGW(TAG, "ESP32 BT addr: %02X:%02X:%02X:%02X:%02X:%02X",
             espBTAddress[5], espBTAddress[4], espBTAddress[3],
             espBTAddress[2], espBTAddress[1], espBTAddress[0]);

    pcktID++;
    writePacketHeader(pcktBuf, 0x01, sixff);
    writePacket(pcktBuf, 0x09, 0xA0, 0, 0xFFFF, 0xFFFFFFFF);
    write32(pcktBuf, 0x00000200);
    write32(pcktBuf, 0);
    write32(pcktBuf, 0);
    writePacketTrailer(pcktBuf);
    writePacketLength(pcktBuf);
    BTsendPacket(pcktBuf);

    if (getPacket(invData.btAddress, 1) != E_OK) return E_INIT;
    if (!validateChecksum()) return E_CHKSUM;

    invData.Serial = get_u32(pcktBuf + 57);
    ESP_LOGW(TAG, "Serial Nr: %lu", invData.Serial);
    return E_OK;
}

void ESP32_SMA_Inverter::logoffSMAInverter() {
    pcktID++;
    writePacketHeader(pcktBuf, 0x01, sixff);
    writePacket(pcktBuf, 0x08, 0xA0, 0x0300, 0xFFFF, 0xFFFFFFFF);
    write32(pcktBuf, 0xFFFD010E);
    write32(pcktBuf, 0xFFFFFFFF);
    writePacketTrailer(pcktBuf);
    writePacketLength(pcktBuf);
    BTsendPacket(pcktBuf);
}

E_RC ESP32_SMA_Inverter::logonSMAInverter() {
    return logonSMAInverter(smaInvPass, USERGROUP);
}

E_RC ESP32_SMA_Inverter::logonSMAInverter(const char *password, const uint8_t user) {
#define MAX_PWLENGTH 12
    uint8_t pw[MAX_PWLENGTH];
    uint8_t encChar = (user == USERGROUP) ? 0x88 : 0xBB;
    uint8_t idx;
    for (idx = 0; password[idx] != 0 && idx < MAX_PWLENGTH; idx++)
        pw[idx] = password[idx] + encChar;
    for (; idx < MAX_PWLENGTH; idx++) pw[idx] = encChar;

    time_t now;
    pcktID++;
    now = time(nullptr);
    writePacketHeader(pcktBuf, 0x01, sixff);
    writePacket(pcktBuf, 0x0E, 0xA0, 0x0100, 0xFFFF, 0xFFFFFFFF);
    write32(pcktBuf, 0xFFFD040C);
    write32(pcktBuf, user);
    write32(pcktBuf, 0x00000384);
    write32(pcktBuf, now);
    write32(pcktBuf, 0);
    writeArray(pcktBuf, pw, sizeof(pw));
    writePacketTrailer(pcktBuf);
    writePacketLength(pcktBuf);
    BTsendPacket(pcktBuf);

    E_RC rc = E_OK;
    if ((rc = getPacket(sixff, 1)) != E_OK) return rc;
    if (!validateChecksum()) return E_CHKSUM;

    uint16_t rcvpcktID = get_u16(pcktBuf + 27) & 0x7FFF;
    if (pcktID == rcvpcktID && get_u32(pcktBuf + 41) == (uint32_t)now) {
        invData.SUSyID = get_u16(pcktBuf + 15);
        invData.Serial = get_u32(pcktBuf + 17);
        ESP_LOGV(TAG, "SUSyID=0x%02X Serial=0x%08X", invData.SUSyID, invData.Serial);
    } else {
        ESP_LOGW(TAG, "Logon unexpected response pcktID=0x%04X rcvpcktID=0x%04X",
                 pcktID, rcvpcktID);
        rc = E_INVRESP;
    }
    return rc;
}

// ============================================================
//  Inverter time sync  (mirrors SBFspot SetPlantTime_V2)
//
//  Step 1 — query: send packet with zeros → inverter returns its
//           current time, tz/dst, and timesetCount in the response.
//  Step 2 — write: send packet with host UTC time + incremented
//           timesetCount → inverter updates its RTC.
//  Response to the write is read and logged for confirmation.
// ============================================================

void ESP32_SMA_Inverter::setInverterTime() {
    time_t hosttime = time(nullptr);
    if (hosttime < 946684800L) {  // before year 2000 — NTP not yet synced
        ESP_LOGW(TAG, "setInverterTime: system clock not synced yet, will retry next cycle");
        sync_time_requested_ = true;  // keep flag set so the loop retries
        return;
    }

    // --- Step 1: query inverter's current time (all-zero timestamps) ---
    // Use espBTAddress (host's own BT address) as destination — required by SMA protocol
    // for time commands (SBFspot added this in v3.1.5 as LocalBTAddress)
    pcktID++;
    writePacketHeader(pcktBuf, 0x01, espBTAddress);
    writePacket(pcktBuf, 0x10, 0xA0, 0, 0xFFFF, 0xFFFFFFFF);
    write32(pcktBuf, 0xF000020A);
    write32(pcktBuf, 0x00236D00); write32(pcktBuf, 0x00236D00); write32(pcktBuf, 0x00236D00);
    write32(pcktBuf, 0); write32(pcktBuf, 0); write32(pcktBuf, 0); write32(pcktBuf, 0);
    write32(pcktBuf, 1); write32(pcktBuf, 1);
    writePacketTrailer(pcktBuf);
    writePacketLength(pcktBuf);
    BTsendPacket(pcktBuf);

    E_RC rc = getPacket(sixff, 1);
    if (rc != E_OK || pcktBufPos < 65) {
        ESP_LOGW(TAG, "setInverterTime: query failed rc=%d len=%d, skipping", rc, pcktBufPos);
        return;
    }

    time_t   invTime      = (time_t)  get_u32(pcktBuf + 45);
    uint32_t tz_dst       =           get_u32(pcktBuf + 57);
    uint32_t timesetCount =           get_u32(pcktBuf + 61);

    printUnixTime(timeBuf, invTime);
    ESP_LOGI(TAG, "setInverterTime: inverter clock = %s (UTC)", timeBuf);
    ESP_LOGI(TAG, "setInverterTime: tz_dst=0x%08X timesetCount=%u", tz_dst, timesetCount);

    hosttime = time(nullptr);
    printUnixTime(timeBuf, hosttime);
    ESP_LOGI(TAG, "setInverterTime: host clock     = %s (UTC), writing to inverter", timeBuf);

    // --- Step 2: write host time to inverter ---
    // Use do-while to retry if FCS bytes happen to be 0x7E/0x7D (would corrupt framing)
    do {
        pcktID++;
        writePacketHeader(pcktBuf, 0x01, espBTAddress);
        writePacket(pcktBuf, 0x10, 0xA0, 0, 0xFFFF, 0xFFFFFFFF);
        write32(pcktBuf, 0xF000020A);
        write32(pcktBuf, 0x00236D00); write32(pcktBuf, 0x00236D00); write32(pcktBuf, 0x00236D00);
        write32(pcktBuf, (uint32_t)hosttime);
        write32(pcktBuf, (uint32_t)hosttime);
        write32(pcktBuf, (uint32_t)hosttime);
        write32(pcktBuf, tz_dst);            // preserve inverter's tz/dst setting
        write32(pcktBuf, timesetCount + 1);  // increment counter
        write32(pcktBuf, 1);
        writePacketTrailer(pcktBuf);
        writePacketLength(pcktBuf);
    } while (!isCrcValid(pcktBuf[pcktBufPos - 3], pcktBuf[pcktBufPos - 2]));
    BTsendPacket(pcktBuf);

    // Give the inverter time to process the write before querying again
    // (some inverters briefly reset their BT module after a time write)
    vTaskDelay(pdMS_TO_TICKS(1000));

    // --- Step 3: verify — send another read query, inverter responds with updated time ---
    pcktID++;
    writePacketHeader(pcktBuf, 0x01, espBTAddress);
    writePacket(pcktBuf, 0x10, 0xA0, 0, 0xFFFF, 0xFFFFFFFF);
    write32(pcktBuf, 0xF000020A);
    write32(pcktBuf, 0x00236D00); write32(pcktBuf, 0x00236D00); write32(pcktBuf, 0x00236D00);
    write32(pcktBuf, 0); write32(pcktBuf, 0); write32(pcktBuf, 0); write32(pcktBuf, 0);
    write32(pcktBuf, 1); write32(pcktBuf, 1);
    writePacketTrailer(pcktBuf);
    writePacketLength(pcktBuf);
    BTsendPacket(pcktBuf);

    rc = getPacket(sixff, 1);
    if (rc == E_OK && pcktBufPos >= 50) {
        time_t newTime = (time_t)get_u32(pcktBuf + 45);
        printUnixTime(timeBuf, newTime);
        ESP_LOGI(TAG, "setInverterTime: inverter clock now = %s (UTC)", timeBuf);
    } else {
        ESP_LOGW(TAG, "setInverterTime: verify query failed rc=%d", rc);
    }
    sync_time_requested_ = false;
}

// ============================================================
//  Packet writing helpers
// ============================================================

void ESP32_SMA_Inverter::writeByte(uint8_t *btbuffer, uint8_t v) {
    fcsChecksum = (fcsChecksum >> 8) ^ fcstab[(fcsChecksum ^ v) & 0xff];
    if (v == 0x7d || v == 0x7e || v == 0x11 || v == 0x12 || v == 0x13) {
        btbuffer[pcktBufPos++] = 0x7d;
        btbuffer[pcktBufPos++] = v ^ 0x20;
    } else {
        btbuffer[pcktBufPos++] = v;
    }
}

void ESP32_SMA_Inverter::write32(uint8_t *btbuffer, uint32_t v) {
    writeByte(btbuffer, (uint8_t)((v >>  0) & 0xFF));
    writeByte(btbuffer, (uint8_t)((v >>  8) & 0xFF));
    writeByte(btbuffer, (uint8_t)((v >> 16) & 0xFF));
    writeByte(btbuffer, (uint8_t)((v >> 24) & 0xFF));
}

void ESP32_SMA_Inverter::write16(uint8_t *btbuffer, uint16_t v) {
    writeByte(btbuffer, (uint8_t)((v >> 0) & 0xFF));
    writeByte(btbuffer, (uint8_t)((v >> 8) & 0xFF));
}

void ESP32_SMA_Inverter::writeArray(uint8_t *btbuffer, const uint8_t bytes[], int loopcount) {
    for (int i = 0; i < loopcount; i++) writeByte(btbuffer, bytes[i]);
}

void ESP32_SMA_Inverter::writePacket(uint8_t *buf, uint8_t longwords, uint8_t ctrl,
                                      uint16_t ctrl2, uint16_t dstSUSyID, uint32_t dstSerial) {
    buf[pcktBufPos++] = 0x7E;
    write32(buf, BTH_L2SIGNATURE);
    writeByte(buf, longwords);
    writeByte(buf, ctrl);
    write16(buf, dstSUSyID);
    write32(buf, dstSerial);
    write16(buf, ctrl2);
    write16(buf, appSUSyID);
    write32(buf, appSerial);
    write16(buf, ctrl2);
    write16(buf, 0);
    write16(buf, 0);
    write16(buf, pcktID | 0x8000);
}

void ESP32_SMA_Inverter::writePacketTrailer(uint8_t *btbuffer) {
    fcsChecksum ^= 0xFFFF;
    btbuffer[pcktBufPos++] = fcsChecksum & 0x00FF;
    btbuffer[pcktBufPos++] = (fcsChecksum >> 8) & 0x00FF;
    btbuffer[pcktBufPos++] = 0x7E;
}

void ESP32_SMA_Inverter::writePacketLength(uint8_t *buf) {
    buf[1] = pcktBufPos & 0xFF;
    buf[2] = (pcktBufPos >> 8) & 0xFF;
    buf[3] = buf[0] ^ buf[1] ^ buf[2];
}

bool ESP32_SMA_Inverter::validateChecksum() {
    fcsChecksum = 0xffff;
    for (int i = 1; i <= pcktBufPos - 4; i++) {
        fcsChecksum = (fcsChecksum >> 8) ^ fcstab[(fcsChecksum ^ pcktBuf[i]) & 0xff];
    }
    fcsChecksum ^= 0xffff;
    if (get_u16(pcktBuf + pcktBufPos - 3) == fcsChecksum) return true;
    ESP_LOGE(TAG, "validateChecksum: got 0x%04X expected 0x%04X",
             fcsChecksum, get_u16(pcktBuf + pcktBufPos - 3));
    return false;
}

// ============================================================
//  Utility helpers
// ============================================================

void ESP32_SMA_Inverter::HexDump(uint8_t *buf, int count, int radix, uint8_t c) {
    char line[(radix * 3) + 10];
    char *lp = line;
    for (int i = 0; i < radix; i++) lp += sprintf(lp, " %02X", i);
    ESP_LOGD(TAG, "---%c----:%s", c, line);
    lp = line;
    for (int i = 0, j = 0; i < count; i++, j = i % radix) {
        if (j == 0) {
            if (lp != line) { ESP_LOGD(TAG, "%s", line); lp = line; }
            lp += sprintf(lp, "%c-%06X:", c, i);
        }
        lp += sprintf(lp, " %02X", buf[i]);
    }
    if (lp != line) ESP_LOGD(TAG, "%s", line);
}

uint8_t ESP32_SMA_Inverter::printUnixTime(char *buf, time_t t) {
    if (t < 1) t = 0;
    uint8_t seconds = t % 60; t /= 60;
    uint8_t minutes = t % 60; t /= 60;
    uint8_t hours   = t % 24; t /= 24;
    uint32_t a = (uint32_t)((4 * t + 102032) / 146097 + 15);
    uint32_t b = (uint32_t)(t + 2442113 + a - (a / 4));
    uint32_t c = (20 * b - 2442) / 7305;
    uint32_t d = b - 365 * c - (c / 4);
    uint32_t e = d * 1000 / 30601;
    uint32_t f = d - e * 30 - e * 601 / 1000;
    if (e <= 13) { c -= 4716; e -= 1; } else { c -= 4715; e -= 13; }
    return snprintf(buf, 31, "%02u.%02u.%u %02u:%02u:%02u",
                    (unsigned)f, (unsigned)e, (unsigned)c,
                    (unsigned)hours, (unsigned)minutes, (unsigned)seconds);
}

uint16_t ESP32_SMA_Inverter::get_u16(uint8_t *buf) {
    uint16_t v = 0;
    v += *(buf + 1); v <<= 8; v += *(buf);
    return v;
}

uint32_t ESP32_SMA_Inverter::get_u32(uint8_t *buf) {
    uint32_t v = 0;
    v += *(buf + 3); v <<= 8; v += *(buf + 2);
    v <<= 8; v += *(buf + 1); v <<= 8; v += *(buf);
    return v;
}

uint64_t ESP32_SMA_Inverter::get_u64(uint8_t *buf) {
    uint64_t v = 0;
    v += *(buf + 7); v <<= 8; v += *(buf + 6);
    v <<= 8; v += *(buf + 5); v <<= 8; v += *(buf + 4);
    v <<= 8; v += *(buf + 3); v <<= 8; v += *(buf + 2);
    v <<= 8; v += *(buf + 1); v <<= 8; v += *(buf);
    return v;
}

void ESP32_SMA_Inverter::get_version(uint32_t version, char *inverter_version_) {
    uint8_t vType  = version & 0xFF;
    vType = vType > 5 ? '?' : "NEABRS"[vType];
    uint8_t vBuild = (version >>  8) & 0xFF;
    uint8_t vMinor = (version >> 16) & 0xFF;
    uint8_t vMajor = (version >> 24) & 0xFF;
    snprintf(inverter_version_, 24, "%c%c.%c%c.%02d.%c",
             '0' + (vMajor >> 4), '0' + (vMajor & 0x0F),
             '0' + (vMinor >> 4), '0' + (vMinor & 0x0F),
             vBuild, vType);
}

} // namespace smabluetooth_solar
} // namespace esphome
