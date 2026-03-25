#pragma once
#ifndef ESP32_SMA_INVERTER_H
#define ESP32_SMA_INVERTER_H
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

// FreeRTOS
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/stream_buffer.h"
#include "freertos/event_groups.h"

// ESP-IDF Bluetooth (Classic / Bluedroid / SPP)
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include "esp_spp_api.h"

#include <string>
#include <cstring>
#include <cstdint>
#include <ctime>

#define DEBUG_SMA 0

namespace esphome {
namespace smabluetooth_solar {

#define tokWh(value64)    (double)(value64)/1000.0
#define tokW(value32)     (float)(value32)/1000.0
#define toW(value32)      (float)(value32)/1.0
#define toHour(value64)   (double)(value64)/3600
#define toAmp(value32)    (float)(value32)/1000
#define toVolt(value32)   (float)(value32)/100
#define toHz(value32)     (float)(value32)/100
#define toPercent(value32)(float)(value32)/100
#define toTemp(value32)   (float)(value32)/100

#define UG_USER      0x07
#define UG_INSTALLER 0x0A

#define ARCH_DAY_SIZE 288
#define COMMBUFSIZE 2048
#define MAX_PCKT_BUF_SIZE COMMBUFSIZE

#define BTH_L2SIGNATURE 0x656003FF
#define USERGROUP UG_USER

#define NaN_S16 0x8000
#define NaN_U16 0xFFFF
#define NaN_S32 (int32_t) 0x80000000
#define NaN_U32 (uint32_t)0xFFFFFFFF
#define NaN_S64 (int64_t) 0x8000000000000000
#define NaN_U64 (uint64_t)0xFFFFFFFFFFFFFFFF

inline const bool is_NaN(const int16_t S16)  { return S16 == NaN_S16; }
inline const bool is_NaN(const uint16_t U16) { return U16 == NaN_U16; }
inline const bool is_NaN(const int32_t S32)  { return S32 == NaN_S32; }
inline const bool is_NaN(const uint32_t U32) { return U32 == NaN_U32; }
inline const bool is_NaN(const int64_t S64)  { return S64 == NaN_S64; }
inline const bool is_NaN(const uint64_t U64) { return U64 == NaN_U64; }

// ---- Event group bits for BT state signalling between task and callbacks ----
#define BT_EVT_SPP_INIT     (BIT0)
#define BT_EVT_DISC_DONE    (BIT1)
#define BT_EVT_CONNECTED    (BIT2)
#define BT_EVT_DISCONNECTED (BIT3)

enum SMA_DATATYPE {
    DT_ULONG  = 0,
    DT_STATUS = 8,
    DT_STRING = 16,
    DT_FLOAT  = 32,
    DT_SLONG  = 64
};

enum E_RC {
    E_OK =          0,
    E_INIT =       -1,
    E_INVPASSW =   -2,
    E_RETRY =      -3,
    E_EOF =        -4,
    E_NODATA =     -5,
    E_OVERFLOW =   -6,
    E_BADARG =     -7,
    E_CHKSUM =     -8,
    E_INVRESP =    -9,
    E_ARCHNODATA = -10,
};

struct InverterData {
    uint8_t btAddress[6];
    uint8_t SUSyID;
    uint32_t Serial;
    uint8_t NetID;
    int32_t Pmax;
    int32_t TotalPac;
    int32_t Pac;
    int32_t Pac1;
    int32_t Pac2;
    int32_t Pac3;
    int32_t Uac1;
    int32_t Uac2;
    int32_t Uac3;
    int32_t Iac1;
    int32_t Iac2;
    int32_t Iac3;
    int32_t Pdc1;
    int32_t Pdc2;
    int32_t Udc1;
    int32_t Udc2;
    int32_t Idc1;
    int32_t Idc2;
    int32_t GridFreq;
    int32_t Eta;
    int32_t InvTemp;
    uint64_t EToday;
    uint64_t ETotal;
    uint64_t dayWh[ARCH_DAY_SIZE];
    time_t  DayStartTime;
    bool hasDayData;
    bool hasMonthData;
    time_t   LastTime;
    uint64_t OperationTime;
    uint64_t FeedInTime;
    int32_t DevStatus;
    int32_t GridRelay;
    E_RC     status;
    uint32_t MeteringGridMsTotWOut;
    uint32_t MeteringGridMsTotWIn;
    time_t WakeupTime;
    std::string DeviceName;
    std::string SWVersion;
    uint32_t DeviceType;
    uint32_t DeviceClass;
    std::string InverterTimestamp;   // human-readable UTC string of inverter's clock
};

struct DisplayData {
    float BTSigStrength;
    float Pmax;
    float TotalPac;
    float Pac;
    float Pac1;
    float Pac2;
    float Pac3;
    float Uac1;
    float Uac2;
    float Uac3;
    float Iac1;
    float Iac2;
    float Iac3;
    float InvTemp;
    float Pdc1;
    float Pdc2;
    float Udc1;
    float Udc2;
    float Idc1;
    float Idc2;
    float GridFreq;
    float EToday;
    float ETotal;
    bool needsMissingValues = false;
};

enum getInverterDataType {
    EnergyProduction    = 1 << 0,
    SpotDCPower         = 1 << 1,
    SpotDCVoltage       = 1 << 2,
    SpotACPower         = 1 << 3,
    SpotACVoltage       = 1 << 4,
    SpotGridFrequency   = 1 << 5,
    SpotACTotalPower    = 1 << 8,
    TypeLabel           = 1 << 9,
    OperationTime       = 1 << 10,
    SoftwareVersion     = 1 << 11,
    DeviceStatus        = 1 << 12,
    GridRelayStatus     = 1 << 13,
    BatteryChargeStatus = 1 << 14,
    BatteryInfo         = 1 << 15,
    InverterTemp        = 1 << 16,
    MeteringGridMsTotW  = 1 << 17,
    sbftest             = 1 << 31
};

enum LriDef {
    OperationHealth                 = 0x2148,
    CoolsysTmpNom                   = 0x2377,
    DcMsWatt                        = 0x251E,
    MeteringTotWhOut                = 0x2601,
    MeteringDyWhOut                 = 0x2622,
    GridMsTotW                      = 0x263F,
    BatChaStt                       = 0x295A,
    OperationHealthSttOk            = 0x411E,
    OperationHealthSttWrn           = 0x411F,
    OperationHealthSttAlm           = 0x4120,
    OperationGriSwStt               = 0x4164,
    OperationRmgTms                 = 0x4166,
    DcMsVol                         = 0x451F,
    DcMsAmp                         = 0x4521,
    MeteringPvMsTotWhOut            = 0x4623,
    MeteringGridMsTotWhOut          = 0x4624,
    MeteringGridMsTotWhIn           = 0x4625,
    MeteringCsmpTotWhIn             = 0x4626,
    MeteringGridMsDyWhOut           = 0x4627,
    MeteringGridMsDyWhIn            = 0x4628,
    MeteringTotOpTms                = 0x462E,
    MeteringTotFeedTms              = 0x462F,
    MeteringGriFailTms              = 0x4631,
    MeteringWhIn                    = 0x463A,
    MeteringWhOut                   = 0x463B,
    MeteringPvMsTotWOut             = 0x4635,
    MeteringGridMsTotWOut           = 0x4636,
    MeteringGridMsTotWIn            = 0x4637,
    MeteringCsmpTotWIn              = 0x4639,
    GridMsWphsA                     = 0x4640,
    GridMsWphsB                     = 0x4641,
    GridMsWphsC                     = 0x4642,
    GridMsPhVphsA                   = 0x4648,
    GridMsPhVphsB                   = 0x4649,
    GridMsPhVphsC                   = 0x464A,
    GridMsAphsA_1                   = 0x4650,
    GridMsAphsB_1                   = 0x4651,
    GridMsAphsC_1                   = 0x4652,
    GridMsAphsA                     = 0x4653,
    GridMsAphsB                     = 0x4654,
    GridMsAphsC                     = 0x4655,
    GridMsHz                        = 0x4657,
    MeteringSelfCsmpSelfCsmpWh      = 0x46AA,
    MeteringSelfCsmpActlSelfCsmp    = 0x46AB,
    MeteringSelfCsmpSelfCsmpInc     = 0x46AC,
    MeteringSelfCsmpAbsSelfCsmpInc  = 0x46AD,
    MeteringSelfCsmpDySelfCsmpInc   = 0x46AE,
    BatDiagCapacThrpCnt             = 0x491E,
    BatDiagTotAhIn                  = 0x4926,
    BatDiagTotAhOut                 = 0x4927,
    BatTmpVal                       = 0x495B,
    BatVol                          = 0x495C,
    BatAmp                          = 0x495D,
    NameplateLocation               = 0x821E,
    NameplateMainModel              = 0x821F,
    NameplateModel                  = 0x8220,
    NameplateAvalGrpUsr             = 0x8221,
    NameplatePkgRev                 = 0x8234,
    InverterWLim                    = 0x832A,
    GridMsPhVphsA2B6100             = 0x464B,
    GridMsPhVphsB2C6100             = 0x464C,
    GridMsPhVphsC2A6100             = 0x464D
};

#pragma pack(push, 1)
typedef struct __attribute__((packed)) PacketHeader {
    uint8_t        SOP;
    unsigned short pkLength;
    uint8_t        pkChecksum;
    uint8_t        SourceAddr[6];
    uint8_t        DestinationAddr[6];
    unsigned short command;
} L1Hdr;
#pragma pack(pop)


class ESP32_SMA_Inverter {
  public:
    static ESP32_SMA_Inverter* getInstance() {
        static ESP32_SMA_Inverter instance;
        return &instance;
    }
    ESP32_SMA_Inverter(const ESP32_SMA_Inverter&) = delete;
    ESP32_SMA_Inverter& operator=(const ESP32_SMA_Inverter&) = delete;

    // Called from ESPHome setup()
    void setup(std::string mac, std::string pw, uint32_t delay_values_ms);

    // Initialise ESP-IDF BT stack (non-blocking; fires BT_EVT_SPP_INIT asynchronously)
    bool begin(const char *localName);

    // Start / stop the FreeRTOS protocol task
    void startBtTask();
    void stopBtTask();

    // Status queries (safe to call from ESPHome main loop)
    bool isDataReady()   const { return data_ready_; }
    void clearDataReady()      { data_ready_ = false; }
    bool hasTaskError()  const { return task_error_; }
    void clearTaskError()      { task_error_ = false; }
    bool isBtConnected() const { return btConnected_; }
    bool isNightModeActive() const { return night_mode_active_; }

    void requestTimeSync()     { sync_time_requested_ = true; }
    void requestTimeFetch()    { fetch_time_requested_ = true; }

    void initPcktID()              { setPcktID(1); }
    void setPcktID(uint8_t id)     { pcktID = id; }

    uint32_t getBtgetByteTimeout() const { return btgetByteTimeout; }
    void setBtgetByteTimeout(uint32_t v) { btgetByteTimeout = v; }

    bool is_nighttime() const;

    // Shared data — written by BT task, read by ESPHome update()
    InverterData invData  = InverterData();
    DisplayData  dispData = DisplayData();

  private:
    ESP32_SMA_Inverter()  {}
    ~ESP32_SMA_Inverter() {}

    // ---- Static callbacks (BT stack calls these from its own tasks) ----
    static void spp_callback(esp_spp_cb_event_t event, esp_spp_cb_param_t *param);
    static void gap_callback(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param);

    // ---- FreeRTOS task entry point ----
    static void btTask(void *pvParameters);

    // ---- Protocol operations (run inside btTask, may block on stream buffer) ----
    bool        isValidSender(const uint8_t expAddr[6], const uint8_t isAddr[6]);
    E_RC        getPacket(const uint8_t expAddr[6], int wait4Command);
    E_RC        getInverterDataCfl(uint32_t command, uint32_t first, uint32_t last);
    E_RC        getInverterData(enum getInverterDataType type);
    bool        getBT_SignalStrength();
    E_RC        initialiseSMAConnection();
    E_RC        logonSMAInverter();
    E_RC        logonSMAInverter(const char *password, const uint8_t user);
    void        logoffSMAInverter();
    void        setInverterTime(bool force = false);
    void        fetchInverterTime();
    E_RC        queryCurrentInverterTime(time_t &invTime, time_t &invLastTimeSet,
                                         uint32_t &tz_dst, uint32_t &timesetCount);

    // ---- Low-level BT I/O (blocking on stream buffer — safe inside btTask) ----
    uint8_t BTgetByte();           // returns 0 and sets readTimeout on timeout
    void    BTsendPacket(uint8_t *btbuffer);
    void    flushRxBuffer();       // drain stream buffer (discard garbage)

    // ---- Packet building helpers ----
    void writePacketHeader(uint8_t *buf, const uint16_t control, const uint8_t *destaddress);
    void writePacket(uint8_t *buf, uint8_t longwords, uint8_t ctrl, uint16_t ctrl2,
                     uint16_t dstSUSyID, uint32_t dstSerial);
    void writePacketTrailer(uint8_t *btbuffer);
    void writePacketLength(uint8_t *buf);
    void writeByte(uint8_t *btbuffer, uint8_t v);
    void write32(uint8_t *btbuffer, uint32_t v);
    void write16(uint8_t *btbuffer, uint16_t v);
    void writeArray(uint8_t *btbuffer, const uint8_t bytes[], int loopcount);
    bool validateChecksum();
    bool isCrcValid(uint8_t lb, uint8_t hb);
    uint32_t getattribute(uint8_t *pcktbuf);

    // ---- FreeRTOS handles ----
    StreamBufferHandle_t rx_stream_buf_  = nullptr;  // SPP RX data → task
    EventGroupHandle_t   bt_event_group_ = nullptr;  // connect/disconnect events
    volatile TaskHandle_t bt_task_handle_ = nullptr; // volatile: read/written across cores

    // ---- SPP state ----
    volatile uint32_t spp_handle_    = 0;
    uint8_t           discovered_scn_ = 1;   // default SCN; overwritten by discovery

    // ---- Cross-task status flags (volatile for visibility across cores) ----
    volatile bool btConnected_        = false;
    volatile bool data_ready_         = false;
    volatile bool task_error_         = false;
    volatile bool stop_task_          = false;
    volatile bool sync_time_requested_  = false;
    volatile bool fetch_time_requested_ = false;
    bool night_mode_active_             = false;
    bool night_mode_time_invalid_logged_ = false;

    // ---- Configuration (written once in setup(), read-only afterward) ----
    uint8_t  smaBTAddress[6];
    char     smaInvPass[12];
    uint32_t delay_values_ms_ = 500;

    // ---- Packet buffers ----
    uint8_t  btrdBuf[COMMBUFSIZE];
    uint8_t  pcktBuf[MAX_PCKT_BUF_SIZE];
    uint16_t pcktBufPos = 0;
    uint16_t pcktBufMax = 0;
    uint8_t  espBTAddress[6];

    // ---- Protocol state ----
    uint16_t pcktID       = 1;
    bool     readTimeout  = false;
    uint16_t fcsChecksum  = 0xffff;
    int32_t  value32      = 0;
    int64_t  value64      = 0;
    uint64_t totalWh      = 0;
    uint64_t totalWh_prev = 0;
    time_t   dateTime     = 0;

    uint32_t btgetByteTimeout = 5000; // ms per-byte receive timeout

    const uint16_t appSUSyID = 125;
    uint32_t       appSerial  = 0;

    char timeBuf[24];
    char charBuf[64];
    const size_t max_buf_size = 64;
    char inverter_version[24];

    const uint8_t sixzeros[6] = {0x00,0x00,0x00,0x00,0x00,0x00};
    const uint8_t sixff[6]    = {0xff,0xff,0xff,0xff,0xff,0xff};
    const char    btPin[5]    = {'0','0','0','0',0};

    // ---- Helpers ----
    void     HexDump(uint8_t *buf, int count, int radix, uint8_t c);
    uint8_t  printUnixTime(char *buf, time_t t);
    uint16_t get_u16(uint8_t *buf);
    uint32_t get_u32(uint8_t *buf);
    uint64_t get_u64(uint8_t *buf);
    void     get_version(uint32_t version, char *inverter_version_);

    // ---- FCS lookup table ----
    const uint16_t fcstab[256] = {
        0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf,
        0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7,
        0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e,
        0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876,
        0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd,
        0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
        0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c,
        0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974,
        0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
        0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3,
        0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a,
        0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
        0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9,
        0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1,
        0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
        0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70,
        0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7,
        0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
        0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036,
        0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e,
        0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
        0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd,
        0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134,
        0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
        0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3,
        0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb,
        0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
        0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a,
        0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1,
        0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
        0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330,
        0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78
    };
};

} // namespace smabluetooth_solar
} // namespace esphome

#endif // ESP32_SMA_INVERTER_H
