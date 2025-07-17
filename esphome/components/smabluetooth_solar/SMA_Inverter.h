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

#include "BluetoothSerial.h"

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
//#define CHAR_BUF_MAX 2048

#define COMMBUFSIZE 2048

//was 512
#define MAX_PCKT_BUF_SIZE COMMBUFSIZE

//unsigned char espBTAddress[6] = {0xE6,0x72,0xCC,0xD1,0x08,0xF0}; // BT address ESP32 F0:08:D1:CC:72:E6
//                        \|E6\|72\|CC\|D1\|08\|F0 };  // BT address  ESP32 F0:08:D1:CC:72:E6
//                        \|d3\|eb\|29\|25\|80\|00 };  // //SMC 6000: 00:80:25:29:eb:d3

#define BTH_L2SIGNATURE 0x656003FF

#define USERGROUP UG_USER



#define NaN_S16 0x8000                          // "Not a Number" representation for int16_t
#define NaN_U16 0xFFFF                          // "Not a Number" representation for uint16_t
#define NaN_S32 (int32_t) 0x80000000            // "Not a Number" representation for int32_t
#define NaN_U32 (uint32_t)0xFFFFFFFF            // "Not a Number" representation for uint32_t
#define NaN_S64 (int64_t) 0x8000000000000000    // "Not a Number" representation for int64_t
#define NaN_U64 (uint64_t)0xFFFFFFFFFFFFFFFF    // "Not a Number" representation for uint64_t

inline const bool is_NaN(const int16_t S16)
{
    return S16 == NaN_S16;
}

inline const bool is_NaN(const uint16_t U16)
{
    return U16 == NaN_U16;
}

inline const bool is_NaN(const int32_t S32)
{
    return S32 == NaN_S32;
}

inline const bool is_NaN(const uint32_t U32)
{
    return U32 == NaN_U32;
}

inline const bool is_NaN(const int64_t S64)
{
    return S64 == NaN_S64;
}

inline const bool is_NaN(const uint64_t U64)
{
    return U64 == NaN_U64;
}
/* // for reference only 
#define PRIu8 "hhu"
#define PRId8 "hhd"
#define PRIx8 "hhx"
#define PRIu16 "hu"
#define PRId16 "hd"
#define PRIx16 "hx"
#define PRIu32 "u"
#define PRId32 "d"
#define PRIx32 "x"
#define PRIu64 "llu" // or possibly "lu"
#define PRId64 "lld" // or possibly "ld"
#define PRIx64 "llx" // or possibly "lx"
*/


enum SMA_DATATYPE
{
    DT_ULONG = 0,
    DT_STATUS = 8,
    DT_STRING = 16,
    DT_FLOAT = 32,
    DT_SLONG = 64
};

enum E_RC {
    E_OK =            0,    // No error
    E_INIT =         -1,    // Unable to initialise
    E_INVPASSW =     -2,    // Invalid password
    E_RETRY =        -3,    // Retry the last action
    E_EOF =          -4,    // End of data
    E_NODATA =       -5,    // no data
    E_OVERFLOW =     -6,    // data buffer overflow
    E_BADARG =       -7,    // Invalid data type
    E_CHKSUM =       -8,    // Invalid checksum
    E_INVRESP =      -9,    // Invalid response
    E_ARCHNODATA =   -10,   // no archive data
};

struct  InverterData {
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
    /*
    int32_t Uac[3];
    int32_t Iac[3];
    int32_t Udc[2];
    int32_t Idc[2];
    int32_t Wdc[2];
    */
    int32_t GridFreq;
    int32_t Eta;
    int32_t InvTemp;
    uint64_t EToday;
    uint64_t ETotal;
  //DayData dayData[288];
    uint64_t dayWh[ARCH_DAY_SIZE];
  //int32_t dayW[ARCH_DAY_SIZE];
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

  /*float Uac[3];
  float Iac[3];
  */
  float InvTemp;

  float Pdc1;
  float Pdc2;

  float Udc1;
  float Udc2;
  float Idc1;
  float Idc2;

  /*
  float Udc[2];
  float Idc[2];
  float Wdc[2];
  */
  float GridFreq;
  float EToday;
  float ETotal;
  //std::string DevStatus;
  bool needsMissingValues = false;
};

enum getInverterDataType {
    EnergyProduction    = 1 << 0,
    SpotDCPower         = 1 << 1,
    SpotDCVoltage       = 1 << 2,
    SpotACPower         = 1 << 3,
    SpotACVoltage       = 1 << 4,
    SpotGridFrequency   = 1 << 5,
    //MaxACPower        = 1 << 6,
    //MaxACPower2       = 1 << 7,
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
    OperationHealth                 = 0x2148,   // *08* Condition (aka INV_STATUS)
    CoolsysTmpNom                   = 0x2377,   // *40* Operating condition temperatures
    DcMsWatt                        = 0x251E,   // *40* DC power input (aka SPOT_PDC1 / SPOT_PDC2)
    MeteringTotWhOut                = 0x2601,   // *00* Total yield (aka SPOT_ETOTAL)
    MeteringDyWhOut                 = 0x2622,   // *00* Day yield (aka SPOT_ETODAY)
    GridMsTotW                      = 0x263F,   // *40* Power (aka SPOT_PACTOT)
    BatChaStt                       = 0x295A,   // *00* Current battery charge status
    OperationHealthSttOk            = 0x411E,   // *00* Nominal power in Ok Mode (deprecated INV_PACMAX1)
    OperationHealthSttWrn           = 0x411F,   // *00* Nominal power in Warning Mode (deprecated INV_PACMAX2)
    OperationHealthSttAlm           = 0x4120,   // *00* Nominal power in Fault Mode (deprecated INV_PACMAX3)
    OperationGriSwStt               = 0x4164,   // *08* Grid relay/contactor (aka INV_GRIDRELAY)
    OperationRmgTms                 = 0x4166,   // *00* Waiting time until feed-in
    DcMsVol                         = 0x451F,   // *40* DC voltage input (aka SPOT_UDC1 / SPOT_UDC2)
    DcMsAmp                         = 0x4521,   // *40* DC current input (aka SPOT_IDC1 / SPOT_IDC2)
    MeteringPvMsTotWhOut            = 0x4623,   // *00* PV generation counter reading
    MeteringGridMsTotWhOut          = 0x4624,   // *00* Grid feed-in counter reading
    MeteringGridMsTotWhIn           = 0x4625,   // *00* Grid reference counter reading
    MeteringCsmpTotWhIn             = 0x4626,   // *00* Meter reading consumption meter
    MeteringGridMsDyWhOut           = 0x4627,   // *00* ?
    MeteringGridMsDyWhIn            = 0x4628,   // *00* ?
    MeteringTotOpTms                = 0x462E,   // *00* Operating time (aka SPOT_OPERTM)
    MeteringTotFeedTms              = 0x462F,   // *00* Feed-in time (aka SPOT_FEEDTM)
    MeteringGriFailTms              = 0x4631,   // *00* Power outage
    MeteringWhIn                    = 0x463A,   // *00* Absorbed energy
    MeteringWhOut                   = 0x463B,   // *00* Released energy
    MeteringPvMsTotWOut             = 0x4635,   // *40* PV power generated
    MeteringGridMsTotWOut           = 0x4636,   // *40* Power grid feed-in
    MeteringGridMsTotWIn            = 0x4637,   // *40* Power grid reference
    MeteringCsmpTotWIn              = 0x4639,   // *40* Consumer power
    GridMsWphsA                     = 0x4640,   // *40* Power L1 (aka SPOT_PAC1)
    GridMsWphsB                     = 0x4641,   // *40* Power L2 (aka SPOT_PAC2)
    GridMsWphsC                     = 0x4642,   // *40* Power L3 (aka SPOT_PAC3)
    GridMsPhVphsA                   = 0x4648,   // *00* Grid voltage phase L1 (aka SPOT_UAC1)
    GridMsPhVphsB                   = 0x4649,   // *00* Grid voltage phase L2 (aka SPOT_UAC2)
    GridMsPhVphsC                   = 0x464A,   // *00* Grid voltage phase L3 (aka SPOT_UAC3)
    GridMsAphsA_1                   = 0x4650,   // *00* Grid current phase L1 (aka SPOT_IAC1)
    GridMsAphsB_1                   = 0x4651,   // *00* Grid current phase L2 (aka SPOT_IAC2)
    GridMsAphsC_1                   = 0x4652,   // *00* Grid current phase L3 (aka SPOT_IAC3)
    GridMsAphsA                     = 0x4653,   // *00* Grid current phase L1 (aka SPOT_IAC1_2)
    GridMsAphsB                     = 0x4654,   // *00* Grid current phase L2 (aka SPOT_IAC2_2)
    GridMsAphsC                     = 0x4655,   // *00* Grid current phase L3 (aka SPOT_IAC3_2)
    GridMsHz                        = 0x4657,   // *00* Grid frequency (aka SPOT_FREQ)
    MeteringSelfCsmpSelfCsmpWh      = 0x46AA,   // *00* Energy consumed internally
    MeteringSelfCsmpActlSelfCsmp    = 0x46AB,   // *00* Current self-consumption
    MeteringSelfCsmpSelfCsmpInc     = 0x46AC,   // *00* Current rise in self-consumption
    MeteringSelfCsmpAbsSelfCsmpInc  = 0x46AD,   // *00* Rise in self-consumption
    MeteringSelfCsmpDySelfCsmpInc   = 0x46AE,   // *00* Rise in self-consumption today
    BatDiagCapacThrpCnt             = 0x491E,   // *40* Number of battery charge throughputs
    BatDiagTotAhIn                  = 0x4926,   // *00* Amp hours counter for battery charge
    BatDiagTotAhOut                 = 0x4927,   // *00* Amp hours counter for battery discharge
    BatTmpVal                       = 0x495B,   // *40* Battery temperature
    BatVol                          = 0x495C,   // *40* Battery voltage
    BatAmp                          = 0x495D,   // *40* Battery current
    NameplateLocation               = 0x821E,   // *10* Device name (aka INV_NAME)
    NameplateMainModel              = 0x821F,   // *08* Device class (aka INV_CLASS)
    NameplateModel                  = 0x8220,   // *08* Device type (aka INV_TYPE)
    NameplateAvalGrpUsr             = 0x8221,   // *  * Unknown
    NameplatePkgRev                 = 0x8234,   // *08* Software package (aka INV_SWVER)
    InverterWLim                    = 0x832A,   // *00* Maximum active power (deprecated INV_PACMAX1_2) (SB3300/SB1200)
    GridMsPhVphsA2B6100             = 0x464B,
    GridMsPhVphsB2C6100             = 0x464C,
    GridMsPhVphsC2A6100             = 0x464D
};

// MultiPacket:
//  Packet-first: Cmd0=08 Byte[18]=0x7E
//  Packet-last : Cmd1=01=cmdcodetowait 
#pragma pack (push, 1)
typedef struct __attribute__ ((packed)) PacketHeader {
    uint8_t   SOP;                // Start Of Packet (0x7E)
    unsigned short  pkLength;
    uint8_t   pkChecksum;
    uint8_t   SourceAddr[6];      // SMA Inverter Address
    uint8_t   DestinationAddr[6]; // Local BT Address
    unsigned short  command;
} L1Hdr;
#pragma pack(pop)

/*
*/
class ESP32BluetoothSerial : public BluetoothSerial {
  public:
    ESP32BluetoothSerial() {

    };
    ~ESP32BluetoothSerial() {
      
    };
};


class ESP32_SMA_Inverter  {
  public: 
    
    // Static method to get the instance of the class.
    static ESP32_SMA_Inverter* getInstance() {
        // This guarantees that the instance is created only once.
        static ESP32_SMA_Inverter instance;
        return &instance;
    }

    // Delete the copy constructor and the assignment operator to prevent cloning.
    ESP32_SMA_Inverter(const ESP32_SMA_Inverter&) = delete;
    ESP32_SMA_Inverter& operator=(const ESP32_SMA_Inverter&) = delete;

    //setup the variables for the inverter
    void setup(std::string mac, std::string pw);


    //Prototypes
    bool isValidSender(uint8_t expAddr[6], uint8_t isAddr[6]);
    E_RC getPacket(uint8_t expAddr[6], int wait4Command);
    void writePacketHeader(uint8_t *buf, const uint16_t control, const uint8_t *destaddress);
    E_RC getInverterDataCfl(uint32_t command, uint32_t first, uint32_t last);
    E_RC getInverterData(enum getInverterDataType type);
    bool getBT_SignalStrength();
    E_RC initialiseSMAConnection();
    E_RC logonSMAInverter();
    E_RC logonSMAInverter(const char *password, const uint8_t user);
    void logoffSMAInverter();

    E_RC ArchiveDayData(time_t startTime);

    bool connect();
    bool connect(uint8_t remoteAddress[]);
    bool disconnect();


    //Prototypes
    uint8_t BTgetByte();
    void BTsendPacket( uint8_t *btbuffer );
    void writeByte(uint8_t *btbuffer, uint8_t v);
    void write32(uint8_t *btbuffer, uint32_t v);
    void write16(uint8_t *btbuffer, uint16_t v);
    void writeArray(uint8_t *btbuffer, const uint8_t bytes[], int loopcount);
    void writePacket(uint8_t *buf, uint8_t longwords, uint8_t ctrl, uint16_t ctrl2, uint16_t dstSUSyID, uint32_t dstSerial);
    void writePacketTrailer(uint8_t *btbuffer);
    void writePacketLength(uint8_t *buf);
    bool validateChecksum();
    bool isCrcValid(uint8_t lb, uint8_t hb);

    uint32_t getattribute(uint8_t *pcktbuf);

    void initPcktID() {
      setPcktID(1);
    }

    void setPcktID(uint8_t pPcktID) {
        pcktID = pPcktID;
    };

    bool isBtConnected() {
      return btConnected;
    }

    bool begin(String localName, bool isMaster);

    InverterData invData = InverterData();
    DisplayData dispData = DisplayData();


  private: 
   // Private constructor to prevent instantiation from outside the class.
    ESP32_SMA_Inverter()  {
    }
    // Destructor (optional, as the singleton instance will be destroyed when the program ends).
    ~ESP32_SMA_Inverter() {}
    void loopNotification();

    BluetoothSerial serialBT = ESP32BluetoothSerial();
    
    uint8_t  btrdBuf[COMMBUFSIZE];    
    uint16_t pcktBufMax = 0; // max. used size of PcktBuf
    uint8_t  espBTAddress[6]; // is retrieved from BT packet

    bool btConnected = false;

    char timeBuf[24]= {0};
    const size_t timeBufLen = 24;
    char charBuf[256]= {0};
    const size_t charBufLen = 256;
    size_t  charLen = 0;


    char smaInvPass[12]= {0};  
    uint8_t smaBTAddress[6]; // SMA bluetooth address

    const uint16_t appSUSyID = 125;
    uint32_t appSerial = 0 ;

    void HexDump(uint8_t *buf, int count, int radix, uint8_t c);
    uint8_t printUnixTime(char *buf, time_t t);
    uint16_t get_u16(uint8_t *buf);
    uint32_t get_u32(uint8_t *buf);
    uint64_t get_u64(uint8_t *buf);
    std::string get_version(uint32_t);

    int32_t  value32 = 0;
    int64_t  value64 = 0;
    uint64_t totalWh = 0;
    uint64_t totalWh_prev = 0;
    time_t   dateTime = 0;

    uint32_t btgetByteTimeout = 5000;

   public:
    uint32_t getBtgetByteTimeout() const { return btgetByteTimeout; }
    void setBtgetByteTimeout(uint32_t btgetByteTimeout) { ESP32_SMA_Inverter::btgetByteTimeout = btgetByteTimeout; }

   private:
    //from SMA_Bluetooth
        uint8_t  pcktBuf[MAX_PCKT_BUF_SIZE];
        uint16_t pcktBufPos = 0;
        uint16_t pcktID = 1;
        bool readTimeout = false;
        uint16_t fcsChecksum=0xffff;
        uint8_t sixzeros[6]= {0x00,0x00,0x00,0x00,0x00,0x00};
        uint8_t sixff[6]   = {0xff,0xff,0xff,0xff,0xff,0xff};
        const char btPin[5] = {'0','0','0','0',0}; // BT pin Always 0000. (not login passcode!)

        PROGMEM prog_uint16_t  fcstab[256]  = {
        0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf,0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7,
        0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e,0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876,
        0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd,0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
        0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c,0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974,
        0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3,
        0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a,0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
        0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9,0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1,
        0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70,
        0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7,0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
        0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036,0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e,
        0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd,
        0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134,0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
        0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3,0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb,
        0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a,
        0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1,0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
        0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330,0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78
        };





};

}
}

#endif