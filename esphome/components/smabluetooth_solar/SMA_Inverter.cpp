/* MIT License

Copyright (c) 2022 Lupo135
Copyright (c) 2023 darrylb123

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

static const char *const INVTAG = "smainverter";

void ESP32_SMA_Inverter::setup(std::string mac, std::string pw) {
// Convert the MAC address string to binary
    ESP_LOGW(INVTAG, "setup inverter to:  %s ", mac.c_str());
    sscanf(mac.c_str(), "%2hhx:%2hhx:%2hhx:%2hhx:%2hhx:%2hhx", 
            &smaBTAddress[0], &smaBTAddress[1], &smaBTAddress[2], &smaBTAddress[3], &smaBTAddress[4], &smaBTAddress[5]);
    // Zero the array, all unused butes must be 0
    for(int i = 0; i < sizeof(smaInvPass);i++)
       smaInvPass[i] ='\0';
    strlcpy(smaInvPass , pw.c_str(), sizeof(smaInvPass));

    invData.SUSyID = 0x7d;
    invData.Serial = 0;

    // reverse inverter BT address
    for(uint8_t i=0; i<6; i++) invData.BTAddress[i] = smaBTAddress[5-i];
    ESP_LOGD("invData.BTAddress: %02X:%02X:%02X:%02X:%02X:%02X\n",
                invData.BTAddress[5], invData.BTAddress[4], invData.BTAddress[3],
                invData.BTAddress[2], invData.BTAddress[1], invData.BTAddress[0]);


}

bool ESP32_SMA_Inverter::begin(String localName, bool isMaster) {
    ESP_LOGD("serialBT begin %s ", localName.c_str());
    boolean bOk = false;
    bOk = serialBT.begin(localName, isMaster);   // "true" creates this device as a BT Master.
    bOk &= serialBT.setPin(&btPin[0]); 
    return bOk;
}

bool ESP32_SMA_Inverter::connect() {
  return connect(smaBTAddress);
}

bool ESP32_SMA_Inverter::connect(uint8_t remoteAddress[]) {
  ESP_LOGD(INVTAG, "connecting %d", );
  bool bGotConnected = serialBT.connect(remoteAddress);
  btConnected = bGotConnected;
  return bGotConnected; 
}

bool ESP32_SMA_Inverter::disconnect() {
  bool bGotDisconnected = serialBT.disconnect();
  btConnected = false;
  return bGotDisconnected;
}

//serialBT.disconnect();

bool ESP32_SMA_Inverter::isValidSender(uint8_t expAddr[6], uint8_t isAddr[6]) {
  for (int i = 0; i < 6; i++)
    if ((isAddr[i] != expAddr[i]) && (expAddr[i] != 0xFF)) {
      
      ESP_LOGV(INVTAG, "\nShoud-Addr: %02X %02X %02X %02X %02X %02X\n   Is-Addr: %02X %02X %02X %02X %02X %02X\n",
        expAddr[5], expAddr[4], expAddr[3], expAddr[2], expAddr[1], expAddr[5],
         isAddr[5],  isAddr[4],  isAddr[3],  isAddr[2],  isAddr[1],  isAddr[5]);
         
        return false;
    }
  return true;
}

// ----------------------------------------------------------------------------------------------
//unsigned int readBtPacket(int index, unsigned int cmdcodetowait) {
E_RC ESP32_SMA_Inverter::getPacket(uint8_t expAddr[6], int wait4Command) {
  //extern BluetoothSerial serialBT;
  ESP_LOGV(INVTAG, "writing");
  ESP_LOGV(INVTAG, "getPacket cmd=0x%04x\n", wait4Command);

  //extern bool readTimeout;
  int index = 0;
  bool hasL2pckt = false;
  E_RC rc = E_OK; 
  L1Hdr *pL1Hdr = (L1Hdr *)&btrdBuf[0];
  do {
    // read L1Hdr
    uint8_t rdCnt=0;
    for (rdCnt=0;rdCnt<18;rdCnt++) {
      btrdBuf[rdCnt]= BTgetByte();
      if (readTimeout)  break;
    }
    ESP_LOGD(INVTAG, "\nL1 Rec=%d bytes pkL=0x%04x=%d Cmd=0x%04x\n",
        rdCnt, pL1Hdr->pkLength, pL1Hdr->pkLength, pL1Hdr->command);

    if (rdCnt<17) {
      ESP_LOGV(INVTAG, "L1<18=%d bytes", rdCnt);
      #if (DEBUG_SMA > 2)
      HexDump(BTrdBuf, rdCnt, 10, 'R');
      #endif
      return E_NODATA;
    }
    // Validate L1 header
    if (!((btrdBuf[0] ^ btrdBuf[1] ^ btrdBuf[2]) == btrdBuf[3])) {
      ESP_LOGW(INVTAG, "\nWrong L1 CRC!!" );
    }

    if (pL1Hdr->pkLength > sizeof(L1Hdr)) { // more bytes to read
      for (rdCnt=18; rdCnt<pL1Hdr->pkLength; rdCnt++) {
        btrdBuf[rdCnt]= BTgetByte();
        if (readTimeout) break;
      }
      ESP_LOGW(INVTAG, "L2 Rec=%d bytes", rdCnt-18);
      #if (DEBUG_SMA > 2)
      HexDump(BTrdBuf, rdCnt, 10, 'R');
      #endif

      //Check if data is coming from the right inverter
      if (isValidSender(expAddr, pL1Hdr->SourceAddr)) {
        rc = E_OK;

        ESP_LOGD(INVTAG, "HasL2pckt: 0x7E?=0x%02X 0x656003FF?=0x%08X\n", btrdBuf[18], get_u32(btrdBuf+19));
        if ((hasL2pckt == 0) && (btrdBuf[18] == 0x7E) && (get_u32(btrdBuf+19) == 0x656003FF)) {
          hasL2pckt = true;
        }

        if (hasL2pckt) {
          //Copy BTrdBuf to pcktBuf
          bool escNext = false;

          for (int i=sizeof(L1Hdr); i<pL1Hdr->pkLength; i++) {
            pcktBuf[index] = btrdBuf[i];
            //Keep 1st byte raw unescaped 0x7E
            if (escNext == true) {
              pcktBuf[index] ^= 0x20;
              escNext = false;
              index++;
            } else {
              if (pcktBuf[index] == 0x7D)
                escNext = true; //Throw away the 0x7d byte
              else
                index++;
            }
            if (index >= MAX_PCKT_BUF_SIZE) {
              ESP_LOGE(INVTAG, "pcktBuf overflow! (%d)\n", index);
            }
          }
          pcktBufPos = index;
        } else {  // no L2pckt
          memcpy(pcktBuf, btrdBuf, rdCnt);
          pcktBufPos = rdCnt;
        }
      } else { // isValidSender()
          rc = E_RETRY;
      }
    } else {  // L1 only
    #if (DEBUG_SMA > 2)
      HexDump(BTrdBuf, rdCnt, 10, 'R');
    #endif
      //Check if data is coming from the right inverter
      if (isValidSender(expAddr, pL1Hdr->SourceAddr)) {
          rc = E_OK;

          memcpy(pcktBuf, btrdBuf, rdCnt);
          pcktBufPos = rdCnt;
      } else { // isValidSender()
          rc = E_RETRY;
      }
    }
    if (btrdBuf[0] != '\x7e') { 
       serialBT.flush();
       ESP_LOGD(INVTAG, "\nCommBuf[0]!=0x7e -> BT-flush");
    }
  } while (((pL1Hdr->command != wait4Command) || (rc == E_RETRY)) && (0xFF != wait4Command));

  if ((rc == E_OK) ) {
  #if (DEBUG_SMA > 1)
    ESP_LOGD(INVTAG, "<<<====Rd Content of pcktBuf =======>>>");
    HexDump(pcktBuf, pcktBufPos, 10, 'P');
    ESP_LOGD(INVTAG, "==>>>");
  #endif
  }

  if (pcktBufPos > pcktBufMax) {
    pcktBufMax = pcktBufPos;
    ESP_LOGD(INVTAG, "pcktBufMax is now %d bytes\n", pcktBufMax);
  }

  return rc;
}

// *************************************************

void ESP32_SMA_Inverter::writePacketHeader(uint8_t *buf, const uint16_t control, const uint8_t *destaddress) {
  //extern uint16_t fcsChecksum;


    pcktBufPos = 0;

    fcsChecksum = 0xFFFF;
    buf[pcktBufPos++] = 0x7E;
    buf[pcktBufPos++] = 0;  //placeholder for len1
    buf[pcktBufPos++] = 0;  //placeholder for len2
    buf[pcktBufPos++] = 0;  //placeholder for checksum
    int i;
    for(i = 0; i < 6; i++) buf[pcktBufPos++] = espBTAddress[i];
    for(i = 0; i < 6; i++) buf[pcktBufPos++] = destaddress[i];

    buf[pcktBufPos++] = (uint8_t)(control & 0xFF);
    buf[pcktBufPos++] = (uint8_t)(control >> 8);
}
// ***********************************************
E_RC ESP32_SMA_Inverter::getInverterDataCfl(uint32_t command, uint32_t first, uint32_t last) {
  //extern uint8_t sixff[6];
    pcktID++;
    writePacketHeader(pcktBuf, 0x01, sixff); //addr_unknown);
    //if (invData.SUSyID == SID_SB240)
    //writePacket(pcktBuf, 0x09, 0xE0, 0, invData.SUSyID, invData.Serial);
    //else
    writePacket(pcktBuf, 0x09, 0xA0, 0, invData.SUSyID, invData.Serial);
    write32(pcktBuf, command);
    write32(pcktBuf, first);
    write32(pcktBuf, last);
    writePacketTrailer(pcktBuf);
    writePacketLength(pcktBuf);

    BTsendPacket(pcktBuf);
    int string[3] = { 0,0,0 }; //String number count

    uint8_t pcktcount = 0;
    bool  validPcktID = false;
    do {
    do {
      invData.status = getPacket(invData.BTAddress, 0x0001);
   
      if (invData.status != E_OK) return invData.status;
      if (validateChecksum()) {
        if ((invData.status = (E_RC)get_u16(pcktBuf + 23)) != E_OK) {
          ESP_LOGD(INVTAG, "Packet status: 0x%02X\n", invData.status);
          return invData.status;
        }
        // *** analyze received data ***
        pcktcount = get_u16(pcktBuf + 25);
        uint16_t rcvpcktID = get_u16(pcktBuf + 27) & 0x7FFF;
        if (pcktID == rcvpcktID) {
          if ((get_u16(pcktBuf + 15) == invData.SUSyID) 
            && (get_u32(pcktBuf + 17) == invData.Serial)) {
            validPcktID = true;
            value32 = 0;
            value64 = 0;
            uint16_t recordsize = 4 * ((uint32_t)pcktBuf[5] - 9) / (get_u32(pcktBuf + 37) - get_u32(pcktBuf + 33) + 1);
            ESP_LOGD(INVTAG, "\npcktID=0x%04x recsize=%d BufPos=%d pcktCnt=%04x", 
                            rcvpcktID,   recordsize, pcktBufPos, pcktcount);
            for (uint16_t ii = 41; ii < pcktBufPos - 3; ii += recordsize) {
              uint8_t *recptr = pcktBuf + ii;
              uint32_t code = get_u32(recptr);
              //LriDef lri = (LriDef)(code & 0x00FFFF00);
              uint16_t lri = (code & 0x00FFFF00) >> 8;
              uint32_t cls = code & 0xFF;
              uint8_t dataType = code >> 24;
              time_t datetime = (time_t)get_u32(recptr + 4);
              ESP_LOGV(INVTAG, "\nlri=0x%04x cls=0x%08X dataType=0x%02x",lri, cls, dataType);
       
              if (recordsize == 16) {
                value64 = get_u64(recptr + 8);
                ESP_LOGV(INVTAG, "\nvalue64=%d=0x%08x",value64, value64);
       
                  //if (is_NaN(value64) || is_NaN((uint64_t)value64)) value64 = 0;
              } else if ((dataType != 16) && (dataType != 8)) { // ((dataType != DT_STRING) && (dataType != DT_STATUS)) {
                value32 = get_u32(recptr + 16);
                if ( value32 < 0) value32 = 0;
                ESP_LOGV(INVTAG, "\nvalue32=%d=0x%08x",value32, value32);
              }
              switch (lri) {
              case GridMsTotW: //SPOT_PACTOT
                  //This function gives us the time when the inverter was switched off
                  invData.LastTime = datetime;
                  invData.Pac = value32;
                  dispData.Pac = tokW(value32);
                  //debug_watt("SPOT_PACTOT", value32, datetime);
                  printUnixTime(timeBuf, datetime);
                  ESP_LOGI(INVTAG, "Pac %15.3f kW %x \n GMT:%s \n", tokW(value32),value32, timeBuf);
                  break;
       
              case GridMsWphsA: //SPOT_PAC1
                  invData.Pmax = value32;
                  dispData.Pmax = tokW(value32);
                  //debug_watt("SPOT_PAC1", value32, datetime);
                  ESP_LOGI(INVTAG, "Pmax %14.2f kW \n", tokW(value32));
                  //printUnixTime(timeBuf, datetime);
                  break;
       
              case GridMsPhVphsA: //SPOT_UAC1
                  invData.Uac[0] = value32;
                  dispData.Uac[0] = toVolt(value32);
                  //debug_volt("SPOT_UAC1", value32, datetime);
                  ESP_LOGI(INVTAG, "UacA %15.2f V  \n", toVolt(value32));
                  //printUnixTime(timeBuf, datetime);
                  break;
              case GridMsPhVphsB: //SPOT_UAC2
                  invData.Uac[1] = value32;
                  dispData.Uac[1] = toVolt(value32);
                  //debug_volt("SPOT_UAC1", value32, datetime);
                  ESP_LOGI(INVTAG, "UacB %15.2f V  \n", toVolt(value32));
                  //printUnixTime(timeBuf, datetime);
                  break;     
                case GridMsPhVphsC: //SPOT_UAC2
                  invData.Uac[2] = value32;
                  dispData.Uac[2] = toVolt(value32);
                  //debug_volt("SPOT_UAC1", value32, datetime);
                  ESP_LOGI(INVTAG, "UacC %15.2f V  \n", toVolt(value32));
                  //printUnixTime(timeBuf, datetime);
                  break;       
              case GridMsAphsA_1: //SPOT_IAC1
              case GridMsAphsA:
                  invData.Iac[0] = value32;
                  dispData.Iac[0] = toAmp(value32);
                  //debug_amp("SPOT_IAC1", value32, datetime);
                  ESP_LOGI(INVTAG, "IacA %15.2f A  \n", toAmp(value32));
                  //printUnixTime(timeBuf, datetime);
                  break;
              case GridMsAphsB_1: //SPOT_IAC1
              case GridMsAphsB:
                  invData.Iac[1] = value32;
                  dispData.Iac[1] = toAmp(value32);
                  //debug_amp("SPOT_IAC1", value32, datetime);
                  ESP_LOGI(INVTAG, "IacB %15.2f A  \n", toAmp(value32));
                  //printUnixTime(timeBuf, datetime);
                  break;
              case GridMsAphsC_1: //SPOT_IAC1
              case GridMsAphsC:
                  invData.Iac[2] = value32;
                  dispData.Iac[2] = toAmp(value32);
                  //debug_amp("SPOT_IAC1", value32, datetime);
                  ESP_LOGI(INVTAG, "IacB %15.2f A  \n", toAmp(value32));
                  //printUnixTime(timeBuf, datetime);
                  break;
       
              case GridMsHz: //SPOT_FREQ
                  invData.Freq = value32;
                  dispData.Freq = toHz(value32);
                  ESP_LOGI(INVTAG, "Freq %14.2f Hz \n", toHz(value32));
                  //printUnixTime(timeBuf, datetime);
                  break;
       
              case DcMsWatt: //SPOT_PDC1 / SPOT_PDC2
                  invData.Wdc[string[0]] = value32;
                  dispData.Wdc[string[0]++] = tokW(value32);
                  ESP_LOGI(INVTAG, "PDC %15.2f kW \n", tokW(value32));
                  //printUnixTime(timeBuf, datetime);
                  break;
       
              case DcMsVol: //SPOT_UDC1 / SPOT_UDC2
                  ESP_LOGI(INVTAG, "Udc %15.2f V (%d) \n", toVolt(value32),string[1]);
                  invData.Udc[string[1]] = value32;
                  dispData.Udc[string[1]++] = toVolt(value32);
                  
                  //printUnixTime(timeBuf, datetime);
                  break;
       
              case DcMsAmp: //SPOT_IDC1 / SPOT_IDC2
                  ESP_LOGI(INVTAG, "Idc %15.2f A (%d)\n", toAmp(value32),string[2]);
                  invData.Idc[string[2]] = value32;
                  dispData.Idc[string[2]++] = toAmp(value32);

                  //printUnixTime(timeBuf, datetime);
                  /* if ((invData.Udc[0]!=0) && (invData.Idc[0] != 0))
                    invData.Eta = ((uint64_t)invData.Uac * (uint64_t)invData.Iac * 10000) /
                                    ((uint64_t)invData.Udc[0] * (uint64_t)invData.Idc[0] );
                  else invData.Eta = 0;
                  ESP_LOGI(INVTAG, "Efficiency %8.2f %%\n", toPercent(invData.Eta)); */
                  break;
       
              case MeteringDyWhOut: //SPOT_ETODAY
                  //This function gives us the current inverter time
                  //invData.InverterDatetime = datetime;
                  invData.EToday = value64;
                  dispData.EToday = tokWh(value64);
                  //debug_kwh("SPOT_ETODAY", value64, datetime);
                  ESP_LOGI(INVTAG, "E-Today %11.3f kWh\n", tokWh(value64));
                  //printUnixTime(timeBuf, datetime);
                  break;
       
              case MeteringTotWhOut: //SPOT_ETOTAL
                  //In case SPOT_ETODAY missing, this function gives us inverter time (eg: SUNNY TRIPOWER 6.0)
                  //invData.InverterDatetime = datetime;
                  invData.ETotal = value64;
                  dispData.ETotal = tokWh(value64);
                  //debug_kwh("SPOT_ETOTAL", value64, datetime);
                  ESP_LOGI(INVTAG, "E-Total %11.3f kWh\n", tokWh(value64));
                  //printUnixTime(timeBuf, datetime);
                  break;
       
              case MeteringTotOpTms: //SPOT_OPERTM
                  invData.OperationTime = value64;
                  //debug_hour("SPOT_OPERTM", value64, datetime);
                  ESP_LOGI(INVTAG, "OperTime  %7.3f h \n", toHour(value64));
                  //printUnixTime(timeBuf, datetime);
                  break;
       
              case MeteringTotFeedTms: //SPOT_FEEDTM
                  invData.FeedInTime = value64;
                  //debug_hour("SPOT_FEEDTM", value64, datetime);
                  ESP_LOGI(INVTAG, "FeedTime  %7.3f h  \n", toHour(value64));
                  //printUnixTime(timeBuf, datetime);
                  break;
       
              case CoolsysTmpNom:
                  invData.InvTemp = value32;
                  dispData.InvTemp = toTemp(value32);
                  ESP_LOGI(INVTAG, "Temp.     %7.3f C \n", toTemp(value32));
                  break;
              case OperationHealth:
                  invData.DevStatus = value32;
                  ESP_LOGI(INVTAG, "Device Status:    %d  \n", value32);
                  break;
              case OperationGriSwStt:
                  invData.GridRelay = value32;
                  ESP_LOGI(INVTAG, "Grid Relay:    %d  \n", value32);
                  break;
              case MeteringGridMsTotWOut:
                  //invData.MeteringGridMsTotWOut = value32;
                  break;
              case MeteringGridMsTotWIn:
                  //invData.MeteringGridMsTotWIn = value32;
                  break;
              default:

                ESP_LOGI(INVTAG, "Caught: %x %d\n",lri,value32);
              }
            } //for
          } else {
            ESP_LOGW(INVTAG, "*** Wrong SUSyID=%04x=%04x Serial=%08x=%08x", 
                 get_u16(pcktBuf + 15), invData.SUSyID, get_u32(pcktBuf + 17),invData.Serial);
          }
        } else {  // wrong PacketID
          ESP_LOGW(INVTAG, "PacketID mismatch: exp=0x%04X is=0x%04X\n", pcktID, rcvpcktID);
          validPcktID = false;
          pcktcount = 0;
        }
      } else { // invalid Checksum
        invData.status = E_CHKSUM;
        return invData.status;
      }
   } while (pcktcount > 0);
   } while (!validPcktID);
   return invData.status;
}
// ***********************************************
E_RC ESP32_SMA_Inverter::getInverterData(enum getInverterDataType type) {
  E_RC rc = E_OK;
  uint32_t command;
  uint32_t first;
  uint32_t last;

  switch (type) {
  case EnergyProduction:
      ESP_LOGD(INVTAG, "\n*** EnergyProduction ***");
      // SPOT_ETODAY, SPOT_ETOTAL
      command = 0x54000200;
      first = 0x00260100;
      last = 0x002622FF;
      break;

  case SpotDCPower:
      ESP_LOGD(INVTAG, "\n*** SpotDCPower ***");
      // SPOT_PDC1, SPOT_PDC2
      command = 0x53800200;
      first = 0x00251E00;
      last = 0x00251EFF;
      break;

  case SpotDCVoltage:
      ESP_LOGD(INVTAG, "\n*** SpotDCVoltage ***");
      // SPOT_UDC1, SPOT_UDC2, SPOT_IDC1, SPOT_IDC2
      command = 0x53800200;
      first = 0x00451F00;
      last = 0x004521FF;
      break;

  case SpotACPower:
      ESP_LOGD(INVTAG, "\n*** SpotACPower ***");
      // SPOT_PAC1, SPOT_PAC2, SPOT_PAC3
      command = 0x51000200;
      first = 0x00464000;
      last = 0x004642FF;
      break;

  case SpotACVoltage:
      ESP_LOGD(INVTAG, "\n*** SpotACVoltage ***");
      // SPOT_UAC1, SPOT_UAC2, SPOT_UAC3, SPOT_IAC1, SPOT_IAC2, SPOT_IAC3
      command = 0x51000200;
      first = 0x00464800;
      last = 0x004655FF;
      break;

  case SpotGridFrequency:
      ESP_LOGD(INVTAG, "\n*** SpotGridFrequency ***");
      // SPOT_FREQ
      command = 0x51000200;
      first = 0x00465700;
      last = 0x004657FF;
      break;

  case SpotACTotalPower:
      ESP_LOGD(INVTAG, "\n*** SpotACTotalPower ***");
      // SPOT_PACTOT
      command = 0x51000200;
      first = 0x00263F00;
      last = 0x00263FFF;
      break;

  case TypeLabel:
      ESP_LOGD(INVTAG, "\n*** TypeLabel ***");
      // INV_NAME, INV_TYPE, INV_CLASS
      command = 0x58000200;
      first = 0x00821E00;
      last = 0x008220FF;
      break;

  case SoftwareVersion:
      ESP_LOGD(INVTAG, "\n*** SoftwareVersion ***");
      // INV_SWVERSION
      command = 0x58000200;
      first = 0x00823400;
      last = 0x008234FF;
      break;

  case DeviceStatus:
      ESP_LOGD(INVTAG, "\n*** DeviceStatus ***");
      // INV_STATUS
      command = 0x51800200;
      first = 0x00214800;
      last = 0x002148FF;
      break;

  case GridRelayStatus:
      ESP_LOGD(INVTAG, "\n*** GridRelayStatus ***");
      // INV_GRIDRELAY
      command = 0x51800200;
      first = 0x00416400;
      last = 0x004164FF;
      break;

  case OperationTime:
      ESP_LOGD(INVTAG, "\n*** OperationTime ***");
      // SPOT_OPERTM, SPOT_FEEDTM
      command = 0x54000200;
      first = 0x00462E00;
      last = 0x00462FFF;
      break;

  case InverterTemp:
      ESP_LOGD(INVTAG, "\n*** InverterTemp ***");
      command = 0x52000200;
      first = 0x00237700;
      last = 0x002377FF;
      break;

  case MeteringGridMsTotW:
      ESP_LOGD(INVTAG, "\n*** MeteringGridMsTotW ***");
      command = 0x51000200;
      first = 0x00463600;
      last = 0x004637FF;
      break;

  default:
      ESP_LOGW(INVTAG, "\nInvalid getInverterDataType!!");
      return E_BADARG;
  };

  // Request data from inverter
  for (uint8_t retries=1;; retries++) {
    rc = getInverterDataCfl(command, first, last);
    if (rc != E_OK) {
      if (retries>1) {
         return rc;
      }
      ESP_LOGI(INVTAG, "\nRetrying.%d",retries);
    } else {
      break;
    } 
  }

  return rc;
}

//-------------------------------------------------------------------------
bool ESP32_SMA_Inverter::getBT_SignalStrength() {
  ESP_LOGI(INVTAG, "\n\n*** SignalStrength ***");
  writePacketHeader(pcktBuf, 0x03, invData.BTAddress);
  writeByte(pcktBuf,0x05);
  writeByte(pcktBuf,0x00);
  writePacketLength(pcktBuf);
  BTsendPacket(pcktBuf);

  getPacket(invData.BTAddress, 4);
  dispData.BTSigStrength = ((float)btrdBuf[22] * 100.0f / 255.0f);
  ESP_LOGI(INVTAG, "BT-Signal %9.1f %%", dispData.BTSigStrength );
  return true;
}

//-------------------------------------------------------------------------
E_RC ESP32_SMA_Inverter::initialiseSMAConnection() {
  //extern uint8_t sixff[6];
  ESP_LOGI(INVTAG, " -> Initialize");
  getPacket(invData.BTAddress, 2); // 1. Receive
  invData.NetID = pcktBuf[22];
  ESP_LOGI(INVTAG, "SMA netID=%02X\n", invData.NetID);
  writePacketHeader(pcktBuf, 0x02, invData.BTAddress);
  write32(pcktBuf, 0x00700400);
  writeByte(pcktBuf, invData.NetID);
  write32(pcktBuf, 0);
  write32(pcktBuf, 1);
  writePacketLength(pcktBuf);

  BTsendPacket(pcktBuf);             // 1. Reply
  getPacket(invData.BTAddress, 5); // 2. Receive

  // Extract ESP32 BT address
  memcpy(espBTAddress, pcktBuf+26,6); 
  ESP_LOGW(INVTAG, "ESP32 BT address: %02X:%02X:%02X:%02X:%02X:%02X\n",
             espBTAddress[5], espBTAddress[4], espBTAddress[3],
             espBTAddress[2], espBTAddress[1], espBTAddress[0]);

  pcktID++;
  writePacketHeader(pcktBuf, 0x01, sixff); //addr_unknown);
  writePacket(pcktBuf, 0x09, 0xA0, 0, 0xFFFF, 0xFFFFFFFF); // anySUSyID, anySerial);
  write32(pcktBuf, 0x00000200);
  write32(pcktBuf, 0);
  write32(pcktBuf, 0);
  writePacketTrailer(pcktBuf);
  writePacketLength(pcktBuf);

  BTsendPacket(pcktBuf);             // 2. Reply
  if (getPacket(invData.BTAddress, 1) != E_OK) // 3. Receive
    return E_INIT;

  if (!validateChecksum())
    return E_CHKSUM;

  invData.Serial = get_u32(pcktBuf + 57);
  ESP_LOGW(INVTAG, "Serial Nr: %lu\n", invData.Serial);
  return E_OK;
}

// log off SMA Inverter - adapted from SBFspot Open Source Project (SBFspot.cpp) by mrtoy-me
void ESP32_SMA_Inverter::logoffSMAInverter()
{
  //extern uint8_t sixff[6];
  pcktID++;
  writePacketHeader(pcktBuf, 0x01, sixff);
  writePacket(pcktBuf, 0x08, 0xA0, 0x0300, 0xFFFF, 0xFFFFFFFF);
  write32(pcktBuf, 0xFFFD010E);
  write32(pcktBuf, 0xFFFFFFFF);
  writePacketTrailer(pcktBuf);
  writePacketLength(pcktBuf);
  BTsendPacket(pcktBuf);
  return;
}

E_RC ESP32_SMA_Inverter::logonSMAInverter() {
  return logonSMAInverter(smaInvPass, USERGROUP);
}

// **** Logon SMA **********
E_RC ESP32_SMA_Inverter::logonSMAInverter(const char *password, const uint8_t user) {
  //extern uint8_t sixff[6];
  #define MAX_PWLENGTH 12
  uint8_t pw[MAX_PWLENGTH];
  E_RC rc = E_OK;

  // Encode password
  uint8_t encChar = (user == USERGROUP)? 0x88:0xBB;
  uint8_t idx;
  for (idx = 0; (password[idx] != 0) && (idx < sizeof(pw)); idx++)
    pw[idx] = password[idx] + encChar;
  for (; idx < MAX_PWLENGTH; idx++) pw[idx] = encChar;

    bool validPcktID = false;
    time_t now;
    pcktID++;
    now = time(NULL);
    writePacketHeader(pcktBuf, 0x01, sixff);
    writePacket(pcktBuf, 0x0E, 0xA0, 0x0100, 0xFFFF, 0xFFFFFFFF); // anySUSyID, anySerial);
    write32(pcktBuf, 0xFFFD040C);
    write32(pcktBuf, user); //userGroup);    // User / Installer
    write32(pcktBuf, 0x00000384); // Timeout = 900sec ?
    write32(pcktBuf, now);
    write32(pcktBuf, 0);
    writeArray(pcktBuf, pw, sizeof(pw));
    writePacketTrailer(pcktBuf);
    writePacketLength(pcktBuf);

    BTsendPacket(pcktBuf);
    if ((rc = getPacket(sixff, 1)) != E_OK) return rc;

    if (!validateChecksum()) return E_CHKSUM;
    uint16_t rcvpcktID = get_u16(pcktBuf+27) & 0x7FFF;
    if ((pcktID == rcvpcktID) && (get_u32(pcktBuf + 41) == now)) {
      invData.SUSyID = get_u16(pcktBuf + 15);
      invData.Serial = get_u32(pcktBuf + 17);
      ESP_LOGV(INVTAG, "Set:->SUSyID=0x%02X ->Serial=0x%02X ", invData.SUSyID, invData.Serial);
      validPcktID = true;
      uint8_t retcode = get_u16(pcktBuf + 23);
      // switch (retcode) {
      //     case 0: rc = E_OK; break;
      //     case 0x0100: rc = E_INVPASSW; break;
      //     default: rc = (E_RC)retcode; break;
      // }
    } else { 
      ESP_LOGW(INVTAG, "Unexpected response  %02X:%02X:%02X:%02X:%02X:%02X pcktID=0x%04X rcvpcktID=0x%04X now=0x%04X", 
                   btrdBuf[9], btrdBuf[8], btrdBuf[7], btrdBuf[6], btrdBuf[5], btrdBuf[4],
                   pcktID, rcvpcktID, now);
      rc = E_INVRESP;
    }
    return rc;
}

/* 
// ******* Archive Day Data **********
E_RC ArchiveDayData(time_t startTime) {
  DEBUG2_PRINT("\n*** ArchiveDayData ***");
  printUnixTime(timeBuf, startTime); DEBUG2_PRINTF("\nStartTime0 GMT:%s", timeBuf);
  // set time to begin of day
  uint8_t minutes = (startTime/60) % 60;
  uint8_t hours = (startTime/(60*60)) % 24;
  startTime -= minutes*60 + hours*60*60;
  printUnixTime(timeBuf, startTime); DEBUG2_PRINTF("\nStartTime2 GMT:%s", timeBuf);

  E_RC rc = E_OK;

  for (unsigned int i = 0; i<ARCH_DAY_SIZE; i++) {
     invData.dayWh[i] = 0;
  }
  invData.hasDayData = false;

  int packetcount = 0;
  bool validPcktID = false;

  E_RC hasData = E_ARCHNODATA;
  pcktID++;
  writePacketHeader(pcktBuf, 0x01, invData.BTAddress);
  writePacket(pcktBuf, 0x09, 0xE0, 0, invData.SUSyID, invData.Serial);
  write32(pcktBuf, 0x70000200);
  write32(pcktBuf, startTime - 300);
  write32(pcktBuf, startTime + 86100);
  writePacketTrailer(pcktBuf);
  writePacketLength(pcktBuf);

  BTsendPacket(pcktBuf);

  do {
    totalWh = 0;
    totalWh_prev = 0;
    dateTime = 0;

    do {
      rc = getPacket(invData.BTAddress, 1);

      if (rc != E_OK) {
         DEBUG3_PRINTF("\ngetPacket error=%d", rc);
         return rc;
      }
      // packetcount=nr of packets left on multi packet transfer n..0
      packetcount = pcktBuf[25];
      DEBUG2_PRINTF("packetcount=%d\n", packetcount);

      //TODO: Move checksum validation to getPacket
      if (!validateChecksum())
        return E_CHKSUM;
      else {
        unsigned short rcvpcktID = get_u16(pcktBuf + 27) & 0x7FFF;
        if (validPcktID || (pcktID == rcvpcktID)) {
          validPcktID = true;
          for (int x = 41; x < (pcktBufPos - 3); x += 12) {
            dateTime = (time_t)get_u32(pcktBuf + x);
            uint16_t idx =((dateTime/3600)%24 * 12)+((dateTime/60)%60/5); //h*12+min/5

            totalWh = get_u64(pcktBuf + x + 4);
            if ((totalWh > 0) && (!invData.hasDayData)) { 
              invData.DayStartTime = dateTime;
              invData.hasDayData = true;
              hasData = E_OK; 
              printUnixTime(timeBuf, dateTime); 
              DEBUG1_PRINTF("\nArchiveDayData %s", timeBuf);
            }
            if (idx < ARCH_DAY_SIZE) {
              invData.dayWh[idx] = totalWh;
              value64 = (totalWh - totalWh_prev) * 60 / 5; // assume 5 min. interval
              DEBUG3_PRINTF("[%03u] %6llu Wh %6llu W\n", idx, totalWh, value64);
            }
            totalWh_prev = totalWh;
          } //for
        } else {
            DEBUG1_PRINTF("Packet ID mismatch. Exp. %d, rec. %d\n", pcktID, rcvpcktID);
            validPcktID = true;
            packetcount = 0;
        }
      }
    } while (packetcount > 0);
  } while (!validPcktID);

  /* print values
  time_t startT = invData.DayStartTime;
  printUnixTime(timeBuf, startT);
  DEBUG2_PRINTF("Day History: %s\n", timeBuf);
  totalWh_prev = 0;

  for (uint16_t i = 0; i<ARCH_DAY_SIZE; i++) {
    totalWh = invData.dayWh[i];
    value32=0;
    if ((totalWh>0) && (totalWh_prev>0)) {
      value32 = (uint32_t)((totalWh - totalWh_prev)*60/5); 
    }
    if (totalWh>0) {
      printUnixTime(timeBuf, startT+3600+i*60*5); // GMT+1 + 5 min. interval
      DEBUG2_PRINTF("[%03d] %11.3f kWh  %7.3f kW %s\n", i, tokWh(totalWh), tokW(value32), timeBuf);
    }
    totalWh_prev = totalWh;
  }
  
  return hasData;
}
*/
// ******* read SMA current data **********
E_RC ESP32_SMA_Inverter::ReadCurrentData() {
  if (!btConnected) {
    charLen += snprintf(charBuf+charLen, CHAR_BUF_MAX-charLen, "Bluetooth offline!\n");
    return E_NODATA;
  }
  if ((getInverterData(SpotACTotalPower)) != E_OK)  {
    charLen += snprintf(charBuf+charLen, CHAR_BUF_MAX-charLen, "SpotACTotalPower error!\n" ); // Pac
    return E_NODATA;
  }
  if ((getInverterData(SpotDCVoltage)) != E_OK)     {
    charLen += snprintf(charBuf+charLen, CHAR_BUF_MAX-charLen, "getSpotDCVoltage error!\n" ); // Udc + Idc
    return E_NODATA;
  }
  if ((getInverterData(SpotACVoltage)) != E_OK)     {
    charLen += snprintf(charBuf+charLen, CHAR_BUF_MAX-charLen, "getSpotACVoltage error!\n" ); // Uac + Iac
    return E_NODATA;
  }
  if ((getInverterData(EnergyProduction)) != E_OK)  {
    charLen += snprintf(charBuf+charLen, CHAR_BUF_MAX-charLen, "EnergyProduction error!\n" ); // E-Total + E-Today
    return E_NODATA;
  }
  if ((getInverterData(SpotGridFrequency)) != E_OK) {
    charLen += snprintf(charBuf+charLen, CHAR_BUF_MAX-charLen, "SpotGridFrequency error!\n");
    return E_NODATA;
  }
  if ((getInverterData(InverterTemp)) != E_OK) {
    charLen += snprintf(charBuf+charLen, CHAR_BUF_MAX-charLen, "InverterTemp error!\n");
    return E_NODATA;
  }
  if ((getInverterData(DeviceStatus)) != E_OK) {
    charLen += snprintf(charBuf+charLen, CHAR_BUF_MAX-charLen, "Device Status error!\n");
    return E_NODATA;
  }
  if ((getInverterData(GridRelayStatus)) != E_OK) {
    charLen += snprintf(charBuf+charLen, CHAR_BUF_MAX-charLen, "Grid Relay Status error!\n");
    return E_NODATA;
  }

//case 5: if ((getInverterData(SpotDCPower)) != E_OK)   DEBUG1_PRINTLN("getSpotDCPower error!"); //pcktBuf[23]=15 error!
//case 6: if ((getInverterData(SpotACPower)) != E_OK)   DEBUG1_PRINTLN("SpotACPower error!"   ); //pcktBuf[23]=15 error!
//case 7: if ((getInverterData(InverterTemp)) != E_OK)  DEBUG1_PRINTLN("InverterTemp error!"  ); //pcktBuf[23]=15 error!
//case 8: if ((getInverterData(OperationTime)) != E_OK) DEBUG1_PRINTLN("OperationTime error!" ); // OperTime + OperTime
  return E_OK;
} 



// **** receive BT byte *******
uint8_t ESP32_SMA_Inverter::BTgetByte() {
  readTimeout = false;
  //Returns a single byte from the bluetooth stream (with error timeout/reset)
  uint32_t time = 20000+millis(); // 20 sec 
  uint8_t  rec = 0;  

  while (!serialBT.available() ) {
    //delay(5);  //Wait for BT byte to arrive
    if (millis() > time) { 
      ESP_LOGD(INVTAG,"BTgetByte Timeout");
      readTimeout = true;
      break;
    }
  }

  if (!readTimeout) rec = serialBT.read();
  return rec;
}
// **** transmit BT buffer ****
void ESP32_SMA_Inverter::BTsendPacket( uint8_t *btbuffer ) {
  //DEBUG2_PRINTLN();
  for(int i=0;i<pcktBufPos;i++) {
    #ifdef DebugBT
    if (i==0) DEBUG2_PRINT("*** sStart=");
    if (i==1) DEBUG2_PRINT(" len=");
    if (i==3) {
      if ((0x7e ^ *(btbuffer+1) ^ *(btbuffer+2)) == *(btbuffer+3)) 
        DEBUG2_PRINT(" checkOK=");
      else  DEBUG2_PRINT(" checkFalse!!=");
    }
    if (i==4)  DEBUG2_PRINTF("\nfrMac[%d]=",i);
    if (i==10) DEBUG2_PRINTF(" toMac[%d]=",i);
    if (i==16) DEBUG2_PRINTF(" Type[%d]=",i);
    if ((i==18)||(i==18+16)||(i==18+32)||(i==18+48)) DEBUG2_PRINTF("\nsDat[%d]=",i);
    DEBUG2_PRINTF("%02X,", *(btbuffer+i)); // Print out what we are sending, in hex, for inspection.
    #endif
    serialBT.write( *(btbuffer+i) );  // Send to SMA via ESP32 bluetooth
  }
  HexDump(btbuffer, pcktBufPos, 10, 'T');
}


//-------------------------------------------------------------------------
// ********************************************************
void ESP32_SMA_Inverter::writeByte(uint8_t *btbuffer, uint8_t v) {
  //Keep a rolling checksum over the payload
  fcsChecksum = (fcsChecksum >> 8) ^ fcstab[(fcsChecksum ^ v) & 0xff];
  if (v == 0x7d || v == 0x7e || v == 0x11 || v == 0x12 || v == 0x13) {
    btbuffer[pcktBufPos++] = 0x7d;
    btbuffer[pcktBufPos++] = v ^ 0x20;
  } else {
    btbuffer[pcktBufPos++] = v;
  }
}
// ********************************************************
void ESP32_SMA_Inverter::write32(uint8_t *btbuffer, uint32_t v) {
  writeByte(btbuffer,(uint8_t)((v >> 0) & 0xFF));
  writeByte(btbuffer,(uint8_t)((v >> 8) & 0xFF));
  writeByte(btbuffer,(uint8_t)((v >> 16) & 0xFF));
  writeByte(btbuffer,(uint8_t)((v >> 24) & 0xFF));
}
// ********************************************************
void ESP32_SMA_Inverter::write16(uint8_t *btbuffer, uint16_t v) {
  writeByte(btbuffer,(uint8_t)((v >> 0) & 0xFF));
  writeByte(btbuffer,(uint8_t)((v >> 8) & 0xFF));
}
// ********************************************************
void ESP32_SMA_Inverter::writeArray(uint8_t *btbuffer, const uint8_t bytes[], int loopcount) {
    for (int i = 0; i < loopcount; i++) writeByte(btbuffer, bytes[i]);
}
// ********************************************************
//  writePacket(pcktBuf, 0x0E, 0xA0, 0x0100, 0xFFFF, 0xFFFFFFFF); // anySUSyID, anySerial);
void ESP32_SMA_Inverter::writePacket(uint8_t *buf, uint8_t longwords, uint8_t ctrl, uint16_t ctrl2, uint16_t dstSUSyID, uint32_t dstSerial) {
  buf[pcktBufPos++] = 0x7E;   //Not included in checksum
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
//-------------------------------------------------------------------------
void ESP32_SMA_Inverter::writePacketTrailer(uint8_t *btbuffer) {
  fcsChecksum ^= 0xFFFF;
  btbuffer[pcktBufPos++] = fcsChecksum & 0x00FF;
  btbuffer[pcktBufPos++] = (fcsChecksum >> 8) & 0x00FF;
  btbuffer[pcktBufPos++] = 0x7E;  //Trailing byte
}
//-------------------------------------------------------------------------
void ESP32_SMA_Inverter::writePacketLength(uint8_t *buf) {
  buf[1] = pcktBufPos & 0xFF;         //Lo-Byte
  buf[2] = (pcktBufPos >> 8) & 0xFF;  //Hi-Byte
  buf[3] = buf[0] ^ buf[1] ^ buf[2];      //checksum
}
//-------------------------------------------------------------------------
bool ESP32_SMA_Inverter::validateChecksum() {
  fcsChecksum = 0xffff;
  //Skip over 0x7e at start and end of packet
  for(int i = 1; i <= pcktBufPos - 4; i++) {
    fcsChecksum = (fcsChecksum >> 8) ^ fcstab[(fcsChecksum ^ pcktBuf[i]) & 0xff];
  }
  fcsChecksum ^= 0xffff;

  if (get_u16(pcktBuf+pcktBufPos-3) == fcsChecksum) {
    return true;
  } else {
    ESP_LOGE(INVTAG, "Invalid validateChecksum 0x%04X not 0x%04X\n", 
    fcsChecksum, get_u16(pcktBuf+pcktBufPos-3));
    return false;
  }
}



void ESP32_SMA_Inverter::HexDump(uint8_t *buf, int count, int radix, uint8_t c) {
  int i, j;
  ESP_LOGD(INVTAG, "\n---%c----:", c);
  for (i=0; i < radix; i++) ESP_LOGD(" %02X", i);
  for (i = 0, j = 0; i < count; i++) {
    if (j % radix == 0) {
      ESP_LOGD(INVTAG, "\n%c-%06d: ", c, j);
    }
    ESP_LOGD(INVTAG, "%02X ", buf[i]);
    j++;
  }
  ESP_LOGD(INVTAG, "\n");
}
//-----------------------------------------------------
 uint8_t ESP32_SMA_Inverter::printUnixTime(char *buf, time_t t) {
    uint32_t a, b, c, d, e, f;
    //Negative Unix time values are not supported
    if(t < 1) t = 0;
  
    //Retrieve hours, minutes and seconds
    uint8_t seconds = t % 60;
    t /= 60;
    uint8_t minutes = t % 60;
    t /= 60;
    uint8_t hours = t % 24;
    t /= 24;
  
    //Convert Unix time to date
    a = (uint32_t) ((4 * t + 102032) / 146097 + 15);
    b = (uint32_t) (t + 2442113 + a - (a / 4));
    c = (20 * b - 2442) / 7305;
    d = b - 365 * c - (c / 4);
    e = d * 1000 / 30601;
    f = d - e * 30 - e * 601 / 1000;
  
    //January and February are counted as months 13 and 14 of the previous year
    if(e <= 13) {
       c -= 4716;
       e -= 1;
    } else {
       c -= 4715;
       e -= 13;
    }
    //Retrieve year, month and day
    uint16_t year = c;
    uint8_t  month = e;
    uint8_t  day = f;
    return snprintf(buf, 31,"%02d.%02d.%d %02d:%02d:%02d",day, month, year, hours, minutes, seconds);
    //ESP_LOGI(" GMT: %02d.%02d.%d %02d:%02d:%02d",day, month, year, hours, minutes, seconds);
 }

//-----------------------------------------------------
uint16_t ESP32_SMA_Inverter::get_u16(uint8_t *buf) {
    register uint16_t shrt = 0;
    shrt += *(buf+1);
    shrt <<= 8;
    shrt += *(buf);
    return shrt;
}
uint32_t ESP32_SMA_Inverter::get_u32(uint8_t *buf) {
    register uint32_t lng = 0;
    lng += *(buf+3);
    lng <<= 8;
    lng += *(buf+2);
    lng <<= 8;
    lng += *(buf+1);
    lng <<= 8;
    lng += *(buf);
    return lng;
}
uint64_t ESP32_SMA_Inverter::get_u64(uint8_t *buf) {
    register uint64_t lnglng = 0;
    lnglng += *(buf+7);
    lnglng <<= 8;
    lnglng += *(buf+6);
    lnglng <<= 8;
    lnglng += *(buf+5);
    lnglng <<= 8;
    lnglng += *(buf+4);
    lnglng <<= 8;
    lnglng += *(buf+3);
    lnglng <<= 8;
    lnglng += *(buf+2);
    lnglng <<= 8;
    lnglng += *(buf+1);
    lnglng <<= 8;
    lnglng += *(buf);
    return lnglng;
}

