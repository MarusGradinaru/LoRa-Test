
// ----- User Settings --------------------------------------

#define DEBUG_MODE   // comment to stop debug messages
#define TX_MODULE    // comment for RX_MODULE  (TX not available for PICO_BOARD)

// Module pin configuration (GPIO)

#if defined(ESP32_BOARD)
  #define PIN_CS     5
  #define PIN_CLK    18
  #define PIN_MOSI   23
  #define PIN_MISO   19
  #define PIN_RESET  27
  #define PIN_BUSY   26
  #define PIN_RX_EN  25
  #define PIN_TX_EN  33
  #define PIN_DIO1   32
#elif defined(PICO_BOARD)
  #define PIN_CS     17 
  #define PIN_CLK    18
  #define PIN_MOSI   19  // TX
  #define PIN_MISO   16  // RX
  #define PIN_RESET  15
  #define PIN_BUSY   14
  #define PIN_RX_EN  21 
  #define PIN_TX_EN  20
  #define PIN_DIO1   13
#endif

// LoRa Module default settings
#define FREQ    869525000 // Frequency: P band, 250KHz, 500mW (27dBm), 10% [863.0 MHz + 1/2 BW .. 870.0 MHz - 1/2 BW] 
#define BW      7         // Bandwidth idx: 125KHz [7.8, 10.4, 15.6, 20.8, 31.25, 41.7, 62.5, 125, 250, 500] KHz
#define SF      4         // Spreading Factor idx: SF9 [5..12]
#define CR      2         // Coding Rate idx: 4/7 [5..8]
#define SYNCW   0xE3u     // Sync Word: Custom [Private: 0x12, Public: 0x34] 
#define TX_PWR  9         // TX Power (dBm) [-9..22] / with 5 dBi antenna = 14 dBm (25 mW)
#define TX_DC   10.0f     // TX Duty Cycle (% percent)
#define PAMB    2         // Preamble Length idx: 8 symb. [6..20] 
#define XOV     1.7f      // TCXO Voltage (V) [1.6, 1.7, 1.8, 2.2, 2.4, 2.7, 3.0, 3.3]
#define LDO     false     // Use LDO only ? [false:LDO and DC-DC, true: just LDO]


// ----- Include Section ------------------------------------

#include <Arduino.h>
#include <SPI.h>
#include <RadioLib.h>

#ifdef ESP32_BOARD
  #define  ISR_ATTR IRAM_ATTR
  #include <esp_partition.h>
#endif

#ifdef PICO_BOARD
  #undef   TX_MODULE
  #define  ISR_ATTR
  #include <FreeRTOS.h>
  #include <semphr.h>
#endif

#ifdef TX_MODULE
  // This build is the TX Module !
  #include <LittleFS.h>
  #include <WiFi.h>
  #include <ESPAsyncWebServer.h>
  #include <ArduinoJson.h>
#else
  // This build is the RX Module !  
#endif


// ----- Debug Functions -----------------------------------

#ifdef DEBUG_MODE

  #define SerialTake()                xSemaphoreTake(serialMutex, portMAX_DELAY)
  #define SerialGive()                xSemaphoreGive(serialMutex);
  #define Print(...)                  Serial.print(__VA_ARGS__)
  #define PrintLn(...)                Serial.println(__VA_ARGS__)
  #define PrintHex(val, dig)          _PrintHex((val), (dig), false)
  #define PrintHexLn(val, dig)        _PrintHex((val), (dig), true)
  #define SafePrintLn(...)            _SafePrintLn(__VA_ARGS__)
  #define PrintBuff(msg, buff, len)   _PrintBuff(msg, buff, len)
  #if defined(ESP32_BOARD) && defined(TX_MODULE)  
    #define ListPartitions()          _listPartitions()
    #define ListDir(fs, dir, lvl)     _listDir(fs, dir, lvl) 
  #endif  

const char debug_Done[]        = " done !";
const char debug_DoneLn[]      = " done !\n";
const char debug_Fail[]        = " failed !";
const char debug_FailLn[]      = " failed !\n";
const char debug_TxBuff[]      = "Transmit buffer";
const char debug_RxBuff[]      = "Received data";
const char debug_MallocFail[]  = "[SYSTEM] Failed to allocate memory.";
const char debug_UpdateCfg[]   = "[SX1262] Updating LoRa configuration...";
const char debug_CfgDone[]     = "[SX1262] Reconfiguration succeeded !";
const char debug_CfgFail[]     = "[SX1262] Reconfiguration failed !";
const char debug_CfgUndo[]     = "[SX1262] Rolling back changes...";
const char debug_SendAckn[]    = "[SX1262] Sending acknowledgement...";
const char debug_SendReply[]   = "[SX1262] Sending reply data...";

SemaphoreHandle_t serialMutex;

void _SafePrintLn(const char* msg) {
  xSemaphoreTake(serialMutex, portMAX_DELAY);
  Serial.println(msg);
  xSemaphoreGive(serialMutex);
}

void _PrintHex(uint32_t val, uint8_t dig, bool line) {
  uint32_t mask = 1UL << ((dig * 4) - 1);  // 4 bits per digit
  for (uint8_t i = 0; i < dig; i++) {
    uint8_t hexDigit = (val >> ((dig - 1 - i) * 4)) & 0xF;
    if (hexDigit < 10) Serial.print((char)('0' + hexDigit));
    else Serial.print((char)('A' + hexDigit - 10)); }
  if (line) Serial.println();
}

void _PrintBuff(const char* msg, const uint8_t* buffer, uint16_t len) {
  Serial.print("[SYSTEM] "); Serial.print(msg); Serial.print(" = ");
  for (int i = 0; i < len; i++) {
    if (buffer[i] < 0x10) Serial.print("0");
    Serial.print(buffer[i], HEX);
    if (i < len-1) Serial.print(", "); }
  Serial.println();
}

#if defined(ESP32_BOARD) && defined(TX_MODULE)

void _listPartitions() {
  Serial.println("Partitions:");
  esp_partition_iterator_t it = esp_partition_find(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_ANY, NULL);
  while (it != NULL) {
    const esp_partition_t *part = esp_partition_get(it);
    Serial.printf(" label='%s'  type=0x%02x  subtype=0x%02x  addr=0x%06x  size=0x%06x\n",
      part->label, part->type, part->subtype, part->address, part->size);
    it = esp_partition_next(it); }
  esp_partition_iterator_release(it);
}

void _listDir(fs::FS& fs, const char* dirname, uint8_t levels) {
  Serial.printf("Listing directory: %s\r\n", dirname);
  File root = fs.open(dirname);
  if (!root) { Serial.println("- failed to open directory"); return; }
  if (!root.isDirectory()) { Serial.println(" - not a directory"); return; }
  File file = root.openNextFile();
  while (file) {
    if (file.isDirectory()) {
      Serial.print("  DIR : "); Serial.println(file.name());
      if (levels) _listDir(fs, file.path(), levels - 1);
    } else {
      Serial.print("  FILE: "); Serial.print(file.name());
      Serial.print("\tSIZE: "); Serial.println(file.size()); }
    file = root.openNextFile(); }
}

#endif

#else  // RELEASE MODE

  #define SerialTake()                    ((void)0)
  #define SerialGive()                    ((void)0)
  #define Print(msg)                      ((void)0)
  #define PrintLn(msg)                    ((void)0)
  #define PrintHex(val, dig)              ((void)0)
  #define PrintHexLn(val, dig)            ((void)0)
  #define SafePrintLn(msg)                ((void)0)
  #define PrintBuff(msg, buff, len)       ((void)0)
  #if defined(ESP32_BOARD) && defined(TX_MODULE)  
    #define ListPartitions()              ((void)0)
    #define ListDir(fs, dir, lvl)         ((void)0) 
  #endif  

#endif


// ----- Basic Functions and Macros ------------------------

#define FlashParams(fstr)   reinterpret_cast<const uint8_t*>(fstr), sizeof(fstr)-1


// ----- CRC-16/CCITT-FALSE Class --------------------------

class CRC16 {
  private:
    static constexpr uint16_t _msbMask    = 0x8000;
    static constexpr uint16_t _init       = 0xFFFF;
    static constexpr uint16_t _polynomial = 0x1021;  
    uint16_t _crc;

    static uint16_t _calc(uint16_t crc, const uint8_t* data, size_t len) { 
      for (size_t i = 0; i < len; i++) {
        uint8_t j = 0x80;
        while (j > 0) {
          uint16_t bit = (uint16_t)(crc & _msbMask);
          crc <<= 1;
          if ((data[i] & j) != 0) bit ^= _msbMask;
          if (bit != 0) crc ^= _polynomial;
          j >>= 1; }}
      return crc;
    }

  public:
    CRC16() { _crc = _init; }
    inline void clear() { _crc = _init; }
    inline uint16_t getValue() { return _crc; }   
    inline void putValue(uint8_t* buff) { memcpy(buff, &_crc, 2); } 
    inline void update(const uint8_t* data, size_t len) { _crc = _calc(_crc, data, len); }
    inline static uint16_t compute(const uint8_t* data, size_t len) { return _calc(_init, data, len); }    
};


// ----- Main LoRa SX1262 Class ----------------------------

#define  HTTP_MSG_SIZE  60
#define  MAX_CFG_JSON   120
#define  MAX_RES_JSON   160
#define  MAX_BLK_JSON   350
#define  DEF_BUFF_SIZE  32
#define  BLK_HEAD_SIZE  9
#define  RX_TIMEOUT     2000

#define  RADIOLIB_ERR_MEM_ALLOC_FAILED  (-3)
#define  RADIOLIB_ERR_OUT_OF_SYNC       (-32001)
#define  RADIOLIB_ERR_BUFF_OVERFLOW     (-32002)
#define  RADIOLIB_ERR_BAD_PROTOCOL      (-32003)
#define  RADIOLIB_ERR_REMOTE_FAILED     (-32004)
#define  RADIOLIB_ERR_INVALID_PARAMS    (-32005)
#define  RADIOLIB_ERR_INVALID_BULK_HDR  (-32006)
#define  RADIOLIB_ERR_BULK_CORRUPTED    (-32007) 

#define  MillisTOA(toa)               (RadioLibTime_t)(((toa) + 999ul) / 1000ul)
#define  BreakAssert(state)           { if ((state) != RADIOLIB_ERR_NONE) break; }
#define  BreakAssertMsg(state, msg)   { if ((state) != RADIOLIB_ERR_NONE) { PrintLn(msg); break; } }

const uint8_t cmdStartTest  = 0xC8;
const uint8_t rplTestRes    = 0x8C;
const uint8_t cmdSetConfig  = 0xA3;
const uint8_t rplConfigRes  = 0x3A;
const uint8_t cmdPing       = 0x51;
const uint8_t rplPing       = 0x15;
const uint8_t cmdBulk       = 0xD6;
const uint8_t rplBulk       = 0x6D;
const uint8_t rplRxBT       = 0x7E;
const uint8_t statSuccess   = 0xFF;
const uint8_t statFailed    = 0x11;

const float listBandwidth[]   = {7.8f, 10.4f, 15.6f, 20.8f, 31.25f, 41.7f, 62.5f, 125.0f, 250.0f, 500.0f};
const uint16_t MAX_BULK_PS    = RADIOLIB_SX126X_MAX_PACKET_LENGTH -2;
const uint16_t MAX_BULK_SIZE  = (0xFF * MAX_BULK_PS) -2;

struct TxBulkTiming {
  RadioLibTime_t 
    off_head,  off_min, off_max,  off_part,  off_reply,  
    toa_full, toa_part;
};

struct RxBulkTiming {
  RadioLibTime_t 
    read_head, work_head,  read_min, read_max, work_min, work_max, 
    read_part, work_part,  read_reply, work_reply; 
};

struct LoraUserCfg {
  uint32_t freq;  // in Hz
  int8_t txpwr;   // in dBm
  uint8_t bandw, spread, cdrate, preamb;  // in list indexes, starting from zero
};

struct LoraFixedCfg {
  uint8_t syncw = SYNCW;
  uint8_t txdc = TX_DC;
  bool useldo = LDO;
  float xovolt = XOV;
};

class MSX1262 : public SX1262 {   //----------------------------------------------------------
  private:
    struct DeviceData {
      bool Running = false;
      bool Result = false;
      bool cfgRemote = true;  // Config params [in]
      uint32_t buffSize;      // Global params [in] 
      uint32_t bulkDelay;     // Bulk Test [in]
      size_t DataRate;        // Bulk Test [out]
      float tx_rssi, rx_rssi, tx_snr, rx_snr, tx_fqerr, rx_fqerr; // Test results [out]
      String Error;           // Global error message [out]
      LoraUserCfg UserCfg;
    };
    SemaphoreHandle_t devMutex = nullptr;  // protect Data, TxBTi, RxBTi

    LoraFixedCfg FCfg; LoraUserCfg UCfg; bool cfgChanging = false; 
    SemaphoreHandle_t cfgMutex = nullptr;  // protect Cfg (UCfg + Data.uParams) & FCfg & cfgChanging

    uint32_t txNext = 0; float psConst;  // TX duty cycle

    int16_t Reconfigure(const LoraUserCfg& params, bool chg, bool forced) {
      cfgTake();
      if (chg) cfgChanging = true;
      int16_t state = RADIOLIB_ERR_NONE;
      do {
        state = standby(); BreakAssert(state);
        if (forced || Cfg->freq != params.freq) {
          state = setFrequency((float)params.freq / 1000000.0f); 
          BreakAssert(state); }
        if (forced || Cfg->txpwr != params.txpwr) { 
          state = setOutputPower(params.txpwr); 
          BreakAssert(state); }
        if (forced || Cfg->bandw != params.bandw) { 
          state = setBandwidth(listBandwidth[params.bandw]); 
          BreakAssert(state); }
        if (forced || Cfg->spread != params.spread) {
          state = setSpreadingFactor(5 + params.spread);
          BreakAssert(state); }
        if (forced || Cfg->cdrate != params.cdrate) {
          state = setCodingRate(5 + params.cdrate);
          BreakAssert(state); }
        if (forced || Cfg->preamb != params.preamb) {
          state = setPreambleLength(6 + params.preamb);
          BreakAssert(state); }  
        Cfg = &params;
        if (!chg) cfgChanging = false;
      } while(false);
      cfgGive(); return state;
    }    

    //--- RSSI Band Monitor ---

    volatile bool bmtDone = false;
    volatile float BandRSSI = 0;
    SemaphoreHandle_t rssiMutex = nullptr;  // protect BandRSSI
    TaskHandle_t taskMonitor = nullptr;

    inline void rssiTake() { xSemaphoreTake(rssiMutex, portMAX_DELAY); }
    inline void rssiGive() { xSemaphoreGive(rssiMutex); }

    static void BandMonitorTask(void* pvParameters) {
      MSX1262& Lora = *static_cast<MSX1262*>(pvParameters); 
      float band_rssi; TickType_t timeout = pdMS_TO_TICKS(1000);
      do {
        Lora.devTake(); if (!Lora.Data.Running) band_rssi = Lora.getRSSI(false); Lora.devGive();
        Lora.rssiTake(); Lora.BandRSSI = band_rssi; Lora.rssiGive();
      } while (ulTaskNotifyTake(pdTRUE, timeout) == 0);
      Lora.bmtDone = true;
      vTaskDelete(NULL);
    }

    //--- Bulk Timing ---

    // protected by DevTake()
    TxBulkTiming TxBTi;
    RxBulkTiming RxBTi;

    // used in the same thread (task)
    RadioLibTime_t btTime = 0;
    bool btFirstPack = true, btTxActive, btRxActive;

  public:   //---------------------------------------------------------------------
    const LoraUserCfg* Cfg = &UCfg;     // read only content
    const TxBulkTiming* TxBT = &TxBTi;  // read only content
    const RxBulkTiming* RxBT = &RxBTi;  // read only content
    DeviceData Data; 

    MSX1262(Module* mod) : SX1262(mod) {
      cfgMutex = xSemaphoreCreateRecursiveMutex();
      devMutex = xSemaphoreCreateMutex();
      rssiMutex = xSemaphoreCreateMutex();
      UCfg = { FREQ, TX_PWR, BW, SF, CR, PAMB };
      Data.Error.reserve(HTTP_MSG_SIZE);
      psConst = (100.0f / TX_DC) - 1.0f;
    }
    
    ~MSX1262() {
      stopBandMonitor();
      if (cfgMutex) vSemaphoreDelete(cfgMutex);
      if (devMutex) vSemaphoreDelete(devMutex);
      if (rssiMutex) vSemaphoreDelete(rssiMutex);
    }    

    inline void cfgTake()  { xSemaphoreTakeRecursive(cfgMutex, portMAX_DELAY); }
    inline void cfgGive()  { xSemaphoreGiveRecursive(cfgMutex); }
    inline void devTake()  { xSemaphoreTake(devMutex, portMAX_DELAY); }
    inline void devGive()  { xSemaphoreGive(devMutex); }

    inline void clearEvents() { xTaskNotifyStateClear(NULL); ulTaskNotifyValueClear(NULL, 0xFFFFFFFF); }

    void getTestJson(char* buff) {  // require devTake() or Data.Running 
      snprintf(buff, MAX_RES_JSON, 
      "{\"tx_rssi\":%.2f,\"rx_rssi\":%.2f,\"tx_snr\":%.2f,\"rx_snr\":%.2f,\"tx_fqerr\":%.1f,\"rx_fqerr\":%.1f}",
      Data.tx_rssi, Data.rx_rssi, Data.tx_snr, Data.rx_snr, Data.tx_fqerr, Data.rx_fqerr);
    }

    uint32_t MemRead[10], MemWork[10]; int iR, iW;  // bulk debug
    uint32_t MemOff[10]; int iF;  // bulk debug

    //--- RSSI Band Monitor ---

    float getBandRSSI() { 
      rssiTake(); float band_rssi = BandRSSI; rssiGive(); 
      return band_rssi;
    }

    void startBandMonitor() {
      if (taskMonitor) return;
      bmtDone = false; bool started = false;
      #if defined(ESP32_BOARD)
        started = xTaskCreatePinnedToCore(BandMonitorTask,
          "BandMonitorTask", 2048, this, 1, &taskMonitor, 1) == pdPASS;
      #elif defined(PICO_BOARD)
        started = xTaskCreateAffinitySet(BandMonitorTask, 
          "BandMonitorTask", 2048, this, 1, 2, &taskMonitor) == pdPASS;
      #endif
      if (!started) taskMonitor = nullptr; 
    }

    void stopBandMonitor() {
      if (!taskMonitor) return;
      xTaskNotifyGive(taskMonitor);
      do delay(5); while (!bmtDone); 
      taskMonitor = nullptr; 
      BandRSSI = 0;
    }

    //--- Configuration functions ---

    void getCfgJson(char* buff) { 
      cfgTake();
      snprintf(buff, MAX_CFG_JSON +1, 
        "{\"freq\":%u,\"txpwr\":%d,\"bandw\":%u,\"spread\":%u,\"cdrate\":%u,\"preamb\":%u}",
        Cfg->freq, Cfg->txpwr, Cfg->bandw, Cfg->spread, Cfg->cdrate, Cfg->preamb);      
      cfgGive();
    }  

    // ApplyUserCfg, CancelConfig, UpdateConfig: require devTake() or Data.Running
    inline int16_t ApplyUserCfg() { return Reconfigure(Data.UserCfg, true, false); }
    inline int16_t CancelConfig() { return Reconfigure(UCfg, false, true); }
    void UpdateConfig() { cfgTake(); UCfg = Data.UserCfg; Cfg = &UCfg; cfgChanging = false; cfgGive(); }
    bool isCfgChanging() { cfgTake(); bool chg = cfgChanging; cfgGive(); return chg; }

    //--- Device functions. All require devTake() or Data.Running ---

    template<typename... Args>
    void FormatError(const char* fmt, Args... args) {
      char buff[HTTP_MSG_SIZE];
      snprintf(buff, sizeof(buff), fmt, args...);
      Data.Error = buff;
    }

    void FormatErrorWait(float wTime) {
      char fbuf[16]; char buff[HTTP_MSG_SIZE];
      dtostrf(wTime, 0, 3, fbuf);
      snprintf(buff, sizeof(buff), "Please wait %s seconds more !", fbuf);
      Data.Error = buff;
    }

    bool ReadyForTx() {
      int32_t delta = (int32_t)(millis() - txNext);
      if (delta >= 0) return true; else {
        float wait_time = ((float)(-delta) / 1000.0f);
        FormatErrorWait(wait_time); return false; }
    }

    void TxDone(uint32_t tx_start, uint32_t tx_toa) {
      if (tx_toa == 0) return;
      uint32_t ps_time = ceil(tx_toa * psConst);
      txNext = tx_start + tx_toa + ps_time;      
    }

    inline int16_t beginDefault() {
      cfgTake();
      int16_t state = begin((float)Cfg->freq / 1000000.0f, listBandwidth[Cfg->bandw], 5 + Cfg->spread, 
        5 + Cfg->cdrate, FCfg.syncw, Cfg->txpwr, 6 + Cfg->preamb, FCfg.xovolt, FCfg.useldo);
      cfgGive();
      return state;
    }

    inline RadioLibTime_t getMaxTOA_ms() { return MillisTOA(getTimeOnAir(RADIOLIB_SX126X_MAX_PACKET_LENGTH)); }
    inline RadioLibTime_t getMaxTOA_us() { return getTimeOnAir(RADIOLIB_SX126X_MAX_PACKET_LENGTH); }
    
    inline int16_t startSingleRx() {
      return startReceive(RADIOLIB_SX126X_RX_TIMEOUT_NONE, RADIOLIB_IRQ_RX_DEFAULT_FLAGS,
        RADIOLIB_IRQ_RX_DEFAULT_MASK, 0);
    }

    void SetErrorMsg(int16_t code) {
      switch (code) {
        case RADIOLIB_ERR_NONE:              Data.Error = ""; break;
        case RADIOLIB_ERR_MEM_ALLOC_FAILED:  Data.Error = "Failed to allocate memory."; break;
        case RADIOLIB_ERR_CRC_MISMATCH:      Data.Error = "LoRa packet is corrupted."; break;
        case RADIOLIB_ERR_RX_TIMEOUT:        Data.Error = "Remote LoRa is not responding."; break;
        case RADIOLIB_ERR_OUT_OF_SYNC:       Data.Error = "The protocol has gone out of sync."; break;
        case RADIOLIB_ERR_BUFF_OVERFLOW:     Data.Error = "The buffer overflowed."; break;
        case RADIOLIB_ERR_BAD_PROTOCOL:      Data.Error = "Invalid LoRa protocol detected."; break;
        case RADIOLIB_ERR_REMOTE_FAILED:     Data.Error = "Remote LoRa failed its job."; break;
        case RADIOLIB_ERR_INVALID_PARAMS:    Data.Error = "Invalid call parameters."; break;
        case RADIOLIB_ERR_INVALID_BULK_HDR:  Data.Error = "Invalid bulk header."; break;
        default:                             FormatError("Local LoRa failed, code %d", code); break;
      }
    }

    bool CheckResult(int16_t state, bool setErr = true) {
      if (state == RADIOLIB_ERR_NONE) { PrintLn(debug_Done); return true; } 
      else { Print(" failed, code "); PrintLn(state); if (setErr) SetErrorMsg(state); return false; }
    }     

    int16_t ReadPacket(uint8_t* data, size_t len, size_t* size = nullptr) {
      uint16_t irq = getIrqFlags(); int16_t state = clearIrqStatus(); RADIOLIB_ASSERT(state);
      int16_t crcState = RADIOLIB_ERR_NONE; int16_t buffState = RADIOLIB_ERR_NONE;
      // Report CRC mismatch when there's a payload CRC error, or a header error and no valid header
      if ((irq & RADIOLIB_SX126X_IRQ_CRC_ERR) ||
        ((irq & RADIOLIB_SX126X_IRQ_HEADER_ERR) && !(irq & RADIOLIB_SX126X_IRQ_HEADER_VALID)))
        crcState = RADIOLIB_ERR_CRC_MISMATCH;
      // get packet length and Rx buffer offset
      uint8_t offset = 0; size_t length = getPacketLength(true, &offset);
      if (size) *size = length;
      if (length > len) { length = len; buffState = RADIOLIB_ERR_BUFF_OVERFLOW; }
      // read packet data starting at offset
      state = readBuffer(data, length, offset); RADIOLIB_ASSERT(state);
      // check if CRC failed or buffer overflow
      RADIOLIB_ASSERT(crcState); RADIOLIB_ASSERT(buffState);
      return(state);
    }

    int16_t receiveEx(uint8_t* data, size_t len, RadioLibTime_t exTimeout = 0, size_t* size = nullptr) {
      int16_t state = standby(); RADIOLIB_ASSERT(state);
      RadioLibTime_t timeout = MillisTOA(getTimeOnAir(len)) + 200ul + exTimeout;
      clearEvents();
      // ---- bulk debug ----- 
      if (btTxActive) {
        btTime = micros() - btTime;
        if (iF < 10) MemOff[iF] = btTime; iF++;
        if (btFirstPack) TxBTi.off_head = btTime;
          else TxBTi.off_reply = btTime; }
      // ---------------------
      state = startSingleRx(); RADIOLIB_ASSERT(state);
      if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(timeout)) == 0) { 
        state = standby(); RADIOLIB_ASSERT(state);
        state = clearIrqStatus(); RADIOLIB_ASSERT(state);
        return RADIOLIB_ERR_RX_TIMEOUT; }
      return ReadPacket(data, len, size);
    }

    int16_t WaitForPacket(uint8_t* buff, size_t* size, RadioLibTime_t timeout = 0) {
      if (!buff || !size) return RADIOLIB_ERR_INVALID_PARAMS; *size = 0;
      TickType_t waitTicks = timeout ? pdMS_TO_TICKS(timeout) : portMAX_DELAY;
      clearEvents(); int16_t state = startSingleRx(); RADIOLIB_ASSERT(state);
      if (ulTaskNotifyTake(pdTRUE, waitTicks) == 0) {
        state = standby(); RADIOLIB_ASSERT(state);
        state = clearIrqStatus(); RADIOLIB_ASSERT(state);
        return RADIOLIB_ERR_RX_TIMEOUT; }
      if (btRxActive) btTime = micros();      // bulk debug
      state = ReadPacket(buff, RADIOLIB_SX126X_MAX_PACKET_LENGTH, size);  
      if (btRxActive) {
        RxBTi.read_head = micros() - btTime;    // bulk debug  
        MemRead[iR] = RxBTi.read_head; iR++;}   // bulk debug
      return state;
    }

    int16_t WaitForReply(uint8_t* buff, uint8_t rplID, size_t nData = 0) {
      if (!buff || nData > RADIOLIB_SX126X_MAX_PACKET_LENGTH -2) return RADIOLIB_ERR_INVALID_PARAMS;
      size_t bSize = 2 + nData; size_t pkLen;
      int16_t state = receiveEx(buff, bSize, RX_TIMEOUT, &pkLen); RADIOLIB_ASSERT(state);
      if (pkLen != bSize || buff[0] != rplID) return RADIOLIB_ERR_BAD_PROTOCOL;
      if (buff[1] == statFailed) return RADIOLIB_ERR_REMOTE_FAILED;
      if (buff[1] != statSuccess) return RADIOLIB_ERR_BAD_PROTOCOL;
      return RADIOLIB_ERR_NONE;
    }

    int16_t BulkTransmit(uint8_t* data, uint16_t len, RadioLibTime_t* txToa = nullptr, 
        uint8_t* buff = nullptr, uint16_t blkID = 0x0000, uint16_t txDelay = 4, bool retTiming = false) {
      if (!data || len > MAX_BULK_SIZE || txDelay > RX_TIMEOUT -100) return RADIOLIB_ERR_INVALID_PARAMS;
      bool ownBuff = !buff; if (ownBuff) {
        buff = new uint8_t[RADIOLIB_SX126X_MAX_PACKET_LENGTH];
        if (!buff) return RADIOLIB_ERR_MEM_ALLOC_FAILED; }
      int16_t result = RADIOLIB_ERR_NONE;
      do {
        CRC16 crc; uint16_t bulk_ps = MAX_BULK_PS, pkSize = RADIOLIB_SX126X_MAX_PACKET_LENGTH; 
        RadioLibTime_t max_toa_us = getMaxTOA_us(); 
        RadioLibTime_t max_toa = MillisTOA(max_toa_us);
        TxBTi.toa_full = max_toa_us;  // bulk debug
        
        // send bulk command header  
        buff[0] = cmdBulk; buff[1] = 0x00; memcpy(&buff[2], &len, 2); 
        memcpy(&buff[4], &blkID, 2); memcpy(&buff[6], &txDelay, 2); memcpy(&buff[8], &retTiming, 1);
        RadioLibTime_t toa_us = getTimeOnAir(BLK_HEAD_SIZE);
        if (txToa) *txToa = MillisTOA(toa_us); 
        result = transmit(buff, BLK_HEAD_SIZE); 
        btTime = micros();  // bulk debug 
        BreakAssert(result);
        if (len) delay(4);  // first packet needs extra processing on the receiver side

        // send bulk command body (data packets)
        if (len) for (uint16_t i = 0; buff[1] < 0xFF; i += MAX_BULK_PS) {
          buff[1]++; 
          if (len >= MAX_BULK_PS) { 
            crc.update(&data[i], bulk_ps); memcpy(&buff[2], &data[i], bulk_ps); 
            len -= bulk_ps; if (txToa) *txToa += max_toa; 
          } else { 
            bulk_ps = len; len = 0;
            if (bulk_ps) { crc.update(&data[i], bulk_ps); memcpy(&buff[2], &data[i], bulk_ps); }
            if (MAX_BULK_PS - bulk_ps >= 2) 
              { crc.putValue(&buff[2+bulk_ps]); bulk_ps += 2; buff[1] = 0xFF; }
            pkSize = 2 + bulk_ps;
            toa_us = getTimeOnAir(pkSize);
            if (txToa) *txToa += MillisTOA(toa_us);
            if (buff[1] == 0xFF) TxBTi.toa_part = toa_us;  // bulk debug
          }
          if (delay) delay(txDelay);
          // --- bulk debug ---
          btTime = micros() - btTime;
          if (iF < 10) MemOff[iF] = btTime; iF++;
          if (btFirstPack) { 
            TxBTi.off_head = btTime;
            btFirstPack = false; 
          } else {
            if (buff[1] < 0xFF || pkSize == RADIOLIB_SX126X_MAX_PACKET_LENGTH) {
              if (btTime < TxBTi.off_min) TxBTi.off_min = btTime;
              if (btTime > TxBTi.off_max) TxBTi.off_max = btTime;
            } else TxBTi.off_part = btTime;
          }
          // -------------------
          result = transmit(buff, pkSize); 
          btTime = micros();  // bulk debug
          BreakAssert(result); 
        }
        BreakAssert(result);
        // wait for transfer acknowledgement
        result = WaitForReply(buff, rplBulk);
      } while (false);
      if (ownBuff) delete[] buff; 
      return result;
    }

    int16_t getBulkHeader(uint8_t* buff, size_t len, uint16_t* blkSize, uint16_t* blkID,
        uint16_t* txDelay, bool* retTiming = nullptr) {
      if (len != BLK_HEAD_SIZE || buff[0] != cmdBulk || buff[1] != 0x00) return RADIOLIB_ERR_INVALID_BULK_HDR;
      memcpy(blkSize, &buff[2], 2); memcpy(blkID, &buff[4], 2); memcpy(txDelay, &buff[6], 2); 
      if (retTiming) memcpy(retTiming, &buff[8], 1);
      return RADIOLIB_ERR_NONE;
    }

    int16_t BulkReceive(uint8_t* data, size_t len, uint16_t txDelay, uint8_t* buff = nullptr, size_t* bRead = nullptr) {
      if (bRead != nullptr) *bRead = 0; if (!data || len > MAX_BULK_SIZE) return RADIOLIB_ERR_INVALID_PARAMS;
      int16_t result = RADIOLIB_ERR_NONE;  CRC16 crc; uint16_t rx_crc = 0; 
      size_t pkSize = 0; bool ownBuff = !buff; RadioLibTime_t xTime;
      size_t ibR = 0; if (bRead == nullptr) bRead = &ibR; bool hasData = len > 0;
      TickType_t timeout = pdMS_TO_TICKS(getMaxTOA_ms() + RX_TIMEOUT);
      if (ownBuff) {
        buff = new uint8_t[RADIOLIB_SX126X_MAX_PACKET_LENGTH];
        if (!buff) return RADIOLIB_ERR_MEM_ALLOC_FAILED; }
      if (hasData) do { 
        clearEvents(); result = startReceive();
        BreakAssert(result); buff[1] = 0x00;
        RadioLibTime_t read_tmp = 0, work_tmp = 0;  // bulk debug 
        for (uint8_t pkIdx = 0x01; buff[1] < 0xFF; pkIdx++) {
          // --- bulk debug [work] ---
          xTime = micros() - btTime; if (iW < 10) MemWork[iW] = xTime; iW++;
          if (!btFirstPack) work_tmp = xTime; 
            else { RxBTi.work_head = xTime; btFirstPack = false; }
          // -------------------------
          uint32_t notifs = ulTaskNotifyTake(pdTRUE, timeout);
          btTime = micros();  // bulk debug
          if (notifs == 0) { result = RADIOLIB_ERR_RX_TIMEOUT; break; }
          if (notifs > 1)  { result = RADIOLIB_ERR_OUT_OF_SYNC; break; }
          result = ReadPacket(buff, RADIOLIB_SX126X_MAX_PACKET_LENGTH, &pkSize); BreakAssert(result);
          // --- bulk debug [read] ---
          if (read_tmp)
            if (buff[1] < 0xFF || pkSize == RADIOLIB_SX126X_MAX_PACKET_LENGTH) {
              if (read_tmp < RxBTi.read_min) RxBTi.read_min = read_tmp;
              if (read_tmp > RxBTi.read_max) RxBTi.read_max = read_tmp;
              if (work_tmp < RxBTi.work_min) RxBTi.work_min = work_tmp;
              if (work_tmp > RxBTi.work_max) RxBTi.work_max = work_tmp;
            } else { RxBTi.read_part = read_tmp; RxBTi.work_part = work_tmp; }
          xTime = micros() - btTime; if (iR < 10) MemRead[iR] = xTime; iR++;
          if (buff[1] < 0xFF) read_tmp = xTime; else RxBTi.read_reply = xTime; 
          // -------------------------
          if (pkSize < 4 || buff[0] != cmdBulk || buff[1] == 0) { result = RADIOLIB_ERR_BAD_PROTOCOL; break; }
          pkSize -= 2; // pkSize = payload size
          if (buff[1] == 0xFF) { pkSize -=2; memcpy(&rx_crc, &buff[2+pkSize], 2); }  
            else if (buff[1] != pkIdx) { result = RADIOLIB_ERR_OUT_OF_SYNC; break; }
          if (pkSize > len) { result = RADIOLIB_ERR_BUFF_OVERFLOW; break; }
          if (pkSize) {
            crc.update(&buff[2], pkSize); 
            memcpy(&data[*bRead], &buff[2], pkSize);
            *bRead += pkSize; len -= pkSize; }
        }
      } while (false);
      if (result == RADIOLIB_ERR_NONE) {
        // send transfer status reply
        buff[0] = rplBulk; 
        if (!hasData || (len == 0 && crc.getValue() == rx_crc)) buff[1] = statSuccess;
          else { buff[1] = statFailed; result = RADIOLIB_ERR_BULK_CORRUPTED; }
        if (delay) delay(txDelay); 
        xTime = micros() - btTime; if (iW < 10) MemWork[iW] = xTime; iW++;        // bulk debug
        if (btFirstPack) RxBTi.work_head = xTime; else RxBTi.work_reply = xTime;  // bulk debug
        int16_t state = transmit(buff, 2);
        if (result == RADIOLIB_ERR_NONE) result = state;
      }
      if (ownBuff) delete[] buff;
      return result;
    }

    void TxBulkTimingInit() {
      btTime = 0; btFirstPack = true; btTxActive = true;
      TxBTi = {}; TxBTi.off_min = 0xFFFFFFFF; 
      memset(MemOff, 0, sizeof(MemOff)); iF = 0;
    }
    inline void TxBulkTimingClose() { 
      btTxActive = false;
    }
    void RxBulkTimingInit() {
      btTime = 0; btFirstPack = true; btRxActive = true;
      RxBTi = {}; RxBTi.read_min = 0xFFFFFFFF; RxBTi.work_min = 0xFFFFFFFF;
      memset(MemRead, 0, sizeof(MemRead)); iR = 0;
      memset(MemWork, 0, sizeof(MemWork)); iW = 0;
    }    
    inline void RxBulkTimingClose() { 
      btRxActive = false;
    }
    inline int16_t TransmitRxBT(uint8_t* fs_buff) {
      delay(100);
      fs_buff[0] = rplRxBT; fs_buff[1] = statSuccess;
      memcpy(&fs_buff[2], &RxBTi, sizeof(RxBTi));
      return transmit(fs_buff, 2 + sizeof(RxBTi));
    }
    inline int16_t ReceiveRxBT(uint8_t* fs_buff) {
      int16_t state = WaitForReply(fs_buff, rplRxBT, sizeof(RxBTi));
      RADIOLIB_ASSERT(state);
      memcpy(&RxBTi, &fs_buff[2], sizeof(RxBTi));
      return RADIOLIB_ERR_NONE;
    }
    inline void UpdateDRate(uint16_t dSize, RadioLibTime_t time_us) { 
      Data.DataRate = 0; 
      if (time_us > 0) 
        Data.DataRate = static_cast<size_t>((static_cast<double>(dSize) / static_cast<double>(time_us)) * 1000000.0);
    }
    inline void getBulkJson(char* buff) {
      snprintf(buff, MAX_BLK_JSON +1, "{\"rate\":%u,"
        "\"ofhd\":%u,\"ofmi\":%u,\"ofmx\":%u,\"ofpt\":%u,\"ofrp\":%u,"
        "\"toaf\":%u,\"toap\":%u,"
        "\"rdhd\":%u,\"wkhd\":%u,\"rdmi\":%u,\"rdmx\":%u,\"wkmi\":%u,\"wkmx\":%u,"
        "\"rdpt\":%u,\"wkpt\":%u,\"rdrp\":%u,\"wkrp\":%u}",
        Data.DataRate, TxBTi.off_head, TxBTi.off_min, TxBTi.off_max, TxBTi.off_part, TxBTi.off_reply,
        TxBTi.toa_full, TxBTi.toa_part,
        RxBTi.read_head, RxBTi.work_head, RxBTi.read_min, RxBTi.read_max, RxBTi.work_min, RxBTi.work_max,
        RxBTi.read_part, RxBTi.work_part, RxBTi.read_reply, RxBTi.work_reply
      );        
    }
};

SPISettings mySPISettings(2000000, MSBFIRST, SPI_MODE0);  // 10 cm long wires SPI
//SX1262 LORA = new Module(PIN_CS, PIN_DIO1, PIN_RESET, PIN_BUSY, SPI, mySPISettings);
Module* mod = new Module(PIN_CS, PIN_DIO1, PIN_RESET, PIN_BUSY, SPI, mySPISettings);
MSX1262 LORA(mod);


// ----- Interrupt Code ------------------------------------

TaskHandle_t hIsrTask = NULL;  // protected by LORA.devTake() in TX_MODULE

void ISR_ATTR IrqDio1(void) { 
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  if (hIsrTask != NULL) vTaskNotifyGiveFromISR(hIsrTask, &xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken); 
}


#ifdef TX_MODULE //----------------------- TX MODULE (http server)---------------------------

const char ssid[]           = "ESP32";
const char password[]       = "loratest";

const char msgAccepted[]    = "Command accepted. Waiting for result...";
const char msgBusy[]        = "LoRa is Busy ! Please wait...";
const char msgBadCmd[]      = "Invalid command syntax.";
const char msgJBuffOver[]   = "JSON buffer overflow.";
const char msgNoJson[]      = "Request body is not JSON."; 
const char msgIntError[]    = "Internal error encountered.";

const char MIME_PLAIN[]     = "text/plain";
const char MIME_HTML[]      = "text/html";
const char MIME_JSON[]      = "application/json";
const char MIME_WOFF2[]     = "application/font-woff2";

const char pathIndex[]      = "/index.html";
const char pathRobo[]       = "/robo-reg.woff2";
const char pathRoboCnd[]    = "/robo-cnd-reg.woff2";
const char pathNextRnd[]    = "/next-rnd-bold.woff2";

#ifdef DEBUG_MODE
const char debug_WaitReply[]   = "[SX1262] Waiting for the reply...";
const char debug_BackListen[]  = "[SX1262] Back to listening mode...";
#endif

// Used to exit an AsyncWebServerRequest handler with a reply
#define StopLora()                  LORA.devTake(); LORA.Data.Running = false; LORA.devGive()
#define EndRplText(code, msg)       { request->send(code, MIME_PLAIN, FlashParams(msg)); return; }
#define EndRplCode(code)            { request->send(code); return; }
#define LoraEndRplText(code, msg)   { LORA.devGive(); request->send(code, MIME_PLAIN, FlashParams(msg)); return; }
#define LoraEndRplCode(code)        { LORA.devGive(); request->send(code); return; }

#define RssiTake()                  xSemaphoreTake(rssiMutex, portMAX_DELAY)
#define RssiGive()                  xSemaphoreGive(rssiMutex)

AsyncWebServer Server(80);

// ------ FILES section --------------------------------------

void handleRoot(AsyncWebServerRequest* request) {
  request->send(LittleFS, pathIndex, MIME_HTML);
}
void handleRoboto(AsyncWebServerRequest* request) {
  request->send(LittleFS, pathRobo, MIME_WOFF2);
}
void handleRoboCnd(AsyncWebServerRequest* request) {
  request->send(LittleFS, pathRoboCnd, MIME_WOFF2);
}
void handleNextRnd(AsyncWebServerRequest* request) {
  request->send(LittleFS, pathNextRnd, MIME_WOFF2);
}

// ------ Band Monitor section --------------------------------

void handleRSSI(AsyncWebServerRequest* request) {
  float band_rssi = LORA.getBandRSSI();
  char result[32];
  snprintf(result, sizeof(result), "{\"rssi\":%.2f}", band_rssi);
  request->send(200, MIME_JSON, result);
}
void handleStartRSSI(AsyncWebServerRequest* request) {
  LORA.startBandMonitor();
  request->send(200);
}
void handleStopRSSI(AsyncWebServerRequest* request) {
  LORA.stopBandMonitor();
  request->send(200);
}

// ------ CONFIG section --------------------------------------

void LoraConfigTask(void* pvParameters);

void handleGetCfg(AsyncWebServerRequest* request) {
  char result[MAX_CFG_JSON +1];  // (+1 for null terminator)
  LORA.getCfgJson(result);
  SerialTake(); Print("[SERVER] JSON Config: "); PrintLn(result); PrintLn(""); SerialGive(); 
  request->send(200, MIME_JSON, result); 
}

void handleSetCfgBody(AsyncWebServerRequest* request, uint8_t* data, size_t len, size_t index, size_t total) {
  static char jBuff[MAX_CFG_JSON +1];  // (+1 for null terminator)
  static size_t jPos = 0;          

  // Perform header checks only on first chunk
  if (index == 0) {
    LORA.devTake();
    if (LORA.Data.Running) LoraEndRplText(429, msgBusy);
    if (!request->hasParam("rem")) LoraEndRplText(400, msgBadCmd);
    if (!request->contentType().equals(MIME_JSON)) LoraEndRplText(400, msgNoJson); 
    if (request->contentLength() > MAX_CFG_JSON) LoraEndRplText(413, msgJBuffOver);
    String strParam = request->getParam("rem")->value();
    if (!strParam.equals("0") && !strParam.equals("1")) LoraEndRplText(400, msgBadCmd);
    LORA.Data.cfgRemote = strParam.equals("1"); LORA.Data.Running = true;
    SafePrintLn("[SERVER] Received valid LoRa reconfiguration request.");
    jPos = 0; jBuff[jPos] = '\0'; 
    LORA.devGive();    
  }
  // Append chunk to buffer
  if (jPos + len > MAX_CFG_JSON) { StopLora(); EndRplText(500, msgJBuffOver); }
  memcpy(jBuff + jPos, data, len); jPos += len;
  // Process only when full body received
  if (index + len == total) {
    jBuff[jPos] = '\0';  // Null-terminate string
    SerialTake(); Print("[SERVER] Received JSON string:"); PrintLn(jBuff); SerialGive();
    JsonDocument jDoc; DeserializationError badJson = deserializeJson(jDoc, jBuff);
    jPos = 0; jBuff[jPos] = '\0';
    bool badSyntax = !jDoc["freq"].is<uint32_t>() || !jDoc["txpwr"].is<int8_t>() || !jDoc["bandw"].is<uint8_t>() || 
      !jDoc["spread"].is<uint8_t>() || !jDoc["cdrate"].is<uint8_t>() || !jDoc["preamb"].is<uint8_t>();
    if (badJson || badSyntax) { StopLora(); EndRplText(400, msgBadCmd) };
    
    LORA.Data.Result = false;
    LORA.Data.UserCfg.freq   = jDoc["freq"].as<uint32_t>(); 
    LORA.Data.UserCfg.txpwr  = jDoc["txpwr"].as<int8_t>(); 
    LORA.Data.UserCfg.bandw  = jDoc["bandw"].as<uint8_t>(); 
    LORA.Data.UserCfg.spread = jDoc["spread"].as<uint8_t>(); 
    LORA.Data.UserCfg.cdrate = jDoc["cdrate"].as<uint8_t>(); 
    LORA.Data.UserCfg.preamb = jDoc["preamb"].as<uint8_t>(); 

    SerialTake(); Print("[SERVER] Starting LoRa Config task...");
    if (xTaskCreatePinnedToCore(LoraConfigTask, "LoraConfig", 4096, NULL, 1, &hIsrTask, 1) == pdPASS) { 
      PrintLn(debug_DoneLn); SerialGive(); 
      EndRplText(200, msgAccepted);
    } else { 
      PrintLn(debug_FailLn); SerialGive();
      StopLora(); EndRplText(500, msgIntError);
    } 
  }
}

void handleCfgRes(AsyncWebServerRequest* request) {
  LORA.devTake();
  if (LORA.Data.Running) LoraEndRplCode(202);
  if (!LORA.Data.Result) { request->send(500, MIME_PLAIN, LORA.Data.Error); LORA.devGive(); return; }
  LORA.devGive();
  SafePrintLn("[SERVER] LoRa was successfully configured.\n"); 
  EndRplCode(200);
}

void LoraConfigTask(void* pvParameters) {
  do {
    if (LORA.Data.cfgRemote && !LORA.ReadyForTx()) break;
    SerialTake();
    
    uint32_t tx_start; RadioLibTime_t tx_toa = 0;
    do {
      size_t bSize = 10; uint8_t buff[bSize]; 
      if (LORA.Data.cfgRemote) {
        // Sending configuration command and data
        buff[0] = cmdSetConfig; 
        memcpy(&buff[1], &LORA.Data.UserCfg.freq, 4);
        buff[5] = (uint8_t)LORA.Data.UserCfg.txpwr;
        buff[6] = LORA.Data.UserCfg.bandw;
        buff[7] = LORA.Data.UserCfg.spread;
        buff[8] = LORA.Data.UserCfg.cdrate;
        buff[9] = LORA.Data.UserCfg.preamb;
        PrintBuff(debug_TxBuff, buff, bSize);
        Print("[SX1262] Sending config buffer...");
        tx_start = millis(); tx_toa = MillisTOA(LORA.getTimeOnAir(bSize));
        if (!LORA.CheckResult(LORA.transmit(buff, bSize))) break;
        // Waiting for acknowledge
        Print(debug_WaitReply);
        if (!LORA.CheckResult(LORA.WaitForReply(buff, rplConfigRes))) break;
      }
      // Apply local configuration
      Print(debug_UpdateCfg);
      if (!LORA.CheckResult(LORA.ApplyUserCfg())) break;
      
      if (LORA.Data.cfgRemote) {
        // Testing the new configuration
        delay(100);
        Print("[SX1262] Sending Ping command...");
        tx_toa += MillisTOA(LORA.getTimeOnAir(2)); 
        if (!LORA.CheckResult(LORA.transmit(&cmdPing, 1))) break;
        Print(debug_WaitReply);
        if (!LORA.CheckResult(LORA.WaitForReply(buff, rplPing))) break;
      }
      // Configuration succeeded, update it
      LORA.UpdateConfig(); LORA.Data.Result = true;
      PrintLn(debug_CfgDone);

    } while (false);
    if (LORA.isCfgChanging()) {
      // Failed, undo reconfiguration
      PrintLn(debug_CfgFail); Print(debug_CfgUndo);
      if (!LORA.CheckResult(LORA.CancelConfig())) break;
    }
    LORA.TxDone(tx_start, tx_toa);
    Print(debug_BackListen);
    LORA.CheckResult(LORA.startReceive(), false);
    PrintLn(""); SerialGive();    
  } while (false);
  LORA.devTake(); LORA.Data.Running = false; hIsrTask = NULL; LORA.devGive();
  vTaskDelete(NULL);
}

// ------ TEST section ----------------------------------------

void LoraTestTask(void* pvParameters);

void handleDoTest(AsyncWebServerRequest* request) {
  LORA.devTake(); 
  if (LORA.Data.Running) LoraEndRplText(429, msgBusy);
  if (!request->hasParam("buff")) LORA.Data.buffSize = DEF_BUFF_SIZE; else {
    String strParam = request->getParam("buff")->value();
    char* endptr; LORA.Data.buffSize = strtoul(strParam.c_str(), &endptr, DEC);
    if (*endptr != '\0') LORA.Data.buffSize = DEF_BUFF_SIZE;
  } 
  LORA.Data.Result = false; 
  SerialTake(); Print("[SERVER] Starting LoRa Test task...");
  if (xTaskCreatePinnedToCore(LoraTestTask, "LoraTest", 4096, NULL, 1, &hIsrTask, 1) == pdPASS) { 
    PrintLn(debug_DoneLn); SerialGive(); 
    LORA.Data.Running = true; 
    LoraEndRplText(200, msgAccepted);
  } else { 
    PrintLn(debug_FailLn); SerialGive();
    LoraEndRplText(500, msgIntError);
  } 
}

void handleTestRes(AsyncWebServerRequest* request) {
  LORA.devTake();
  if (LORA.Data.Running) LoraEndRplCode(202);
  if (!LORA.Data.Result) { request->send(500, MIME_PLAIN, LORA.Data.Error); LORA.devGive(); return; }
  char result[MAX_RES_JSON]; LORA.getTestJson(result); LORA.devGive(); 
  SerialTake(); Print("[SERVER] JSON Result: "); PrintLn(result); PrintLn(""); SerialGive(); 
  request->send(200, MIME_JSON, result);
}

void LoraTestTask(void* pvParameters) {
  do {
    if (!LORA.ReadyForTx()) break;
    SerialTake();

    uint32_t tx_start; RadioLibTime_t tx_toa = 0; 
    do {
      // Sending test command and data
      size_t bSize = max(LORA.Data.buffSize, 14u); uint8_t buff[bSize]; 
      buff[0] = cmdStartTest;
      for (int i = 1; i < bSize; i++) buff[i] = i; 
      PrintBuff(debug_TxBuff, buff, bSize);
      Print("[SX1262] Sending test buffer...");
      tx_start = millis(); tx_toa = MillisTOA(LORA.getTimeOnAir(bSize));
      if (!LORA.CheckResult(LORA.transmit(buff, bSize))) break;
      // Waiting for the same size test buffer reply
      Print(debug_WaitReply);
      if (!LORA.CheckResult(LORA.WaitForReply(buff, rplTestRes, bSize-2))) break;
      // Updating results
      memcpy(&LORA.Data.tx_rssi, &buff[2], 4);   LORA.Data.rx_rssi  = LORA.getRSSI(); 
      memcpy(&LORA.Data.tx_snr, &buff[6], 4);    LORA.Data.rx_snr   = LORA.getSNR();
      memcpy(&LORA.Data.tx_fqerr, &buff[10], 4); LORA.Data.rx_fqerr = LORA.getFrequencyError(); 
      LORA.Data.Result = true; 
    } while (false); 
    LORA.TxDone(tx_start, tx_toa);

    Print(debug_BackListen);
    LORA.CheckResult(LORA.startReceive(), false);
    PrintLn(""); SerialGive();    
  } while (false);
  LORA.devTake(); LORA.Data.Running = false; hIsrTask = NULL; LORA.devGive();
  vTaskDelete(NULL);
}

// ------ BULK section ----------------------------------------

void LoraBulkTask(void* pvParameters);

void handleBulkTest(AsyncWebServerRequest* request) {
  LORA.devTake(); 
  if (LORA.Data.Running) LoraEndRplText(429, msgBusy);
  String strParam; strParam.reserve(16); char* endptr;
  if (!request->hasParam("buff") || !request->hasParam("delay")) LoraEndRplText(400, msgBadCmd);
  strParam = request->getParam("buff")->value();
  LORA.Data.buffSize = strtoul(strParam.c_str(), &endptr, DEC);
  if (*endptr != '\0') LoraEndRplText(400, msgBadCmd);
  strParam = request->getParam("delay")->value();
  LORA.Data.bulkDelay = strtoul(strParam.c_str(), &endptr, DEC);
  if (*endptr != '\0') LoraEndRplText(400, msgBadCmd);
  LORA.Data.Result = false; 
  SerialTake(); Print("[SERVER] Starting LoRa Bulk Test task...");
  if (xTaskCreatePinnedToCore(LoraBulkTask, "LoraBulkTest", 4096, NULL, 1, &hIsrTask, 1) == pdPASS) { 
    PrintLn(debug_DoneLn); SerialGive(); 
    LORA.Data.Running = true; 
    LoraEndRplText(200, msgAccepted);
  } else { 
    PrintLn(debug_FailLn); SerialGive();
    LoraEndRplText(500, msgIntError);
  } 
}

void handleBulkRes(AsyncWebServerRequest* request) {
  LORA.devTake();
  if (LORA.Data.Running) LoraEndRplCode(202);
  if (!LORA.Data.Result) { request->send(500, MIME_PLAIN, LORA.Data.Error); LORA.devGive(); return; }
  char* result = new char[MAX_BLK_JSON]; if (!result) LoraEndRplText(500, msgIntError);
  LORA.getBulkJson(result); LORA.devGive(); 
  SerialTake(); PrintLn("[SERVER] Bulk JSON Result:"); PrintLn(result); PrintLn(""); SerialGive(); 
  request->send(200, MIME_JSON, result);
}

void LoraBulkTask(void* pvParameters) {
  do {
    if (!LORA.ReadyForTx()) break;

    SerialTake();
    Print("Test params:  Bulk Size = "); Print(LORA.Data.buffSize);
    Print(",  Packet Delay = "); PrintLn(LORA.Data.bulkDelay);

    uint32_t tx_start; RadioLibTime_t tx_toa = 0;
    uint16_t dSize = LORA.Data.buffSize;
    uint8_t* data = new uint8_t[dSize];
    uint8_t* buff = new uint8_t[RADIOLIB_SX126X_MAX_PACKET_LENGTH]; 
    do {
      if (!data || !buff) { PrintLn(debug_MallocFail); LORA.SetErrorMsg(RADIOLIB_ERR_MEM_ALLOC_FAILED); break; }

      // fill the data buffer with some data
      uint8_t val = 0x01;
      for (uint16_t i = 0; i < dSize; i += 16) {
        uint16_t blockSize = (dSize - i >= 16) ? 16 : (dSize - i);
        memset(data + i, val, blockSize);        
        val++; if (val == 0x00) val = 0x01; }      

      // start the Bulk test  
      tx_start = millis();  
      LORA.TxBulkTimingInit();  // bulk debug
      Print("[SX1262] Transmitting bulk data packets...");
      RadioLibTime_t time_us = micros(); 
      int16_t state = LORA.BulkTransmit(data, dSize, &tx_toa, buff, 0xABCD, LORA.Data.bulkDelay, true);
      time_us = micros() - time_us;
      LORA.TxBulkTimingClose();  // bulk debug
      if (!LORA.CheckResult(state)) break; 

      // receive timing results
      Print("[SX1262] Waiting for timing results...");
      state = LORA.ReceiveRxBT(buff);
      LORA.UpdateDRate(dSize, time_us);
      if (!LORA.CheckResult(state)) break;

      Print("Transfer time: "); Print(static_cast<double>(time_us) / 1000.0, 2); 
      Print(" ms,  Data rate: "); Print(LORA.Data.DataRate); PrintLn(" bytes/s");

      PrintLn("\nDebug:");
      Print("Off Time = "); for (int i = 0; i < 10; i++) { Print(LORA.MemOff[i]); if (i < 9) Print(", "); else PrintLn("\n"); }

      LORA.Data.Result = true;

    } while (false); 
    LORA.TxDone(tx_start, tx_toa);
    if (data) delete[] data; if (buff) delete[] buff;

    Print(debug_BackListen);
    LORA.CheckResult(LORA.startReceive(), false);
    PrintLn(""); SerialGive();    
  } while (false);

  LORA.devTake(); LORA.Data.Running = false; hIsrTask = NULL; LORA.devGive();
  vTaskDelete(NULL);
}


#else  //----------------------------- RX MODULE (LoRa server) -----------------------------

void LoraServerTask(void* pvParameters) {
  size_t bSize; int16_t state;
  uint8_t* buff = new uint8_t[RADIOLIB_SX126X_MAX_PACKET_LENGTH]; 
  if (!buff) SafePrintLn(debug_MallocFail);
  else while (true) {

    // Wait for RF command (DIO1 interrupt)
    SafePrintLn("[SX1262] Listening for packets...");
    LORA.RxBulkTimingInit(); // bulk debug
    state = LORA.WaitForPacket(buff, &bSize);
    if (state == RADIOLIB_ERR_CRC_MISMATCH) {
      SafePrintLn("[SX1262] Corrupt packet has been received and ignored.\n"); continue; }
    else if (state != RADIOLIB_ERR_NONE) { 
      SerialTake(); Print("[SX1262] Something really bad happened. Error code: "); 
      PrintLn(state); PrintLn(""); SerialGive(); break; }
    SerialTake(); 
    if (buff[0] != cmdBulk) PrintLn("[SX1262] Valid packet has been received. Processing...");   // bulk debug

    // Handle the requested command
    switch (buff[0]) {

      case cmdSetConfig: {
        PrintLn("[SYSTEM] Module reconfiguration requested.");
        if (bSize != 10) { Print("[SYSTEM] Invalid packet size: "); Print(bSize);  PrintLn(" bytes."); break; }
        PrintBuff(debug_RxBuff, buff, bSize);
        // Reading configuration data
        memcpy(&LORA.Data.UserCfg.freq, &buff[1], 4);   
        LORA.Data.UserCfg.txpwr  = (int8_t)buff[5];
        LORA.Data.UserCfg.bandw  = buff[6];
        LORA.Data.UserCfg.spread = buff[7];
        LORA.Data.UserCfg.cdrate = buff[8];
        LORA.Data.UserCfg.preamb = buff[9];
        // Sending acknowledge
        buff[0] = rplConfigRes; buff[1] = statSuccess; bSize = 2;
        delay(100); Print(debug_SendAckn);
        if (!LORA.CheckResult(LORA.transmit(buff, bSize), false)) break;
        do {
          // Apply local configuration
          Print(debug_UpdateCfg);
          if (!LORA.CheckResult(LORA.ApplyUserCfg(), false)) break;
          // Testing the new configuration
          Print("[SX1262] Waiting for Ping...");
          if (!LORA.CheckResult(LORA.receiveEx(buff, sizeof(buff), RX_TIMEOUT, &bSize), false)) break;
          if (bSize != 1 || buff[0] != cmdPing) { PrintLn("[SYSTEM] Ping not received."); break; }
          buff[0] = rplPing; buff[1] = statSuccess; bSize = 2; delay(100);
          Print(debug_SendAckn);
          if (!LORA.CheckResult(LORA.transmit(buff, bSize), false)) break;
          // Configuration succeeded, update it
          LORA.UpdateConfig(); LORA.Data.Result = true;
          PrintLn(debug_CfgDone);
        } while (false);
        if (LORA.isCfgChanging()) {
          // Failed, undo reconfiguration
          PrintLn(debug_CfgFail); Print(debug_CfgUndo);
          LORA.CheckResult(LORA.CancelConfig(), false);
        }      
        break;
      }

      case cmdStartTest: {
        PrintLn("[SYSTEM] Test command requested.");
        PrintBuff(debug_RxBuff, buff, bSize);
        buff[0] = rplTestRes;
        buff[1] = statSuccess; 
        float Data = LORA.getRSSI(); memcpy(&buff[2], &Data, 4);
        Data = LORA.getSNR(); memcpy(&buff[6], &Data, 4);
        Data = LORA.getFrequencyError(); memcpy(&buff[10], &Data, 4);
        for (int i = 14; i < bSize; i++) { buff[i] = i - 13; }
        PrintBuff(debug_TxBuff, buff, bSize);
        delay(100); Print(debug_SendReply);
        LORA.CheckResult(LORA.transmit(buff, bSize), false);        
        break;
      }

      case cmdBulk: {
        uint16_t blkSize, blkID, txDelay; bool retTiming;
        state = LORA.getBulkHeader(buff, bSize, &blkSize, &blkID, &txDelay, &retTiming);
        BreakAssertMsg(state, "[SYSTEM] Invalid bulk test request.");
        //Print("[SYSTEM] Bulk test requested: "); Print(blkSize); Print(" bytes, "); 
        //PrintHex(blkID, 4); Print(" ID, "); Print(txDelay); PrintLn(" ms"); 
        uint8_t* data = new uint8_t[blkSize]; if (!data) { PrintLn(debug_MallocFail); break; }
        do {
          state = LORA.BulkReceive(data, blkSize, txDelay, buff);
          LORA.RxBulkTimingClose();
          BreakAssertMsg(state, "[SYSTEM] LoRa bulk test failed.");
          PrintLn("[SYSTEM] The transfer was completed successfully !");
          if (retTiming) {
            Print("[SX1262] Sending timing results..."); 
            LORA.CheckResult(LORA.TransmitRxBT(buff), false);
          }
          PrintLn("\nDebug:");
          Print("Read = "); for (int i = 0; i < 10; i++) { Print(LORA.MemRead[i]); if (i < 9) Print(", "); else PrintLn(""); }
          Print("Work = "); for (int i = 0; i < 10; i++) { Print(LORA.MemWork[i]); if (i < 9) Print(", "); else PrintLn(""); }
        } while (false);
        delete[] data;
        break;
      }

      default: PrintLn("[SYSTEM] Unknown command.");
    }

    PrintLn(""); SerialGive();
  }
  SafePrintLn("[SYSTEM] LoRa server stopped.");
  hIsrTask = NULL; vTaskDelete(NULL);
}

#endif 


// ========================= PROGRAM STARTING POINT ==============================

void setup() {
  #ifdef DEBUG_MODE
    Serial.begin(115200); delay(1000);
    serialMutex = xSemaphoreCreateMutex();
  #endif

  #if !defined(ESP32_BOARD) && !defined(PICO_BOARD)
    PrintLn("Unsupported board !"); return;
  #endif

  #ifdef TX_MODULE   // ----------- TX MODULE SETUP --------------

    PrintLn("[SYSTEM] Starting program for TX Module (Client)...");

    //listPartitions();
    Print("[SYSTEM] Mouning file system [LittleFS]...");
    if (LittleFS.begin(true, "/LFS", 5, "littlefs")) PrintLn(debug_Done);
      else { PrintLn(debug_Fail); return; }
    //listDir(LittleFS, "/", 10);  

    Print("[SYSTEM] Starting Access Point...");  
    if (WiFi.softAP(ssid, password)) PrintLn(debug_Done);
      else { PrintLn(debug_Fail); return; }
    Print("[SYSTEM] Access Point IP: "); PrintLn(WiFi.softAPIP());    

    Server.on("/", HTTP_GET, handleRoot);
    Server.on(pathRobo, HTTP_GET, handleRoboto);
    Server.on(pathRoboCnd, HTTP_GET, handleRoboCnd);
    Server.on(pathNextRnd, HTTP_GET, handleNextRnd);
    Server.on("/getcfg", HTTP_GET, handleGetCfg);
    Server.on("/setcfg", HTTP_POST, [](AsyncWebServerRequest *request) {}, nullptr, handleSetCfgBody);
    Server.on("/rescfg", HTTP_GET, handleCfgRes);
    Server.on("/dotest", HTTP_POST, handleDoTest);
    Server.on("/restest", HTTP_GET, handleTestRes);
    Server.on("/rssi", HTTP_GET, handleRSSI);
    Server.on("/rssion", HTTP_POST, handleStartRSSI);
    Server.on("/rssioff", HTTP_POST, handleStopRSSI);
    Server.on("/bulk", HTTP_POST, handleBulkTest);
    Server.on("/resbulk", HTTP_GET, handleBulkRes);    

  #else              // ------------ RX MODULE SETUP --------------

    PrintLn("[SYSTEM] Starting program for RX Module (Server)...");

  #endif             // ------------ COMMON CODE INIT -------------
                     
  #if defined(ESP32_BOARD) 
    SPI.begin(PIN_CLK, PIN_MISO, PIN_MOSI, PIN_CS);
  #elif defined(PICO_BOARD)
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH); 
    SPI.setSCK(PIN_CLK);
    SPI.setRX(PIN_MISO);  // SPI0 RX (MISO)
    SPI.setTX(PIN_MOSI);  // SPIO TX (MOSI)
    SPI.begin();
  #endif  

  Print("[SX1262] Initializing LoRa...");
  LORA.setRfSwitchPins(PIN_RX_EN, PIN_TX_EN);
  LORA.setDio1Action(IrqDio1);
  if (!LORA.CheckResult(LORA.beginDefault(), false)) return;
  Print("[SX1262] Setup RX boosted gain mode...");
  if (!LORA.CheckResult(LORA.setRxBoostedGainMode(true), false)) return;

  #ifdef TX_MODULE   // ------------- TX MODULE LAUNCH ------------

    SerialTake();

    Print("[SX1262] Entering listening mode...");
    if (!LORA.CheckResult(LORA.startReceive(), false)) return;

    Print("[SYSTEM] Starting HTTP Server...");
    Server.begin(); PrintLn(debug_Done);      

    PrintLn(""); SerialGive();

  #else              // ------------- RX MODULE LAUNCH ------------- 

    SerialTake();
    
    #if defined(ESP32_BOARD)
      Print("[SYSTEM] Starting LoRa Server task...");
      if (xTaskCreatePinnedToCore(LoraServerTask, "LoraServerTask", 4096, NULL, 1, &hIsrTask, 1) == pdPASS) 
        PrintLn(debug_Done); else PrintLn(debug_Fail);
    #elif defined(PICO_BOARD)
      Print("[SYSTEM] Starting LoRa Server task...");
      if (xTaskCreateAffinitySet(LoraServerTask, "LoraServerTask", 4096, NULL, 1, 2, &hIsrTask) == pdPASS) 
        PrintLn(debug_Done); else PrintLn(debug_Fail);
    #endif

    PrintLn(""); SerialGive();

  #endif             // ----------------------------------------------
}

void loop() { delay(1000); }


/* ------ TO DO ------------------------------
 - better duty cycle handling, taking into account the band
 - use hardware CRC
 */