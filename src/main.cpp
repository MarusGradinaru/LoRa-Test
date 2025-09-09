
// ----- User Settings ----------------------------------------------------------

#define DEBUG_MODE   // comment to stop debug messages
#define TX_MODULE    // comment for RX_MODULE  (TX not available for PICO_BOARD)

// Module pin configuration (GPIO)

#ifdef ESP32_BOARD
  #define PIN_CS     5
  #define PIN_CLK    18
  #define PIN_MOSI   23
  #define PIN_MISO   19
  #define PIN_RESET  27
  #define PIN_BUSY   26
  #define PIN_RX_EN  25
  #define PIN_TX_EN  33
  #define PIN_DIO1   32
#endif

#ifdef PICO_BOARD
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


//-------------------------------------------------------------------------------

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


// ------ Debug Functions ----------------------------------

#ifdef DEBUG_MODE

  #define SerialTake()                xSemaphoreTake(serialMutex, portMAX_DELAY)
  #define SerialGive()                xSemaphoreGive(serialMutex);
  #define Print(msg)                  Serial.print(msg)
  #define PrintLn(msg)                Serial.println(msg)
  #define SafePrintLn(msg)            _SafePrintLn(msg)
  #define PrintBuff(msg, buff, len)   _PrintBuff(msg, buff, len)
  
SemaphoreHandle_t serialMutex;

void _SafePrintLn(const char* msg) {
  xSemaphoreTake(serialMutex, portMAX_DELAY);
  Serial.println(msg);
  xSemaphoreGive(serialMutex);
}

void _PrintBuff(const char* msg, const uint8_t* buffer, uint16_t len) {
  Serial.print("[SYSTEM] "); Serial.print(msg); Serial.print(" = ");
  for (int i = 0; i < len; i++) {
    if (buffer[i] < 0x10) Serial.print("0");
    Serial.print(buffer[i], HEX);
    if (i < len-1) Serial.print(", ");
  }
  Serial.println();
}

#else  // RELEASE MODE

  #define SerialTake()                    ((void)0)
  #define SerialGive()                    ((void)0)
  #define Print(msg)                      ((void)0)
  #define PrintLn(msg)                    ((void)0)
  #define SafePrintLn(msg)                ((void)0)
  #define PrintBuff(msg, buff, len)       ((void)0)

#endif


// ------ Basic Functions -------------------------------------------------

// Some useful macros
#define FlashParams(fstr)  reinterpret_cast<const uint8_t*>(fstr), sizeof(fstr)-1
#define MillisTOA(toa)     (RadioLibTime_t)((toa + 999ul) / 1000ul)

bool CheckLoraResult(int16_t state, bool errStop = false) {
  if (state == RADIOLIB_ERR_NONE) {
    PrintLn("done !"); return true;
  } else {
    Print("failed, code "); PrintLn(state);
    if (errStop) { while (true) delay(1000); }
    return false;
  }
} 

#ifdef ESP32_BOARD

void listPartitions() {
  Serial.println("Partitions:");
  esp_partition_iterator_t it = esp_partition_find(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_ANY, NULL);
  while (it != NULL) {
    const esp_partition_t *part = esp_partition_get(it);
    Serial.printf(" label='%s'  type=0x%02x  subtype=0x%02x  addr=0x%06x  size=0x%06x\n",
      part->label, part->type, part->subtype, part->address, part->size);
    it = esp_partition_next(it);
  }
  esp_partition_iterator_release(it);
}

void listDir(fs::FS& fs, const char* dirname, uint8_t levels) {
  Serial.printf("Listing directory: %s\r\n", dirname);

  File root = fs.open(dirname);
  if (!root) { Serial.println("- failed to open directory"); return; }
  if (!root.isDirectory()) { Serial.println(" - not a directory"); return; }

  File file = root.openNextFile();
  while (file) {
    if (file.isDirectory()) {
      Serial.print("  DIR : ");
      Serial.println(file.name());
      if (levels) { listDir(fs, file.path(), levels - 1); }
    } else {
      Serial.print("  FILE: ");
      Serial.print(file.name());
      Serial.print("\tSIZE: ");
      Serial.println(file.size());
    }
    file = root.openNextFile();
  }
}

#endif


// ------ Main LoRa SX1262 Class ------------------------------------------

#define  HTTP_MSG_SIZE  60
#define  MAX_CFG_JSON   120
#define  MAX_RES_JSON   160
#define  DEF_BUFF_SIZE  32

const uint16_t cmdStartTest = 0xC1E8u;
const uint16_t rplTestRes   = 0xE8C1u;
const uint16_t cmdSetConfig = 0x7A94u;
const uint16_t rplConfigRes = 0x947Au;
const uint16_t cmdPing      = 0xAA55u;
const uint16_t rplPing      = 0x55AAu;
const uint16_t statSuccess  = 0x2664u;
const uint16_t statFailed   = 0x6246u;

const char msgDCWait[]      = "Please wait %s seconds more !";   
const char msgLoraTimeout[] = "The other LoRa is not responding !";
const char msgLLFail[]      = "Local LoRa%s failed, code %d";
const char msgRLFail[]      = "Remote LoRa failed its job.";
const char msgBadLRpl[]     = "Invalid LoRa response.";

const float listBandwidth[] = {7.8f, 10.4f, 15.6f, 20.8f, 31.25f, 41.7f, 62.5f, 125.0f, 250.0f, 500.0f};

struct LoraUserCfg {
  uint32_t freq;  // in Hz
  int8_t txpwr;   // in dBm
  uint8_t bandw, spread, cdrate, preamb;  // in list indexes, starting from zero
};

struct LoraFixedCfg {
  uint8_t syncw = SYNCW;
  uint8_t txdc = TX_DC;
  float xovolt = XOV;
  bool useldo = LDO;
};

class MSX1262 : public SX1262 {   //----------------------------------------------------------
  private:
    struct DeviceData {
      bool Running = false;
      bool Result = false;
      String Error;
      uint32_t rxTimeout = 0;
      uint32_t buffSize = DEF_BUFF_SIZE;
      // Test results [out]
      float tx_rssi, rx_rssi, tx_snr, rx_snr, tx_fqerr, rx_fqerr;
      // Config params [in]
      bool cfgRemote = true;
      LoraUserCfg UserCfg;
    };
    SemaphoreHandle_t devMutex = nullptr;  // protect Data

    LoraFixedCfg FCfg; LoraUserCfg UCfg; bool cfgChanging = false; 
    SemaphoreHandle_t cfgMutex = nullptr;  // protect Cfg (UCfg + Data.uParams) & FCfg & cfgChanging

    uint32_t txNext = 0; float psConst;  // TX duty cycle

    int16_t Reconfigure(const LoraUserCfg& params, bool chg, bool forced) {
      cfgTake();
      if (chg) cfgChanging = true;
      int16_t state = RADIOLIB_ERR_NONE;
      do {
        state = standby(); if (state != RADIOLIB_ERR_NONE) break;
        if (forced || Cfg->freq != params.freq) {
          state = setFrequency((float)params.freq / 1000000.0f); 
          if (state != RADIOLIB_ERR_NONE) break; 
        }
        if (forced || Cfg->txpwr != params.txpwr) { 
          state = setOutputPower(params.txpwr); 
          if (state != RADIOLIB_ERR_NONE) break; 
        }
        if (forced || Cfg->bandw != params.bandw) { 
          state = setBandwidth(listBandwidth[params.bandw]); 
          if (state != RADIOLIB_ERR_NONE) break; 
        }
        if (forced || Cfg->spread != params.spread) {
          state = setSpreadingFactor(5 + params.spread);
          if (state != RADIOLIB_ERR_NONE) break; 
        }
        if (forced || Cfg->cdrate != params.cdrate) {
          state = setCodingRate(5 + params.cdrate);
          if (state != RADIOLIB_ERR_NONE) break; 
        }
        if (forced || Cfg->preamb != params.preamb) {
          state = setPreambleLength(6 + params.preamb);
          if (state != RADIOLIB_ERR_NONE) break; 
        }  
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
      float band_rssi;
      do {
        Lora.devTake(); if (!Lora.Data.Running) band_rssi = Lora.getRSSI(false); Lora.devGive();
        Lora.rssiTake(); Lora.BandRSSI = band_rssi; Lora.rssiGive();
      } while (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1000)) == 0);
      Lora.bmtDone = true;
      vTaskDelete(NULL);
    }

  public:   //---------------------------------------------------------------------
    const LoraUserCfg* Cfg = &UCfg;  // read only content
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

    void getTestJson(char* buff) {  // require devTake() or Data.Running 
      snprintf(buff, MAX_RES_JSON, 
      "{\"tx_rssi\":%.2f,\"rx_rssi\":%.2f,\"tx_snr\":%.2f,\"rx_snr\":%.2f,\"tx_fqerr\":%.1f,\"rx_fqerr\":%.1f}",
      Data.tx_rssi, Data.rx_rssi, Data.tx_snr, Data.rx_snr, Data.tx_fqerr, Data.rx_fqerr);
    }

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
      snprintf(buff, MAX_CFG_JSON, 
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
      snprintf(buff, sizeof(buff), msgDCWait, fbuf);
      Data.Error = buff;
    }

    bool ReadyForTx() {
      int32_t delta = (int32_t)(millis() - txNext);
      if (delta >= 0) return true; else {
        float wait_time = ((float)(-delta) / 1000.0f);
        FormatErrorWait(wait_time); return false; 
      }
    }

    void TxDone(uint32_t tx_start, uint32_t tx_toa) {
      if (tx_toa == 0) return;
      uint32_t ps_time = ceil(tx_toa * psConst);
      txNext = tx_start + tx_toa + ps_time;      
    }

    int16_t beginDefault() {
      cfgTake();
      int16_t state = begin((float)Cfg->freq / 1000000.0f, listBandwidth[Cfg->bandw], 5 + Cfg->spread, 
      5 + Cfg->cdrate, FCfg.syncw, Cfg->txpwr, 6 + Cfg->preamb, FCfg.xovolt, FCfg.useldo);
      cfgGive();
      return state;
    }
    
    // Works only for explicit header !
    int16_t receiveEx(uint8_t* data, size_t len, RadioLibTime_t exTimeout) {
      int16_t state = standby(); RADIOLIB_ASSERT(state);
      RadioLibTime_t hwTimeout = 0; RadioLibTime_t swTimeout = 0;
      if (getPacketType() != RADIOLIB_SX126X_PACKET_TYPE_LORA) return RADIOLIB_ERR_UNKNOWN;
      // calculate hardware timeout (100 LoRa symbols, the default for SX127x series)
      cfgTake(); float symbolLength = (float)(uint32_t(1) << (5+Cfg->spread)) / 
        (listBandwidth[Cfg->bandw] * 1000); cfgGive();
      hwTimeout = (RadioLibTime_t)(symbolLength * 100.0f);
      if (hwTimeout < 100) hwTimeout = 100;  // limit it to a resonable minimum of 100 ms
      // calculate software timeout, taking into account the full packet length and 200 ms safety margin
      swTimeout = (RadioLibTime_t)((getTimeOnAir(len) + 999ul) / 1000ul) + 200ul;
      hwTimeout += exTimeout; swTimeout += exTimeout;
      // start reception
      uint32_t timeoutValue = (uint32_t)(((float)hwTimeout * 1000.0f) / 15.625f);
      if (timeoutValue > 0xFFDC00) timeoutValue = 0xFFDC00;  // limit it to max 262s - from datasheet !
      state = startReceive(timeoutValue); RADIOLIB_ASSERT(state);
      // wait for packet reception or timeout
      uint8_t pinIrq = getMod()->getIrq();
      bool softTimeout = false; RadioLibTime_t start = millis();
      while (!digitalRead(pinIrq)) 
        { delay(10); if (millis() - start > swTimeout) { softTimeout = true; break; } }
      // if it was a timeout, this will return an error code
      state = standby();
      if ((state != RADIOLIB_ERR_NONE) && (state != RADIOLIB_ERR_SPI_CMD_TIMEOUT)) return(state);
      // check whether this was a timeout or not
      if ((getIrqFlags() & RADIOLIB_SX126X_IRQ_TIMEOUT) || softTimeout)
        { standby(); clearIrqStatus(); return RADIOLIB_ERR_RX_TIMEOUT; }
      // read the received data
      return(readData(data, len));
    }

    bool WaitForReplay(uint8_t* buff, size_t len, uint16_t rplID, size_t nData = 0) {
      Print("[SX1262] Waiting for replay... ");
      int16_t state = receiveEx(buff, len, Data.rxTimeout);
      CheckLoraResult(state);
      if (state == RADIOLIB_ERR_RX_TIMEOUT) { Data.Error = msgLoraTimeout; return false; }  
      if (state != RADIOLIB_ERR_NONE) { FormatError(msgLLFail, " RX", state); return false; }
      size_t pkLen = getPacketLength();
      PrintBuff("Received buffer", buff, min(len, pkLen));
      if ((pkLen != 4 + nData ) || (memcmp(buff, &rplID, 2) != 0)) 
        { Data.Error = msgBadLRpl; return false; }
      if (memcmp(&buff[2], &statFailed, 2) == 0)
        { Data.Error = msgRLFail; return false; }  
      if (memcmp(&buff[2], &statSuccess, 2) != 0)
        { Data.Error = msgBadLRpl; return false; }
      return true;  
    }    
};

SPISettings mySPISettings(2000000, MSBFIRST, SPI_MODE0);  // 10 cm long wires SPI
//SX1262 LORA = new Module(PIN_CS, PIN_DIO1, PIN_RESET, PIN_BUSY, SPI, mySPISettings);
Module* mod = new Module(PIN_CS, PIN_DIO1, PIN_RESET, PIN_BUSY, SPI, mySPISettings);
MSX1262 LORA(mod);


// ------ Interrupt Code -----------------------------------

TaskHandle_t hIsrTask = NULL;  // protected by LORA.devTake() in TX_MODULE

void ISR_ATTR IrqDio1(void) { 
  if (hIsrTask != NULL) vTaskNotifyGiveFromISR(hIsrTask, NULL); 
}


#ifdef TX_MODULE //----------------------- TX MODULE (http server)---------------------------

const char ssid[]           = "ESP32";
const char password[]       = "loratest";

const char msgAccepted[]    = "Command accepted. Waiting for result...";
const char msgBusy[]        = "LoRa is Busy ! Please wait...";
const char msgBadCmd[]      = "Invalid command syntax.";
const char msgIntError[]    = "Internal error encountered.";
const char msgJBuffOver[]   = "JSON buffer overflow.";
const char msgNoJson[]      = "Request body is not JSON."; 

const char MIME_PLAIN[]     = "text/plain";
const char MIME_HTML[]      = "text/html";
const char MIME_JSON[]      = "application/json";
const char MIME_WOFF2[]     = "application/font-woff2";

const char pathIndex[]      = "/index.html";
const char pathRobo[]       = "/robo-reg.woff2";
const char pathRoboCnd[]    = "/robo-cnd-reg.woff2";
const char pathNextRnd[]    = "/next-rnd-bold.woff2";

// Used to exit an AsyncWebServerRequest handler with a replay
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
  char result[MAX_CFG_JSON]; 
  LORA.getCfgJson(result);
  SerialTake(); Print("[SERVER] JSON Config: "); PrintLn(result); PrintLn(""); SerialGive(); 
  request->send(200, MIME_JSON, result); 
}

void handleSetCfgBody(AsyncWebServerRequest* request, uint8_t* data, size_t len, size_t index, size_t total) {
  static char jBuff[MAX_CFG_JSON + 1];  // (+1 for null terminator)
  static size_t jPos = 0;          

  // Perform header checks only on first chunk
  if (index == 0) {
    LORA.devTake();
    if (LORA.Data.Running) LoraEndRplText(429, msgBusy);
    if (!request->hasParam("rem")) LoraEndRplText(400, msgBadCmd);
    if (!request->contentType().equals(MIME_JSON)) LoraEndRplText(400, msgNoJson); 
    if (request->contentLength() > MAX_CFG_JSON) LoraEndRplText(413, msgJBuffOver);
    String strParam; strParam.reserve(16);
    if (!request->hasParam("timeout")) LORA.Data.rxTimeout = 0; else {
      strParam = request->getParam("timeout")->value();
      char* endptr; LORA.Data.rxTimeout = strtoul(strParam.c_str(), &endptr, DEC);
      if (*endptr != '\0') LORA.Data.rxTimeout = 0; }     
    strParam = request->getParam("rem")->value();
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
      PrintLn("done !\n"); SerialGive(); 
      EndRplText(200, msgAccepted);
    } else { 
      PrintLn("failed !\n"); SerialGive();
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
      int16_t state; 
      size_t bSize = 11; uint8_t buff[bSize]; 
      if (LORA.Data.cfgRemote) {
        // Sending configuration command and data
        memcpy(buff, &cmdSetConfig, 2); 
        memcpy(&buff[2], &LORA.Data.UserCfg.freq, 4);
        buff[6]  = LORA.Data.UserCfg.txpwr;
        buff[7]  = LORA.Data.UserCfg.bandw;
        buff[8]  = LORA.Data.UserCfg.spread;
        buff[9]  = LORA.Data.UserCfg.cdrate;
        buff[10] = LORA.Data.UserCfg.preamb;
        PrintBuff("Transmit buffer", buff, bSize);
        Print("[SX1262] Sending config buffer... ");
        tx_start = millis(); tx_toa = MillisTOA(LORA.getTimeOnAir(bSize));
        state = LORA.transmit(buff, bSize);
        if (!CheckLoraResult(state)) { LORA.FormatError(msgLLFail, " TX", state); break; }
        // Waiting for acknowledge
        if (!LORA.WaitForReplay(buff, sizeof(buff), rplConfigRes)) break;
      }
      // Apply local configuration
      Print("[SX1262] Updating LoRa configuration... ");
      state = LORA.ApplyUserCfg(); 
      if (!CheckLoraResult(state)) { LORA.FormatError(msgLLFail, " Cfg", state); break; }

      if (LORA.Data.cfgRemote) {
        // Testing the new configuration
        delay(100);
        Print("[SX1262] Sending Ping command... ");
        tx_toa += MillisTOA(LORA.getTimeOnAir(2)); 
        state = LORA.transmit(reinterpret_cast<const uint8_t*>(&cmdPing), 2);
        if (!CheckLoraResult(state)) { LORA.FormatError(msgLLFail, " TX", state); break; }
        if (!LORA.WaitForReplay(buff, sizeof(buff), rplPing)) break;
      }
      // Configuration succeeded, update it
      LORA.UpdateConfig(); LORA.Data.Result = true;
      PrintLn("[SX1262] Reconfiguration succeeded !");

    } while (false);
    if (LORA.isCfgChanging()) {
      // Failed, undo reconfiguration
      PrintLn("[SX1262] Reconfiguration failed !");
      Print("[SX1262] Rolling back changes... ");
      CheckLoraResult(LORA.CancelConfig());
    }
    LORA.TxDone(tx_start, tx_toa);
    Print("[SX1262] Back to listening mode... ");
    CheckLoraResult(LORA.startReceive());
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
  String strParam; strParam.reserve(10);
  if (!request->hasParam("buff")) LORA.Data.buffSize = DEF_BUFF_SIZE;
  else {
    strParam = request->getParam("buff")->value();
    char* endptr; LORA.Data.buffSize = strtoul(strParam.c_str(), &endptr, DEC);
    if (*endptr != '\0') LORA.Data.buffSize = DEF_BUFF_SIZE;
  } 
  if (!request->hasParam("timeout")) LORA.Data.rxTimeout = 0;
  else {
    strParam = request->getParam("timeout")->value();
    char* endptr; LORA.Data.rxTimeout = strtoul(strParam.c_str(), &endptr, DEC);
    if (*endptr != '\0') LORA.Data.rxTimeout = 0;
  } 
  LORA.Data.Result = false; 
  SerialTake(); Print("[SERVER] Starting LoRa Test task...");
  if (xTaskCreatePinnedToCore(LoraTestTask, "LoraTest", 4096, NULL, 1, &hIsrTask, 1) == pdPASS) { 
    PrintLn("done !\n"); SerialGive(); 
    LORA.Data.Running = true; 
    LoraEndRplText(200, msgAccepted);
  } else { 
    PrintLn("failed !\n"); SerialGive();
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
    if (LORA.Data.cfgRemote && !LORA.ReadyForTx()) break;
    SerialTake();

    uint32_t tx_start; RadioLibTime_t tx_toa = 0; 
    do {
      // Sending test command and data
      size_t bSize = max(LORA.Data.buffSize, 16u); uint8_t buff[bSize]; 
      memcpy(buff, &cmdStartTest, 2);
      for (int i = 2; i < bSize; i++) buff[i] = i - 1; 
      PrintBuff("Transmit buffer", buff, bSize);
      Print("[SX1262] Sending test buffer... ");
      tx_start = millis(); tx_toa = MillisTOA(LORA.getTimeOnAir(bSize));
      int16_t state = LORA.transmit(buff, bSize);
      if (!CheckLoraResult(state)) { LORA.FormatError(msgLLFail, " TX", state); break; }
      // Waiting for the same size test buffer replay
      if (!LORA.WaitForReplay(buff, bSize, rplTestRes, bSize-4)) break;
      // Updating results
      memcpy(&LORA.Data.tx_rssi, &buff[4], 4);   LORA.Data.rx_rssi  = LORA.getRSSI(); 
      memcpy(&LORA.Data.tx_snr, &buff[8], 4);    LORA.Data.rx_snr   = LORA.getSNR();
      memcpy(&LORA.Data.tx_fqerr, &buff[12], 4); LORA.Data.rx_fqerr = LORA.getFrequencyError(); 
      LORA.Data.Result = true; 
    } while (false); 
    LORA.TxDone(tx_start, tx_toa);

    Print("[SX1262] Back to listening mode... ");
    CheckLoraResult(LORA.startReceive());
    PrintLn(""); SerialGive();    
  } while (false);
  LORA.devTake(); LORA.Data.Running = false; hIsrTask = NULL; LORA.devGive();
  vTaskDelete(NULL);
}


#else  //----------------------------- RX MODULE (LoRa server) -----------------------------

void LoraServerTask(void* pvParameters) {
  uint16_t CMD = 0;
  while (true) {
    // wait for RF command (DIO1 interrupt)
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    SerialTake(); PrintLn("[SX1262] New packet received.");

    // read the packet from SX1262 device
    uint8_t buff[255] = {0}; size_t bSize = LORA.getPacketLength();
    int16_t state = LORA.readData(buff, bSize);
    if (state != RADIOLIB_ERR_NONE) {
      LORA.clearIrqFlags(RADIOLIB_SX126X_IRQ_ALL);
      Print("[SX1262] Failed to read packet. Error code: "); PrintLn(state);
      Print(""); SerialGive(); continue;
    }  
    PrintLn("[SX1262] The packet has been successfully read.");

    // handle the requested command
    memcpy(&CMD, buff, 2);
    switch (CMD) {

      case cmdSetConfig: {
        PrintLn("[SYSTEM] Module reconfiguration requested.");
        if (bSize != 11) {
          Print("[SYSTEM] Invalid packet size: "); Print(bSize); 
          PrintLn(" bytes.\n"); SerialGive(); continue;
        }
        PrintBuff("Command Data", buff, bSize);
        memcpy(&LORA.Data.UserCfg.freq, &buff[2], 4);   
        LORA.Data.UserCfg.txpwr  = (int8_t) buff[6];
        LORA.Data.UserCfg.bandw  = buff[7];
        LORA.Data.UserCfg.spread = buff[8];
        LORA.Data.UserCfg.cdrate = buff[9];
        LORA.Data.UserCfg.preamb = buff[10];
        memcpy(buff, &rplConfigRes, 2);
        memcpy(&buff[2], &statSuccess, 2); 
        bSize = 4;
        break;
      }

      case cmdStartTest: {
        PrintLn("[SYSTEM] Test command requested.");
        PrintBuff("Command Data", buff, bSize);
        memcpy(buff, &rplTestRes, 2);
        memcpy(&buff[2], &statSuccess, 2); 
        float Data = LORA.getRSSI(); memcpy(&buff[4], &Data, 4);
        Data = LORA.getSNR(); memcpy(&buff[8], &Data, 4);
        Data = LORA.getFrequencyError(); memcpy(&buff[12], &Data, 4);
        for (int i = 16; i < bSize; i++) { buff[i] = i - 15; }
        break;
      }

      default: {
        PrintLn("[SYSTEM] Unknown command.\n"); 
        SerialGive(); continue;
      }
    }

    PrintBuff("Replay Data", buff, bSize);
    LORA.clearDio1Action(); delay(100);
    Print("[SX1262] Sending replay data... ");
    state = LORA.transmit(buff, bSize); CheckLoraResult(state);

    if (state == RADIOLIB_ERR_NONE && CMD == cmdSetConfig) {
      do {
        // Apply local configuration
        Print("[SX1262] Updating LoRa configuration... ");
        if (!CheckLoraResult(LORA.ApplyUserCfg())) break;
        // Testing the new configuration
        Print("[SX1262] Waiting for Ping... ");
        if (!CheckLoraResult(LORA.receiveEx(buff, sizeof(buff), 2000))) break;
        bSize = LORA.getPacketLength();
        if (bSize != 2 || memcmp(buff, &cmdPing, 2) != 0) { PrintLn("[SYSTEM] Ping not received"); break; }
        memcpy(buff, &rplPing, 2); memcpy(&buff[2], &statSuccess, 2); bSize = 4; delay(100);
        Print("[SX1262] Sending replay data... ");
        if (!CheckLoraResult(LORA.transmit(buff, bSize))) break;
        // Configuration succeeded, update it
        LORA.UpdateConfig(); LORA.Data.Result = true;
        PrintLn("[SX1262] Reconfiguration succeeded !");
      } while (false);
      if (LORA.isCfgChanging()) {
        // Failed, undo reconfiguration
        PrintLn("[SX1262] Reconfiguration failed !");
        Print("[SX1262] Rolling back changes... ");
        CheckLoraResult(LORA.CancelConfig());
      }      
    }

    LORA.setDio1Action(IrqDio1);
    Print("[SX1262] Back to listening mode... ");
    CheckLoraResult(LORA.startReceive());
    PrintLn(""); SerialGive();
  }
}

#endif //-------------------------------------------------------------------------------

void setup() {
  #ifdef DEBUG_MODE
    Serial.begin(115200); delay(1000);
    serialMutex = xSemaphoreCreateMutex();
  #endif

  #ifdef TX_MODULE

    PrintLn("[SYSTEM] Starting program for TX Module (Client)...");

    //listPartitions();
    Print("[SYSTEM] Mouning file system [LittleFS]... ");
    if (LittleFS.begin(true, "/LFS", 5, "littlefs")) PrintLn("done !");
      else { PrintLn("failed !"); while(true) delay(1000); }
    //listDir(LittleFS, "/", 10);  

    Print("[SYSTEM] Starting Access Point... ");  
    if (WiFi.softAP(ssid, password)) PrintLn("done !");
      else { PrintLn("failed !"); while(true) delay(1000); }
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

  #else // RX MODULE 

    PrintLn("[SYSTEM] Starting program for RX Module (Server)...");

  #endif

  #if defined(ESP32_BOARD) 
    SPI.begin(PIN_CLK, PIN_MISO, PIN_MOSI, PIN_CS);
  #elif defined(PICO_BOARD)
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH); 
    SPI.setSCK(PIN_CLK);
    SPI.setRX(PIN_MISO);  // SPI0 RX (MISO)
    SPI.setTX(PIN_MOSI);  // SPIO TX (MOSI)
    SPI.begin();
  #else
    PrintLn("Unsupported board !")
    while (true) delay(1000);
  #endif  

  Print("[SX1262] Initializing LoRa... ");
  CheckLoraResult(LORA.beginDefault(), true);
  LORA.setRfSwitchPins(PIN_RX_EN, PIN_TX_EN);

  Print("[SX1262] Setup RX boosted gain mode... ");
  CheckLoraResult(LORA.setRxBoostedGainMode(true));

  #ifdef TX_MODULE   // ---------- TX MODULE SETUP --------------------------------

    SerialTake();

    Print("[SX1262] Entering listening mode... ");
    CheckLoraResult(LORA.startReceive(), true);

    Print("[SYSTEM] Starting HTTP Server... ");
    Server.begin(); PrintLn("done !");      

    PrintLn(""); SerialGive();

  #else              // ---------- RX MODULE SETUP -------------------------------- 

    SerialTake();

    #ifdef ESP32_BOARD
      Print("[SYSTEM] Starting LoRa Server task... ");
      if (xTaskCreatePinnedToCore(LoraServerTask, "LoraServerTask", 2048, NULL, 1, &hIsrTask, 1) == pdPASS) 
        { PrintLn("done !"); } else { PrintLn("failed !"); while (true) delay(1000); }
    #endif
    #ifdef PICO_BOARD
      Print("[SYSTEM] Starting LoRa Server task... ");
      if (xTaskCreateAffinitySet(LoraServerTask, "LoraServerTask", 2048, NULL, 1, 2, &hIsrTask) == pdPASS) 
        { PrintLn("done !"); } else { PrintLn("failed !"); while (true) delay(1000); }      
    #endif

    Print("[SX1262] Starting to listen... ");
    LORA.setDio1Action(IrqDio1); 
    CheckLoraResult(LORA.startReceive(), true);
    Print(""); SerialGive();

  #endif             // -----------------------------------------------------------
}

void loop() { delay(1000); }

/* ------ TO DO ------------------------------
 - maybe switch to 1 byte commands ?
 - better duty cycle handling, taking into account the band
*/