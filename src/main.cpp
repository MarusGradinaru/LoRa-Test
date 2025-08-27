
// ----- User Settings ----------------------------------------------------------
#define DEBUG_MODE   // comment to stop debug messages
#define TX_MODULE    // comment for RX_MODULE  (TX not available for PICO_BOARD)
//-------------------------------------------------------------------------------

#include <Arduino.h>
#include <SPI.h>
#include <RadioLib.h>

#ifdef PICO_BOARD
  #undef TX_MODULE
  #include <FreeRTOS.h>
  #include <semphr.h>
#else  
  #include <esp_partition.h>
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
#else
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

// Module default settings
#define FREQ    869525000ul // Frequency: P band, 250KHz, 500mW (27dBm), 10% [863.0 MHz + 1/2 BW .. 870.0 MHz - 1/2 BW] 
#define BW      7           // Bandwidth idx: 125KHz [7.8, 10.4, 15.6, 20.8, 31.25, 41.7, 62.5, 125, 250, 500] KHz
#define SF      4           // Spreading Factor idx: SF9 [5..12]
#define CR      2           // Coding Rate idx: 4/7 [5..8]
#define SYNCW   0xE3u       // Sync Word: Custom [Private: 0x12, Public: 0x34] 
#define TX_PWR  9           // TX Power (dBm) [-9..22] / with 5 dBi antenna = 14 dBm (25 mW)
#define TX_DC   10.0f       // TX Duty Cycle (% percent)
#define PAMB    2           // Preamble Length idx: 8 symb. [6..20] 
#define XOV     1.7f        // TCXO Voltage (V) [1.6, 1.7, 1.8, 2.2, 2.4, 2.7, 3.0, 3.3]
#define LDO     false       // Use LDO only ? [false:LDO and DC-DC, true: just LDO]

#define LoraTake()      xSemaphoreTake(loraMutex, portMAX_DELAY)
#define LoraGive()      xSemaphoreGive(loraMutex)
#define CfgTake()       xSemaphoreTake(cfgMutex, portMAX_DELAY)
#define CfgGive()       xSemaphoreGive(cfgMutex)

#define FlashParams(fstr) reinterpret_cast<const uint8_t*>(fstr), sizeof(fstr)-1

// LoRa command codes
#define testBuffSize 32u
const uint16_t cmdStartTest = 0xC1E8u;
const uint16_t rplTestRes   = 0xE8C1u;
const uint16_t cmdSetConfig = 0x7A94u;
const uint16_t rplConfigRes = 0x947Au;
const uint16_t statSuccess  = 0x2664u;
const uint16_t statFailed   = 0x6246u;


// ------ Classes ------------------------------------------

class MSX1262 : public SX1262 {
  public:
    using SX1262::SX1262;
    
};

// ------ Global objects and init --------------------------

const float   listBandwidth[]  = {7.8f, 10.4f, 15.6f, 20.8f, 31.25f, 41.7f, 62.5f, 125.0f, 250.0f, 500.0f};
const uint8_t listSpreading[]  = {5, 6, 7, 8, 9, 10, 11, 12};
const uint8_t listCodingRate[] = {5, 6, 7, 8};
const uint8_t listPreamble[]   = {6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20};

typedef struct {
  uint32_t freq;  // in Hz
  int8_t txpwr; 
  uint8_t bandw, spread, cdrate, preamb;
} LoraCfgParams;

typedef struct {
  uint32_t freq = FREQ;  // in Hz
  uint8_t bandw = BW;
  uint8_t spread = SF;
  uint8_t cdrate = CR;
  uint8_t syncw = SYNCW;
  int8_t txpwr = TX_PWR;
  uint8_t txdc = TX_DC;
  uint8_t preamb = PAMB;
  float xovolt = XOV;
  bool useldo = LDO;
} LoraConfiguration;
SemaphoreHandle_t cfgMutex = xSemaphoreCreateMutex();
LoraConfiguration LoraCfg;

SPISettings mySPISettings(2000000, MSBFIRST, SPI_MODE0);  // 10 cm long wires SPI
SX1262 LORA = new Module(PIN_CS, PIN_DIO1, PIN_RESET, PIN_BUSY, SPI, mySPISettings);
//Module* mod = new Module(PIN_CS, PIN_DIO1, PIN_RESET, PIN_BUSY, SPI, mySPISettings);
//MSX1262 LORA(mod);

// ------ Debug functions ----------------------------------

#ifdef DEBUG_MODE

  #define SerialTake()                xSemaphoreTake(serialMutex, portMAX_DELAY)
  #define SerialGive()                xSemaphoreGive(serialMutex);
  #define Print(msg)                  Serial.print(msg)
  #define PrintLn(msg)                Serial.println(msg)
  #define SafePrintLn(msg)            _SafePrintLn(msg)
  #define PrintBuff(msg, buff, len)   _PrintBuff(msg, buff, len)
  
SemaphoreHandle_t serialMutex;

void _SafePrintLn(const char *msg) {
  xSemaphoreTake(serialMutex, portMAX_DELAY);
  Serial.println(msg);
  xSemaphoreGive(serialMutex);
}

void _PrintBuff(const char *msg, uint8_t *buffer, uint16_t len) {
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


// ------ Common functions ----------------------------------

#ifdef ESP32_BOARD
  #define ISR_ATTR IRAM_ATTR
#else
  #define ISR_ATTR
#endif

TaskHandle_t hIsrTask = NULL;  // protected by LoraTake() in TX_MODULE

void ISR_ATTR IrqDio1(void) { 
  if (hIsrTask != NULL) vTaskNotifyGiveFromISR(hIsrTask, NULL); 
}

bool CheckLoraResult(int16_t state, bool errStop = false) {
  if (state == RADIOLIB_ERR_NONE) {
    PrintLn("done !");
    return true;
  } else {
    Print("failed, code "); PrintLn(state);
    if (errStop) { while (true) delay(1000); }
    return false;
  }
} 

bool UpdateLoraCfg(const LoraCfgParams &params) {  // Needs LoraTake() or loraData.Running
  CfgTake();
  int16_t state = RADIOLIB_ERR_NONE;
  do {
    state = LORA.standby(); if (state != RADIOLIB_ERR_NONE) break;
    if (LoraCfg.freq != params.freq) {
      state = LORA.setFrequency((float)params.freq / 1000000.0f); 
      if (state != RADIOLIB_ERR_NONE) break; 
      LoraCfg.freq = params.freq;
    }
    if (LoraCfg.txpwr != params.txpwr) { 
      state = LORA.setOutputPower(params.txpwr); 
      if (state != RADIOLIB_ERR_NONE) break; 
      LoraCfg.txpwr = params.txpwr;
    }
    if (LoraCfg.bandw != params.bandw) { 
      state = LORA.setBandwidth(listBandwidth[params.bandw]); 
      if (state != RADIOLIB_ERR_NONE) break; 
      LoraCfg.bandw = params.bandw;
    }
    if (LoraCfg.spread != params.spread) {
      state = LORA.setSpreadingFactor(listSpreading[params.spread]);
      if (state != RADIOLIB_ERR_NONE) break; 
      LoraCfg.spread = params.spread; 
    }
    if (LoraCfg.cdrate != params.cdrate) {
      state = LORA.setCodingRate(listCodingRate[params.cdrate]);
      if (state != RADIOLIB_ERR_NONE) break; 
      LoraCfg.cdrate = params.cdrate; 
    }
    if (LoraCfg.preamb != params.preamb) {
      state = LORA.setPreambleLength(listPreamble[params.preamb]);
      if (state != RADIOLIB_ERR_NONE) break; 
      LoraCfg.preamb = params.preamb;
    }  
  } while(false);
  CfgGive();
  return state;
}

#ifdef TX_MODULE

bool ReadyForTX(uint32_t deadline, float &wait_time) {
  int32_t delta = (int32_t)(millis() - deadline);
  if (delta >= 0) { wait_time = 0.0f; return true; } 
   else {wait_time = ((float)(-delta) / 1000.0f); return false; }
}

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

void listDir(fs::FS &fs, const char *dirname, uint8_t levels) {
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


#ifdef TX_MODULE //-------------------------------- TX MODULE ----------------------------------

const char ssid[]           = "ESP32";
const char password[]       = "loratest";

#define  HTTP_MSG_SIZE 60
const char msgAccepted[]    = "Command accepted. Waiting for result...";
const char msgBusy[]        = "LoRa is Busy ! Please wait...";
const char msgDCWait[]      = "Please wait %s seconds more !";    
const char msgBadCmd[]      = "Invalid command syntax.";
const char msgBadLRpl[]     = "Invalid LoRa response.";
const char msgIntError[]    = "Internal error encountered.";
const char msgLoraTimeout[] = "The other LoRa is not responding !";
const char msgLLFail[]      = "Local LoRa%s failed, code %d";
const char msgRLFail[]      = "Remote LoRa%s failed, code %d";
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

const float psConst = (100.0f / TX_DC) - 1.0f;

// Used to exit an AsyncWebServerRequest handler with a replay
#define StopLora()                  LoraTake(); loraData.Running = false; LoraGive()
#define EndRplText(code, msg)       { request->send(code, MIME_PLAIN, FlashParams(msg)); return; }
#define EndRplCode(code)            { request->send(code); return; }
#define LoraEndRplText(code, msg)   { LoraGive(); request->send(code, MIME_PLAIN, FlashParams(msg)); return; }
#define LoraEndRplCode(code)        { LoraGive(); request->send(code); return; }

typedef struct {
  bool Running = false;
  bool Result = false;
  uint32_t txNext = 0;
  String Error = "";
  uint32_t rxTimeout = 0;
  uint32_t buffSize = testBuffSize;
  // Test results [out]
  uint32_t tx_toa, rx_toa;
  float tx_rssi, rx_rssi, tx_snr, rx_snr, tx_fqerr, rx_fqerr;
  // Config params [in]
  bool isLocal = false;
  LoraCfgParams cfgParams;
} LoraShareData;
SemaphoreHandle_t loraMutex = xSemaphoreCreateMutex();
LoraShareData loraData; 

const size_t MAX_JSON_SIZE = 120;  
char jBuff[MAX_JSON_SIZE + 1];  // (+1 for null terminator)
size_t jPos = 0;          

AsyncWebServer Server(80);

// Forward declarations
void LoraConfigTask(void* pvParameters);
void LoraTestTask(void* pvParameters);

template<typename... Args>
void FormatError(const char* fmt, Args... args) {
  char buff[HTTP_MSG_SIZE];
  snprintf(buff, sizeof(buff), fmt, args...);
  loraData.Error = buff;
}

void FormatErrorWait(float wTime) {
  char fbuf[16]; char buff[HTTP_MSG_SIZE];
  dtostrf(wTime, 0, 3, fbuf);
  snprintf(buff, sizeof(buff), msgDCWait, fbuf);
  loraData.Error = buff;
}

// ------ FILES section --------------------------------------

void handleRoot(AsyncWebServerRequest *request) {
  request->send(LittleFS, pathIndex, MIME_HTML);
}
void handleRoboto(AsyncWebServerRequest *request) {
  request->send(LittleFS, pathRobo, MIME_WOFF2);
}
void handleRoboCnd(AsyncWebServerRequest *request) {
  request->send(LittleFS, pathRoboCnd, MIME_WOFF2);
}
void handleNextRnd(AsyncWebServerRequest *request) {
  request->send(LittleFS, pathNextRnd, MIME_WOFF2);
}

// ------ CONFIG section --------------------------------------

void handleGetCfg(AsyncWebServerRequest *request) {
  char result[MAX_JSON_SIZE]; CfgTake();
  snprintf(result, sizeof(result), 
    "{\"freq\":%u,\"txpwr\":%d,\"bandw\":%u,\"spread\":%u,\"cdrate\":%u,\"preamb\":%u}",
    LoraCfg.freq, LoraCfg.txpwr, LoraCfg.bandw, LoraCfg.spread, LoraCfg.cdrate, LoraCfg.preamb);      
  CfgGive();  
  SerialTake(); Print("[SERVER] JSON Config: "); PrintLn(result); PrintLn(""); SerialGive(); 
  request->send(200, MIME_JSON, result); 
}

void handleSetCfgBody(AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total) {
  // Perform header checks only on first chunk
  if (index == 0) {
    LoraTake();
    if (loraData.Running) LoraEndRplText(429, msgBusy);
    if (!request->hasParam("loc")) LoraEndRplText(400, msgBadCmd);
    if (!request->contentType().equals(MIME_JSON)) LoraEndRplText(400, msgNoJson); 
    if (request->contentLength() > MAX_JSON_SIZE) LoraEndRplText(413, msgJBuffOver);
    String strParam; strParam.reserve(16);
    if (!request->hasParam("timeout")) loraData.rxTimeout = 0; else {
      strParam = request->getParam("timeout")->value();
      char* endptr; loraData.rxTimeout = strtoul(strParam.c_str(), &endptr, DEC);
      if (*endptr != '\0') loraData.rxTimeout = 0; }     
    strParam = request->getParam("loc")->value();
    if (!strParam.equals("0") && !strParam.equals("1")) LoraEndRplText(400, msgBadCmd);
    loraData.isLocal = strParam.equals("1"); loraData.Running = true;
    SafePrintLn("[SERVER] Received valid LoRa reconfiguration request.");
    jPos = 0; jBuff[jPos] = '\0'; 
    LoraGive();    
  }
  // Append chunk to buffer
  if (jPos + len > MAX_JSON_SIZE) { StopLora(); EndRplText(500, msgJBuffOver); }
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
    
    loraData.Result = false;
    loraData.cfgParams.freq   = jDoc["freq"].as<uint32_t>(); 
    loraData.cfgParams.txpwr  = jDoc["txpwr"].as<int8_t>(); 
    loraData.cfgParams.bandw  = jDoc["bandw"].as<uint8_t>(); 
    loraData.cfgParams.spread = jDoc["spread"].as<uint8_t>(); 
    loraData.cfgParams.cdrate = jDoc["cdrate"].as<uint8_t>(); 
    loraData.cfgParams.preamb = jDoc["preamb"].as<uint8_t>(); 

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

void handleCfgRes(AsyncWebServerRequest *request) {
  LoraTake();
  if (loraData.Running) LoraEndRplCode(202);
  if (!loraData.Result) { request->send(500, MIME_PLAIN, loraData.Error); LoraGive(); return; }
  LoraGive();
  SafePrintLn("[SERVER] LoRa was successfully configured.\n"); 
  EndRplCode(200);
}

void LoraConfigTask(void* pvParameters) {
  do {
    if (!loraData.isLocal) {
      float wTime;
      if (!ReadyForTX(loraData.txNext, wTime)) { FormatErrorWait(wTime); break; }
    }
    SerialTake();

    do {
      int16_t state;
      if (!loraData.isLocal) {
        // Sending command and data
        size_t bSize = 11; uint8_t buff[bSize]; 
        memcpy(buff, &cmdSetConfig, 2); 
        memcpy(&buff[2], &loraData.cfgParams.freq, 4);
        buff[6]  = loraData.cfgParams.txpwr;
        buff[7]  = loraData.cfgParams.bandw;
        buff[8]  = loraData.cfgParams.spread;
        buff[9]  = loraData.cfgParams.cdrate;
        buff[10] = loraData.cfgParams.preamb;
        PrintBuff("Transmit buffer", buff, bSize);
        Print("[SX1262] Sending config buffer...");
        uint32_t tx_start = millis();
        state = LORA.transmit(buff, bSize);
        uint32_t tx_toa = millis() - tx_start;
        if (!CheckLoraResult(state)) { FormatError(msgLLFail, " TX", state); break; }
        uint32_t ps_time = ceil(tx_toa * psConst);
        loraData.txNext = tx_start + tx_toa + ps_time;    

        // Receiving requested data
        memset(buff, 0x00, bSize);
        if (loraData.rxTimeout) 
          { Print("[SYSTEM] Using additional RX timeout: "); PrintLn(loraData.rxTimeout); }
        Print("[SX1262] Waiting for replay... ");
        state = LORA.receive(buff, sizeof(buff), loraData.rxTimeout);
        CheckLoraResult(state);
        if (state == RADIOLIB_ERR_RX_TIMEOUT) { loraData.Error = msgLoraTimeout; break; }  
        if (state != RADIOLIB_ERR_NONE) { FormatError(msgLLFail, " RX", state); break; }
        bSize = LORA.getPacketLength();
        PrintBuff("Received buffer", buff, bSize);
        if ((bSize < 4) || (memcmp(buff, &rplConfigRes, 2) != 0)) 
          { loraData.Error = msgBadLRpl; break; }
        if (memcmp(&buff[2], &statFailed, 2) == 0)
          { memcpy(&state, &buff[4], 2); FormatError(msgRLFail, " Cfg", state); break; }  
        if (memcmp(&buff[2], &statSuccess, 2) != 0)
          { loraData.Error = msgBadLRpl; break; }
      }

      // Updating local configuration
      state = UpdateLoraCfg(loraData.cfgParams);
      if (state != RADIOLIB_ERR_NONE) { FormatError(msgLLFail, " Cfg", state); break; }
      loraData.Result = true;         
    } while (false);

    LORA.standby();
    PrintLn("[SX1262] Back to standby.\n");
    SerialGive();    
  } while (false);
  LoraTake(); loraData.Running = false; hIsrTask = NULL; LoraGive();
  vTaskDelete(NULL);
}

// ------ TEST section ----------------------------------------

void handleDoTest(AsyncWebServerRequest *request) {
  LoraTake(); 
  if (loraData.Running) LoraEndRplText(429, msgBusy);
  String strParam; strParam.reserve(10);
  if (!request->hasParam("buff")) loraData.buffSize = testBuffSize;
  else {
    strParam = request->getParam("buff")->value();
    char* endptr; loraData.buffSize = strtoul(strParam.c_str(), &endptr, DEC);
    if (*endptr != '\0') loraData.buffSize = testBuffSize;
  } 
  if (!request->hasParam("timeout")) loraData.rxTimeout = 0;
  else {
    strParam = request->getParam("timeout")->value();
    char* endptr; loraData.rxTimeout = strtoul(strParam.c_str(), &endptr, DEC);
    if (*endptr != '\0') loraData.rxTimeout = 0;
  } 
  loraData.Result = false; 
  SerialTake(); Print("[SERVER] Starting LoRa Test task...");
  if (xTaskCreatePinnedToCore(LoraTestTask, "LoraTest", 4096, NULL, 1, &hIsrTask, 1) == pdPASS) { 
    PrintLn("done !\n"); SerialGive(); 
    loraData.Running = true; 
    LoraEndRplText(200, msgAccepted);
  } else { 
    PrintLn("failed !\n"); SerialGive();
    LoraEndRplText(500, msgIntError);
  } 
}

void handleTestRes(AsyncWebServerRequest *request) {
  LoraTake();
  if (loraData.Running) LoraEndRplCode(202);
  if (!loraData.Result) { request->send(500, MIME_PLAIN, loraData.Error); LoraGive(); return; }
  char result[180];
  snprintf(result, sizeof(result), 
    "{\"tx_toa\":%lu,\"rx_toa\":%lu,\"tx_rssi\":%.2f,\"rx_rssi\":%.2f,\"tx_snr\":%.2f,\"rx_snr\":%.2f,\"tx_fqerr\":%.1f,\"rx_fqerr\":%.1f}",
    loraData.tx_toa, loraData.rx_toa, loraData.tx_rssi, loraData.rx_rssi, loraData.tx_snr, loraData.rx_snr, loraData.tx_fqerr, loraData.rx_fqerr);      
  LoraGive(); 
  SerialTake(); Print("[SERVER] JSON Result: "); PrintLn(result); PrintLn(""); SerialGive(); 
  request->send(200, MIME_JSON, result);
}

void LoraTestTask(void* pvParameters) {
  do {
    float wTime;
    if (!ReadyForTX(loraData.txNext, wTime)) { FormatErrorWait(wTime); break; }
    SerialTake();

    do {
      // Sending command and data
      size_t bSize = max(loraData.buffSize, 14u); uint8_t buff[bSize]; 
      memcpy(buff, &cmdStartTest, 2);
      for (int i = 2; i < bSize; i++) buff[i] = i - 1; 
      PrintBuff("Transmit buffer", buff, bSize);
      Print("[SX1262] Sending test buffer...");
      uint32_t tx_start = millis();
      int16_t state = LORA.transmit(buff, bSize);
      uint32_t tx_toa = millis() - tx_start;
      if (!CheckLoraResult(state)) { FormatError(msgLLFail, " TX", state); break; }
      uint32_t ps_time = ceil(tx_toa * psConst);
      loraData.txNext = tx_start + tx_toa + ps_time;

      // Receiving requested data
      memset(buff, 0x00, bSize);
      if (loraData.rxTimeout) 
        { Print("[SYSTEM] Using additional RX timeout: "); PrintLn(loraData.rxTimeout); }
      Print("[SX1262] Waiting for replay... ");
      uint32_t rx_toa = millis();
      state = LORA.receive(buff, sizeof(buff), loraData.rxTimeout);
      rx_toa = millis() - rx_toa;
      CheckLoraResult(state);
      if (state == RADIOLIB_ERR_RX_TIMEOUT) { loraData.Error = msgLoraTimeout; break; }  
      if (state != RADIOLIB_ERR_NONE) { FormatError(msgLLFail, " RX", state); break; }
      PrintBuff("Received buffer", buff, bSize);
      if ((LORA.getPacketLength() != bSize) || (memcmp(buff, &rplTestRes, 2) != 0)) 
        { loraData.Error = msgBadLRpl; break; }
      
      // Updating results
      loraData.tx_toa = tx_toa;                 loraData.rx_toa = rx_toa;
      memcpy(&loraData.tx_rssi, &buff[2], 4);   loraData.rx_rssi = LORA.getRSSI(); 
      memcpy(&loraData.tx_snr, &buff[6], 4);    loraData.rx_snr = LORA.getSNR();
      memcpy(&loraData.tx_fqerr, &buff[10], 4); loraData.rx_fqerr = LORA.getFrequencyError(); 
      loraData.Result = true; 
    } while (false); 

    LORA.standby();
    PrintLn("[SX1262] Back to standby.\n");
    SerialGive();
  } while (false);
  LoraTake(); loraData.Running = false; hIsrTask = NULL; LoraGive();
  vTaskDelete(NULL);
}


#else  //------------------------------------------ RX MODULE ------------------------------------

void LoraServerTask(void* pvParameters) {
  uint16_t CMD = 0;
  while (true) {
    // wait for RF command (DIO1 interrupt)
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    SerialTake(); PrintLn("[SX1262] New packet received.");

    // read the packet from SX1262 device
    uint8_t buff[256] = {0}; size_t size = LORA.getPacketLength();
    int16_t state = LORA.readData(buff, size);
    if (state != RADIOLIB_ERR_NONE) {
      LORA.clearIrqFlags(RADIOLIB_SX126X_IRQ_ALL);
      Print("[SX1262] Failed to read packet. Error code: "); PrintLn(state);
      Print(""); SerialGive(); continue;
    }  
    PrintLn("[SX1262] The packet has been successfully read.");

    // handle the requested command
    memcpy(&CMD, buff, 2); LoraCfgParams params;
    switch (CMD) {

      case cmdSetConfig: {
        PrintLn("[SYSTEM] Module reconfiguration requested.");
        if (size != 11) {
          Print("[SYSTEM] Invalid packet size: "); Print(size); 
          PrintLn(" bytes.\n"); SerialGive(); continue;
        }
        PrintBuff("Command Data", buff, size);
        memcpy(&params.freq, &buff[2], 4);   
        params.txpwr  = (int8_t) buff[6];
        params.bandw  = buff[7];
        params.spread = buff[8];
        params.cdrate = buff[9];
        params.preamb = buff[10];
        memcpy(buff, &rplConfigRes, 2);
        memcpy(&buff[2], &statSuccess, 2); 
        size = 4;
        break;
      }

      case cmdStartTest: {
        PrintLn("[SYSTEM] Test command requested.");
        PrintBuff("Command Data", buff, size);
        memcpy(buff, &rplTestRes, 2);
        float Data = LORA.getRSSI(); memcpy(&buff[2], &Data, 4);
        Data = LORA.getSNR(); memcpy(&buff[6], &Data, 4);
        Data = LORA.getFrequencyError(); memcpy(&buff[10], &Data, 4);
        for (int i = 14; i < size; i++) { buff[i] = i - 13; }
        break;
      }

      default: {
        PrintLn("[SYSTEM] Unknown command.\n"); 
        SerialGive(); continue;
      }
    }

    PrintBuff("Replay Data", buff, size);
    LORA.clearDio1Action();
    Print("[SX1262] Sending replay data... ");
    if (CheckLoraResult(LORA.transmit(buff, size)) && (CMD == cmdSetConfig)) {
      Print("[SX1262] Updating LoRa configuration... ");
      CheckLoraResult(UpdateLoraCfg(params), true); }
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
    loraData.Error.reserve(HTTP_MSG_SIZE);

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

  #else // RX MODULE 

    PrintLn("[SYSTEM] Starting program for RX Module (Server)...");

  #endif

  #ifdef ESP32_BOARD 
    SPI.begin(PIN_CLK, PIN_MISO, PIN_MOSI, PIN_CS);
  #else
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH); 
    SPI.setSCK(PIN_CLK);
    SPI.setRX(PIN_MISO);  // SPI0 RX (MISO)
    SPI.setTX(PIN_MOSI);  // SPIO TX (MOSI)
    SPI.begin();
  #endif  

  Print("[SX1262] Initializing LoRa... ");
  CheckLoraResult(LORA.begin((float)LoraCfg.freq / 1000000.0f, listBandwidth[LoraCfg.bandw], 
    listSpreading[LoraCfg.spread], listCodingRate[LoraCfg.cdrate], LoraCfg.syncw, LoraCfg.txpwr, 
    listPreamble[LoraCfg.preamb], LoraCfg.xovolt, LoraCfg.useldo), true);
  LORA.setRfSwitchPins(PIN_RX_EN, PIN_TX_EN);

  #if defined(TX_MODULE)  // ---------- TX MODULE SETUP --------------------------------

    SerialTake();

    Print("[SX1262] Entering standby mode... ");
    CheckLoraResult(LORA.standby(), true);

    Print("[SYSTEM] Starting HTTP Server... ");
    Server.begin(); PrintLn("done !");      

    PrintLn(""); SerialGive();

  #else                   // ---------- RX MODULE SETUP -------------------------------- 

    SerialTake();
    Print("[SYSTEM] Starting LoRa Server task... ");

    #ifdef ESP32_BOARD
      if (xTaskCreatePinnedToCore(LoraServerTask, "LoraServerTask", 2048, NULL, 1, &hIsrTask, 1) == pdPASS) 
        { PrintLn("done !"); } else { PrintLn("failed !"); while (true) delay(1000); }
    #else
      if (xTaskCreateAffinitySet(LoraServerTask, "LoraServerTask", 2048, NULL, 1, 2, &hIsrTask) == pdPASS) 
        { PrintLn("done !"); } else { PrintLn("failed !"); while (true) delay(1000); }      
    #endif

    Print("[SX1262] Starting to listen... ");
    LORA.setDio1Action(IrqDio1); 
    CheckLoraResult(LORA.startReceive(), true);
    Print(""); SerialGive();

  #endif                  // -----------------------------------------------------------
}

void loop() { delay(1000); }
