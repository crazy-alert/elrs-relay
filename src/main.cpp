/*
  ==============================================================================
   ELRS РЕТРАНСЛЯТОР
  ==============================================================================
*/

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <ArduinoJson.h>
#include <esp_task_wdt.h>
#include <LittleFS.h>
#include <FS.h>
#include <SD.h>
#include <driver/uart.h>    // для uart_set_line_inverse и uart_set_mode
#include <DNSServer.h>
#include <map>
#include <vector>

// Пины
#define RELAY_PIN     5
#define LED_PIN       2
#define SD_CS_PIN     15
#define TX_MODULE_TX_PIN 32   // ESP32 -> инвертор -> S.Port (к передатчику)
#define TX_MODULE_RX_PIN 33   // S.Port -> инвертор -> ESP32 (от передатчика)

// Параметры
#define DEFAULT_BAUD       400000
#define TIMEOUT_MS         500
#define WDT_TIMEOUT_MS     5000
#define POWER_MODE_AUTO       0
#define POWER_MODE_ALWAYS_ON  1
#define POWER_MODE_ALWAYS_OFF 2
#define DEBUG_RX2_VERBOSE     0
#define DEBUG_TX1_VERBOSE     0
#define DEBUG_TX1_RAW_CAPTURE 1
#define AP_CHANNEL            6
#define WIFI_CONNECT_TIMEOUT_MS 10000
#define WIFI_RECONNECT_INTERVAL_MS 15000

// CRSF
// CRSF endpoint addresses (ELRS/Crossfire):
// 0xEE - TX module, 0xEA - handset/radio, 0xEC - receiver
#define CRSF_ADDRESS_MODULE      0xEE
#define CRSF_ADDRESS_HANDSET     0xEA
#define CRSF_ADDRESS_HANDSET_ALT 0xEF
#define CRSF_ADDRESS_RECEIVER    0xEC
#define CRSF_FRAMETYPE_DEVICE_PING       0x28
#define CRSF_FRAMETYPE_DEVICE_INFO       0x29
#define CRSF_FRAMETYPE_PARAMETER_DATA    0x2B
#define CRSF_FRAMETYPE_PARAMETER_REQUEST 0x2C
#define CRSF_FRAMETYPE_PARAMETER_WRITE   0x2D
#define CRSF_FRAMETYPE_ELRS_STATUS       0x2E

// Режим диагностики - игнорирование CRC для тестирования протокола
#define TX_DIAGNOSTICS_MODE 1
// Не игнорируем CRC: после корректного парсинга это индикатор реального качества линии.
#define TX_SKIP_CRC_CHECK 0

// Estructura для хранения пункта меню из menu.json
struct MenuItem {
    int id;
    String name;
    int type;           // 0-16 (uint8, int8, ..., command, etc)
    int parent;         // parent folder id (0 = root)
    bool isFolder;
    bool isCommand;
    String value;
    String options;     // for select type: "opt1;opt2;opt3"
    int minVal, maxVal; // for numeric types
    String unit;
    String description;
};

// Класс управления передатчиком (полудуплекс, открытый сток)
class TransmitterController {
public:
    TransmitterController();
    void begin(unsigned long baud);
    void update();
    void sendRawPacket(uint8_t* payload, uint8_t len, uint8_t destAddr);
    void sendRawFrame(const uint8_t* packet, size_t len);
    bool isConnected() const { return _connected; }  // линк есть уже после ping/ответов
    String getDeviceName() const { return _deviceName; }
    int getTxPower() const;
    String getTxPowerRead() const;
    String getRfModeRead() const;
    String getSwitchModeRead() const;
    String getAntennaModeRead() const;
    String getTelemRatioRead() const;
    String getTxPowerSet() const { return _txPowerSet; }
    String getRfModeSet() const { return _rfModeSet; }
    String getSwitchModeSet() const { return _switchModeSet; }
    String getAntennaModeSet() const { return _antennaModeSet; }
    String getTelemRatioSet() const { return _telemRatioSet; }
    int getPktRate() const;
    bool setTxPowerByName(const String& value);
    bool setRfModeByName(const String& value);
    bool setSwitchModeByName(const String& value);
    bool setAntennaModeByName(const String& value);
    bool setTelemRatioByName(const String& value);
    String getMenuJson(int parentId = 0);
    bool setValue(int fieldId, const String& value);
    bool executeCommand(int fieldId);
    void refresh();
    void loadMenuFromJson(const char* jsonPath);

private:
    struct CRSFField {
        uint8_t fieldId, fieldType;
        String name, value, options;
        bool isFolder, isCommand;
    };
    HardwareSerial* _serial;
    bool _connected;
    unsigned long _lastResponseTime;
    String _deviceName;
    bool _deviceInfoReceived, _fieldsRequested;
    std::map<uint8_t, CRSFField> _fields;
    std::vector<MenuItem> _menuItems;
    bool _menuFromFile;
    String _txPowerSet, _rfModeSet, _switchModeSet, _antennaModeSet, _telemRatioSet;
    uint8_t _lastOutA[64], _lastOutB[64];
    uint8_t _lastOutALen, _lastOutBLen;
    unsigned long _lastOutAMs, _lastOutBMs;

    uint8_t _crc8(const uint8_t* d, size_t len);
    void _rememberOutFrame(const uint8_t* frame, uint8_t len);
    bool _isLikelyEchoFrame(const uint8_t* frame, uint8_t len) const;
    int _findFieldIdByKeys(const std::vector<String>& keys) const;
    String _getFieldValueByKeys(const std::vector<String>& keys) const;
    bool _setFieldByKeys(const std::vector<String>& keys, const String& value, String& shadow);
    void _sendCrsfPacket(uint8_t* p, uint8_t len, uint8_t type);
    void _sendDevicePing();
    void _requestAllFields();
    void _requestField(uint8_t id);
    void _writeParameter(uint8_t id, uint32_t val);
    void _writeParameter(uint8_t id, const uint8_t* data, uint8_t len);
    void _processIncoming();
    void _handleDeviceInfo(uint8_t* d, uint8_t len);
    void _handleParameterData(uint8_t* d, uint8_t len);
    void _handleElrsStatus(uint8_t* d, uint8_t len);
    void _buildMenuItems(JsonArray items, int parentId);
};

// Глобальные объекты
Preferences prefs;
WebServer server(80);
DNSServer dnsServer;
IPAddress apIP(192,168,4,1);
const byte DNS_PORT = 53;
HardwareSerial receiverSerial(2);
TransmitterController txController;

unsigned long lastPacketTime = 0;
bool linkToRadio = false;
unsigned long packetCount = 0;
int currentPowerMode = POWER_MODE_AUTO;
int currentBaudRate = DEFAULT_BAUD;
uint8_t rxPacketBuffer[64];
uint8_t rxPacketIndex = 0;
bool rxInPacket = false;
uint8_t rxExpectedLen = 0;
unsigned long lastLedBlink = 0;
bool ledState = false;
bool sdCardAvailable = false;
String sdStatusMsg = "Not connected";

String wifiSSID = "";
String wifiPass = "";
unsigned long lastWiFiAttemptMs = 0;
unsigned long lastApCheckMs = 0;
unsigned long txFramesIn = 0;
unsigned long txFramesForwarded = 0;
unsigned long txFramesCrcBad = 0;
unsigned long txBytesIn = 0;
unsigned long txAddrFramesIn = 0;
unsigned long txStdFramesIn = 0;
uint32_t txTypeSeen[256] = {0};
uint32_t txTypeForwarded[256] = {0};
unsigned long lastCopterTelemetryMs = 0;
unsigned long lastTxDiagPrintMs = 0;
unsigned long prevTxBytesIn = 0;
unsigned long prevTxFramesIn = 0;
unsigned long prevTxFramesForwarded = 0;
unsigned long prevTxFramesCrcBad = 0;
unsigned long prevTxAddrFramesIn = 0;
unsigned long prevTxStdFramesIn = 0;

// CRC8
uint8_t crsfCrc8(const uint8_t* data, size_t len) {
    uint8_t crc = 0;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t b = 0; b < 8; b++) crc = (crc & 0x80) ? (crc << 1) ^ 0xD5 : (crc << 1);
    }
    return crc;
}

// Отправка в приёмник
void sendCrsfPacketToReceiver(uint8_t* payload, uint8_t len) {
    uint8_t pkt[64], idx=0;
    pkt[idx++]=0xC8; pkt[idx++]=len+2; pkt[idx++]=CRSF_ADDRESS_RECEIVER; pkt[idx++]=CRSF_ADDRESS_HANDSET;
    memcpy(pkt+idx, payload, len); idx+=len;
    pkt[idx]=crsfCrc8(pkt+2, idx-2); idx++;
    for(int i=0; i<idx; i++) { while(!receiverSerial.availableForWrite()) yield(); receiverSerial.write(pkt[i]); }
    receiverSerial.flush();
}

// Прозрачный форвардинг уже готового CRSF пакета в приёмник (телеметрия от TX)
void forwardRawPacketToReceiver(const uint8_t* packet, size_t len) {
    if (len == 0) return;
    receiverSerial.write(packet, len);
    receiverSerial.flush();
}

void forwardFrameToReceiverByType(uint8_t frameType, const uint8_t* payload, size_t payloadLen) {
    // Стандартный CRSF кадр (без address extension):
    // [sync][len][type][payload...][crc]
    if (payloadLen + 4 > 64) return;
    uint8_t out[64];
    size_t idx = 0;
    out[idx++] = 0xC8;
    out[idx++] = static_cast<uint8_t>(payloadLen + 2); // type + crc
    out[idx++] = frameType;
    if (payloadLen > 0 && payload != nullptr) {
        memcpy(out + idx, payload, payloadLen);
        idx += payloadLen;
    }
    out[idx++] = crsfCrc8(out + 2, idx - 2);
    receiverSerial.write(out, idx);
    receiverSerial.flush();
}

static bool isValidCrsfSyncByte(uint8_t b) {
    return (b >= 0xC8 && b <= 0xEF);
}

static void printHexFrame(const char* prefix, const uint8_t* data, uint8_t len) {
    Serial.printf("%s len=%u: ", prefix, len);
    for (uint8_t i = 0; i < len; i++) {
        Serial.printf("%02X", data[i]);
        if (i + 1 < len) Serial.print(" ");
    }
    Serial.println();
}

static bool isDroneTelemetryFrameType(uint8_t frameType) {
    // Для режима "максимум телеметрии": пропускаем всё, кроме сервисных
    // кадров Lua/Device API (0x28..0x2E), чтобы не засорять поток управления модулем.
    return !(frameType >= CRSF_FRAMETYPE_DEVICE_PING && frameType <= CRSF_FRAMETYPE_ELRS_STATUS);
}

static bool isCopterTelemetryEvidenceType(uint8_t frameType) {
    // Типы, которые обычно приходят именно от FC/дрона, а не от линк-статистики ретранслятора.
    switch (frameType) {
        case 0x02: // GPS
        case 0x07: // Vario
        case 0x08: // Battery
        case 0x09: // Baro altitude
        case 0x0B: // Heartbeat
        case 0x1E: // Attitude
        case 0x21: // Flight mode
            return true;
        default:
            return false;
    }
}

static bool hasCopterAirLinkByLinkStats(uint8_t frameType, const uint8_t* payload, uint8_t payloadLen) {
    // CRSF Link Statistics (0x14), payload[8] = downlink link quality.
    // Для реального линка с бортом ожидаем downlink LQ > 0.
    if (frameType == 0x14 && payload != nullptr && payloadLen >= 9) {
        return payload[8] > 0;
    }
    return false;
}

// Обработка данных от приёмника и ретрансляция
void processReceiverData() {
    static unsigned long lastDiag = 0;
    while (receiverSerial.available()) {
        uint8_t b = receiverSerial.read();

        if (!rxInPacket && isValidCrsfSyncByte(b)) {
            rxInPacket = true;
            rxPacketIndex = 0;
            rxExpectedLen = 0;
        }

        if (rxInPacket) {
            if (rxPacketIndex < sizeof(rxPacketBuffer)) rxPacketBuffer[rxPacketIndex++] = b;
            if (rxPacketIndex == 2) {
                uint8_t frameLen = rxPacketBuffer[1];
                if (frameLen < 2 || frameLen > 62) {
                    rxInPacket = false;
                    continue;
                }
                rxExpectedLen = frameLen + 2;
            }

            if (rxPacketIndex == rxExpectedLen && rxExpectedLen > 0) {
                uint8_t calcCrc = crsfCrc8(rxPacketBuffer + 2, rxPacketIndex - 3);
                uint8_t expCrc = rxPacketBuffer[rxPacketIndex - 1];

#if DEBUG_RX2_VERBOSE
                String hexDump = "RX2: len=" + String(rxPacketIndex) + " HEX: ";
                for (uint8_t i = 0; i < rxPacketIndex; i++) {
                    if (rxPacketBuffer[i] < 0x10) hexDump += "0";
                    hexDump += String(rxPacketBuffer[i], HEX);
                    hexDump += " ";
                }
                hexDump += "crc_calc=0x" + String(calcCrc, HEX) + " exp=0x" + String(expCrc, HEX) + " ";
                hexDump += (calcCrc == expCrc) ? "OK" : "BAD";
                Serial.println(hexDump);
#else
                if (calcCrc != expCrc && millis() - lastDiag > 1000) {
                    Serial.printf("RX2 CRC BAD: len=%u calc=0x%02X exp=0x%02X\n", rxPacketIndex, calcCrc, expCrc);
                    lastDiag = millis();
                }
#endif

                if (calcCrc == expCrc) {
                    lastPacketTime = millis();
                    packetCount++;
                    // Forward validated CRSF frame transparently RX -> TX.
                    // Repacking here breaks frame type/length for standard frames (e.g. type 0x16).
                    txController.sendRawFrame(rxPacketBuffer, rxPacketIndex);
                }
                rxInPacket = false;
            } else if (rxPacketIndex >= sizeof(rxPacketBuffer)) {
                rxInPacket = false;
            }
        }
    }
}

void updateRelay() {
    bool on = (currentPowerMode==POWER_MODE_ALWAYS_ON) ? true :
              (currentPowerMode==POWER_MODE_ALWAYS_OFF) ? false : linkToRadio;
    digitalWrite(RELAY_PIN, on ? HIGH : LOW);
}

void loadSettings() {
    prefs.begin("elrs", false);
    currentPowerMode = prefs.getInt("pmode", POWER_MODE_AUTO);
    wifiSSID = prefs.getString("wifi_ssid", "");
    wifiPass = prefs.getString("wifi_pass", "");
    prefs.end();
}
void savePowerMode(int m) { prefs.begin("elrs", false); prefs.putInt("pmode", m); prefs.end(); delay(50); currentPowerMode=m; }
void saveWiFiSettings(const String& ssid, const String& pass) {
    prefs.begin("elrs", false);
    prefs.putString("wifi_ssid", ssid);
    prefs.putString("wifi_pass", pass);
    prefs.end();
    wifiSSID = ssid;
    wifiPass = pass;
}

bool startSoftAP() {
    bool apCfgOk = WiFi.softAPConfig(apIP, apIP, IPAddress(255,255,255,0));
    bool apOk = WiFi.softAP("ELRS_Repeater_Pro", "12345678", AP_CHANNEL, 0);
    Serial.printf("AP config: %s, AP start: %s\n", apCfgOk ? "OK" : "FAIL", apOk ? "OK" : "FAIL");
    if (apOk) dnsServer.start(DNS_PORT, "*", apIP);
    Serial.println("AP SSID: " + WiFi.softAPSSID());
    Serial.println("AP IP: " + WiFi.softAPIP().toString());
    return apOk;
}

void connectHomeWiFi(bool blocking) {
    if (wifiSSID.length() == 0) return;
    if (WiFi.status() == WL_CONNECTED) return;

    WiFi.disconnect(false, false);
    WiFi.begin(wifiSSID.c_str(), wifiPass.c_str());

    if (!blocking) {
        Serial.println("STA reconnect started");
        return;
    }

    Serial.print("Connecting to WiFi");
    unsigned long started = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - started < WIFI_CONNECT_TIMEOUT_MS) {
        esp_task_wdt_reset();
        delay(250);
        Serial.print(".");
    }
    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("\nConnected! STA IP: " + WiFi.localIP().toString());
    } else {
        Serial.printf("\nFailed to connect. WiFi.status()=%d\n", (int)WiFi.status());
    }
}

// SD-карта
bool initSDCard() {
    if(!SD.begin(SD_CS_PIN)) {
        sdStatusMsg = "Init failed";
        Serial.println("SD Card init failed");
        return false;
    }
    File root = SD.open("/");
    if(!root) {
        sdStatusMsg = "No card";
        Serial.println("No SD card");
        return false;
    }
    root.close();
    sdStatusMsg = "OK";
    Serial.println("SD mounted");
    return true;
}
bool streamFileFromPriority(const String& path, const String& contentType) {
    if(sdCardAvailable && SD.exists(path)) {
        File f = SD.open(path, FILE_READ);
        if(f) { server.streamFile(f, contentType); f.close(); return true; }
    }
    if(LittleFS.exists(path)) {
        fs::File f = LittleFS.open(path, "r");
        if(f) { server.streamFile(f, contentType); f.close(); return true; }
    }
    return false;
}

// Веб-интерфейс
void handleRoot() {
    String html = R"rawliteral(
<!DOCTYPE html>
<html><head><meta charset="UTF-8"><title>ELRS Repeater Pro</title><meta name="viewport" content="width=device-width, initial-scale=1">
<style>
*{box-sizing:border-box} body{font-family:'Segoe UI',Arial;margin:0;padding:20px;background:#0a0e1a;color:#e0e0e0}
.container{max-width:900px;margin:0 auto} h1{color:#e94560;text-align:center}
.tabs{display:flex;gap:5px;margin-bottom:20px;flex-wrap:wrap} .tab-btn{padding:12px 24px;background:#1a1f2e;border:none;border-radius:8px 8px 0 0;color:#aaa;cursor:pointer} .tab-btn.active{background:#e94560;color:#fff}
.tab-content{display:none;background:#11161f;border-radius:0 12px 12px 12px;padding:20px} .tab-content.active{display:block}
.card{background:#1a1f2e;border-radius:12px;padding:20px;margin-bottom:20px} .card h3{margin-top:0;color:#e94560;border-bottom:1px solid #2a3040;padding-bottom:10px}
table{width:100%;border-collapse:collapse} td,th{padding:12px;border-bottom:1px solid #2a3040;text-align:left} th{color:#e94560}
button{background:#e94560;border:none;padding:10px 20px;border-radius:8px;color:#fff;cursor:pointer} button:hover{background:#c0314a}
button.secondary{background:#2a3040} input{background:#2a3040;border:none;padding:10px;border-radius:6px;color:#fff}
.status-ok{color:#2ecc71} .status-bad{color:#e74c3c} .flex{display:flex;gap:10px;flex-wrap:wrap}
.menu-tree{list-style:none;padding:0} .menu-item{padding:8px;border-bottom:1px solid #2a3040;cursor:pointer} .menu-folder{font-weight:bold} .menu-value{float:right;color:#e94560}
</style>
<script>
let curParent=0;

async function refreshDisplay(){
    let r=await fetch('/api/status'), d=await r.json();
    document.getElementById('linkRadio').innerHTML=d.linkRadio?'✅ CONNECTED':'❌ DISCONNECTED';
    document.getElementById('linkCopter').innerHTML=d.linkCopter?'✅ CONNECTED':'❌ DISCONNECTED';
    document.getElementById('packets').innerText=d.packets;
    document.getElementById('txpower').innerText=(d.txPowerRead||'---')+' ('+(d.txPowerSet||'-')+')';
    document.getElementById('rfmd').innerText=(d.rfmdRead||'---')+' ('+(d.rfmdSet||'-')+')';
    document.getElementById('switchMode').innerText=(d.switchModeRead||'---')+' ('+(d.switchModeSet||'-')+')';
    document.getElementById('antennaMode').innerText=(d.antennaModeRead||'---')+' ('+(d.antennaModeSet||'-')+')';
    document.getElementById('telemRatio').innerText=(d.telemRatioRead||'---')+' ('+(d.telemRatioSet||'-')+')';
    document.getElementById('pktrate').innerText=d.pktrate+' Hz';
    document.getElementById('deviceName').innerText=d.deviceName;
    document.getElementById('baudrate').innerText=d.baudrate;
    let sdElem = document.getElementById('sdStatus');
    if(d.sd === 'OK'){
        sdElem.innerHTML = '✅ Connected';
        sdElem.className = 'status-ok';
    } else {
        sdElem.innerHTML = '❌ ' + d.sd;
        sdElem.className = 'status-bad';
    }
    
    let p=await fetch('/api/powerMode'), pd=await p.json();
    document.getElementById('relayState').innerHTML=pd.relayState?'✅ ON':'❌ OFF';
    document.getElementById('powerModeText').innerHTML = pd.mode==0?'Auto':(pd.mode==1?'Always ON':'Always OFF');
}

async function fullUpdate(){
    await refreshDisplay();
    let r=await fetch('/api/status'), d=await r.json();
    let p=await fetch('/api/powerMode'), pd=await p.json();
    document.getElementById('modeAuto').checked = (pd.mode==0);
    document.getElementById('modeAlwaysOn').checked = (pd.mode==1);
    document.getElementById('modeAlwaysOff').checked = (pd.mode==2);
}

async function applyPower(){
    let m=document.getElementById('modeAuto').checked?0:(document.getElementById('modeAlwaysOn').checked?1:2);
    await fetch('/api/setPowerMode?mode='+m);
    fullUpdate();
}
async function bindRx(){ if(confirm('Bind receiver?')) await fetch('/api/bindRx'); }
async function resetRx(){ if(confirm('Reset bind?')) await fetch('/api/resetRxBind'); }
async function enableRxWifi(){ await fetch('/api/enableRxWifi'); alert('WiFi enabled'); }

function switchTab(id){
    document.querySelectorAll('.tab-content').forEach(e=>e.classList.remove('active'));
    document.querySelectorAll('.tab-btn').forEach(e=>e.classList.remove('active'));
    document.getElementById('tab-'+id).classList.add('active');
    document.querySelector(`.tab-btn[data-tab="${id}"]`).classList.add('active');
    if(id==='transmitter') loadTxMenu(0);
    if(id==='wifi') fetchWiFi();
    if(id==='dashboard') fullUpdate();
}

async function loadTxMenu(parent){
    try {
        let r = await fetch('/api/tx/menu?parent='+parent);
        if(!r.ok) throw new Error('HTTP '+r.status);
        let m = await r.json();
        curParent = parent;
        let html = parent!==0?'<button class="secondary" onclick="loadTxMenu(0)">← Назад</button>':'';
        html += '<ul class="menu-tree">';
        (m.items || []).forEach(i=>{
            const name = String(i.name || '');
            const value = String(i.value || '');
            if(i.type==='folder') html+=`<li class="menu-item menu-folder" onclick="loadTxMenu(${i.id})">📁 ${name}</li>`;
            else if(i.type==='command') html+=`<li class="menu-item" onclick="execCmd(${i.id})">⚡ ${name}</li>`;
            else html+=`<li class="menu-item" onclick="editField(${i.id},'${name.replace(/'/g,"\\'")}','${value.replace(/'/g,"\\'")}')">${name} <span class="menu-value">${value}</span></li>`;
        });
        html += '</ul><hr><button class="secondary" onclick="loadTxMenu(curParent)">Refresh</button>';
        document.getElementById('tx-menu').innerHTML=html;
    } catch(e) {
        document.getElementById('tx-menu').innerHTML = '<div style="color:#f66">Error loading TX menu: '+e.message+'</div>';
        console.error(e);
    }
}
function execCmd(id){ if(confirm('Execute?')) fetch('/api/tx/command?id='+id).then(()=>loadTxMenu(curParent)); }
function editField(id,name,val){
    let nv=prompt('New value for '+name,val);
    if(nv!==null) fetch('/api/tx/set?id='+id+'&value='+encodeURIComponent(nv)).then(()=>loadTxMenu(curParent));
}
async function fetchWiFi(){
    let r=await fetch('/api/wifi'), d=await r.json();
    document.getElementById('wifiStatus').innerHTML=d.connected?'✅ Connected':'❌ Disconnected';
    document.getElementById('wifiIP').innerText=d.ip||'---';
    document.getElementById('wifiSSID').value=d.ssid||'';
}
function saveWiFi(){
    let ssid=document.getElementById('wifiSSID').value;
    let pass=document.getElementById('wifiPass').value;
    if(!ssid) return alert('SSID required');
    fetch('/api/wifi',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:'ssid='+encodeURIComponent(ssid)+'&pass='+encodeURIComponent(pass)})
    .then(()=>{alert('Saved. Restarting...');setTimeout(()=>location.reload(),2000);});
}

async function applyTxNamed(apiPath,inputId){
    const v = document.getElementById(inputId).value;
    const r = await fetch(apiPath+'?value='+encodeURIComponent(v));
    if(!r.ok){ alert('TX apply failed ('+r.status+')'); }
    await refreshDisplay();
}

setInterval(refreshDisplay, 1000);
window.onload=()=>{ fullUpdate(); };
</script>
</head><body>
<div class="container"><h1>ELRS Repeater Pro</h1>
<div class="tabs">
    <button class="tab-btn active" data-tab="dashboard" onclick="switchTab('dashboard')">Dashboard</button>
    <button class="tab-btn" data-tab="transmitter" onclick="switchTab('transmitter')">Transmitter</button>
    <button class="tab-btn" data-tab="receiver" onclick="switchTab('receiver')">Receiver</button>
    <button class="tab-btn" data-tab="wifi" onclick="switchTab('wifi')">WiFi</button>
</div>
<div id="tab-dashboard" class="tab-content active">
    <div class="card"><h3>Status</h3><table>
        <tr><th>Device</th><td id="deviceName">---</td></tr>
        <tr><th>Link Radio</th><td id="linkRadio">---</td></tr>
        <tr><th>Link Copter</th><td id="linkCopter">---</td></tr>
        <tr><th>Packets</th><td id="packets">---</td></tr>
        <tr><th>TX Power</th><td id="txpower">---</td></tr>
        <tr><th>RFMD</th><td id="rfmd">---</td></tr>
        <tr><th>Switch Mode</th><td id="switchMode">---</td></tr>
        <tr><th>Antenna Mode</th><td id="antennaMode">---</td></tr>
        <tr><th>Telem Ratio</th><td id="telemRatio">---</td></tr>
        <tr><th>Packet Rate</th><td id="pktrate">---</td></tr>
        <tr><th>CRSF Baudrate</th><td id="baudrate">---</td></tr>
        <tr><th>SD Card</th><td id="sdStatus">---</td></tr>
    </table></div>
    <div class="card"><h3>Power Control</h3>
        <label><input type="radio" name="pm" value="0" id="modeAuto"> Auto (Bind only)</label><br>
        <label><input type="radio" name="pm" value="1" id="modeAlwaysOn"> Always ON</label><br>
        <label><input type="radio" name="pm" value="2" id="modeAlwaysOff"> Always OFF</label><br>
        <button onclick="applyPower()">Apply</button>
        <hr><table><tr><th>Relay State</th><td id="relayState">---</td></tr>
        <tr><th>Selected Mode</th><td id="powerModeText">---</td></tr></table>
    </div>
    <div class="card"><h3>TX Quick Control</h3>
        <div class="flex" style="align-items:center">
            <label>TX Power:</label>
            <select id="setTxPower"><option value="10">10</option><option value="25">25</option><option value="50">50</option><option value="100">100</option><option value="250">250</option><option value="500">500</option><option value="1000">1000</option></select>
            <button onclick="applyTxNamed('/api/tx/setPower','setTxPower')">Apply</button>
        </div>
        <br>
        <div class="flex" style="align-items:center">
            <label>RFMD:</label>
            <select id="setRfmd">
                <option value="25Hz">0 - 900 - 25Hz LoRa -123dBm</option>
                <option value="50Hz">1 - 900 - 50Hz LoRa -120dBm</option>
                <option value="100Hz">2 - 900 - 100Hz LoRa -117dBm</option>
                <option value="100Hz Full">3 - 900 - 100Hz Full -112dBm</option>
                <option value="200Hz">5 - 900 - 200Hz LoRa -112dBm</option>
                <option value="200Hz Full">6 - 900 - 200Hz Full -111dBm</option>
                <option value="250Hz">7 - 900 - 250Hz LoRa -111dBm</option>
                <option value="D50">10 - 900 - D50 -112dBm</option>
                <option value="K1000 Full">11 - 900 - K1000 Full -101dBm</option>

                <option value="50Hz">21 - 2.4 - 50Hz LoRa -115dBm</option>
                <option value="100Hz Full">23 - 2.4 - 100Hz Full -112dBm</option>
                <option value="150Hz">24 - 2.4 - 150Hz LoRa -112dBm</option>
                <option value="250Hz">27 - 2.4 - 250Hz LoRa -108dBm</option>
                <option value="333Hz Full">28 - 2.4 - 333Hz Full -105dBm</option>
                <option value="500Hz">29 - 2.4 - 500Hz LoRa -105dBm</option>
                <option value="D250">30 - 2.4 - D250 -104dBm</option>
                <option value="D500">31 - 2.4 - D500 -104dBm</option>
                <option value="F500">32 - 2.4 - F500 -104dBm</option>
                <option value="F1000">33 - 2.4 - F1000 -104dBm</option>
                <option value="DK250">34 - 2.4 - DK250 -103dBm</option>
                <option value="DK500">35 - 2.4 - DK500 -103dBm</option>
                <option value="K1000">36 - 2.4 - K1000 -103dBm</option>

                <option value="100Hz Full">100 - X-Band - 100Hz Full -112dBm</option>
                <option value="150Hz">101 - X-Band - 150Hz -112dBm</option>
            </select>
            <button onclick="applyTxNamed('/api/tx/setRfmd','setRfmd')">Apply</button>
        </div>
        <br>
        <div class="flex" style="align-items:center">
            <label>Switch Mode:</label>
            <select id="setSwitchMode">
                <option value="Hybrid">Hybrid</option><option value="Wide">Wide</option>
                <option value="8ch">Full Res 8ch</option><option value="12ch Mixed">Full Res 12ch Mixed</option>
                <option value="16ch Rate/2">Full Res 16ch Rate/2</option>
            </select>
            <button onclick="applyTxNamed('/api/tx/setSwitchMode','setSwitchMode')">Apply</button>
        </div>
        <br>
        <div class="flex" style="align-items:center">
            <label>Antenna Mode:</label>
            <select id="setAntennaMode">
                <option value="Gemini">Gemini</option><option value="Ant 1">Ant 1</option>
                <option value="Ant 2">Ant 2</option><option value="Switch">Switch</option>
            </select>
            <button onclick="applyTxNamed('/api/tx/setAntennaMode','setAntennaMode')">Apply</button>
        </div>
        <br>
        <div class="flex" style="align-items:center">
            <label>Telem Ratio:</label>
            <select id="setTelemRatio">
                <option value="Std">Std</option><option value="1:2">1:2</option><option value="1:4">1:4</option>
                <option value="1:8">1:8</option><option value="1:16">1:16</option><option value="1:32">1:32</option>
                <option value="1:64">1:64</option><option value="1:128">1:128</option>
                <option value="Race">Race</option><option value="Off">Off</option>
            </select>
            <button onclick="applyTxNamed('/api/tx/setTelemRatio','setTelemRatio')">Apply</button>
        </div>
    </div>
    <div class="card"><h3>System</h3>
    </div>
</div>
<div id="tab-transmitter" class="tab-content">
    <div class="card"><h3>TX Settings</h3><div id="tx-menu">Loading...</div></div>
    <div class="card"><h3>Lua / Custom TX UI</h3>
        <button onclick="window.location.href='/tx_interface.html'">Open Lua Interface</button>
        <button class="secondary" onclick="window.location.href='/lua'">Open Lua Alias</button>
        <p style="margin-top:10px;color:#bbb;font-size:0.9em">If a custom tx_interface.html exists on SD:/www, it will be loaded here.</p>
    </div>
</div>
<div id="tab-receiver" class="tab-content">
    <div class="card"><h3>Receiver</h3>
        <button onclick="bindRx()">Bind</button>
        <button class="secondary" onclick="resetRx()">Reset Bind</button>
        <button onclick="enableRxWifi()">Enable WiFi</button>
    </div>
</div>
<div id="tab-wifi" class="tab-content">
    <div class="card"><h3>Home WiFi</h3>
        <div>Status: <span id="wifiStatus">---</span></div>
        <div>IP: <span id="wifiIP">---</span></div>
        <hr>
        <input id="wifiSSID" placeholder="SSID" value=""><br><br>
        <input id="wifiPass" type="password" placeholder="Password"><br><br>
        <button onclick="saveWiFi()">Save & Restart</button>
    </div>
</div>
</div></body></html>
)rawliteral";
    server.send(200, "text/html", html);
}

// API
void handleApiStatus() {
    DynamicJsonDocument d(640);
    d["linkRadio"] = linkToRadio;
    d["linkCopter"] = (millis() - lastCopterTelemetryMs) < 2000;
    d["packets"] = packetCount;
    d["txpower"] = txController.getTxPower();
    d["pktrate"] = txController.getPktRate();
    d["txPowerRead"] = txController.getTxPowerRead();
    d["txPowerSet"] = txController.getTxPowerSet();
    d["rfmdRead"] = txController.getRfModeRead();
    d["rfmdSet"] = txController.getRfModeSet();
    d["switchModeRead"] = txController.getSwitchModeRead();
    d["switchModeSet"] = txController.getSwitchModeSet();
    d["antennaModeRead"] = txController.getAntennaModeRead();
    d["antennaModeSet"] = txController.getAntennaModeSet();
    d["telemRatioRead"] = txController.getTelemRatioRead();
    d["telemRatioSet"] = txController.getTelemRatioSet();
    d["deviceName"] = txController.getDeviceName();
    d["baudrate"] = currentBaudRate;
    d["sd"] = sdStatusMsg;
    d["txIn"] = txFramesIn;
    d["txFwd"] = txFramesForwarded;
    d["txCrcBad"] = txFramesCrcBad;
    d["txBytesIn"] = txBytesIn;
    String s; serializeJson(d, s); server.send(200, "application/json", s);
}
void handleApiPowerMode() {
    DynamicJsonDocument d(128);
    d["mode"] = currentPowerMode;
    d["relayState"] = digitalRead(RELAY_PIN);
    String s; serializeJson(d, s); server.send(200, "application/json", s);
}
void handleApiSetPowerMode() {
    if(server.hasArg("mode")){ int m=server.arg("mode").toInt(); if(m>=0&&m<=2){ savePowerMode(m); server.send(200); return; }}
    server.send(400);
}
void handleApiBindRx() { uint8_t p[]={0x10,0x01}; sendCrsfPacketToReceiver(p,2); server.send(200); }
void handleApiResetRxBind() {
    String c="set expresslrs_uid = 0"; uint8_t p[64]; int i=0;
    p[i++]=0x10; p[i++]=0x08; for(char ch:c) p[i++]=ch;
    sendCrsfPacketToReceiver(p,i); server.send(200);
}
void handleApiEnableRxWifi() {
    for(int i=0;i<3;i++){ digitalWrite(RELAY_PIN,LOW); delay(200); digitalWrite(RELAY_PIN,HIGH); delay(200); }
    uint8_t p[]={0x00,0x00,0x01,0x01}; sendCrsfPacketToReceiver(p,4); server.send(200);
}
void handleApiTxMenu() {
    int p=server.hasArg("parent")?server.arg("parent").toInt():0;
    server.send(200,"application/json",txController.getMenuJson(p));
}
void handleApiTxSet() {
    if(server.hasArg("id")&&server.hasArg("value"))
        server.send(txController.setValue(server.arg("id").toInt(),server.arg("value"))?200:400);
    else server.send(400);
}
void handleApiTxSetPower() {
    if (!server.hasArg("value")) { server.send(400, "text/plain", "Missing value"); return; }
    server.send(txController.setTxPowerByName(server.arg("value")) ? 200 : 400);
}
void handleApiTxSetRfmd() {
    if (!server.hasArg("value")) { server.send(400, "text/plain", "Missing value"); return; }
    server.send(txController.setRfModeByName(server.arg("value")) ? 200 : 400);
}
void handleApiTxSetSwitchMode() {
    if (!server.hasArg("value")) { server.send(400, "text/plain", "Missing value"); return; }
    server.send(txController.setSwitchModeByName(server.arg("value")) ? 200 : 400);
}
void handleApiTxSetAntennaMode() {
    if (!server.hasArg("value")) { server.send(400, "text/plain", "Missing value"); return; }
    server.send(txController.setAntennaModeByName(server.arg("value")) ? 200 : 400);
}
void handleApiTxSetTelemRatio() {
    if (!server.hasArg("value")) { server.send(400, "text/plain", "Missing value"); return; }
    server.send(txController.setTelemRatioByName(server.arg("value")) ? 200 : 400);
}
void handleApiTxCommand() {
    if(server.hasArg("id")) server.send(txController.executeCommand(server.arg("id").toInt())?200:400);
    else server.send(400);
}
void handleTxInterface() {
    if(!streamFileFromPriority("/www/tx_interface.html","text/html")) {
        String html = R"rawliteral(<html><body style="background:#111;color:#eee;font-family:Segoe UI"><div id="tx">Default. Put tx_interface.html on SD:/www/</div><script>
async function load(p=0){
    try {
        let r = await fetch('/api/tx/menu?parent='+p);
        if(!r.ok) throw new Error('HTTP '+r.status);
        let m = await r.json();
        let h = '';
        if(p) h += '<button onclick="load(0)">Back</button>';
        h += '<ul>';
        m.items.forEach(i => {
            if(i.type == 'folder') h += '<li onclick="load('+i.id+')">📁'+i.name+'</li>';
            else if(i.type == 'command') h += '<li onclick="fetch(\'/api/tx/command?id='+i.id+'\')">⚡'+i.name+'</li>';
            else h += '<li onclick="edit('+i.id+',\''+i.name+'\',\''+i.value+'\')">'+i.name+' <span>'+i.value+'</span></li>';
        });
        h += '</ul>';
        document.getElementById('tx').innerHTML = h;
    } catch(e) {
        document.getElementById('tx').innerHTML = '<div style="color:#f66">Error loading TX menu: '+e.message+'</div>';
        console.error(e);
    }
}
function edit(i,n,v){let nv=prompt(n,v);if(nv)fetch('/api/tx/set?id='+i+'&value='+nv).then(()=>load(curParent));}
let curParent=0;load(0);</script></body></html>)rawliteral";
        server.send(200,"text/html",html);
    }
}
void handleApiWiFiGet() {
    DynamicJsonDocument d(256);
    d["ssid"] = wifiSSID;
    d["connected"] = (WiFi.status() == WL_CONNECTED);
    d["ip"] = WiFi.localIP().toString();
    String s; serializeJson(d, s);
    server.send(200, "application/json", s);
}
void handleApiWiFiPost() {
    if (server.hasArg("ssid") && server.hasArg("pass")) {
        saveWiFiSettings(server.arg("ssid"), server.arg("pass"));
        server.send(200, "text/plain", "OK. Restarting...");
        delay(500); ESP.restart();
    } else server.send(400, "text/plain", "Missing ssid/pass");
}

void setupWebServer() {
    server.on("/", handleRoot);
    server.on("/api/status", handleApiStatus);
    server.on("/api/powerMode", handleApiPowerMode);
    server.on("/api/setPowerMode", handleApiSetPowerMode);
    server.on("/api/bindRx", handleApiBindRx);
    server.on("/api/resetRxBind", handleApiResetRxBind);
    server.on("/api/enableRxWifi", handleApiEnableRxWifi);
    server.on("/api/tx/menu", handleApiTxMenu);
    server.on("/api/tx/set", handleApiTxSet);
    server.on("/api/tx/setPower", handleApiTxSetPower);
    server.on("/api/tx/setRfmd", handleApiTxSetRfmd);
    server.on("/api/tx/setSwitchMode", handleApiTxSetSwitchMode);
    server.on("/api/tx/setAntennaMode", handleApiTxSetAntennaMode);
    server.on("/api/tx/setTelemRatio", handleApiTxSetTelemRatio);
    server.on("/api/tx/command", handleApiTxCommand);
    server.on("/tx_interface.html", handleTxInterface);
    server.on("/lua", handleTxInterface);
    server.on("/api/wifi", HTTP_GET, handleApiWiFiGet);
    server.on("/api/wifi", HTTP_POST, handleApiWiFiPost);
    server.onNotFound([]() {
        server.sendHeader("Location", String("http://") + apIP.toString(), true);
        server.send(302, "text/plain", "Redirecting");
    });
    server.begin();
    Serial.println("Web server started");
}

void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("\n=== ELRS Repeater Pro Starting ===");
    
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW); delay(200); digitalWrite(LED_PIN, HIGH);
    
    esp_task_wdt_init(WDT_TIMEOUT_MS/1000, true);
    esp_task_wdt_add(NULL);
    
    if(!LittleFS.begin(true)) Serial.println("LittleFS mount failed");
    else Serial.println("LittleFS mounted");
    
    loadSettings();
    Serial.printf("Settings: baud=%d, powerMode=%d, WiFi SSID=%s\n", currentBaudRate, currentPowerMode, wifiSSID.c_str());
    
    pinMode(RELAY_PIN, OUTPUT);
    digitalWrite(RELAY_PIN, LOW);
    
    receiverSerial.begin(currentBaudRate, SERIAL_8N1, 16, 17);
    uart_set_line_inverse(UART_NUM_2, UART_SIGNAL_RXD_INV | UART_SIGNAL_TXD_INV);
    txController.begin(currentBaudRate);
    
    sdCardAvailable = initSDCard();
    Serial.printf("SD Card: %s\n", sdStatusMsg.c_str());
    
    WiFi.mode(WIFI_AP_STA);
    WiFi.setSleep(false);
    WiFi.setAutoReconnect(false);

    startSoftAP();
    if (wifiSSID.length() > 0) connectHomeWiFi(true);
    else Serial.println("Home WiFi SSID is empty, STA connect skipped.");
    
    setupWebServer();
    lastPacketTime = millis();
    Serial.println("Setup complete");
}

void loop() {
    processReceiverData();
    txController.update();
    linkToRadio = (millis()-lastPacketTime < TIMEOUT_MS);
    updateRelay();
    if (!linkToRadio) {
        if (millis() - lastLedBlink > 200) {
            lastLedBlink = millis();
            ledState = !ledState;
            digitalWrite(LED_PIN, ledState ? LOW : HIGH);
        }
    } else {
        digitalWrite(LED_PIN, HIGH);
        lastLedBlink = millis();
    }
    dnsServer.processNextRequest();
    server.handleClient();

    unsigned long now = millis();
    if (now - lastApCheckMs > WIFI_RECONNECT_INTERVAL_MS) {
        lastApCheckMs = now;
        if (WiFi.softAPSSID().length() == 0) {
            Serial.println("AP disappeared, restarting AP...");
            startSoftAP();
        }
    }

    if (now - lastTxDiagPrintMs > 2000) {
        unsigned long dBytes = txBytesIn - prevTxBytesIn;
        unsigned long dIn = txFramesIn - prevTxFramesIn;
        unsigned long dFwd = txFramesForwarded - prevTxFramesForwarded;
        unsigned long dBad = txFramesCrcBad - prevTxFramesCrcBad;
        unsigned long dAddr = txAddrFramesIn - prevTxAddrFramesIn;
        unsigned long dStd = txStdFramesIn - prevTxStdFramesIn;
        prevTxBytesIn = txBytesIn;
        prevTxFramesIn = txFramesIn;
        prevTxFramesForwarded = txFramesForwarded;
        prevTxFramesCrcBad = txFramesCrcBad;
        prevTxAddrFramesIn = txAddrFramesIn;
        prevTxStdFramesIn = txStdFramesIn;
        lastTxDiagPrintMs = now;

        Serial.printf(
            "TXDIAG total[bytes=%lu in=%lu fwd=%lu bad=%lu addr=%lu std=%lu] delta[bytes=%lu in=%lu fwd=%lu bad=%lu addr=%lu std=%lu]\n",
            txBytesIn, txFramesIn, txFramesForwarded, txFramesCrcBad, txAddrFramesIn, txStdFramesIn,
            dBytes, dIn, dFwd, dBad, dAddr, dStd
        );
        Serial.printf(
            "TXTYPES in[02=%lu 07=%lu 08=%lu 09=%lu 0B=%lu 14=%lu 1C=%lu 1E=%lu 21=%lu 29=%lu 2B=%lu] fwd[02=%lu 07=%lu 08=%lu 09=%lu 0B=%lu 14=%lu 1C=%lu 1E=%lu 21=%lu 29=%lu 2B=%lu]\n",
            txTypeSeen[0x02], txTypeSeen[0x07], txTypeSeen[0x08], txTypeSeen[0x09], txTypeSeen[0x0B],
            txTypeSeen[0x14], txTypeSeen[0x1C], txTypeSeen[0x1E], txTypeSeen[0x21], txTypeSeen[0x29], txTypeSeen[0x2B],
            txTypeForwarded[0x02], txTypeForwarded[0x07], txTypeForwarded[0x08], txTypeForwarded[0x09], txTypeForwarded[0x0B],
            txTypeForwarded[0x14], txTypeForwarded[0x1C], txTypeForwarded[0x1E], txTypeForwarded[0x21], txTypeForwarded[0x29], txTypeForwarded[0x2B]
        );
    }

    esp_task_wdt_reset();
    yield();
}

// ========== РЕАЛИЗАЦИЯ TransmitterController (полудуплекс, открытый сток) ==========
TransmitterController::TransmitterController() :
    _serial(nullptr), _connected(false), _lastResponseTime(0),
    _deviceInfoReceived(false), _fieldsRequested(false),
    _txPowerSet("-"), _rfModeSet("-"), _switchModeSet("-"), _antennaModeSet("-"), _telemRatioSet("-"),
    _lastOutALen(0), _lastOutBLen(0), _lastOutAMs(0), _lastOutBMs(0) {}

void TransmitterController::begin(unsigned long baud) {
    // Настраиваем пин как открытый сток
    pinMode(TX_MODULE_TX_PIN, OUTPUT);
    pinMode(TX_MODULE_RX_PIN, INPUT);

    // Инициализация UART с одним пином для RX и TX
    _serial = &Serial1;
    _serial->begin(baud, SERIAL_8N1, TX_MODULE_RX_PIN, TX_MODULE_TX_PIN);

    // Инверсию делает внешний инвертор на 2N7002.
    uart_set_mode(UART_NUM_1, UART_MODE_UART);
    uart_set_line_inverse(UART_NUM_1, UART_SIGNAL_INV_DISABLE);

    Serial.printf("TX UART started: RX=%d TX=%d baud=%lu\n", TX_MODULE_RX_PIN, TX_MODULE_TX_PIN, baud);
    delay(100);
    _sendDevicePing();
    _lastResponseTime = millis();
}

uint8_t TransmitterController::_crc8(const uint8_t* data, size_t len) {
    uint8_t crc = 0;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t bit = 0; bit < 8; bit++) {
            if (crc & 0x80) crc = (crc << 1) ^ 0xD5;
            else crc = (crc << 1);
        }
    }
    return crc;
}

void TransmitterController::_rememberOutFrame(const uint8_t* frame, uint8_t len) {
    if (!frame || len == 0 || len > sizeof(_lastOutA)) return;
    memcpy(_lastOutB, _lastOutA, _lastOutALen);
    _lastOutBLen = _lastOutALen;
    _lastOutBMs = _lastOutAMs;
    memcpy(_lastOutA, frame, len);
    _lastOutALen = len;
    _lastOutAMs = millis();
}

bool TransmitterController::_isLikelyEchoFrame(const uint8_t* frame, uint8_t len) const {
    if (!frame || len == 0) return false;
    unsigned long now = millis();
    const unsigned long windowMs = 300;
    if (_lastOutALen == len && (now - _lastOutAMs) <= windowMs && memcmp(_lastOutA, frame, len) == 0) return true;
    if (_lastOutBLen == len && (now - _lastOutBMs) <= windowMs && memcmp(_lastOutB, frame, len) == 0) return true;
    return false;
}

void TransmitterController::_sendCrsfPacket(uint8_t* payload, uint8_t len, uint8_t packetType) {
    auto sendOne = [&](uint8_t originAddr) {
        uint8_t packet[64];
        uint8_t idx = 0;
        packet[idx++] = CRSF_ADDRESS_MODULE;
        // Extended CRSF frame:
        // [sync][len][type][dest][origin][payload...][crc]
        packet[idx++] = len + 4;
        packet[idx++] = packetType;
        packet[idx++] = CRSF_ADDRESS_MODULE;
        packet[idx++] = originAddr;
        if (len > 0 && payload != nullptr) {
            memcpy(packet + idx, payload, len);
            idx += len;
        }
        packet[idx] = _crc8(packet + 2, idx - 2);
        idx++;

        Serial.printf("TX1: Sending pkt len=%u type=0x%02X origin=0x%02X crc=0x%02X\n",
            idx, packetType, originAddr, packet[idx - 1]);
#if DEBUG_TX1_RAW_CAPTURE
        printHexFrame("TX1 RAW OUT", packet, idx);
#endif
        _rememberOutFrame(packet, idx);
        _serial->write(packet, idx);
        _serial->flush();
    };

    sendOne(CRSF_ADDRESS_HANDSET);
    // ELRS 4+ compatibility: some stacks identify host as 0xEF (handset)
    if (packetType >= CRSF_FRAMETYPE_DEVICE_PING && packetType <= CRSF_FRAMETYPE_PARAMETER_WRITE &&
        CRSF_ADDRESS_HANDSET_ALT != CRSF_ADDRESS_HANDSET) {
        delay(1);
        sendOne(CRSF_ADDRESS_HANDSET_ALT);
    }
}

void TransmitterController::sendRawPacket(uint8_t* payload, uint8_t len, uint8_t destAddr) {
    uint8_t packet[64];
    uint8_t idx = 0;
    packet[idx++] = CRSF_ADDRESS_MODULE;
    packet[idx++] = len + 4;
    packet[idx++] = destAddr;
    packet[idx++] = CRSF_ADDRESS_MODULE;
    packet[idx++] = CRSF_ADDRESS_HANDSET;
    if (len > 0 && payload != nullptr) {
        memcpy(packet + idx, payload, len);
        idx += len;
    }
    packet[idx] = _crc8(packet + 2, idx - 2);
    idx++;

    _rememberOutFrame(packet, idx);
    _serial->write(packet, idx);
    _serial->flush();
}

void TransmitterController::sendRawFrame(const uint8_t* packet, size_t len) {
    if (!_serial || !packet || len == 0) return;
    _serial->write(packet, len);
    _serial->flush();
}

void TransmitterController::_sendDevicePing() {
    Serial.println("TX1: Sending DEVICE_PING...");
    _sendCrsfPacket(nullptr, 0, CRSF_FRAMETYPE_DEVICE_PING);
}

void TransmitterController::_requestField(uint8_t fieldId) {
    // Parameter request payload: [fieldId][chunk]
    uint8_t p[] = {fieldId, 0x00};
    _sendCrsfPacket(p, 2, CRSF_FRAMETYPE_PARAMETER_REQUEST);
}

void TransmitterController::_requestAllFields() {
    // On first sync _fields is empty, so we must actively probe parameter IDs.
    // ELRS modules typically expose parameters in low ID range.
    for (uint8_t id = 0; id <= 80; id++) {
        _requestField(id);
        delay(2);
    }
    Serial.println("TX1: Requested parameter fields 0..80");
    _fieldsRequested = true;
}

void TransmitterController::_writeParameter(uint8_t fieldId, uint32_t value) {
    uint8_t p[5] = {fieldId};
    p[1] = value & 0xFF;
    p[2] = (value >> 8) & 0xFF;
    p[3] = (value >> 16) & 0xFF;
    p[4] = (value >> 24) & 0xFF;
    _sendCrsfPacket(p, 5, CRSF_FRAMETYPE_PARAMETER_WRITE);
}

void TransmitterController::_writeParameter(uint8_t fieldId, const uint8_t* data, uint8_t len) {
    uint8_t p[64] = {fieldId};
    memcpy(p + 1, data, len);
    _sendCrsfPacket(p, 1 + len, CRSF_FRAMETYPE_PARAMETER_WRITE);
}

void TransmitterController::_processIncoming() {
    static uint8_t buf[128], idx=0;
    static bool inPacket=false;
    static uint8_t expLen=0;
    static uint16_t rawDumpBudget = 80;
    while (_serial->available()) {
        uint8_t b = _serial->read();
        txBytesIn++;
        if (!inPacket) {
            if (!isValidCrsfSyncByte(b)) continue;
            inPacket = true;
            idx = 0;
            expLen = 0;
        }
        if (inPacket) {
            if (idx < sizeof(buf)) buf[idx++] = b;
            if (idx == 2) {
                uint8_t frameLen = buf[1];
                if (frameLen < 2 || frameLen > 62) {
                    inPacket = false;
                    continue;
                }
                expLen = frameLen + 2;
            }
            if (idx == expLen && expLen>0) {
                if (_isLikelyEchoFrame(buf, idx)) {
#if DEBUG_TX1_RAW_CAPTURE
                    Serial.println("TX1: Echo frame dropped");
#endif
                    inPacket = false;
                    continue;
                }
                txFramesIn++;
                uint8_t expCrc = buf[idx-1];
                // Supported layouts:
                // 1) standard: [sync][len][type][payload...][crc]
                // 2) extended: [sync][len][type][dest][origin][payload...][crc]
                // 3) legacy addressed (older local format): [sync][len][to][from][type][payload...][crc]
                bool isLegacyAddressed = (
                    idx >= 6 &&
                    (buf[2] >= 0xC8) &&
                    (buf[3] >= 0xC8)
                );
                bool isExtended = (
                    !isLegacyAddressed &&
                    idx >= 6 &&
                    (buf[2] >= CRSF_FRAMETYPE_DEVICE_PING && buf[2] <= CRSF_FRAMETYPE_ELRS_STATUS) &&
                    (buf[3] >= 0xE0) &&
                    (buf[4] >= 0xE0)
                );

                if (isLegacyAddressed || isExtended) txAddrFramesIn++;
                else txStdFramesIn++;

                uint8_t frameType = isLegacyAddressed ? buf[4] : buf[2];
                txTypeSeen[frameType]++;

                uint8_t* payloadPtr = nullptr;
                uint8_t payloadLen = 0;
                uint8_t calcCrc = 0;

                if (isLegacyAddressed) {
                    payloadPtr = buf + 5;
                    payloadLen = idx - 6;
                    calcCrc = _crc8(buf + 4, payloadLen + 1);      // type+payload
                } else if (isExtended) {
                    payloadPtr = buf + 5;
                    payloadLen = idx - 6;
                    calcCrc = _crc8(buf + 2, idx - 3);             // type+dest+origin+payload
                } else {
                    payloadPtr = buf + 3;
                    payloadLen = idx - 4;
                    calcCrc = _crc8(buf + 2, idx - 3);             // type+payload
                }
#if DEBUG_TX1_VERBOSE
                String hexDump = "HEX: ";
                for (uint8_t i = 0; i < idx; i++) {
                    if (buf[i] < 0x10) hexDump += "0";
                    hexDump += String(buf[i], HEX);
                    hexDump += " ";
                }
                Serial.printf("TX1: RX len=%u type=0x%02X from=0x%02X to=0x%02X crc_calc=0x%02X exp=0x%02X %s\n",
                    idx, frameType, buf[2], buf[3], calcCrc, expCrc, calcCrc==expCrc?"OK":"BAD");
                Serial.println(hexDump);
#endif
                
                bool crcOk = (calcCrc == expCrc);
                if (!crcOk) txFramesCrcBad++;

#if DEBUG_TX1_RAW_CAPTURE
                bool isInterestingType =
                    frameType == CRSF_FRAMETYPE_DEVICE_PING ||
                    frameType == CRSF_FRAMETYPE_DEVICE_INFO ||
                    frameType == CRSF_FRAMETYPE_PARAMETER_DATA ||
                    frameType == CRSF_FRAMETYPE_PARAMETER_REQUEST ||
                    frameType == CRSF_FRAMETYPE_PARAMETER_WRITE ||
                    frameType == CRSF_FRAMETYPE_ELRS_STATUS;
                if ((rawDumpBudget > 0 && isInterestingType) || !crcOk) {
                    const char* layout = isExtended ? "ext" : (isLegacyAddressed ? "legacy" : "std");
                    Serial.printf("TX1 RAW IN meta: layout=%s type=0x%02X crc=%s calc=0x%02X exp=0x%02X\n",
                        layout, frameType, crcOk ? "OK" : "BAD", calcCrc, expCrc);
                    printHexFrame("TX1 RAW IN", buf, idx);
                    if (rawDumpBudget > 0) rawDumpBudget--;
                }
#endif
                
                // TX module may have wrong/missing CRC in ping response, so we process it anyway
                bool processPkt = true;  // Always process TX packets for now
                if (!crcOk && frameType != CRSF_FRAMETYPE_DEVICE_PING) {
                    Serial.printf("TX1: Warning - CRC mismatch (calc=0x%02X exp=0x%02X) for type 0x%02X\n", calcCrc, expCrc, frameType);
                }

                // Ignore local echo of our own extended requests:
                // [type][dest=MODULE][origin=HANDSET]...
                bool isOwnEcho = false;
                if (isExtended && payloadLen >= 0) {
                    uint8_t dest = buf[3];
                    uint8_t origin = buf[4];
                    bool isDeviceCmd =
                        frameType == CRSF_FRAMETYPE_DEVICE_PING ||
                        frameType == CRSF_FRAMETYPE_DEVICE_INFO ||
                        frameType == CRSF_FRAMETYPE_PARAMETER_REQUEST ||
                        frameType == CRSF_FRAMETYPE_PARAMETER_WRITE;
                    if (isDeviceCmd &&
                        dest == CRSF_ADDRESS_MODULE &&
                        (origin == CRSF_ADDRESS_HANDSET || origin == CRSF_ADDRESS_HANDSET_ALT)) {
                        isOwnEcho = true;
                    }
                }

                if (processPkt && !isOwnEcho) {
                    if (frameType == CRSF_FRAMETYPE_DEVICE_INFO || frameType == CRSF_FRAMETYPE_PARAMETER_DATA) {
                        if (isExtended) {
                            Serial.printf("TX1: RX type=0x%02X ext dest=0x%02X origin=0x%02X payloadLen=%u\n",
                                frameType, buf[3], buf[4], payloadLen);
                        } else if (isLegacyAddressed) {
                            Serial.printf("TX1: RX type=0x%02X legacy to=0x%02X from=0x%02X payloadLen=%u\n",
                                frameType, buf[2], buf[3], payloadLen);
                        } else {
                            Serial.printf("TX1: RX type=0x%02X std payloadLen=%u\n", frameType, payloadLen);
                        }
                    }
                    _lastResponseTime = millis();
                    _connected = true;

                    // Прозрачная двусторонняя ретрансляция: телеметрия TX -> RX
                    if ((crcOk || TX_SKIP_CRC_CHECK) && isDroneTelemetryFrameType(frameType)) {
                        forwardFrameToReceiverByType(frameType, payloadPtr, payloadLen);
                        txFramesForwarded++;
                        txTypeForwarded[frameType]++;
                        if (isCopterTelemetryEvidenceType(frameType) ||
                            hasCopterAirLinkByLinkStats(frameType, payloadPtr, payloadLen)) {
                            lastCopterTelemetryMs = millis();
                        }
                    }

                    switch(frameType) {
                        case CRSF_FRAMETYPE_DEVICE_INFO: 
                            Serial.println("TX1: Got DEVICE_INFO");
                            _handleDeviceInfo(payloadPtr, payloadLen);
                            if (!_fieldsRequested) {
                                Serial.println("TX1: Requesting all fields (after DEVICE_INFO)...");
                                _requestAllFields();
                            }
                            break;
                        case CRSF_FRAMETYPE_PARAMETER_DATA: 
                            _handleParameterData(payloadPtr, payloadLen);
                            Serial.printf("TX1: Got PARAM_DATA len=%u, fields=%u\n", payloadLen, (unsigned)_fields.size());
                            break;
                        case CRSF_FRAMETYPE_ELRS_STATUS: 
                            _handleElrsStatus(payloadPtr, payloadLen); 
                            break;
                        case CRSF_FRAMETYPE_DEVICE_PING:
                            Serial.println("TX1: Got DEVICE_PING response - connected!");
                            break;
                        default:
                            break;
                    }
                }
                inPacket = false;
            } else if (idx >= sizeof(buf)) {
                Serial.printf("TX1: packet too long (%u), resync\n", idx);
                inPacket = false;
            }
        }
    }
}

void TransmitterController::_handleDeviceInfo(uint8_t* data, uint8_t len) {
    if (!_deviceInfoReceived) {
        _fields.clear();
        _fieldsRequested = false;
    }
    _deviceInfoReceived = true;

    // Some modules return short device-info payloads; keep flow alive anyway.
    if (len >= 4) {
        _deviceName = String((char*)(data + 3));
    } else if (_deviceName.length() == 0) {
        _deviceName = "ELRS TX";
    }
    Serial.printf("Device info received: %s (len=%u)\n", _deviceName.c_str(), len);
}

void TransmitterController::_handleParameterData(uint8_t* data, uint8_t len) {
    if (len < 6) return;
    uint8_t fieldId = data[2];
    uint8_t fieldType = data[4];

    // CRSF Parameter Data layout:
    // [dst][src][fieldId][parentId][fieldType][name\0][type-specific...]
    size_t nameStart = 5;
    size_t nameEnd = nameStart;
    while (nameEnd < len && data[nameEnd] != 0x00) nameEnd++;
    if (nameEnd >= len) return;  // malformed (name without '\0')

    CRSFField f;
    f.fieldId = fieldId;
    f.fieldType = fieldType;
    f.isFolder = false;
    f.isCommand = false;
    f.name = String((char*)(data + nameStart));

    size_t p = nameEnd + 1;
    switch(fieldType) {
        case 0x00:  // UINT8
        case 0x01:  // INT8
            if (p < len) f.value = String(data[p]);
            break;
        case 0x02:  // UINT16
        case 0x03:  // INT16
            if (p + 1 < len) f.value = String(data[p] | (data[p + 1] << 8));
            break;
        case 0x07:  // SELECT
            if (p < len) {
                f.value = String(data[p]);
                if (p + 1 < len) f.options = String((char*)(data + p + 1));
            }
            break;
        case 0x08:  // STRING
            if (p < len) f.value = String((char*)(data + p));
            break;
        case 0x09:  // FOLDER
            f.isFolder = true;
            break;
        case 0x0A:  // COMMAND
            f.isCommand = true;
            break;
        default:
            return;
    }
    _fields[fieldId] = f;
}

void TransmitterController::_handleElrsStatus(uint8_t* data, uint8_t len) {}

void TransmitterController::update() {
    _processIncoming();
    // Timeout for losing connection (increased to 5 seconds)
    if (millis() - _lastResponseTime > 5000) {
        if (_connected) {
            _connected = false;
            _deviceInfoReceived = false;
            _fieldsRequested = false;
            Serial.println("TX1: Connection timeout!");
        }
    }
    static unsigned long lastPing = 0;
    static unsigned long lastInfoRequest = 0;
    if (millis() - lastPing > 3000) {
        lastPing = millis();
        if (!_connected) {
            Serial.printf("TX1: Not connected, pinging... (_serial=%p, available=%d)\n", _serial, _serial->available());
            _sendDevicePing();
            lastInfoRequest = 0;  // Reset info request timer
        }
        else if (_connected && !_deviceInfoReceived && (millis() - lastInfoRequest > 500)) {
            // After ping response, request device info (but only once every 500ms)
            Serial.println("TX1: Requesting device info...");
            lastInfoRequest = millis();
            uint8_t p[] = {};
            _sendCrsfPacket(p, 0, CRSF_FRAMETYPE_DEVICE_INFO);
        }
        else if (_deviceInfoReceived && !_fieldsRequested && (millis() - lastInfoRequest > 1000)) {
            Serial.println("TX1: Requesting all fields...");
            lastInfoRequest = millis();
            _requestAllFields();
        }
    }
}

int TransmitterController::getTxPower() const {
    for (auto& p : _fields)
        if (p.second.name.indexOf("Power") >= 0) return p.second.value.toInt();
    return 0;
}

int TransmitterController::_findFieldIdByKeys(const std::vector<String>& keys) const {
    for (const auto& p : _fields) {
        String n = p.second.name;
        n.toLowerCase();
        for (const auto& key : keys) {
            String k = key;
            k.toLowerCase();
            if (n.indexOf(k) >= 0) return p.first;
        }
    }
    return -1;
}

String TransmitterController::_getFieldValueByKeys(const std::vector<String>& keys) const {
    int id = _findFieldIdByKeys(keys);
    if (id < 0) return "---";
    auto it = _fields.find((uint8_t)id);
    if (it == _fields.end()) return "---";
    return it->second.value.length() ? it->second.value : "---";
}

bool TransmitterController::_setFieldByKeys(const std::vector<String>& keys, const String& value, String& shadow) {
    int id = _findFieldIdByKeys(keys);
    if (id < 0) return false;
    auto it = _fields.find((uint8_t)id);
    if (it == _fields.end()) return false;

    String v = value;
    // For SELECT fields allow passing readable option labels from docs/UI.
    if (it->second.fieldType == 0x07) {
        bool isNumeric = v.length() > 0;
        for (size_t i = 0; i < v.length(); i++) {
            if (v[i] < '0' || v[i] > '9') { isNumeric = false; break; }
        }
        if (!isNumeric) {
            String opts = it->second.options;
            int start = 0;
            int idx = 0;
            String needle = v;
            needle.toLowerCase();
            while (start <= opts.length()) {
                int sep = opts.indexOf(';', start);
                if (sep < 0) sep = opts.length();
                String opt = opts.substring(start, sep);
                String cmp = opt;
                cmp.toLowerCase();
                if (cmp == needle) {
                    v = String(idx);
                    break;
                }
                idx++;
                start = sep + 1;
            }
        }
    }

    if (!setValue(id, v)) return false;
    shadow = value;
    return true;
}

String TransmitterController::getTxPowerRead() const { return _getFieldValueByKeys({"power"}); }
String TransmitterController::getRfModeRead() const { return _getFieldValueByKeys({"rfmd", "rf mode"}); }
String TransmitterController::getSwitchModeRead() const { return _getFieldValueByKeys({"switch mode"}); }
String TransmitterController::getAntennaModeRead() const { return _getFieldValueByKeys({"antenna mode", "ant mode"}); }
String TransmitterController::getTelemRatioRead() const { return _getFieldValueByKeys({"telem ratio", "telemetry ratio"}); }

bool TransmitterController::setTxPowerByName(const String& value) {
    return _setFieldByKeys({"power"}, value, _txPowerSet);
}
bool TransmitterController::setRfModeByName(const String& value) {
    return _setFieldByKeys({"rfmd", "rf mode"}, value, _rfModeSet);
}
bool TransmitterController::setSwitchModeByName(const String& value) {
    return _setFieldByKeys({"switch mode"}, value, _switchModeSet);
}
bool TransmitterController::setAntennaModeByName(const String& value) {
    return _setFieldByKeys({"antenna mode", "ant mode"}, value, _antennaModeSet);
}
bool TransmitterController::setTelemRatioByName(const String& value) {
    return _setFieldByKeys({"telem ratio", "telemetry ratio"}, value, _telemRatioSet);
}

int TransmitterController::getPktRate() const {
    for (auto& p : _fields)
        if (p.second.name.indexOf("Rate") >= 0) return p.second.value.toInt();
    return 0;
}

void TransmitterController::_buildMenuItems(JsonArray items, int parentId) {
    if (parentId == 0) {
        for (auto& p : _fields) {
            if (p.second.isFolder) {
                JsonObject o = items.add<JsonObject>();
                o["id"] = p.first; o["name"] = p.second.name; o["type"] = "folder";
            }
        }
    }
    for (auto& p : _fields) {
        if (!p.second.isFolder && parentId == 0) {
            JsonObject o = items.add<JsonObject>();
            o["id"] = p.first; o["name"] = p.second.name; o["value"] = p.second.value;
            o["type"] = p.second.isCommand ? "command" : "parameter";
            if (p.second.fieldType == 0x07) o["options"] = p.second.options;
        }
    }
}

String TransmitterController::getMenuJson(int parentId) {
    DynamicJsonDocument doc(2048);
    JsonArray items = doc.createNestedArray("items");
    doc["parent"] = parentId;
    
    // If TX fields are available, use them
    if (!_fields.empty()) {
        _buildMenuItems(items, parentId);
    } else {
        // Fallback: Return built-in ELRS TX menu structure
        // Power settings
        JsonObject pwrItem = items.add<JsonObject>();
        pwrItem["id"] = 1;
        pwrItem["name"] = "Power";
        pwrItem["value"] = "25";
        pwrItem["type"] = "parameter";
        pwrItem["min"] = 0;
        pwrItem["max"] = 250;
        pwrItem["unit"] = "mW";
        
        // Packet rate
        JsonObject rateItem = items.add<JsonObject>();
        rateItem["id"] = 2;
        rateItem["name"] = "Packet Rate";
        rateItem["value"] = "200";
        rateItem["type"] = "parameter";
        rateItem["options"] = "4;25;50;100;150;200;250;500";
        
        // Telemetry
        JsonObject telItem = items.add<JsonObject>();
        telItem["id"] = 3;
        telItem["name"] = "Telemetry";
        telItem["value"] = "1";
        telItem["type"] = "parameter";
        telItem["options"] = "Off;1/2;On";
        
        // Status string
        JsonObject statusItem = items.add<JsonObject>();
        statusItem["id"] = 4;
        statusItem["name"] = "TX Status";
        statusItem["value"] = _connected ? (_deviceInfoReceived ? "Connected" : "Connecting...") : "Disconnected";
        statusItem["type"] = "info";
    }
    
    String out; 
    serializeJson(doc, out); 
    return out;
}

bool TransmitterController::setValue(int fieldId, const String& value) {
    auto it = _fields.find(fieldId);
    if (it == _fields.end()) return false;
    if (it->second.fieldType == 0x07) _writeParameter(fieldId, (uint32_t)value.toInt());
    else if (it->second.fieldType == 0x02) _writeParameter(fieldId, (uint32_t)value.toInt());
    else if (it->second.fieldType == 0x03) _writeParameter(fieldId, (uint32_t)value.toInt());
    else if (it->second.fieldType == 0x08) _writeParameter(fieldId, (const uint8_t*)value.c_str(), value.length());
    else return false;
    return true;
}

bool TransmitterController::executeCommand(int fieldId) {
    auto it = _fields.find(fieldId);
    if (it == _fields.end() || !it->second.isCommand) return false;
    _writeParameter(fieldId, (uint32_t)1);
    return true;
}

void TransmitterController::refresh() {
    _deviceInfoReceived = false;
    _fieldsRequested = false;
    _fields.clear();
    _sendDevicePing();
}


