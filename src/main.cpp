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
#define TX_PIN        32      // Единая линия данных передатчика (S.Port)

// Параметры
#define DEFAULT_BAUD       400000
#define TIMEOUT_MS         500
#define WDT_TIMEOUT_MS     5000
#define POWER_MODE_AUTO       0
#define POWER_MODE_ALWAYS_ON  1
#define POWER_MODE_ALWAYS_OFF 2

// CRSF
#define CRSF_ADDRESS_MODULE      0xEC
#define CRSF_ADDRESS_HANDSET     0xEE
#define CRSF_ADDRESS_RECEIVER    0xEF
#define CRSF_FRAMETYPE_DEVICE_PING       0x28
#define CRSF_FRAMETYPE_DEVICE_INFO       0x29
#define CRSF_FRAMETYPE_PARAMETER_DATA    0x2B
#define CRSF_FRAMETYPE_PARAMETER_REQUEST 0x2C
#define CRSF_FRAMETYPE_PARAMETER_WRITE   0x2D
#define CRSF_FRAMETYPE_ELRS_STATUS       0x2E

// Класс управления передатчиком (полудуплекс, открытый сток)
class TransmitterController {
public:
    TransmitterController();
    void begin(unsigned long baud);
    void update();
    void sendRawPacket(uint8_t* payload, uint8_t len, uint8_t destAddr);
    bool isConnected() const { return _connected; }
    String getDeviceName() const { return _deviceName; }
    int getTxPower() const;
    int getPktRate() const;
    String getMenuJson(int parentId = 0);
    bool setValue(int fieldId, const String& value);
    bool executeCommand(int fieldId);
    void refresh();

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

    uint8_t _crc8(const uint8_t* d, size_t len);
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

// Обработка данных от приёмника и ретрансляция
void processReceiverData() {
    while(receiverSerial.available()) {
        uint8_t b = receiverSerial.read();
        if(!rxInPacket && b==0xC8) { rxInPacket=true; rxPacketIndex=0; rxExpectedLen=0; }
        if(rxInPacket) {
            if(rxPacketIndex < sizeof(rxPacketBuffer)) rxPacketBuffer[rxPacketIndex++] = b;
            if(rxPacketIndex==2) rxExpectedLen = rxPacketBuffer[1]+2;
            if(rxPacketIndex==rxExpectedLen && rxExpectedLen>0) {
                uint8_t calcCrc = crsfCrc8(rxPacketBuffer+2, rxPacketIndex-3);
                uint8_t expCrc = rxPacketBuffer[rxPacketIndex-1];
                if(calcCrc == expCrc) {
                    lastPacketTime = millis();
                    packetCount++;
                    txController.sendRawPacket(rxPacketBuffer+4, rxPacketIndex-5, rxPacketBuffer[3]);
                }
                rxInPacket=false;
            } else if(rxPacketIndex>64) rxInPacket=false;
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
    currentBaudRate = prefs.getInt("baud", DEFAULT_BAUD);
    currentPowerMode = prefs.getInt("pmode", POWER_MODE_AUTO);
    wifiSSID = prefs.getString("wifi_ssid", "");
    wifiPass = prefs.getString("wifi_pass", "");
    prefs.end();
}
void saveBaudRate(int b) { prefs.begin("elrs", false); prefs.putInt("baud", b); prefs.end(); delay(50); currentBaudRate=b; }
void savePowerMode(int m) { prefs.begin("elrs", false); prefs.putInt("pmode", m); prefs.end(); delay(50); currentPowerMode=m; }
void saveWiFiSettings(const String& ssid, const String& pass) {
    prefs.begin("elrs", false);
    prefs.putString("wifi_ssid", ssid);
    prefs.putString("wifi_pass", pass);
    prefs.end();
    wifiSSID = ssid;
    wifiPass = pass;
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
    document.getElementById('txpower').innerText=d.txpower+' mW';
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
    document.getElementById('newBaud').value = d.baudrate;
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
async function setBaud(){
    let b=document.getElementById('newBaud').value;
    await fetch('/api/setBaud?baud='+b); alert('Restarting...'); setTimeout(()=>location.reload(),2000);
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
    let r=await fetch('/api/tx/menu?parent='+parent), m=await r.json();
    curParent=parent;
    let html=parent!==0?'<button class="secondary" onclick="loadTxMenu(0)">← Назад</button>':'';
    html+='<ul class="menu-tree">';
    m.items.forEach(i=>{
        if(i.type==='folder') html+=`<li class="menu-item menu-folder" onclick="loadTxMenu(${i.id})">📁 ${i.name}</li>`;
        else if(i.type==='command') html+=`<li class="menu-item" onclick="execCmd(${i.id})">⚡ ${i.name}</li>`;
        else html+=`<li class="menu-item" onclick="editField(${i.id},'${i.name}','${i.value}')">${i.name} <span class="menu-value">${i.value}</span></li>`;
    });
    html+='</ul><hr><button class="secondary" onclick="loadTxMenu(curParent)">Refresh</button>';
    document.getElementById('tx-menu').innerHTML=html;
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
    <div class="card"><h3>System</h3>
        <div class="flex"><input id="newBaud" value="460800"> <button onclick="setBaud()">Set Baud & Restart</button></div>
    </div>
</div>
<div id="tab-transmitter" class="tab-content">
    <div class="card"><h3>TX Settings</h3><div id="tx-menu">Loading...</div></div>
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
    JsonDocument d;
    d["linkRadio"] = linkToRadio;
    d["linkCopter"] = txController.isConnected();
    d["packets"] = packetCount;
    d["txpower"] = txController.getTxPower();
    d["pktrate"] = txController.getPktRate();
    d["deviceName"] = txController.getDeviceName();
    d["baudrate"] = currentBaudRate;
    d["sd"] = sdStatusMsg;
    String s; serializeJson(d, s); server.send(200, "application/json", s);
}
void handleApiSetBaud() {
    if(server.hasArg("baud")){ int b=server.arg("baud").toInt(); if(b>=9600&&b<=921600){ saveBaudRate(b); server.send(200); delay(200); ESP.restart(); }}
    server.send(400);
}
void handleApiPowerMode() {
    JsonDocument d; d["mode"]=currentPowerMode; d["relayState"]=digitalRead(RELAY_PIN);
    String s; serializeJson(d,s); server.send(200,"application/json",s);
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
void handleApiTxCommand() {
    if(server.hasArg("id")) server.send(txController.executeCommand(server.arg("id").toInt())?200:400);
    else server.send(400);
}
void handleTxInterface() {
    if(!streamFileFromPriority("/www/tx_interface.html","text/html")) {
        String html = R"rawliteral(<html><body style="background:#111;color:#eee;font-family:Segoe UI"><div id="tx">Default. Put tx_interface.html on SD:/www/</div><script>
async function load(p=0){let r=await fetch('/api/tx/menu?parent='+p),m=await r.json();let h='';if(p)h+='<button onclick="load(0)">Back</button>';h+='<ul>';m.items.forEach(i=>{if(i.type=='folder')h+='<li onclick="load('+i.id+')">📁'+i.name;else if(i.type=='command')h+='<li onclick="fetch(\'/api/tx/command?id='+i.id+'\')">⚡'+i.name;else h+='<li onclick="edit('+i.id+',\''+i.name+'\',\''+i.value+'\')">'+i.name+' <span>'+i.value+'</span>';});h+='</ul>';document.getElementById('tx').innerHTML=h;}
function edit(i,n,v){let nv=prompt(n,v);if(nv)fetch('/api/tx/set?id='+i+'&value='+nv).then(()=>load(curParent));}
let curParent=0;load(0);</script></body></html>)rawliteral";
        server.send(200,"text/html",html);
    }
}
void handleApiWiFiGet() {
    JsonDocument d;
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
    server.on("/api/setBaud", handleApiSetBaud);
    server.on("/api/powerMode", handleApiPowerMode);
    server.on("/api/setPowerMode", handleApiSetPowerMode);
    server.on("/api/bindRx", handleApiBindRx);
    server.on("/api/resetRxBind", handleApiResetRxBind);
    server.on("/api/enableRxWifi", handleApiEnableRxWifi);
    server.on("/api/tx/menu", handleApiTxMenu);
    server.on("/api/tx/set", handleApiTxSet);
    server.on("/api/tx/command", handleApiTxCommand);
    server.on("/tx_interface.html", handleTxInterface);
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
    if (wifiSSID.length() > 0) {
        WiFi.begin(wifiSSID.c_str(), wifiPass.c_str());
        Serial.print("Connecting to WiFi");
        int attempts = 0;
        while (WiFi.status() != WL_CONNECTED && attempts < 20) { delay(500); Serial.print("."); attempts++; }
        if (WiFi.status() == WL_CONNECTED) Serial.println("\nConnected! IP: " + WiFi.localIP().toString());
        else Serial.println("\nFailed to connect.");
    }
    WiFi.softAPConfig(apIP, apIP, IPAddress(255,255,255,0));
    WiFi.softAP("ELRS_Repeater_Pro", "12345678", 1, 0);
    dnsServer.start(DNS_PORT, "*", apIP);
    Serial.print("AP IP: "); Serial.println(WiFi.softAPIP());
    
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
    esp_task_wdt_reset();
    yield();
}

// ========== РЕАЛИЗАЦИЯ TransmitterController (полудуплекс, открытый сток) ==========
TransmitterController::TransmitterController() :
    _serial(nullptr), _connected(false), _lastResponseTime(0),
    _deviceInfoReceived(false), _fieldsRequested(false) {}

void TransmitterController::begin(unsigned long baud) {
    // Настраиваем пин как открытый сток
    pinMode(TX_PIN, OUTPUT_OPEN_DRAIN);
    digitalWrite(TX_PIN, HIGH);  // Высокоимпедансное состояние для приёма

    // Инициализация UART с одним пином для RX и TX
    _serial = &Serial1;
    _serial->begin(baud, SERIAL_8N1, TX_PIN, TX_PIN);

    // Включаем инверсию данных (требуется для CRSF)
    uart_set_line_inverse(UART_NUM_1, UART_SIGNAL_RXD_INV | UART_SIGNAL_TXD_INV);

    // Включаем полудуплексный режим RS485 (автоматическое управление направлением)
    uart_set_mode(UART_NUM_1, UART_MODE_RS485_HALF_DUPLEX);

    Serial.printf("TX UART (half-duplex open-drain) started on pin %d\n", TX_PIN);
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

void TransmitterController::_sendCrsfPacket(uint8_t* payload, uint8_t len, uint8_t packetType) {
    uint8_t packet[64];
    uint8_t idx = 0;
    packet[idx++] = 0xC8;
    packet[idx++] = len + 2;
    packet[idx++] = CRSF_ADDRESS_MODULE;
    packet[idx++] = CRSF_ADDRESS_HANDSET;
    packet[idx++] = packetType;
    memcpy(packet + idx, payload, len);
    idx += len;
    packet[idx] = _crc8(packet + 2, idx - 2);
    idx++;

    // Отправляем без ручного управления пином — всё делает RS485
    _serial->write(packet, idx);
    _serial->flush();
}

void TransmitterController::sendRawPacket(uint8_t* payload, uint8_t len, uint8_t destAddr) {
    uint8_t packet[64];
    uint8_t idx = 0;
    packet[idx++] = 0xC8;
    packet[idx++] = len + 2;
    packet[idx++] = CRSF_ADDRESS_MODULE;
    packet[idx++] = CRSF_ADDRESS_HANDSET;
    packet[idx++] = destAddr;
    memcpy(packet + idx, payload, len);
    idx += len;
    packet[idx] = _crc8(packet + 2, idx - 2);
    idx++;

    _serial->write(packet, idx);
    _serial->flush();
}

void TransmitterController::_sendDevicePing() {
    uint8_t p[] = {0x00, 0x00};
    _sendCrsfPacket(p, 2, CRSF_FRAMETYPE_DEVICE_PING);
}

void TransmitterController::_requestField(uint8_t fieldId) {
    uint8_t p[] = {0x00, 0x00, fieldId, 0x00};
    _sendCrsfPacket(p, 4, CRSF_FRAMETYPE_PARAMETER_REQUEST);
}

void TransmitterController::_requestAllFields() {
    for (auto& pair : _fields) _requestField(pair.first);
    _fieldsRequested = true;
}

void TransmitterController::_writeParameter(uint8_t fieldId, uint32_t value) {
    uint8_t p[7] = {0x00, 0x00, fieldId};
    p[3] = value & 0xFF; p[4] = (value>>8)&0xFF; p[5] = (value>>16)&0xFF; p[6] = (value>>24)&0xFF;
    _sendCrsfPacket(p, 7, CRSF_FRAMETYPE_PARAMETER_WRITE);
}

void TransmitterController::_writeParameter(uint8_t fieldId, const uint8_t* data, uint8_t len) {
    uint8_t p[64] = {0x00, 0x00, fieldId};
    memcpy(p+3, data, len);
    _sendCrsfPacket(p, 3+len, CRSF_FRAMETYPE_PARAMETER_WRITE);
}

void TransmitterController::_processIncoming() {
    static uint8_t buf[128], idx=0;
    static bool inPacket=false;
    static uint8_t expLen=0;
    while (_serial->available()) {
        uint8_t b = _serial->read();
        if (!inPacket && b == 0xC8) { inPacket=true; idx=0; expLen=0; }
        if (inPacket) {
            if (idx < sizeof(buf)) buf[idx++] = b;
            if (idx == 2) expLen = buf[1] + 2;
            if (idx == expLen && expLen>0) {
                bool crcOk = (_crc8(buf+2, idx-3) == buf[idx-1]);
                Serial.printf("TX1: packet len=%u exp=%u type=0x%02X crc=%s\n", idx, expLen, buf[3], crcOk?"OK":"BAD");
                if (crcOk) {
                    _lastResponseTime = millis();
                    _connected = true;
                    switch(buf[3]) {
                        case CRSF_FRAMETYPE_DEVICE_INFO: _handleDeviceInfo(buf+4, idx-5); break;
                        case CRSF_FRAMETYPE_PARAMETER_DATA: _handleParameterData(buf+4, idx-5); break;
                        case CRSF_FRAMETYPE_ELRS_STATUS: _handleElrsStatus(buf+4, idx-5); break;
                    }
                }
                inPacket = false;
            } else if (idx > 64) inPacket = false;
        }
    }
}

void TransmitterController::_handleDeviceInfo(uint8_t* data, uint8_t len) {
    if (len >= 3) {
        _deviceName = String((char*)(data+3));
        _deviceInfoReceived = true;
        _fields.clear();
        _fieldsRequested = false;
        Serial.println("Device info received: " + _deviceName);
    }
}

void TransmitterController::_handleParameterData(uint8_t* data, uint8_t len) {
    if (len < 6) return;
    uint8_t fieldId = data[2];
    uint8_t fieldType = data[4];
    CRSFField f;
    f.fieldId = fieldId;
    f.fieldType = fieldType;
    f.isFolder = false;
    f.isCommand = false;
    switch(fieldType) {
        case 0x00: case 0x01:
            f.name = String((char*)(data+5));
            { int semi = f.name.indexOf(';');
              if(semi>0){ f.value = f.name.substring(semi+1); f.name = f.name.substring(0,semi); }
              else f.value = f.name; }
            break;
        case 0x02: if(len>=6) f.value = String(data[5]); break;
        case 0x03: if(len>=7) f.value = String(data[5] | (data[6]<<8)); break;
        case 0x07: if(len>=6) { f.value = String(data[5]); if(len>6) f.options = String((char*)(data+6)); } break;
        case 0x08: f.value = String((char*)(data+5)); break;
        case 0x09: f.isFolder = true; f.name = String((char*)(data+5)); break;
        case 0x0A: f.isCommand = true; f.name = String((char*)(data+5)); break;
        default: return;
    }
    _fields[fieldId] = f;
}

void TransmitterController::_handleElrsStatus(uint8_t* data, uint8_t len) {}

void TransmitterController::update() {
    _processIncoming();
    if (millis() - _lastResponseTime > 2000) _connected = false;
    static unsigned long lastPing = 0;
    if (millis() - lastPing > 5000) {
        lastPing = millis();
        if (!_connected) _sendDevicePing();
        else if (_deviceInfoReceived && !_fieldsRequested) _requestAllFields();
    }
}

int TransmitterController::getTxPower() const {
    for (auto& p : _fields)
        if (p.second.name.indexOf("Power") >= 0) return p.second.value.toInt();
    return 0;
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
    JsonDocument doc;
    JsonArray items = doc["items"].to<JsonArray>();
    doc["parent"] = parentId;
    _buildMenuItems(items, parentId);
    String out; serializeJson(doc, out); return out;
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