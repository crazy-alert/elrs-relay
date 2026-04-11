/*
  ==============================================================================
   ELRS РЕТРАНСЛЯТОР PRO на ESP32-WROOM-32 (ФИНАЛЬНАЯ ВЕРСИЯ)
   ==============================================================================
   
   ФУНКЦИОНАЛ:
   - Полная ретрансляция CRSF между приёмником (full-duplex) и передатчиком (S.Port half-duplex)
   - Двусторонняя телеметрия
   - ВЕБ-ИНТЕРФЕЙС С ВКЛАДКАМИ:
        * Dashboard - общая информация о состоянии
        * Transmitter - управление мощностью, packet rate, бинд, активация WiFi
        * Receiver - управление биндом, активация WiFi, CLI-команды
        * Power - управление питанием передатчика (Auto/Always ON/Always OFF)
   - Поддержка CRSF Config Protocol (0x2C/0x2B) для получения параметров модуля
   - Сохранение настроек (битрейт, режим питания) во flash
   - Активация WiFi на передатчике и приёмнике через CRSF-команды
   - Режим AP+STA для ESP32 (одновременная работа точки доступа и клиента)

   АВТОР: Опытный разработчик
   ВЕРСИЯ: 8.0 (полная интеграция всех функций)

   ==============================================================================
   СХЕМА ПОДКЛЮЧЕНИЯ (псевдографика)
   ==============================================================================

   ┌────────────────────────────────────────────────────────────────────────────┐
   │                          ESP32-WROOM-32 DevKit v1                          │
   │  ┌─────────────────────────────────────────────────────────────────────┐   │
   │  │  USB Type-C (питание и отладка)                                     │   │
   │  └─────────────────────────────────────────────────────────────────────┘   │
   │                                                                            │
   │  ┌─────────┐         ┌─────────┐          ┌─────────┐        ┌──────────┐  │
   │  │ GPIO16  │◄────────┤ TX      │          │ GPIO5   ├────────┤ IN (реле)│  │
   │  │ (RX2)   │         │  ELRS   │          │         │        └──────────┘  │
   │  └─────────┘         │ ПРИЁМНИК│          └─────────┘              │       │
   │  ┌─────────┐         │ (full-  │          ┌─────────┐              │       │
   │  │ GPIO17  ├─────────┤  duplex)│          │ GPIO2   ├──────────────┼───┐   │
   │  │ (TX2)   │         │         │          │ (LED)   │              │   │   │
   │  └─────────┘         └─────────┘          └─────────┘              │   │   │
   │       │                   │                   │                    │   │   │
   │       │                   │             ┌─────┴─────┐              │   │   │
   │       │                   │             │   Реле    │              │   │   │
   │       │                   │             │  катушка  │              │   │   │
   │       │                   │             └───────────┘              │   │   │
   │       │                   │                                        │   │   │
   │  ┌─────────┐              │              ┌─────────────────────┐   │   │   │
   │  │ GPIO18  ├────────┬─────┼──────────────┤  ELRS ПЕРЕДАТЧИК    │   │   │   │
   │  │ (TX1)   │        │     │              │  (S.Port, half‑dup) │   │   │   │
   │  └─────────┘        │     │              └─────────────────────┘   │   │   │
   │       │             │     │                            │           │   │   │
   │  ┌─────────┐        │     │                            │           │   │   │
   │  │ GPIO19  ├────────┼─────┼────────────────────────────┤ S.Port    │   │   │
   │  │ (RX1)   │        │     │                            │           │   │   │
   │  └─────────┘        │     │                            │           │   │   │
   │       │             │     │                            │           │   │   │
   │  ┌─────────┐        │     │                      ┌─────┴─────┐     │   │   │
   │  │  GND    ├────────┼─────┼──────────────────────┤ GND       │     │   │   │
   │  └─────────┘        │     │                      └───────────┘     │   │   │
   │       │             │     │                                        │   │   │
   │  ┌─────────┐        │     │                      ┌───────────┐     │   │   │
   │  │  3.3V   ├────────┼─────┼──────────────────────┤ VCC       │     │   │   │
   │  └─────────┘        │     │                      └───────────┘     │   │   │
   │                     │     │                                        │   │   │
   └─────────────────────┼─────┼────────────────────────────────────────┼───────┘
                         │     │                                        │
                         │     │    ┌─────────┐   ┌─────────┐           │
                         │     └────┤  R1     ├───┤ 4.7kΩ   ├───────┐   │
                         │          │  1kΩ    │   └─────────┘       │   │
                         │          └─────────┘                     │   │
                         │              │                           │   │
                         └──────────────┼───────────────────────────┼───┘
                                        │                           │
                               ┌────────┴────────┐        ┌─────────┴────────┐
                               │     +3.3V       │        │    S.Port line   │
                               └─────────────────┘        └──────────────────┘

   ==============================================================================
   Вот оптимальная схема размещения конденсаторов:

На входе питания ESP32 (5V и GND): Электролитический конденсатор 100–470 мкФ (16В). Он сглаживает общие пульсации и компенсирует броски тока при старте WiFi или реле.

На линии питания ELRS-передатчика (после реле): Электролитический конденсатор 100–470 мкФ (10В). Передатчик может потреблять до 500 мА и более при передаче на полной мощности. Этот конденсатор служит локальным буфером, снижая нагрузку на входное питание.

Рядом с пинами VCC/GND на ESP32: Керамический конденсатор 100 нФ (104). Он фильтрует высокочастотные помехи, которые могут наводиться на провода.

На линии питания ELRS-приёмника (3.3V): Керамический конденсатор 10–47 мкФ (танталовый или низкоимпедансный электролит) + 100 нФ параллельно. Это обеспечит чистое питание для чувствительного приёмного тракта.

💡 Важные технические нюансы
Реле и искрение: При коммутации питания передатчика через реле возникает искрение контактов, создающее широкий спектр помех. Электролит (470 мкФ) после реле гасит эти выбросы, а керамика (100 нФ) — ВЧ-составляющую.

Разводка проводов: Конденсатор для передатчика нужно ставить физически близко к нему. Если соединительные провода длинные, добавьте еще один конденсатор 47–100 мкФ прямо у пина питания на ESP32.

Пусковые токи: ELRS-модули при старте могут потреблять кратковременный ток выше номинала (inrush current). Большой электролит на входе питания ESP32 (470 мкФ) обеспечит эту энергию без просадки основного источника (USB или BEC).

Альтернатива: Если у вас нет электролита 470 мкФ, можно использовать 220 мкФ + 220 мкФ параллельно (емкости суммируются).

Итог по схеме: ESP32 питается через 470 мкФ, после реле на передатчике — еще один 470 мкФ. Параллельно каждому из них — керамика 100 нФ. Это стандартная и проверенная практика для стабилизации цифровых и RF устройств.
*/
/*
  ==============================================================================
   ELRS РЕТРАНСЛЯТОР PRO на ESP32-WROOM-32 (ИСПРАВЛЕННАЯ ВЕРСИЯ)
   ==============================================================================
   
   ИСПРАВЛЕНИЯ:
   - Буферизация целых CRSF-пакетов для ретрансляции.
   - Поддержка полудуплекса S.Port через DIR_PIN (GPIO4).
   - Убран delay(1), watchdog включён.
   - Корректное определение потери телеметрии (linkToCopter).
   - Очистка полей устройства при повторном опросе.
   - Проверка availableForWrite().
   - Обработка ответов от приёмника (обновление lastPacketTime).
   
   ==============================================================================
   СХЕМА ПОДКЛЮЧЕНИЯ (дополненная)
   ==============================================================================
   
   ESP32:
     GPIO16 (RX2) <-- TX приёмника (full-duplex)
     GPIO17 (TX2) --> RX приёмника
     ?GPIO18 (TX0) --> через резистор 1 кОм --> S.Port передатчика (half-duplex)
     ?GPIO19 (RX0) <-- через резистор 1 кОм <-- S.Port передатчика
     GPIO4  (DIR)  --> управление ключом (транзистором) для TX1: HIGH = передача, LOW = приём
     GPIO5  (RELAY) --> реле (питание передатчика)
     GPIO2  (LED)  --> индикация (активный LOW)
     
   Подтяжка S.Port к 3.3V через 4.7 кОм.
   Транзисторный ключ: база через 1 кОм к GPIO4, эмиттер на GND, коллектор на линию TX1 (после резистора 1 кОм).
   В режиме приёма (DIR=LOW) транзистор закрыт, TX1 не влияет на линию.
   В режиме передачи (DIR=HIGH) транзистор открывается и притягивает TX1 к GND? Нет, нужно уточнить: управление должно подключать/отключать выход TX1 от линии. Лучше использовать аналоговый ключ (например, 74LVC1G3157). Для простоты можно оставить только резисторы и не использовать DIR, если передатчик ELRS настроен в full-duplex. Но код поддерживает DIR.
*/

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <ArduinoJson.h>
#include <esp_task_wdt.h>
#include <map>
#include <vector>

// ========== ПИНЫ ==========
#define RELAY_PIN   5
#define LED_PIN     2
#define DIR_PIN     4        // управление направлением для полудуплекса

// ========== ПАРАМЕТРЫ ==========
#define DEFAULT_BAUD     460800
#define TIMEOUT_MS       500       // таймаут связи с радио (receiver)
#define TELEM_TIMEOUT_MS 1000      // таймаут телеметрии от передатчика
#define WDT_TIMEOUT_MS   5000      // сторожевой таймер 5 секунд

// ========== РЕЖИМЫ ПИТАНИЯ ==========
#define POWER_MODE_AUTO      0
#define POWER_MODE_ALWAYS_ON 1
#define POWER_MODE_ALWAYS_OFF 2

int currentPowerMode = POWER_MODE_AUTO;

// ========== CRSF АДРЕСА И КОМАНДЫ ==========
#define CRSF_ADDRESS_RADIO       0xEA
#define CRSF_ADDRESS_MODULE      0xEC
#define CRSF_ADDRESS_HANDSET     0xEE
#define CRSF_ADDRESS_RECEIVER    0xEF

#define CRSF_FRAMETYPE_DEVICE_PING       0x28
#define CRSF_FRAMETYPE_DEVICE_INFO       0x29
#define CRSF_FRAMETYPE_PARAMETER_DATA    0x2B
#define CRSF_FRAMETYPE_PARAMETER_REQUEST 0x2C
#define CRSF_FRAMETYPE_PARAMETER_WRITE   0x2D
#define CRSF_FRAMETYPE_ELRS_STATUS       0x2E
#define CRSF_FRAMETYPE_COMMAND           0x32

#define CRSF_COMMAND_SUBCMD_RX_BIND      0x01
#define CRSF_COMMAND_REALM_RX            0x10

// ========== ОБЪЕКТЫ ==========
Preferences prefs;
WebServer server(80);

HardwareSerial receiverSerial(2);   // UART2: RX=16, TX=17
HardwareSerial transmitterSerial(0); // UART0: TX=18?, RX=19?

// ========== СТРУКТУРЫ ==========
struct CRSFField {
  uint8_t fieldId;
  uint8_t fieldType;
  uint8_t parentId;
  String name;
  String value;
  String options;
  uint32_t minVal;
  uint32_t maxVal;
  float floatVal;
  bool isCommand;
  bool isFolder;
};

std::map<uint8_t, CRSFField> deviceFields;
uint8_t deviceFieldCount = 0;
String deviceName = "";
bool deviceInfoReceived = false;
bool fieldsRequested = false;

// Статусы
unsigned long lastPacketTime = 0;      // последний пакет от receiver (радио)
unsigned long lastTelemPacketTime = 0; // последний пакет от transmitter (телеметрия)
bool linkToRadio = false;
bool linkToCopter = false;
int rssi = 0;
unsigned long packetCount = 0;
int packetsBad = 0;
int packetsGood = 0;
uint8_t elrsFlags = 0;
String elrsMessage = "";

// Настройки
int currentBaudRate = DEFAULT_BAUD;
int currentTxPower = 100;
int currentPktRate = 250;

// Таймеры периодических задач
unsigned long lastParamRefresh = 0;
unsigned long lastElrsStatusRequest = 0;
unsigned long lastLedBlink = 0;
bool ledState = false;

// Буфер для накопления пакета от receiver (ретрансляция)
uint8_t rxPacketBuffer[64];
uint8_t rxPacketIndex = 0;
bool rxInPacket = false;
uint8_t rxExpectedLen = 0;

// ========== CRC8 ==========
uint8_t crsfCrc8(const uint8_t* data, size_t len) {
  uint8_t crc = 0;
  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (uint8_t bit = 0; bit < 8; bit++) {
      if (crc & 0x80) crc = (crc << 1) ^ 0x31;
      else crc = (crc << 1);
    }
  }
  return crc;
}

// ========== ОТПРАВКА CRSF ПАКЕТОВ (с управлением направлением) ==========
// Для отправки в передатчик (полудуплекс)
void sendCrsfPacketToTransmitter(uint8_t* payload, uint8_t len, uint8_t destAddr = CRSF_ADDRESS_MODULE) {
  uint8_t packet[64];
  uint8_t idx = 0;
  packet[idx++] = 0xC8;
  packet[idx++] = len + 2;
  packet[idx++] = destAddr;
  packet[idx++] = CRSF_ADDRESS_HANDSET;
  memcpy(packet + idx, payload, len);
  idx += len;
  packet[idx] = crsfCrc8(packet + 2, idx - 2);
  idx++;
  
  // Переключаем направление на передачу
  digitalWrite(DIR_PIN, HIGH);
  delayMicroseconds(10);  // небольшая задержка для установки ключа
  
  // Отправляем с проверкой свободного места в буфере
  for (int i = 0; i < idx; i++) {
    while (transmitterSerial.availableForWrite() == 0) {
      yield(); // ждём освобождения буфера
    }
    transmitterSerial.write(packet[i]);
  }
  transmitterSerial.flush(); // ждём отправки (необходимо для полудуплекса)
  
  // Переключаем обратно на приём
  digitalWrite(DIR_PIN, LOW);
  delayMicroseconds(10);
}

// Отправка в приёмник (полный дуплекс, без переключения направления)
void sendCrsfPacketToReceiver(uint8_t* payload, uint8_t len) {
  uint8_t packet[64];
  uint8_t idx = 0;
  packet[idx++] = 0xC8;
  packet[idx++] = len + 2;
  packet[idx++] = CRSF_ADDRESS_RECEIVER;
  packet[idx++] = CRSF_ADDRESS_HANDSET;
  memcpy(packet + idx, payload, len);
  idx += len;
  packet[idx] = crsfCrc8(packet + 2, idx - 2);
  idx++;
  
  for (int i = 0; i < idx; i++) {
    while (receiverSerial.availableForWrite() == 0) yield();
    receiverSerial.write(packet[i]);
  }
  receiverSerial.flush();
}

// Обёртки для совместимости
void sendCrsfPacket(uint8_t* payload, uint8_t len, uint8_t destAddr = CRSF_ADDRESS_MODULE) {
  sendCrsfPacketToTransmitter(payload, len, destAddr);
}

void sendDevicePing() {
  uint8_t payload[] = { 0x00, 0x00 };
  sendCrsfPacket(payload, sizeof(payload), CRSF_FRAMETYPE_DEVICE_PING);
  Serial.println("Sent Device Ping");
}

void requestField(uint8_t fieldId) {
  uint8_t payload[] = { 0x00, 0x00, fieldId, 0x00 };
  sendCrsfPacket(payload, sizeof(payload), CRSF_FRAMETYPE_PARAMETER_REQUEST);
}

void requestAllFields() {
  for (auto& pair : deviceFields) requestField(pair.first);
  fieldsRequested = true;
}

void executeCommand(uint8_t fieldId, uint8_t value = 1) {
  uint8_t payload[] = { 0x00, 0x00, fieldId, value };
  sendCrsfPacket(payload, sizeof(payload), CRSF_FRAMETYPE_PARAMETER_WRITE);
}

void setFieldValue(uint8_t fieldId, uint32_t value) {
  uint8_t payload[8] = { 0x00, 0x00, fieldId };
  payload[3] = value & 0xFF;
  payload[4] = (value >> 8) & 0xFF;
  payload[5] = (value >> 16) & 0xFF;
  payload[6] = (value >> 24) & 0xFF;
  sendCrsfPacket(payload, 7, CRSF_FRAMETYPE_PARAMETER_WRITE);
}

int findFieldId(const String& searchName) {
  for (auto& pair : deviceFields)
    if (pair.second.name.indexOf(searchName) >= 0) return pair.first;
  return -1;
}

void requestElrsStatus() {
  uint8_t payload[] = { 0x00, 0x00, 0x00, 0x00 };
  sendCrsfPacket(payload, sizeof(payload), CRSF_FRAMETYPE_PARAMETER_WRITE);
}

// Активация WiFi на передатчике (поиск поля "WiFi" или "Enable WiFi")
void enableTransmitterWiFi() {
  int wifiField = findFieldId("WiFi");
  if (wifiField >= 0) {
    executeCommand(wifiField, 1);
    Serial.println("WiFi enabled on transmitter via field ID");
  } else {
    // fallback: старый метод
    uint8_t payload[] = { 0x00, 0x00, 0x01, 0x01 };
    sendCrsfPacket(payload, sizeof(payload), CRSF_FRAMETYPE_PARAMETER_WRITE);
    Serial.println("WiFi enabled on transmitter (fallback)");
  }
}

// Активация WiFi на приёмнике через реле (три быстрых переключения)
void enableReceiverWiFi() {
  for (int i = 0; i < 3; i++) {
    digitalWrite(RELAY_PIN, LOW);
    delay(200);
    digitalWrite(RELAY_PIN, HIGH);
    delay(200);
  }
  // Также отправим CRSF-команду
  uint8_t payload[] = { 0x00, 0x00, 0x01, 0x01 };
  sendCrsfPacketToReceiver(payload, sizeof(payload));
  Serial.println("WiFi enabled on receiver");
}

void bindTransmitter() {
  int bindField = findFieldId("Bind");
  if (bindField >= 0) executeCommand(bindField, 1);
  else Serial.println("Bind field not found");
}

void bindReceiver() {
  uint8_t payload[2] = { CRSF_COMMAND_REALM_RX, CRSF_COMMAND_SUBCMD_RX_BIND };
  sendCrsfPacketToReceiver(payload, sizeof(payload));
}

void resetReceiverBind() {
  String cmd = "set expresslrs_uid = 0";
  uint8_t payload[64];
  uint8_t idx = 0;
  payload[idx++] = CRSF_COMMAND_REALM_RX;
  payload[idx++] = 0x08;  // CLI command type
  for (int i = 0; i < cmd.length(); i++) payload[idx++] = cmd[i];
  sendCrsfPacketToReceiver(payload, idx);
}

// ========== ОБРАБОТКА ВХОДЯЩИХ CRSF ОТ ПЕРЕДАТЧИКА ==========
void processDeviceInfo(uint8_t* data, uint8_t len) {
  if (len >= 3) {
    deviceFieldCount = data[2];
    deviceName = String((char*)(data + 3));
    deviceInfoReceived = true;
    // Очищаем старые поля перед запросом новых
    deviceFields.clear();
    fieldsRequested = false;
    requestAllFields();
  }
}

void processParameterData(uint8_t* data, uint8_t len) {
  if (len < 6) return;
  uint8_t fieldId = data[2];
  uint8_t fieldType = data[4];
  uint8_t* valueData = data + 5;
  uint8_t valueLen = len - 5;
  CRSFField field;
  field.fieldId = fieldId;
  field.fieldType = fieldType;
  switch (fieldType) {
    case 0x00: case 0x01: field.value = String((char*)valueData); break;
    case 0x02: if (valueLen>=1) field.value = String(valueData[0]); break;
    case 0x03: if (valueLen>=2) field.value = String(valueData[0] | (valueData[1]<<8)); break;
    case 0x07:
      if (valueLen>=1) {
        field.value = String(valueData[0]);
        field.options = String((char*)(valueData+1));
      }
      break;
    case 0x08: field.value = String((char*)valueData); break;
  }
  deviceFields[fieldId] = field;
}

void processElrsStatus(uint8_t* data, uint8_t len) {
  if (len >= 4) {
    packetsBad = data[0];
    packetsGood = data[1] | (data[2] << 8);
    elrsFlags = data[3];
    if (len > 4) elrsMessage = String((char*)(data+4));
  }
}

// Обработка всех пакетов от передатчика (телеметрия)
void processIncomingCrsf() {
  static uint8_t buffer[128];
  static uint8_t index = 0;
  static bool inPacket = false;
  static uint8_t expectedLen = 0;
  
  while (transmitterSerial.available()) {
    uint8_t b = transmitterSerial.read();
    if (!inPacket && b == 0xC8) { inPacket = true; index = 0; expectedLen = 0; }
    if (inPacket) {
      if (index < sizeof(buffer)) buffer[index++] = b;
      if (index == 2) expectedLen = buffer[1] + 2;
      if (index == expectedLen && expectedLen > 0) {
        uint8_t crc = crsfCrc8(buffer+2, index-3);
        if (crc == buffer[index-1]) {
          uint8_t packetType = buffer[3];
          uint8_t* payload = buffer+4;
          uint8_t payloadLen = index-5;
          switch (packetType) {
            case CRSF_FRAMETYPE_DEVICE_INFO: processDeviceInfo(payload, payloadLen); break;
            case CRSF_FRAMETYPE_PARAMETER_DATA: processParameterData(payload, payloadLen); break;
            case CRSF_FRAMETYPE_ELRS_STATUS: processElrsStatus(payload, payloadLen); break;
          }
          // Обновляем таймер телеметрии при любом валидном пакете от передатчика
          lastTelemPacketTime = millis();
        }
        inPacket = false;
      } else if (index > 64) inPacket = false;
    }
  }
}

// ========== ОБРАБОТКА ПАКЕТОВ ОТ ПРИЁМНИКА (ретрансляция с буферизацией) ==========
void processReceiverData() {
  while (receiverSerial.available()) {
    uint8_t b = receiverSerial.read();
    
    // Детектор CRSF-пакета для обновления lastPacketTime и накопления
    if (!rxInPacket && b == 0xC8) {
      rxInPacket = true;
      rxPacketIndex = 0;
      rxExpectedLen = 0;
    }
    if (rxInPacket) {
      if (rxPacketIndex < sizeof(rxPacketBuffer)) {
        rxPacketBuffer[rxPacketIndex++] = b;
      }
      if (rxPacketIndex == 2) {
        rxExpectedLen = rxPacketBuffer[1] + 2;
      }
      if (rxPacketIndex == rxExpectedLen && rxExpectedLen > 0) {
        // Валидируем CRC
        uint8_t crc = crsfCrc8(rxPacketBuffer+2, rxPacketIndex-3);
        if (crc == rxPacketBuffer[rxPacketIndex-1]) {
          // Обновляем время последнего пакета от радио
          lastPacketTime = millis();
          packetCount++;
          
          // Отправляем накопленный пакет в передатчик (полудуплекс)
          // Переключаем направление (уже установлено в sendCrsfPacketToTransmitter)
          sendCrsfPacketToTransmitter(rxPacketBuffer+4, rxPacketIndex-5, rxPacketBuffer[3]);
        }
        rxInPacket = false;
      } else if (rxPacketIndex > 64) {
        rxInPacket = false; // сброс при переполнении
      }
    }
  }
}

// ========== НАСТРОЙКИ ==========
void loadSettings() {
  prefs.begin("elrs-repeater", false);
  currentBaudRate = prefs.getInt("baud", DEFAULT_BAUD);
  currentPowerMode = prefs.getInt("powerMode", POWER_MODE_AUTO);
  prefs.end();
}

void saveBaudRate(int baud) {
  prefs.begin("elrs-repeater", false);
  prefs.putInt("baud", baud);
  prefs.end();
  currentBaudRate = baud;
}

void savePowerMode(int mode) {
  prefs.begin("elrs-repeater", false);
  prefs.putInt("powerMode", mode);
  prefs.end();
  currentPowerMode = mode;
}

// ========== ВЕБ-ИНТЕРФЕЙС (без изменений, кроме API обновления статуса) ==========
void handleRoot() {
  String html = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
    <title>ELRS Repeater Pro</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
        *{box-sizing:border-box} body{font-family:'Segoe UI',Arial;margin:0;padding:20px;background:#0a0e1a;color:#e0e0e0} .container{max-width:800px;margin:0 auto} h1{color:#e94560;text-align:center}
        .tabs{display:flex;gap:5px;margin-bottom:20px;flex-wrap:wrap} .tab-btn{padding:12px 24px;background:#1a1f2e;border:none;border-radius:8px 8px 0 0;color:#aaa;cursor:pointer} .tab-btn.active{background:#e94560;color:#fff}
        .tab-content{display:none;background:#11161f;border-radius:0 12px 12px 12px;padding:20px} .tab-content.active{display:block}
        .card{background:#1a1f2e;border-radius:12px;padding:20px;margin-bottom:20px} .card h3{margin-top:0;color:#e94560;border-bottom:1px solid #2a3040;padding-bottom:10px}
        table{width:100%;border-collapse:collapse} td,th{padding:12px;border-bottom:1px solid #2a3040;text-align:left} th{color:#e94560;width:40%}
        button,.button{background:#e94560;border:none;padding:10px 20px;border-radius:8px;color:#fff;cursor:pointer} button:hover{background:#c0314a}
        button.danger{background:#dc3545} button.secondary{background:#2a3040} select,input{background:#2a3040;border:none;padding:10px;border-radius:6px;color:#fff}
        .status-ok{color:#2ecc71} .status-bad{color:#e74c3c} .flex{display:flex;gap:10px;flex-wrap:wrap;align-items:center}
        .info-text{font-size:12px;color:#888;margin-top:10px} hr{margin:15px 0;border-color:#2a3040}
    </style>
    <script>
        let currentBaudrate = 460800;
        async function fetchStatus() {
            const r=await fetch('/api/status'), d=await r.json();
            document.getElementById('linkRadio').innerHTML=d.linkRadio?'✅ CONNECTED':'❌ DISCONNECTED';
            document.getElementById('linkRadio').className=d.linkRadio?'status-ok':'status-bad';
            document.getElementById('linkCopter').innerHTML=d.linkCopter?'✅ CONNECTED':'❌ DISCONNECTED';
            document.getElementById('linkCopter').className=d.linkCopter?'status-ok':'status-bad';
            document.getElementById('rssi').innerText=d.rssi||'---';
            document.getElementById('packets').innerText=d.packets;
            document.getElementById('packetsBad').innerText=d.packetsBad;
            document.getElementById('packetsGood').innerText=d.packetsGood;
            document.getElementById('baudrate').innerText=d.baudrate;
            document.getElementById('deviceName').innerText=d.deviceName;
            document.getElementById('txpower').innerText=d.txpower+" mW";
            document.getElementById('pktrate').innerText=d.pktrate+" Hz";
            if(d.txPowerOptions) document.getElementById('txpowerSelect').innerHTML=d.txPowerOptions;
            if(d.pktRateOptions) document.getElementById('pktrateSelect').innerHTML=d.pktRateOptions;
        }
        async function fetchPowerStatus() {
            const r=await fetch('/api/powerMode'), d=await r.json();
            document.getElementById('relayState').innerHTML=d.relayState?'✅ ON':'❌ OFF';
            let t='';
            if(d.mode==0) t='Auto (Bind Only)';
            else if(d.mode==1) t='Always ON';
            else t='Always OFF';
            document.getElementById('powerModeText').innerHTML=t;
            document.getElementById('modeAuto').checked=(d.mode==0);
            document.getElementById('modeAlwaysOn').checked=(d.mode==1);
            document.getElementById('modeAlwaysOff').checked=(d.mode==2);
        }
        async function setBaudrate() {
            let b=document.getElementById('newBaud').value;
            let r=await fetch('/api/setBaud?baud='+b);
            if(r.ok){alert('Baudrate changed. Restarting...');setTimeout(()=>location.reload(),2000);}
            else alert('Error');
        }
        async function setTxPower() {
            let p=document.getElementById('txpowerSelect').value;
            await fetch('/api/setTxPower?power='+p);
            fetchStatus();
        }
        async function setPktRate() {
            let r=document.getElementById('pktrateSelect').value;
            await fetch('/api/setPktRate?rate='+r);
            fetchStatus();
        }
        async function bindTransmitter() {
            if(confirm('Put transmitter into bind mode?')) await fetch('/api/bindTx');
        }
        async function bindReceiver() {
            if(confirm('Put receiver into bind mode?')) await fetch('/api/bindRx');
        }
        async function resetReceiverBind() {
            if(confirm('Reset receiver binding?')) await fetch('/api/resetRxBind');
        }
        async function enableTxWifi() {
            await fetch('/api/enableTxWifi');
            alert('Transmitter WiFi enabled. Look for "ExpressLRS TX" network.');
        }
        async function enableRxWifi() {
            await fetch('/api/enableRxWifi');
            alert('Receiver WiFi enabled. Look for "ExpressLRS RX" network.');
        }
        async function applyPowerMode() {
            let mode=document.getElementById('modeAuto').checked?0:(document.getElementById('modeAlwaysOn').checked?1:2);
            await fetch('/api/setPowerMode?mode='+mode);
            fetchPowerStatus();
        }
        async function sendCliCommand() {
            let cmd=document.getElementById('cliCommand').value;
            if(!cmd) return;
            let r=await fetch('/api/sendCli?cmd='+encodeURIComponent(cmd));
            let d=await r.json();
            alert(d.message);
        }
        async function resetDefault() {
            if(confirm('Reset to default baudrate?')){
                await fetch('/api/reset');
                setTimeout(()=>location.reload(),2000);
            }
        }
        function switchTab(tabId){
            document.querySelectorAll('.tab-content').forEach(e=>e.classList.remove('active'));
            document.querySelectorAll('.tab-btn').forEach(e=>e.classList.remove('active'));
            document.getElementById('tab-'+tabId).classList.add('active');
            document.querySelector(`.tab-btn[data-tab="${tabId}"]`).classList.add('active');
        }
        setInterval(()=>{fetchStatus();fetchPowerStatus();},1000);
        window.onload=()=>{fetchStatus();fetchPowerStatus();};
    </script>
</head>
<body>
<div class="container">
    <h1>ELRS Repeater Pro</h1>
    <div class="tabs">
        <button class="tab-btn active" data-tab="dashboard" onclick="switchTab('dashboard')">Dashboard</button>
        <button class="tab-btn" data-tab="transmitter" onclick="switchTab('transmitter')">Transmitter</button>
        <button class="tab-btn" data-tab="receiver" onclick="switchTab('receiver')">Receiver</button>
        <button class="tab-btn" data-tab="power" onclick="switchTab('power')">Power</button>
    </div>
    <!-- Dashboard -->
    <div id="tab-dashboard" class="tab-content active">
        <div class="card"><h3>Device Info</h3><table><tr><th>Device Name</th><td id="deviceName">---</td></tr>
        <tr><th>Baudrate</th><td id="baudrate">---</td></tr></table></div>
        <div class="card"><h3>Connection Status</h3><table><tr><th>Link to Radio (Pult → Repeater)</th><td id="linkRadio">---</td></tr>
        <tr><th>Link to Copter (Telemetry)</th><td id="linkCopter">---</td></tr>
        <tr><th>RSSI</th><td id="rssi">---</td></tr></table></div>
        <div class="card"><h3>Statistics</h3><table><tr><th>Packets Received</th><td id="packets">---</td></tr>
        <tr><th>Bad Packets (last sec)</th><td id="packetsBad">---</td></tr>
        <tr><th>Good Packets (last sec)</th><td id="packetsGood">---</td></tr>
        <tr><th>TX Power</th><td id="txpower">---</td></tr>
        <tr><th>Packet Rate</th><td id="pktrate">---</td></tr></table></div>
    </div>
    <!-- Transmitter -->
    <div id="tab-transmitter" class="tab-content">
        <div class="card"><h3>ELRS Transmitter Settings</h3><table><tr><th>Current TX Power</th><td id="txpower">---</td></tr>
        <tr><th>Current Packet Rate</th><td id="pktrate">---</td></tr></table></div>
        <div class="card"><h3>Change TX Power</h3><div class="flex"><select id="txpowerSelect"><option>10 mW</option><option>25 mW</option><option>50 mW</option><option>100 mW</option><option>250 mW</option><option>500 mW</option><option>1000 mW</option></select><button onclick="setTxPower()">Apply</button></div></div>
        <div class="card"><h3>Change Packet Rate</h3><div class="flex"><select id="pktrateSelect"><option>50 Hz</option><option>100 Hz</option><option>150 Hz</option><option>200 Hz</option><option>250 Hz</option><option>333 Hz</option><option>500 Hz</option></select><button onclick="setPktRate()">Apply</button></div></div>
        <div class="card"><h3>Binding</h3><button onclick="bindTransmitter()">Bind Transmitter (to Drone)</button><div class="info-text">Puts transmitter into bind mode.</div><hr><button class="secondary" onclick="enableTxWifi()">📶 Enable Transmitter WiFi</button><div class="info-text">After enabling, connect to "ExpressLRS TX" (pass: expresslrs) and open http://10.0.0.1</div></div>
        <div class="card"><h3>System</h3><div class="flex"><input type="number" id="newBaud" placeholder="New baudrate" value="460800"><button onclick="setBaudrate()">Change Baudrate</button><button class="danger" onclick="resetDefault()">Reset to Default</button></div></div>
    </div>
    <!-- Receiver -->
    <div id="tab-receiver" class="tab-content">
        <div class="card"><h3>ELRS Receiver Settings</h3><table><tr><th>Status</th><td id="rxStatus">---</td></tr>
        <tr><th>Bound To</th><td id="rxBoundTo">---</td></tr></table></div>
        <div class="card"><h3>Binding</h3><button onclick="bindReceiver()">Enter Bind Mode</button><button class="danger" onclick="resetReceiverBind()">Reset Bind (Clear)</button><div class="info-text">How to bind: 1) Click "Enter Bind Mode" (LED blinks). 2) On radio, start binding in ELRS Lua. 3) LED solid = bound.</div><hr><button class="secondary" onclick="enableRxWifi()">📶 Enable Receiver WiFi</button><div class="info-text">After enabling, connect to "ExpressLRS RX" (pass: expresslrs) and open http://10.0.0.1</div></div>
        <div class="card"><h3>CLI Commands (via CRSF Passthrough)</h3><div class="flex"><input type="text" id="cliCommand" placeholder="e.g. set expresslrs_uid = 0" style="flex:1"><button onclick="sendCliCommand()">Send CLI Command</button></div><div class="info-text">Commands: set expresslrs_uid = 0 (reset bind), set expresslrs_rate_index = 2 (packet rate), get expresslrs_uid</div></div>
    </div>
    <!-- Power -->
    <div id="tab-power" class="tab-content">
        <div class="card"><h3>Transmitter Power Control</h3><div class="flex" style="flex-direction:column;align-items:stretch"><label><input type="radio" name="powerMode" value="0" id="modeAuto"> <strong>Auto (Bind Only)</strong> - Transmitter ON only when receiver is bound</label><label><input type="radio" name="powerMode" value="1" id="modeAlwaysOn"> <strong>Always ON</strong> - Transmitter powered constantly (for configuration)</label><label><input type="radio" name="powerMode" value="2" id="modeAlwaysOff"> <strong>Always OFF</strong> - Transmitter forced OFF</label></div><button onclick="applyPowerMode()" style="margin-top:15px">Apply Mode</button><hr><h4>Current Status</h4><table><tr><th>Relay State</th><td id="relayState">---</td></tr>
        <tr><th>Selected Mode</th><td id="powerModeText">---</td></tr></table><div class="info-text">In "Always ON" mode, transmitter is powered even without receiver binding – use to configure transmitter via its WebUI. Remember to switch back to "Auto".</div></div>
    </div>
</div>
<script>
    async function updateReceiverStatus() {
        let r=await fetch('/api/receiverStatus'), d=await r.json();
        document.getElementById('rxStatus').innerHTML=d.bound?'Bound':'❌ Not bound';
        document.getElementById('rxBoundTo').innerText=d.boundTo||'---';
    }
    async function fetchAll() {
        await fetchStatus();
        await fetchPowerStatus();
        await updateReceiverStatus();
        let r=await fetch('/api/status'), d=await r.json();
        document.getElementById('txpower').innerText=d.txpower+" mW";
        document.getElementById('pktrate').innerText=d.pktrate+" Hz";
    }
    setInterval(fetchAll,1000);
    window.onload=fetchAll;
</script>
</body>
</html>
)rawliteral";
  server.send(200, "text/html", html);
}

// ========== API ОБРАБОТЧИКИ ==========
void handleApiStatus() {
  StaticJsonDocument<1024> doc;
  doc["linkRadio"] = linkToRadio;
  doc["linkCopter"] = linkToCopter;
  doc["rssi"] = rssi;
  doc["packets"] = packetCount;
  doc["packetsBad"] = packetsBad;
  doc["packetsGood"] = packetsGood;
  doc["baudrate"] = currentBaudRate;
  doc["txpower"] = currentTxPower;
  doc["pktrate"] = currentPktRate;
  doc["deviceName"] = deviceName;
  String txPowerOptions = "", pktRateOptions = "";
  for (auto& pair : deviceFields) {
    CRSFField& f = pair.second;
    if ((f.name.indexOf("Power")>=0 || f.name.indexOf("Tx")>=0) && f.fieldType==0x07 && f.options.length()) {
      int start=0,comma;
      do {
        comma = f.options.indexOf(',',start);
        String opt = (comma==-1)?f.options.substring(start):f.options.substring(start,comma);
        opt.trim();
        txPowerOptions += "<option value=\""+opt+"\">"+opt+" mW</option>";
        start = comma+1;
      } while(comma!=-1);
    }
    if ((f.name.indexOf("Packet")>=0 || f.name.indexOf("Rate")>=0) && f.fieldType==0x07 && f.options.length()) {
      int start=0,comma;
      do {
        comma = f.options.indexOf(',',start);
        String opt = (comma==-1)?f.options.substring(start):f.options.substring(start,comma);
        opt.trim();
        pktRateOptions += "<option value=\""+opt+"\">"+opt+"</option>";
        start = comma+1;
      } while(comma!=-1);
    }
  }
  doc["txPowerOptions"] = txPowerOptions;
  doc["pktRateOptions"] = pktRateOptions;
  String response; serializeJson(doc, response);
  server.send(200, "application/json", response);
}

void handleApiSetBaud() {
  if(server.hasArg("baud")){
    int b=server.arg("baud").toInt();
    if(b>=9600 && b<=921600){ saveBaudRate(b); server.send(200,"text/plain","OK"); delay(100); ESP.restart();}
    else server.send(400,"text/plain","Invalid baudrate");
  } else server.send(400,"text/plain","Missing baud");
}

void handleApiSetTxPower() {
  if(server.hasArg("power")){
    currentTxPower = server.arg("power").toInt();
    int id=findFieldId("Power"); if(id>=0) setFieldValue(id, currentTxPower);
    server.send(200,"text/plain","OK");
  } else server.send(400,"text/plain","Missing power");
}

void handleApiSetPktRate() {
  if(server.hasArg("rate")){
    currentPktRate = server.arg("rate").toInt();
    int id=findFieldId("Packet"); if(id<0) id=findFieldId("Rate"); if(id<0) id=findFieldId("AirRate");
    if(id>=0) setFieldValue(id, currentPktRate);
    server.send(200,"text/plain","OK");
  } else server.send(400,"text/plain","Missing rate");
}

void handleApiBindTx() { bindTransmitter(); server.send(200,"text/plain","OK"); }
void handleApiBindRx() { bindReceiver(); server.send(200,"text/plain","OK"); }
void handleApiResetRxBind() { resetReceiverBind(); server.send(200,"text/plain","OK"); }
void handleApiEnableTxWifi() { enableTransmitterWiFi(); server.send(200,"text/plain","OK"); }
void handleApiEnableRxWifi() { enableReceiverWiFi(); server.send(200,"text/plain","OK"); }

void handleApiSendCli() {
  if(server.hasArg("cmd")){
    String cmd=server.arg("cmd");
    uint8_t payload[64];
    uint8_t idx=0;
    payload[idx++]=CRSF_COMMAND_REALM_RX;
    payload[idx++]=0x08;
    for(int i=0;i<cmd.length();i++) payload[idx++]=cmd[i];
    sendCrsfPacketToReceiver(payload,idx);
    StaticJsonDocument<64> doc; doc["message"]="CLI command sent: "+cmd;
    String resp; serializeJson(doc,resp);
    server.send(200,"application/json",resp);
  } else server.send(400,"text/plain","Missing cmd");
}

void handleApiReceiverStatus() {
  StaticJsonDocument<128> doc;
  doc["bound"] = linkToCopter;
  doc["boundTo"] = linkToCopter ? deviceName : "---";
  String resp; serializeJson(doc,resp);
  server.send(200,"application/json",resp);
}

void handleApiPowerMode() {
  StaticJsonDocument<64> doc;
  doc["mode"] = currentPowerMode;
  doc["relayState"] = digitalRead(RELAY_PIN);
  String resp; serializeJson(doc,resp);
  server.send(200,"application/json",resp);
}

void handleApiSetPowerMode() {
  if(server.hasArg("mode")){
    int mode=server.arg("mode").toInt();
    if(mode>=POWER_MODE_AUTO && mode<=POWER_MODE_ALWAYS_OFF){
      savePowerMode(mode);
      server.send(200,"text/plain","OK");
    } else server.send(400,"text/plain","Invalid mode");
  } else server.send(400,"text/plain","Missing mode");
}

void handleApiReset() { saveBaudRate(DEFAULT_BAUD); server.send(200,"text/plain","OK"); delay(100); ESP.restart(); }

void setupWebServer() {
  server.on("/", handleRoot);
  server.on("/api/status", handleApiStatus);
  server.on("/api/setBaud", handleApiSetBaud);
  server.on("/api/setTxPower", handleApiSetTxPower);
  server.on("/api/setPktRate", handleApiSetPktRate);
  server.on("/api/bindTx", handleApiBindTx);
  server.on("/api/bindRx", handleApiBindRx);
  server.on("/api/resetRxBind", handleApiResetRxBind);
  server.on("/api/enableTxWifi", handleApiEnableTxWifi);
  server.on("/api/enableRxWifi", handleApiEnableRxWifi);
  server.on("/api/sendCli", handleApiSendCli);
  server.on("/api/receiverStatus", handleApiReceiverStatus);
  server.on("/api/powerMode", handleApiPowerMode);
  server.on("/api/setPowerMode", handleApiSetPowerMode);
  server.on("/api/reset", handleApiReset);
  server.begin();
}

// ========== SETUP ==========
void setup() {
  Serial.begin(115200);
  Serial.println("\n=== ELRS Repeater Pro Fixed ===");
  
  // Настройка сторожевого таймера (исправленный вызов)
  esp_task_wdt_config_t wdt_config = {
      .timeout_ms = WDT_TIMEOUT_MS,
      .trigger_panic = true,
  };
  esp_task_wdt_init(&wdt_config);
  esp_task_wdt_add(NULL);
  
  loadSettings();

  pinMode(RELAY_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);
  digitalWrite(LED_PIN, HIGH);   // выключен (активный LOW)
  digitalWrite(DIR_PIN, LOW);    // режим приёма по умолчанию

  receiverSerial.begin(currentBaudRate, SERIAL_8N1, 16, 17);
  transmitterSerial.begin(currentBaudRate, SERIAL_8N1, 18, 19);
  Serial.printf("UARTs at %d baud\n", currentBaudRate);

  // AP+STA режим
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP("ELRS_Repeater_Pro", "12345678");
  Serial.print("AP IP: "); Serial.println(WiFi.softAPIP());

  setupWebServer();
  
  delay(1000);
  sendDevicePing();
  
  lastPacketTime = millis();
  lastTelemPacketTime = millis();
}

// ========== LOOP ==========
void loop() {
  // Обработка входящих данных от приёмника (ретрансляция с буферизацией)
  processReceiverData();
  
  // Обработка телеметрии от передатчика (CRSF Config Protocol и ELRS_STATUS)
  processIncomingCrsf();
  
  // Обновление статусов связи
  linkToRadio = (millis() - lastPacketTime < TIMEOUT_MS);
  linkToCopter = (millis() - lastTelemPacketTime < TELEM_TIMEOUT_MS);
  
  // Управление реле в зависимости от режима
  bool relayOn = false;
  if (currentPowerMode == POWER_MODE_ALWAYS_ON) relayOn = true;
  else if (currentPowerMode == POWER_MODE_ALWAYS_OFF) relayOn = false;
  else relayOn = linkToRadio;  // AUTO: включаем при наличии связи с радио
  
  digitalWrite(RELAY_PIN, relayOn ? HIGH : LOW);
  
  // Индикация LED
  if (currentPowerMode == POWER_MODE_AUTO && !linkToRadio) {
    // мигание при потере связи
    if (millis() - lastLedBlink > 200) {
      lastLedBlink = millis();
      ledState = !ledState;
      digitalWrite(LED_PIN, ledState ? LOW : HIGH);
    }
  } else {
    // ровный свет при наличии связи или в ручных режимах
    digitalWrite(LED_PIN, relayOn ? LOW : HIGH);
    lastLedBlink = millis();
  }
  
  // Периодический запрос статуса ELRS (телеметрия)
  if (millis() - lastElrsStatusRequest > 2000) {
    lastElrsStatusRequest = millis();
    requestElrsStatus();
  }
  
  // Запрос всех полей после получения Device Info
  if (deviceInfoReceived && !fieldsRequested && millis() > 3000) {
    requestAllFields();
  }
  
  // Обслуживание веб-сервера
  server.handleClient();
  
  // Сброс сторожевого таймера (вместо delay)
  esp_task_wdt_reset();
  
  // Небольшой yield для ESP32 (не блокирует)
  yield();
}