#include <Arduino.h>
#include <WiFi.h>
#include <NimBLEDevice.h>
#include <USB.h>
#include <WiFiServer.h>
#include <Wire.h>
#include <Adafruit_NeoPixel.h>
#include <ArduinoOTA.h>
#include <Preferences.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <ESPmDNS.h>
#include <cerrno>
#include <sys/socket.h>
#include <string>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

extern "C" {
#include "esp_bt.h"
#include "esp_task_wdt.h"
#include "nvs_flash.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_idf_version.h"
#include "esp_core_dump.h"
#include "esp_heap_caps.h"
#include "esp_sleep.h"
#include "esp_system.h"
}
#include "display.h"
#include "buzzer.h"
#include "connect_webserver.h"
#include "ota_config.h"

#include "terminal_transport.h"
#include "usb_host_bridge.h"

// -----------------------------------------------------------------------------
// Firmware Version (automatisch aus Git via git_version.py)
// -----------------------------------------------------------------------------
#ifndef GIT_VERSION
#define GIT_VERSION "dev"
#endif
#ifndef GIT_COMMIT
#define GIT_COMMIT "unknown"
#endif
#ifndef BUILD_TIME
#define BUILD_TIME "unknown"
#endif
#define FIRMWARE_VERSION GIT_VERSION

namespace WiFiConfig {
  enum class Mode : uint8_t {
    AccessPoint = 0,
    Client = 1,
  };

  constexpr const char* MDNS_HOSTNAME = "connect";
  constexpr unsigned long CLIENT_CONNECT_TIMEOUT_MS = 15000;
}

// -----------------------------------------------------------------------------
// Timing-Konstanten (eliminiert Magic Numbers)
// -----------------------------------------------------------------------------
namespace Timing {
  constexpr unsigned long BUTTON_SHORT_MAX_MS   = 500;   // Max Dauer für kurzen Klick
  constexpr unsigned long BUTTON_DOUBLE_GAP_MS  = 400;   // Max Pause zwischen Doppelklicks
  constexpr unsigned long BUTTON_LONG_MS        = 1000;  // Ab hier: langer Druck (BLE Toggle)
  constexpr unsigned long BUTTON_OTA_MS         = 5000;  // Ab hier: OTA Update
  constexpr unsigned long BUTTON_FEEDBACK_BLE   = 1000;  // Feedback "BLE Toggle"
  constexpr unsigned long BUTTON_FEEDBACK_OTA   = 3000;  // Feedback "OTA Update"
  constexpr unsigned long BUTTON_ARM_STABLE_MS  = 1000;
  constexpr unsigned long BUTTON_OTA_CONFIRM_DELAY_MS = 1000;
  constexpr unsigned long BUTTON_OTA_CONFIRM_WINDOW_MS = 10000;
  
  constexpr unsigned long DISPLAY_UPDATE_MS     = 1000;  // Display Refresh Rate
  constexpr unsigned long BLE_WELCOME_DELAY_MS  = 5000;  // Zeit bis BLE Welcome
  constexpr unsigned long MESSAGE_DISPLAY_MS    = 2000;  // Anzeigedauer für Meldungen (2s)
  constexpr unsigned long SPLASH_DISPLAY_MS     = 2000;  // Logo-Anzeigedauer
  
  constexpr unsigned long BATTERY_READ_INTERVAL = 500;   // Batterie-Messintervall
  constexpr unsigned long USB_TASK_INTERVAL     = 1;     // USB Host Task Intervall
  constexpr unsigned long OTA_WIFI_TIMEOUT_MS   = 15000; // WiFi Verbindungs-Timeout für OTA
  constexpr unsigned long TCP_IDLE_TIMEOUT_MS   = 900000; // TCP Idle Timeout (15 Minuten)
  constexpr size_t        TELNET_CMD_BUFFER_MAX = 256;   // Max. Telnet Command Buffer Länge
  constexpr size_t        SERIAL_DRAIN_BUDGET   = 2048;  // RX-Bytes pro Pump, damit WiFi/BLE Luft behalten
  constexpr size_t        BLE_RX_QUEUE_LEN      = 2048;
  constexpr size_t        BLE_RX_PROCESS_BUDGET = 1024;
  constexpr size_t        TELNET_TX_BUFFER_BYTES = 8192;
  constexpr size_t        TELNET_CONTROL_RESERVE = 2048;
  constexpr size_t        BLE_TX_BUFFER_BYTES    = 8192;
  constexpr size_t        BLE_CONTROL_RESERVE    = 1024;
  constexpr unsigned long BLE_PACKET_IDLE_MS     = 10;
  constexpr size_t        TARGET_TX_BUFFER_BYTES = 4096;
  constexpr size_t        TARGET_TX_PUMP_BUDGET  = 2048;
  constexpr size_t        TARGET_CONTROL_RESERVE = 512;
}

// -----------------------------------------------------------------------------
// Baudrate-Konfiguration (zentrale Definition)
// -----------------------------------------------------------------------------
namespace BaudConfig {
  // Alle gültigen Baudraten
  constexpr uint32_t ALL_RATES[] = {300, 1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600};
  constexpr int ALL_RATES_COUNT = sizeof(ALL_RATES) / sizeof(ALL_RATES[0]);
  
  // Rotation für Doppelklick (häufig verwendete)
  constexpr uint32_t CYCLE_RATES[] = {9600, 19200, 38400, 57600, 115200, 230400};
  constexpr int CYCLE_RATES_COUNT = sizeof(CYCLE_RATES) / sizeof(CYCLE_RATES[0]);
  
  constexpr uint32_t DEFAULT_BAUD = 115200;
}

namespace SerialBufferConfig {
  constexpr size_t RS232_RX_BYTES = 8192;
}

namespace PowerConfig {
  constexpr uint8_t CRITICAL_SLEEP_PERCENT = 3;
  constexpr float CRITICAL_SLEEP_VOLTAGE = 3.05f;
  constexpr uint32_t CRITICAL_SLEEP_GRACE_MS = 120000;
  constexpr uint32_t CRITICAL_SLEEP_MIN_UPTIME_MS = 120000;
  constexpr uint64_t CRITICAL_SLEEP_US = 60ULL * 1000000ULL;
}

namespace BatteryConfig {
  constexpr float VALID_MIN_VOLTAGE = 2.70f;
  constexpr float VALID_MAX_VOLTAGE = 4.60f;
  constexpr float STARTUP_FALLBACK_VOLTAGE = 3.80f;
  constexpr uint8_t MIN_VALID_SAMPLES_FOR_SLEEP = 8;
}

namespace HardwareConfig {
  constexpr bool MAX3232_ALWAYS_ENABLED = true;
  constexpr bool MAX3232_ACTIVE_LEVEL_HIGH = true;
}

// -----------------------------------------------------------------------------
// Pins
// -----------------------------------------------------------------------------
#define MODE_BUTTON_PIN 0
#define BUZZER_PIN      1
#define MAX3232_EN_PIN  2
#define BATTERY_PIN     3

#ifndef RGB_LED_PIN
#define RGB_LED_PIN 48
#endif
#ifndef RGB_LED_BRIGHTNESS
#define RGB_LED_BRIGHTNESS 32
#endif

#define UART_RX 5
#define UART_TX 6

// I2C Display (SDA=8, SCL=9)
#define I2C_SDA 8
#define I2C_SCL 9

// -----------------------------------------------------------------------------
// WiFi
// -----------------------------------------------------------------------------
#define WIFI_SSID     "ConNect"
#define WIFI_PASSWORD "12345678"
#define WIFI_IP       "192.168.32.1"
#define WIFI_CHANNEL  6

// -----------------------------------------------------------------------------
// Globals
// -----------------------------------------------------------------------------
Preferences prefs;  // NVS-Storage für persistente Einstellungen
Display display;
HardwareSerial RS232Serial(1);
static terminal::ByteRingBuffer<Timing::TARGET_TX_BUFFER_BYTES> targetTxBuffer;
WiFiServer tcpServer(23);
WiFiClient tcpClient;
static String telnetCmdBuffer;
static bool telnetSessionActive = false;
static terminal::ByteRingBuffer<Timing::TELNET_TX_BUFFER_BYTES> telnetTxBuffer;
static terminal::TelnetDecoder telnetDecoder;
static terminal::NewlineDecoder telnetNewlineDecoder;
static bool telnetAtLineStart = true;
static bool telnetCmdMode = false;
static uint8_t telnetCmdPrefix = '`';
static uint32_t lastTcpActivity = 0;
static std::atomic<uint32_t> stat_telnet_tx_rejected{0};

NimBLEServer* pServer = nullptr;
NimBLECharacteristic* pTxCharacteristic = nullptr;
std::atomic<bool> deviceConnected{false};
bool oldDeviceConnected = false;
std::atomic<bool> bleAdvertising{false};
static std::atomic<bool> bleNotificationsEnabled{false};
static terminal::ByteRingBuffer<Timing::BLE_TX_BUFFER_BYTES> bleTxBuffer;
static std::atomic<bool> bleTxResetRequested{true};
static uint32_t bleTxLastEnqueueMs = 0;
static bool bleTxFlushRequested = false;

bool isUSBMode = true;   // USB-Host (CDC/FTDI/CP210x/PL2303) vs RS232
static WiFiConfig::Mode configuredWiFiMode = WiFiConfig::Mode::AccessPoint;
static bool wifiFallbackAccessPoint = false;
static bool mdnsRunning = false;
static uint32_t lastMdnsAttempt = 0;

char lastReceivedData = 0;
String bleCmdBuffer;
uint32_t currentBaudRate = 115200;
static QueueHandle_t bleRxQueue = nullptr;
static std::atomic<uint32_t> stat_ble_rx_dropped{0};
static std::atomic<uint32_t> stat_ble_tx_rejected{0};
static bool usbHostStarted = false;
static bool usbModeRestartRequested = false;
static uint32_t usbModeRestartRequestedAt = 0;
static bool runtimeServicesReady = false;

// Stats (extern für webserver.cpp). Atomare Zaehler, da sie aus
// BLE-Callback-Task und loop() inkrementiert werden.
std::atomic<uint32_t> stat_usb_rx{0}, stat_usb_tx{0};
std::atomic<uint32_t> stat_rs232_rx{0}, stat_rs232_tx{0};

// Aktivitäts-Tracking für Display (RX/TX in letztem Intervall)
static uint32_t lastRxActivity = 0;
static uint32_t lastTxActivity = 0;

static esp_reset_reason_t bootResetReason = ESP_RST_UNKNOWN;
static esp_sleep_wakeup_cause_t bootWakeCause = ESP_SLEEP_WAKEUP_UNDEFINED;
static uint32_t bootCount = 0;

// USB Verbindungsstatus-Tracking für Töne
static bool lastUsbConnected = false;

// Terminal defaults (fixed like PuTTY: character mode, echo off, EOL=CR)
static const uint8_t  telnetEolMode   = 1;     // 0=CRLF, 1=CR, 2=LF (fixed CR)
static bool           localEcho       = false; // false: Echo kommt vom seriellen Ziel

// BLE UART (Nordic) UUIDs
#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

// -----------------------------------------------------------------------------
// Forward decls
// -----------------------------------------------------------------------------
static void handleInternalCommand(const String& cmd);
static void bleNotifyLine(const String& line);
static void requestUsbModeRestart();

// -----------------------------------------------------------------------------
// Helpers
// -----------------------------------------------------------------------------
static inline void doBeep() { beep(BUZZER_PIN); }

void setSerialTargetMode(bool usbMode) {
  isUSBMode = usbMode;
  bool enHigh = HardwareConfig::MAX3232_ALWAYS_ENABLED
                  ? HardwareConfig::MAX3232_ACTIVE_LEVEL_HIGH
                  : !usbMode;
  pinMode(MAX3232_EN_PIN, OUTPUT);
  digitalWrite(MAX3232_EN_PIN, enHigh ? HIGH : LOW);
  gpio_set_direction((gpio_num_t)MAX3232_EN_PIN, GPIO_MODE_OUTPUT);
  gpio_set_level((gpio_num_t)MAX3232_EN_PIN, enHigh ? 1 : 0);
  if (usbMode && runtimeServicesReady && !usbHostStarted) {
    requestUsbModeRestart();
  }
}

static void requestUsbModeRestart() {
  if (usbModeRestartRequested) return;
  usbModeRestartRequested = true;
  usbModeRestartRequestedAt = millis();
  display.showMessage("USB Host", "Neustart...", "", 1200);
}

static bool expectedMax3232EnHigh() {
  return HardwareConfig::MAX3232_ALWAYS_ENABLED
           ? HardwareConfig::MAX3232_ACTIVE_LEVEL_HIGH
           : !isUSBMode;
}

static void forceMax3232EnLevel(bool high) {
  pinMode(MAX3232_EN_PIN, OUTPUT);
  digitalWrite(MAX3232_EN_PIN, high ? HIGH : LOW);
  gpio_set_direction((gpio_num_t)MAX3232_EN_PIN, GPIO_MODE_OUTPUT);
  gpio_set_level((gpio_num_t)MAX3232_EN_PIN, high ? 1 : 0);
}

static void enqueueBleRxByte(uint8_t b) {
  if (!bleRxQueue) return;
  if (xQueueSend(bleRxQueue, &b, 0) == pdPASS) return;
  stat_ble_rx_dropped.fetch_add(1, std::memory_order_relaxed);
}

// Check if character is a command prefix (backtick recommended, apostrophe also works)
static inline bool isCmdPrefix(char c) {
  unsigned char uc = (unsigned char)c;
  return uc == '`' ||   // Backtick (recommended for mobile)
         uc == '\'' ||  // Apostrophe
         uc == 0xB4;    // Acute accent from single-byte mobile keyboards
}

static const char* resetReasonToStr(esp_reset_reason_t reason) {
  switch (reason) {
    case ESP_RST_POWERON:   return "POWERON";
    case ESP_RST_EXT:       return "EXT";
    case ESP_RST_SW:        return "SW";
    case ESP_RST_PANIC:     return "PANIC";
    case ESP_RST_INT_WDT:   return "INT_WDT";
    case ESP_RST_TASK_WDT:  return "TASK_WDT";
    case ESP_RST_WDT:       return "WDT";
    case ESP_RST_DEEPSLEEP: return "DEEPSLEEP";
    case ESP_RST_BROWNOUT:  return "BROWNOUT";
    case ESP_RST_SDIO:      return "SDIO";
    default:                return "UNKNOWN";
  }
}

static const char* wakeCauseToStr(esp_sleep_wakeup_cause_t cause) {
  switch (cause) {
    case ESP_SLEEP_WAKEUP_UNDEFINED: return "UNDEFINED";
    case ESP_SLEEP_WAKEUP_EXT0:      return "EXT0";
    case ESP_SLEEP_WAKEUP_EXT1:      return "EXT1";
    case ESP_SLEEP_WAKEUP_TIMER:     return "TIMER";
    case ESP_SLEEP_WAKEUP_TOUCHPAD:  return "TOUCHPAD";
    case ESP_SLEEP_WAKEUP_ULP:       return "ULP";
    case ESP_SLEEP_WAKEUP_GPIO:      return "GPIO";
    case ESP_SLEEP_WAKEUP_UART:      return "UART";
    case ESP_SLEEP_WAKEUP_WIFI:      return "WIFI";
    default:                         return "OTHER";
  }
}

static String getConfiguredOTAManifestUrl() {
  String url = prefs.getString("otaUrl", OTAConfig::DEFAULT_MANIFEST_URL);
  url.trim();
  if (url.length() == 0) {
    url = OTAConfig::DEFAULT_MANIFEST_URL;
  }
  return url;
}

static String extractJsonStringField(const String& payload, const char* field, int searchFrom = 0) {
  String needle = String("\"") + field + "\"";
  int fieldIdx = payload.indexOf(needle, searchFrom);
  if (fieldIdx < 0) {
    return "";
  }

  int colonIdx = payload.indexOf(':', fieldIdx + needle.length());
  if (colonIdx < 0) {
    return "";
  }

  int valueStart = payload.indexOf('"', colonIdx + 1);
  if (valueStart < 0) {
    return "";
  }

  valueStart += 1;
  int valueEnd = valueStart;
  bool escaped = false;
  while (valueEnd < payload.length()) {
    char current = payload[valueEnd];
    if (current == '"' && !escaped) {
      break;
    }
    if (current == '\\' && !escaped) {
      escaped = true;
    } else {
      escaped = false;
    }
    valueEnd++;
  }

  if (valueEnd <= valueStart || valueEnd >= payload.length()) {
    return "";
  }

  return payload.substring(valueStart, valueEnd);
}

static bool isValidMD5(const String& value) {
  if (value.length() != 32) {
    return false;
  }
  for (size_t i = 0; i < value.length(); ++i) {
    char c = value[i];
    bool isHex = (c >= '0' && c <= '9') ||
                 (c >= 'a' && c <= 'f') ||
                 (c >= 'A' && c <= 'F');
    if (!isHex) {
      return false;
    }
  }
  return true;
}

static String buildAbsoluteUrl(const String& sourceUrl, const String& location) {
  if (location.length() == 0) {
    return "";
  }

  if (location.startsWith("http://") || location.startsWith("https://")) {
    return location;
  }

  int schemeIdx = sourceUrl.indexOf("://");
  if (schemeIdx < 0) {
    return location;
  }

  int hostStart = schemeIdx + 3;
  int hostEnd = sourceUrl.indexOf('/', hostStart);
  String origin = hostEnd >= 0 ? sourceUrl.substring(0, hostEnd) : sourceUrl;

  if (location.startsWith("/")) {
    return origin + location;
  }

  int lastSlash = sourceUrl.lastIndexOf('/');
  String baseDir = lastSlash >= 0 ? sourceUrl.substring(0, lastSlash + 1) : sourceUrl + "/";
  return baseDir + location;
}

// -----------------------------------------------------------------------------
// Battery measurement with filtering and calibration
// -----------------------------------------------------------------------------
// Spannungsteiler: 33k (oben) + 15k (unten) mit 100nF parallel
// Verhältnis: Vbat * 15k / (33k + 15k) = Vbat * 0.3125
// Rückrechnung: Vadc * (33k + 15k) / 15k = Vadc * 3.2

static float batteryVoltageFiltered = 0.0f;  // EMA gefilterte Spannung
static float batteryVoltageRawLast = 0.0f;
static int   batteryLevelFiltered = -1;      // Gefilterter Prozentwert (-1 = nicht initialisiert)
static uint32_t lastBatteryRead = 0;
static uint8_t batteryValidSamples = 0;
static bool batteryMeasurementValid = false;

static bool isPlausibleBatteryVoltage(float voltage) {
  return voltage >= BatteryConfig::VALID_MIN_VOLTAGE && voltage <= BatteryConfig::VALID_MAX_VOLTAGE;
}

static float readBatteryVoltageRaw() {
  // Spannungsteiler-Kalibrierung
  // Kalibriert: vADC=1.169V bei vBat=4.080V (gemessen mit Multimeter)
  // Verhältnis = 4.080 / 1.169 = 3.49
  constexpr float DIVIDER_RATIO = 3.49f;
  
  // ESP32-S3 ADC Konfiguration
  constexpr float ADC_VREF = 3.3f;
  constexpr int ADC_MAX = 4095;
  
  // Mehrfach-Sampling für Rauschunterdrückung.
  // Kein delayMicroseconds mehr zwischen Samples: analogRead() auf dem
  // ESP32-S3 braucht selbst ~15..30 us pro Konversion, das ist Abstand
  // genug. So bleibt die Gesamtdauer unter 0.5 ms statt ~1.6 ms und die
  // Funktion wird loop-freundlicher.
  constexpr int NUM_SAMPLES = 16;
  uint32_t rawSum = 0;

  for (int i = 0; i < NUM_SAMPLES; i++) {
    rawSum += analogRead(BATTERY_PIN);
  }
  
  float rawAvg = static_cast<float>(rawSum) / NUM_SAMPLES;
  
  // ESP32 ADC Linearitätskorrektur (Polynom-Korrektur für bessere Genauigkeit)
  // Der ESP32 ADC ist bekannt für Nichtlinearität, besonders < 0.1V und > 3.1V
  float rawNorm = rawAvg / ADC_MAX;
  // Leichte Korrektur für Nichtlinearität im mittleren Bereich
  float corrected = rawNorm + 0.015f * (rawNorm - 0.5f) * (1.0f - rawNorm);
  
  float vAdc = corrected * ADC_VREF;
  return vAdc * DIVIDER_RATIO;
}

static float readBatteryVoltage() {
  // Exponentieller gleitender Mittelwert (EMA) Filter
  // Alpha = 0.03 = sehr langsame Reaktion, extrem stabile Anzeige
  // Bei 500ms Update-Intervall: ~50 Sekunden für 90% Anpassung
  constexpr float EMA_ALPHA = 0.03f;
  
  float rawVoltage = readBatteryVoltageRaw();
  batteryVoltageRawLast = rawVoltage;

  if (!isPlausibleBatteryVoltage(rawVoltage)) {
    batteryMeasurementValid = false;
    if (batteryVoltageFiltered > 0.0f) {
      return batteryVoltageFiltered;
    }
    return BatteryConfig::STARTUP_FALLBACK_VOLTAGE;
  }

  if (batteryValidSamples < 255) {
    batteryValidSamples++;
  }
  batteryMeasurementValid = true;
  
  if (batteryVoltageFiltered == 0.0f) {
    // Erste Messung: Initialisiere mit aktuellem Wert
    batteryVoltageFiltered = rawVoltage;
  } else {
    // EMA Filter: new = alpha * raw + (1-alpha) * old
    batteryVoltageFiltered = EMA_ALPHA * rawVoltage + (1.0f - EMA_ALPHA) * batteryVoltageFiltered;
  }
  
  return batteryVoltageFiltered;
}

static bool batteryCanTriggerCriticalSleep() {
  return batteryMeasurementValid &&
         batteryValidSamples >= BatteryConfig::MIN_VALID_SAMPLES_FOR_SLEEP &&
         batteryVoltageFiltered > 0.0f &&
         batteryVoltageFiltered <= PowerConfig::CRITICAL_SLEEP_VOLTAGE &&
         millis() >= PowerConfig::CRITICAL_SLEEP_MIN_UPTIME_MS;
}

uint8_t getBatteryLevel() {
  // 18650 Li-Ion Spannungskurve (typisch)
  // 4.20V = 100% (voll geladen)
  // 3.70V = ~50% (Nennspannung)
  // 3.30V = ~10% (fast leer)
  // 3.00V = 0% (Entladeschlussspannung)
  constexpr float VBAT_EMPTY = 3.00f;
  
  // Nur alle 500ms neu messen (reduziert Schwankungen weiter)
  uint32_t now = millis();
  if (batteryLevelFiltered >= 0 && (now - lastBatteryRead) < 500) {
    return batteryLevelFiltered;
  }
  lastBatteryRead = now;
  
  float vBat = readBatteryVoltage();
  
  // Nichtlineare Spannungs-zu-Kapazität Umrechnung für Li-Ion
  // Die Entladekurve ist nicht linear!
  float pct;
  if (vBat >= 4.10f) {
    // 4.10V - 4.20V: 90% - 100%
    pct = 90.0f + (vBat - 4.10f) * 100.0f;
  } else if (vBat >= 3.70f) {
    // 3.70V - 4.10V: 20% - 90% (relativ linear)
    pct = 20.0f + (vBat - 3.70f) * (70.0f / 0.40f);
  } else if (vBat >= 3.30f) {
    // 3.30V - 3.70V: 5% - 20%
    pct = 5.0f + (vBat - 3.30f) * (15.0f / 0.40f);
  } else {
    // 3.00V - 3.30V: 0% - 5%
    pct = (vBat - VBAT_EMPTY) * (5.0f / 0.30f);
  }
  
  int newLevel = constrain(static_cast<int>(pct + 0.5f), 0, 100);
  
  // Hysterese: Nur ändern wenn Differenz > 3% (verhindert Flackern)
  if (batteryLevelFiltered < 0) {
    batteryLevelFiltered = newLevel;
  } else if (abs(newLevel - batteryLevelFiltered) > 3) {
    // Sanfter Übergang: Maximal 1% pro Update
    if (newLevel > batteryLevelFiltered) {
      batteryLevelFiltered++;
    } else {
      batteryLevelFiltered--;
    }
  }
  
  return static_cast<uint8_t>(batteryLevelFiltered);
}

// Erkennt ob der Akku geladen wird (Spannung > 4.15V oder steigend)
bool isBatteryCharging() {
  static float lastVoltage = 0.0f;
  static uint32_t lastCheck = 0;
  static bool chargingState = false;
  static int chargingCounter = 0;  // Zähler für stabile Erkennung
  
  // Nur alle 1 Sekunde prüfen
  if (millis() - lastCheck < 1000 && lastVoltage > 0) {
    return chargingState;
  }
  lastCheck = millis();
  
  float currentVoltage = batteryVoltageFiltered;
  
  // Ladeerkennung:
  // 1. Spannung > 4.18V = definitiv am Laden (Li-Ion Ladeschlussspannung ~4.2V)
  // 2. Spannung steigt merklich = wahrscheinlich am Laden
  bool chargingNow = false;
  
  if (currentVoltage > 4.18f) {
    chargingNow = true;  // Hohe Spannung = Laden
  } else if (lastVoltage > 0 && currentVoltage > lastVoltage + 0.005f) {
    chargingNow = true;  // Spannung steigt = Laden
  }
  
  // Zähler-basierte Hysterese für stabiles Verhalten
  if (chargingNow) {
    chargingCounter = min(chargingCounter + 2, 5);  // Schnell hochzählen
  } else {
    chargingCounter = max(chargingCounter - 1, 0);  // Langsam runterzählen
  }
  
  // Status wechseln nur bei klaren Schwellwerten
  if (chargingCounter >= 3 && !chargingState) {
    chargingState = true;   // Laden erkannt nach ~2s
  } else if (chargingCounter == 0 && chargingState) {
    chargingState = false;  // Nicht mehr laden nach ~3-5s
  }
  
  lastVoltage = currentVoltage;
  return chargingState;
}

static inline bool isValidBaud(uint32_t baud) {
  for (int i = 0; i < BaudConfig::ALL_RATES_COUNT; i++) {
    if (BaudConfig::ALL_RATES[i] == baud) return true;
  }
  return false;
}

void rs232ReinitUart() {
  RS232Serial.setRxBufferSize(SerialBufferConfig::RS232_RX_BYTES);
  RS232Serial.begin(currentBaudRate, SERIAL_8N1, UART_RX, UART_TX);
  RS232Serial.end();
  delay(10);
  gpio_reset_pin((gpio_num_t)UART_TX);
  pinMode(UART_TX, OUTPUT);
  digitalWrite(UART_TX, HIGH);
  delay(2);
  digitalWrite(UART_TX, LOW);
  delay(2);
  digitalWrite(UART_TX, HIGH);
  delay(2);
  pinMode(UART_TX, INPUT);
  gpio_reset_pin((gpio_num_t)UART_RX);
  delay(10);
  RS232Serial.setRxBufferSize(SerialBufferConfig::RS232_RX_BYTES);
  RS232Serial.begin(currentBaudRate, SERIAL_8N1, UART_RX, UART_TX);
  uart_set_pin(UART_NUM_1, UART_TX, UART_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
  RS232Serial.setRxFIFOFull(64);
}

static void cycleBaudRate() {
  // Finde aktuelle Position und gehe zur nächsten
  int idx = 0;
  for (int i = 0; i < BaudConfig::CYCLE_RATES_COUNT; i++) {
    if (BaudConfig::CYCLE_RATES[i] == currentBaudRate) { idx = i; break; }
  }
  idx = (idx + 1) % BaudConfig::CYCLE_RATES_COUNT;
  currentBaudRate = BaudConfig::CYCLE_RATES[idx];
  if (isUSBMode) RS232Serial.updateBaudRate(currentBaudRate);
  else           rs232ReinitUart();
  usbHostSetLineCoding(currentBaudRate);
  usbHostSetSignals(true, true);
  // Speichern in NVS
  prefs.putUInt("baudRate", currentBaudRate);
}

static inline void setBaudRate(uint32_t baud) {
  if (!isValidBaud(baud)) return;
  currentBaudRate = baud;
  if (isUSBMode) RS232Serial.updateBaudRate(baud);
  else           rs232ReinitUart();
  usbHostSetLineCoding(baud);
  usbHostSetSignals(true, true);
}

// -----------------------------------------------------------------------------
// Client-Eingaben werden gepuffert und blockweise zum aktiven Ziel gepumpt.
// Bei vollem Ring lesen Telnet/BLE/Web nicht weiter und erzeugen Backpressure.
// -----------------------------------------------------------------------------
static void pumpTargetTx() {
  size_t budget = Timing::TARGET_TX_PUMP_BUDGET;
  while (!targetTxBuffer.empty() && budget > 0) {
    size_t chunkLen = min(targetTxBuffer.contiguousSize(), budget);
    size_t written = 0;
    if (isUSBMode) {
      if (!usbHostConnected()) return;
      written = usbHostWrite(targetTxBuffer.frontData(), chunkLen);
      if (written > 0) stat_usb_tx.fetch_add(written, std::memory_order_relaxed);
    } else {
      int writable = RS232Serial.availableForWrite();
      if (writable <= 0) return;
      chunkLen = min(chunkLen, static_cast<size_t>(writable));
      written = RS232Serial.write(targetTxBuffer.frontData(), chunkLen);
      if (written > 0) stat_rs232_tx.fetch_add(written, std::memory_order_relaxed);
    }
    if (written == 0) return;
    targetTxBuffer.consume(written);
    budget -= written;
  }
}

static inline bool writeToTarget(uint8_t value) {
  return targetTxBuffer.push(value);
}

static bool forwardCommandCandidate(const uint8_t* prefix, size_t prefixLen,
                                    const String& command, bool appendEnter) {
  size_t required = prefixLen + command.length() + (appendEnter ? 1 : 0);
  if (required > targetTxBuffer.freeSpace()) return false;
  targetTxBuffer.push(prefix, prefixLen);
  targetTxBuffer.push(reinterpret_cast<const uint8_t*>(command.c_str()), command.length());
  if (appendEnter) targetTxBuffer.push('\r');
  return true;
}

static bool forwardCommandCandidate(uint8_t prefix, const String& command, bool appendEnter) {
  return forwardCommandCandidate(&prefix, 1, command, appendEnter);
}

// -----------------------------------------------------------------------------
// Gepufferte Terminal-Ausgabe (Zielgeraet -> Telnet/BLE)
// -----------------------------------------------------------------------------
static void closeTelnetSession(bool countPending = true) {
  if (countPending && !telnetTxBuffer.empty()) {
    stat_telnet_tx_rejected.fetch_add(telnetTxBuffer.size(), std::memory_order_relaxed);
  }
  telnetTxBuffer.clear();
  tcpClient.stop();
  telnetSessionActive = false;
  telnetCmdBuffer = "";
  telnetDecoder.reset();
  telnetNewlineDecoder.reset();
  telnetAtLineStart = true;
  telnetCmdMode = false;
}

static void pumpTelnetTx() {
  if (!telnetSessionActive) return;

  size_t budget = 4096;
  while (!telnetTxBuffer.empty() && budget > 0) {
    size_t chunkLen = telnetTxBuffer.contiguousSize();
    chunkLen = min(chunkLen, static_cast<size_t>(512));
    chunkLen = min(chunkLen, budget);
    errno = 0;
    int written = send(tcpClient.fd(), telnetTxBuffer.frontData(), chunkLen, MSG_DONTWAIT);
    if (written < 0) {
      bool fatal = errno == EPIPE || errno == ECONNRESET || errno == ENOTCONN ||
                   errno == ECONNABORTED || errno == EBADF;
      if (fatal) {
        closeTelnetSession();
      }
      return;
    }
    if (written == 0) return;
    telnetTxBuffer.consume(static_cast<size_t>(written));
    budget -= static_cast<size_t>(written);
    lastTcpActivity = millis();
  }
}

class TelnetBufferedPrint : public Print {
public:
  size_t write(uint8_t value) override {
    return write(&value, 1);
  }

  size_t write(const uint8_t* data, size_t len) override {
    if (!telnetSessionActive || !data || len == 0) return 0;
    size_t required = len;
    for (size_t i = 0; i < len; ++i) {
      if (data[i] == terminal::TELNET_IAC) ++required;
    }
    pumpTelnetTx();
    if (required > telnetTxBuffer.freeSpace()) {
      stat_telnet_tx_rejected.fetch_add(len, std::memory_order_relaxed);
      return 0;
    }
    for (size_t i = 0; i < len; ++i) {
      terminal::enqueueTelnetData(telnetTxBuffer, data[i]);
    }
    return len;
  }
};

static TelnetBufferedPrint telnetOut;

enum class TelnetInputError {
  None,
  TargetDisconnected,
  QueueFull,
};

static TelnetInputError telnetInputError = TelnetInputError::None;

static bool sendTelnetTargetByte(uint8_t value, bool targetReady) {
  bool sent = targetReady && writeToTarget(value);
  if (sent) {
    telnetInputError = TelnetInputError::None;
    return true;
  }

  TelnetInputError error = targetReady
                             ? TelnetInputError::QueueFull
                             : TelnetInputError::TargetDisconnected;
  if (error != telnetInputError) {
    telnetOut.print(error == TelnetInputError::TargetDisconnected
                      ? "\r\n[USB device not connected - input not sent]\r\n> "
                      : "\r\n[Target TX queue full - input not sent]\r\n> ");
    telnetInputError = error;
  }
  return false;
}

static void setTelnetLocalEcho(bool enabled) {
  localEcho = enabled;
  if (!telnetSessionActive || telnetTxBuffer.freeSpace() < 3) return;
  // Der Telnet-Client darf nie selbst lokal echoen: Bei localEcho=false kommt
  // das Echo vom seriellen Ziel und wird zurueckgeleitet, bei true von ConNect.
  const uint8_t negotiation[] = {
    terminal::TELNET_IAC,
    terminal::TELNET_WILL,
    1,
  };
  telnetTxBuffer.push(negotiation, sizeof(negotiation));
}

static void handleTelnetNegotiation(const terminal::TelnetEvent& event) {
  constexpr uint8_t OPTION_BINARY = 0;
  constexpr uint8_t OPTION_ECHO = 1;
  constexpr uint8_t OPTION_SUPPRESS_GO_AHEAD = 3;
  uint8_t responseCommand = 0;

  if (event.command == terminal::TELNET_DO &&
      event.option != OPTION_BINARY && event.option != OPTION_ECHO &&
      event.option != OPTION_SUPPRESS_GO_AHEAD) {
    responseCommand = terminal::TELNET_WONT;
  } else if (event.command == terminal::TELNET_WILL &&
             event.option != OPTION_BINARY && event.option != OPTION_SUPPRESS_GO_AHEAD) {
    responseCommand = terminal::TELNET_DONT;
  }

  if (responseCommand != 0) {
    const uint8_t response[] = {terminal::TELNET_IAC, responseCommand, event.option};
    pumpTelnetTx();
    telnetTxBuffer.push(response, sizeof(response));
  }
}

static inline uint16_t bleChunkLen(){
  uint16_t mtu = NimBLEDevice::getMTU();
  if (mtu < 23) mtu = 23;
  uint16_t n = mtu - 3;        // ATT-Overhead
  if (n > 244) n = 244;        // passend zu setMTU(247)
  return n;
}

static void pumpBleTx(bool force = false) {
  if (bleTxResetRequested.exchange(false, std::memory_order_relaxed)) {
    if (!bleTxBuffer.empty()) {
      stat_ble_tx_rejected.fetch_add(bleTxBuffer.size(), std::memory_order_relaxed);
    }
    bleTxBuffer.clear();
    bleTxFlushRequested = false;
  }
  if (!deviceConnected.load() || !bleNotificationsEnabled.load() || !pTxCharacteristic) {
    if (!bleTxBuffer.empty()) {
      stat_ble_tx_rejected.fetch_add(bleTxBuffer.size(), std::memory_order_relaxed);
    }
    bleTxBuffer.clear();
    bleTxFlushRequested = false;
    return;
  }
  if (bleTxBuffer.empty()) return;

  uint16_t maxChunk = bleChunkLen();
  bool idleDue = millis() - bleTxLastEnqueueMs >= Timing::BLE_PACKET_IDLE_MS;
  if (!force && !bleTxFlushRequested && !idleDue && bleTxBuffer.size() < maxChunk) {
    return;
  }

  size_t packetsLeft = 4;
  while (!bleTxBuffer.empty() && packetsLeft-- > 0) {
    size_t chunkLen = min(bleTxBuffer.contiguousSize(), static_cast<size_t>(maxChunk));
    if (!pTxCharacteristic->notify(bleTxBuffer.frontData(), chunkLen)) {
      return;
    }
    bleTxBuffer.consume(chunkLen);
    if (!force && bleTxBuffer.size() < maxChunk) {
      break;
    }
  }
  if (bleTxBuffer.empty()) {
    bleTxFlushRequested = false;
  }
}

static bool bleOutputReady() {
  return bleNotificationsEnabled.load(std::memory_order_relaxed) &&
         !bleTxResetRequested.load(std::memory_order_relaxed) &&
         pTxCharacteristic;
}

static inline bool bleSend(const uint8_t* data, size_t len){
  if (!deviceConnected.load() || !bleOutputReady() || !data || len == 0) return false;
  pumpBleTx(true);
  if (!bleTxBuffer.push(data, len)) {
    stat_ble_tx_rejected.fetch_add(len, std::memory_order_relaxed);
    return false;
  }
  bleTxLastEnqueueMs = millis();
  bleTxFlushRequested = true;
  return true;
}

static inline void bleNotifyLine(const String& line) {
  String t = line + "\r\n";
  webConsolePushRxBulk(t.c_str(), t.length());
  if (bleOutputReady()) {
    bleSend((const uint8_t*)t.c_str(), t.length());
  }
}

static inline bool bleAppendFromDevice(uint8_t value){
  if (!bleOutputReady()) return true;
  if (!bleTxBuffer.push(value)) return false;
  bleTxLastEnqueueMs = millis();
  if (value == '\r' || value == '\n') {
    bleTxFlushRequested = true;
  }
  return true;
}

static inline void bleFlushLineIdle(){
  pumpBleTx();
}

static bool terminalOutputsCanAcceptByte() {
  if (telnetSessionActive &&
      telnetTxBuffer.freeSpace() <= Timing::TELNET_CONTROL_RESERVE) {
    return false;
  }
  if (bleOutputReady() &&
      bleTxBuffer.freeSpace() <= Timing::BLE_CONTROL_RESERVE) {
    return false;
  }
  return true;
}
// RGB LED (1x NeoPixel) status logic
//  - BLE advertising (no client): slow red pulse
//  - BLE connected: solid red
//  - WiFi AP active, no Telnet clients: slow blue pulse
//  - Telnet client connected: solid cyan
//  - Activity (RX/TX): short white pulse overlay
// -----------------------------------------------------------------------------
static Adafruit_NeoPixel g_rgb(1, RGB_LED_PIN, NEO_GRB + NEO_KHZ800);
static bool g_rgbInit=false; 
static uint32_t g_rgbPulseUntil=0; 
static uint32_t g_rgbLastBlink=0; 
static bool g_rgbBlink=false;
static inline void rgbBegin(){ if(!g_rgbInit){ g_rgb.begin(); g_rgb.setBrightness(RGB_LED_BRIGHTNESS); g_rgb.clear(); g_rgb.show(); g_rgbInit=true;} }
static inline void rgbShow(uint8_t r,uint8_t g,uint8_t b){ if(!g_rgbInit) rgbBegin(); g_rgb.setPixelColor(0,g_rgb.Color(r,g,b)); g_rgb.show(); }
static inline void rgbPulseActivity(uint16_t ms){ uint32_t until=millis()+ms; if(until>g_rgbPulseUntil) g_rgbPulseUntil=until; }
static inline void rgbUpdate(){
  if(!g_rgbInit) rgbBegin();
  uint32_t now=millis(); 
  if(now-g_rgbLastBlink>=500){ g_rgbLastBlink=now; g_rgbBlink=!g_rgbBlink; }

  bool bleConn = deviceConnected.load();
  bool bleAdv = bleAdvertising.load();
  bool wifiAP = (WiFi.getMode() & WIFI_MODE_AP) && (WiFi.softAPIP() != IPAddress(0,0,0,0));
  bool wifiClient = isWiFiClientConnected();
  bool telnetConn = telnetSessionActive;

  uint8_t r=0,g=0,b=0;
  // Base layers
  if (wifiAP || wifiClient) {
    // WiFi AP active: slow blue pulse without Telnet, solid cyan with client
    if (telnetConn) { b = 200; g = 120; } // cyan
    else { b = g_rgbBlink ? 180 : 10; }
  }
  if (bleConn) {
    r = 220;
  } else if (bleAdv) {
    r = g_rgbBlink ? 180 : 10;
  }

  // Activity pulse overlay (short white flash)
  if((int32_t)(g_rgbPulseUntil-now)>0){ r=255; g=255; b=255; }
  rgbShow(r,g,b);
}

static void enterCriticalBatterySleep() {
  String msg = "Akku kritisch - Sleep\r\n";
  webConsolePushRxBulk(msg.c_str(), msg.length());
  if (telnetSessionActive) {
    telnetOut.print(msg);
    pumpTelnetTx();
  }
  if (bleNotificationsEnabled.load() && pTxCharacteristic) bleNotifyLine("Akku kritisch - Sleep");

  display.showMessage("Akku leer", "Sleep", "Laden", 1000);
  delay(300);

  if (telnetSessionActive) closeTelnetSession();
  if (NimBLEDevice::isInitialized()) {
    if (pServer) pServer->setCallbacks(nullptr, false);
    NimBLEDevice::deinit(true);
    bleAdvertising = false;
    deviceConnected = false;
    pServer = nullptr;
    pTxCharacteristic = nullptr;
  }
  WiFi.softAPdisconnect(true);
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);

  digitalWrite(MAX3232_EN_PIN, LOW);
  rgbShow(0, 0, 0);
  display.clear();
  display.refresh();

  esp_sleep_enable_timer_wakeup(PowerConfig::CRITICAL_SLEEP_US);
  delay(50);
  esp_deep_sleep_start();
  while (true) delay(1000);
}

// -----------------------------------------------------------------------------
// Help
// -----------------------------------------------------------------------------
static void printHelp(Print* out){
  static const char* lines[] = {
    "=== ConNect Commands ===",
    "Prefix: ` oder '",
    "",
    "`usb / `rs232  - Ziel wechseln",
    "`baud N        - Baudrate",
    "`status        - Systemstatus",
    "`power         - Boot/Power-Diagnose",
    "`heap          - Speicherinfo",
    "`version       - FW-Version",
    "`wlan SSID PW  - WLAN + OTA",
    "`otaurl URL    - OTA Manifest-Link",
    "`ota           - OTA starten",
    "`coredump      - letzten Crash anzeigen",
    "`coredump erase- Crashlog loeschen",
    "`pins          - GPIO State (Diagnose)",
    "`en high/low   - MAX3232_EN Diagnose",
    "`loopback      - RS232 TX->RX Test",
    "`echo on/off   - lokales Echo (Telnet)",
    "",
    "Taste:",
    "1x = USB/RS232",
    "USB-Start ggf. mit Neustart",
    "2x = Baudrate rotieren",
    "3x = Hotspot/WLAN",
    "1-5s = BLE an/aus",
    ">5s + kurz = OTA"
  };
  for (const char* line : lines) {
    if (out) {
      out->print(line);
      out->print("\r\n");
    }
    bleNotifyLine(line);
  }
}

// -----------------------------------------------------------------------------
// BLE
// -----------------------------------------------------------------------------
// Buzzer Funktion aus buzzer.h/cpp

class ServerCallbacks: public NimBLEServerCallbacks {
  void onConnect(NimBLEServer*, NimBLEConnInfo&) override { 
    bleTxResetRequested.store(true, std::memory_order_relaxed);
    bleNotificationsEnabled.store(false, std::memory_order_relaxed);
    deviceConnected = true; 
    bleAdvertising = false;
    beepPattern(2, 1500, 50, 50);  // 2x kurzer hoher Ton bei Connect
  }
  void onDisconnect(NimBLEServer*, NimBLEConnInfo&, int) override {
    deviceConnected = false;
    bleNotificationsEnabled.store(false, std::memory_order_relaxed);
    bleTxResetRequested.store(true, std::memory_order_relaxed);
    beepPattern(1, 800, 150, 0);   // 1x tiefer Ton bei Disconnect
    NimBLEAdvertising* adv = NimBLEDevice::getAdvertising();
    if (adv && adv->start()) { bleAdvertising = true; }
  }
};

class TxCallbacks : public NimBLECharacteristicCallbacks {
  void onSubscribe(NimBLECharacteristic*, NimBLEConnInfo&, uint16_t subValue) override {
    bool enabled = (subValue & 0x01) != 0;
    bool wasEnabled = bleNotificationsEnabled.load(std::memory_order_relaxed);
    if (enabled != wasEnabled) {
      bleTxResetRequested.store(true, std::memory_order_relaxed);
      bleNotificationsEnabled.store(enabled, std::memory_order_relaxed);
    }
  }
};

class RxCallbacks : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic* c, NimBLEConnInfo&) override {
    std::string val = c->getValue();
    size_t queueBytes = val.size();
    if (!bleRxQueue || uxQueueSpacesAvailable(bleRxQueue) < queueBytes) {
      stat_ble_rx_dropped.fetch_add(queueBytes, std::memory_order_relaxed);
      return;
    }

    for (size_t i = 0; i < val.size(); ++i) {
      enqueueBleRxByte(static_cast<uint8_t>(val[i]));
    }
  }
};


// Statische Callback-Instanzen (verhindert Memory Leak bei BLE-Neustart)
static ServerCallbacks serverCallbacksInstance;
static RxCallbacks rxCallbacksInstance;
static TxCallbacks txCallbacksInstance;

static void setupBLE(){
  NimBLEDevice::init("ConNect");
  NimBLEDevice::setPower(ESP_PWR_LVL_P9);
  NimBLEDevice::setMTU(247);

  pServer = NimBLEDevice::createServer();
  if (!pServer) return;
  pServer->setCallbacks(&serverCallbacksInstance);

  NimBLEService* pService = pServer->createService(SERVICE_UUID);
  if (!pService) return;

  pTxCharacteristic = pService->createCharacteristic(CHARACTERISTIC_UUID_TX, NIMBLE_PROPERTY::NOTIFY);
  if (!pTxCharacteristic) return;
  pTxCharacteristic->setCallbacks(&txCallbacksInstance);

  NimBLECharacteristic* pRx = pService->createCharacteristic(
      CHARACTERISTIC_UUID_RX, NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR);
  if (!pRx) return;
  pRx->setCallbacks(&rxCallbacksInstance);

  if (!pServer->start()) return;

  NimBLEAdvertising* adv = NimBLEDevice::getAdvertising();
  if (!adv) return;
  adv->addServiceUUID(SERVICE_UUID);
  if (!adv->start()) {
    display.showMessage("BLE", "Adv-Start", "FEHLER", 3000);
    return;
  }
  bleAdvertising = true;
}

// Forward declaration
static void setupWiFi();
static void setupBLE();
static void stopMdns();

// -----------------------------------------------------------------------------
// HTTP OTA Update (Server-Manifest)
// -----------------------------------------------------------------------------
static bool bleWasAdvertising = false;  // Merken ob BLE vorher aktiv war

static bool otaConnectToWiFi() {
  String ssid = prefs.getString("otaSSID", "");
  String pass = prefs.getString("otaPass", "");
  
  if (ssid.length() == 0) {
    display.showMessage("Kein WLAN", "Konfiguriere via", "192.168.32.1");
    delay(Timing::MESSAGE_DISPLAY_MS);
    return false;
  }
  
  display.showMessage("OTA Update", "Verbinde WiFi...", ssid.c_str());

  // BLE komplett deinitialisieren - nicht nur Advertising stoppen.
  // mbedTLS-Handshake braucht ~30-50 KB zusammenhaengenden Heap. Mit
  // aktivem NimBLE-Stack stehen typ. nur ~30 KB groesster Block bereit
  // -> Handshake schlaegt mit "connection refused" (-1) fehl.
  // Deinit gibt ~40 KB frei. Nach OTA wird BLE in otaRestoreAP() neu
  // aufgebaut.
  bleWasAdvertising = bleAdvertising;
  if (NimBLEDevice::isInitialized()) {
    // ServerCallbacks ist ein STATISCHES Objekt (.bss). NimBLEServer ruft
    // im Destruktor per Default `delete m_pCallbacks` -> assert in heap_caps_free
    // ("free target outside heap areas"). Daher Callbacks vorher abmelden mit
    // deleteCallbacks=false.
    if (pServer) pServer->setCallbacks(nullptr, false);
    NimBLEDevice::deinit(true);  // true = clear data
    bleAdvertising = false;
    deviceConnected = false;
    pServer = nullptr;
    pTxCharacteristic = nullptr;
  }
  delay(100);
  
  // AP stoppen
  stopMdns();
  WiFi.softAPdisconnect(true);
  WiFi.mode(WIFI_OFF);
  delay(200);
  
  // Station-Modus mit Modem Sleep (WICHTIG für BLE Koexistenz!)
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(WIFI_PS_MIN_MODEM);  // Modem sleep aktivieren
  delay(50);
  WiFi.begin(ssid.c_str(), pass.c_str());
  
  unsigned long startTime = millis();
  while (WiFi.status() != WL_CONNECTED) {
    if (millis() - startTime > Timing::OTA_WIFI_TIMEOUT_MS) {
      display.showMessage("WiFi Fehler", "Timeout", "SSID/Passwort ok?");
      delay(Timing::MESSAGE_DISPLAY_MS);
      return false;
    }
    esp_task_wdt_reset();
    delay(250);
  }
  
  display.showMessage("WiFi OK", WiFi.localIP().toString().c_str());
  delay(500);
  return true;
}

static void otaRestoreAP() {
  // Nach OTA (Erfolg oder Fehler) ist der State zu fragil fuer einen
  // sauberen Restore: WiFi war STA, BLE wurde deinit, Heap fragmentiert,
  // Update.cpp hat ggf. die Bootloader-Partitionstabelle angefasst. Ein
  // STA->OFF->AP Switch direkt nach NimBLE-Reinit hat reproduzierbar in
  // ieee80211_hostap_attach gepanic't. Reboot ist hier robust und
  // benutzerfreundlich (Display/BLE/Web sind nach <2 s wieder verfuegbar).
  display.showMessage("OTA", "Neustart...", "", 1500);
  delay(1500);
  ESP.restart();
}

class ScopedLoopWatchdogPause {
public:
  ScopedLoopWatchdogPause()
      : paused(esp_task_wdt_delete(nullptr) == ESP_OK) {}

  ~ScopedLoopWatchdogPause() {
    if (paused) esp_task_wdt_add(nullptr);
  }

private:
  bool paused;
};

static void performHTTPOTA() {
  String manifestUrl = getConfiguredOTAManifestUrl();

  if (manifestUrl.length() == 0) {
    display.showMessage("OTA Fehler", "Link fehlt", "192.168.32.1");
    delay(3000);
    return;
  }

  if (!otaConnectToWiFi()) {
    otaRestoreAP();
    return;
  }

  // Host aus URL extrahieren und DNS aufloesen, bevor wir HTTPClient bemuehen.
  // So trennen wir DNS-Fehler sauber von TCP/TLS-Fehlern.
  int schemeEnd = manifestUrl.indexOf("://");
  if (schemeEnd > 0) {
    int hostStart = schemeEnd + 3;
    int hostEnd = manifestUrl.indexOf('/', hostStart);
    if (hostEnd < 0) hostEnd = manifestUrl.length();
    int portSep = manifestUrl.indexOf(':', hostStart);
    if (portSep > 0 && portSep < hostEnd) hostEnd = portSep;
    String host = manifestUrl.substring(hostStart, hostEnd);
    IPAddress resolved;
    if (!WiFi.hostByName(host.c_str(), resolved)) {
      display.showMessage("Update Fehler", "DNS-Fehler", host.c_str());
      delay(4000);
      otaRestoreAP();
      return;
    }
  }

  display.showMessage("OTA Update", "Lade Manifest...");
  delay(200);

  // Manifest in eigenem Block: secureClient + http muessen ZERSTOERT sein,
  // bevor der Download seinen TLS-Handshake versucht. Sonst halten beide
  // gleichzeitig mbedTLS-Buffer (>30 KB) und der Download bekommt -1.
  String payload;
  {
    bool isHttps = manifestUrl.startsWith("https://");
    WiFiClient       basicClient;
    WiFiClientSecure secureClient;
    if (isHttps) {
      secureClient.setInsecure();
      secureClient.setTimeout(15);
      secureClient.setHandshakeTimeout(15);
    } else {
      basicClient.setTimeout(15);
    }

    HTTPClient http;
    bool began = isHttps ? http.begin(secureClient, manifestUrl)
                         : http.begin(basicClient, manifestUrl);
    if (!began) {
      display.showMessage("Update Fehler", "URL ungueltig", "");
      delay(3000);
      otaRestoreAP();
      return;
    }
    http.addHeader("Accept", "application/json");
    http.addHeader("User-Agent", "ConNect-OTA/1.0");
    http.setFollowRedirects(HTTPC_FORCE_FOLLOW_REDIRECTS);
    http.setTimeout(15000);

    int httpCode = http.GET();
    if (httpCode != HTTP_CODE_OK) {
      const char* hint = "";
      if (httpCode == 404) hint = "Manifest fehlt";
      else if (httpCode == HTTPC_ERROR_CONNECTION_REFUSED) hint = "Verbindung abgelehnt";
      else if (httpCode == HTTPC_ERROR_SEND_HEADER_FAILED ||
               httpCode == HTTPC_ERROR_SEND_PAYLOAD_FAILED) hint = "Sende-Fehler";
      else if (httpCode == HTTPC_ERROR_NOT_CONNECTED) hint = "Nicht verbunden";
      else if (httpCode == HTTPC_ERROR_CONNECTION_LOST) hint = "Verb. verloren";
      else if (httpCode == HTTPC_ERROR_READ_TIMEOUT) hint = "Lese-Timeout";
      else if (httpCode < 0) hint = "Netzwerk-Fehler";
      char errBuf[24];
      snprintf(errBuf, sizeof(errBuf), "Code: %d", httpCode);
      display.showMessage("Update Fehler", hint[0] ? hint : errBuf, hint[0] ? errBuf : "");
      http.end();
      delay(4000);
      otaRestoreAP();
      return;
    }

    payload = http.getString();
    http.end();
  } // <- hier sterben secureClient/basicClient/http -> mbedTLS-Heap frei
  
  String tagName = extractJsonStringField(payload, "version");
  if (tagName.length() == 0) {
    tagName = extractJsonStringField(payload, "tag");
  }

  String firmwareLocation = extractJsonStringField(payload, "firmware");
  if (firmwareLocation.length() == 0) {
    firmwareLocation = extractJsonStringField(payload, "url");
  }

  String firmwareMd5 = extractJsonStringField(payload, "md5");
  firmwareMd5.trim();
  if (firmwareMd5.length() > 0 && !isValidMD5(firmwareMd5)) {
    display.showMessage("Update Fehler", "Manifest MD5", "ungueltig");
    delay(4000);
    otaRestoreAP();
    return;
  }

  String downloadUrl = buildAbsoluteUrl(manifestUrl, firmwareLocation);
  if (downloadUrl.length() == 0) {
    display.showMessage("Kein Update", "Firmware-Link fehlt", "im Manifest");
    delay(4000);
    otaRestoreAP();
    return;
  }

  if (tagName.length() == 0) {
    tagName = OTAConfig::FIRMWARE_FILENAME;
  }
  
  // Versions-Prüfung: Vergleiche mit aktueller Firmware
  // Problem: GIT_VERSION wird zur Compile-Zeit gesetzt. Nach OTA enthält die neue
  // Firmware möglicherweise eine alte Version wenn sie nicht exakt auf dem Tag gebaut wurde.
  // Lösung: Nach erfolgreichem OTA speichern wir die Tag-Version in NVS.
  
  // Prüfe ob wir eine gespeicherte OTA-Version haben (diese hat Vorrang)
  String installedVersion = prefs.getString("otaVersion", "");
  String currentVersion = installedVersion.length() > 0 ? installedVersion : String(GIT_VERSION);

  // Normalisiere Versionen (entferne führendes 'v' wenn vorhanden)
  String remoteVer = tagName;
  if (remoteVer.startsWith("v") || remoteVer.startsWith("V")) remoteVer = remoteVer.substring(1);
  String localVer = currentVersion;
  if (localVer.startsWith("v") || localVer.startsWith("V")) localVer = localVer.substring(1);
  
  // Extrahiere Base-Version aus lokalem String (z.B. "1.0.6" aus "1.0.6-2-g1234567")
  // Format: TAG[-COMMITS-gHASH][-dirty]
  String localBaseVer = localVer;
  int dashIdx = localVer.indexOf('-');
  if (dashIdx > 0) {
    localBaseVer = localVer.substring(0, dashIdx);
  }

  // Prüfe ob gleiche Version
  // - dirty: immer Update erlauben (lokale Änderungen)
  // - dev/unknown/nur Hash: immer Update erlauben (kein Git Tag)
  // - Gleiche Base-Version (z.B. 1.0.6 == 1.0.6): bereits aktuell
  bool isDirty = currentVersion.indexOf("dirty") >= 0;
  bool isDevBuild = currentVersion == "dev" || currentVersion == "unknown" || currentVersion.length() < 3;
  // Prüfe ob Version nur ein Commit-Hash ist (keine Punkte = kein semver Tag)
  bool isHashOnly = localBaseVer.indexOf('.') < 0;
  bool sameBaseVersion = (remoteVer == localBaseVer);
  
  // Nur "bereits aktuell" wenn: exakt gleiche Base-Version UND kein dirty UND kein dev/hash-only Build
  bool skipUpdate = sameBaseVersion && !isDirty && !isDevBuild && !isHashOnly;

  if (skipUpdate) {
    display.showMessage("OTA", "Bereits aktuell", tagName.c_str(), 3000);
    delay(3000);
    otaRestoreAP();
    return;
  }
  
  display.showMessage("Update:", (tagName + " -> " + currentVersion.substring(0,10)).c_str(), "Lade...");
  delay(1500);

  // Download-Client: identisches Stack-Muster wie beim Manifest, damit
  // HTTPClient garantiert vor dem WiFiClient zerstoert wird.
  bool dlHttps = downloadUrl.startsWith("https://");
  WiFiClient       dlBasic;
  WiFiClientSecure dlSecure;
  if (dlHttps) {
    dlSecure.setInsecure();
    dlSecure.setTimeout(60);
    dlSecure.setHandshakeTimeout(15);
  } else {
    dlBasic.setTimeout(60);
  }

  HTTPClient dlHttp;
  bool dlBegan = dlHttps ? dlHttp.begin(dlSecure, downloadUrl)
                         : dlHttp.begin(dlBasic, downloadUrl);
  if (!dlBegan) {
    display.showMessage("Update Fehler", "Download-URL", "ungueltig");
    delay(3000);
    otaRestoreAP();
    return;
  }
  dlHttp.addHeader("Accept", "application/octet-stream");
  dlHttp.addHeader("User-Agent", "ConNect-OTA/1.0");
  dlHttp.setFollowRedirects(HTTPC_FORCE_FOLLOW_REDIRECTS);
  dlHttp.setTimeout(60000);

  int dlCode = dlHttp.GET();
  if (dlCode != HTTP_CODE_OK) {
    char errBuf[32];
    snprintf(errBuf, sizeof(errBuf), "Download: %d", dlCode);
    display.showMessage("Update Fehler", errBuf, "Nochmal versuchen");
    dlHttp.end();
    delay(4000);
    otaRestoreAP();
    return;
  }

  int contentLength = dlHttp.getSize();
  bool hasKnownSize = contentLength > 0;

  display.showMessage("Update", "Starte Flash...", hasKnownSize ? (String(contentLength / 1024) + " KB").c_str() : "Stream");
  delay(500);

  // Stream für Update holen
  WiFiClient* stream = dlHttp.getStreamPtr();
  if (!stream) {
    display.showMessage("Update Fehler", "Kein Stream", "");
    dlHttp.end();
    delay(3000);
    otaRestoreAP();
    return;
  }

  // Update.begin loescht die Inactive-Partition. Bei UPDATE_SIZE_UNKNOWN
  // nimmt die ESP-Update-Lib die volle Partitionsgroesse an.
  if (!Update.begin(hasKnownSize ? contentLength : UPDATE_SIZE_UNKNOWN)) {
    display.showMessage("Update Fehler", "Begin fehlg.", Update.errorString());
    dlHttp.end();
    delay(4000);
    otaRestoreAP();
    return;
  }

  if (firmwareMd5.length() > 0 && !Update.setMD5(firmwareMd5.c_str())) {
    display.showMessage("Update Fehler", "Manifest MD5", "abgelehnt");
    Update.abort();
    dlHttp.end();
    delay(4000);
    otaRestoreAP();
    return;
  }

  // writeStream ist die offizielle Variante: blockiert bis EOF / Fehler,
  // ruft intern Update.write() in passenden Bloecken. Flash-Schreibvorgaenge
  // koennen laenger als der loopTask-Watchdog dauern, daher wird nur dessen
  // Subscription fuer diesen Scope pausiert und auf Fehlerpfaden restauriert.
  ScopedLoopWatchdogPause watchdogPause;
  size_t bytesWritten = Update.writeStream(*stream);

  if (Update.hasError()) {
    display.showMessage("Update Fehler", "Schreiben fehlg.", Update.errorString());
    Update.abort();
    dlHttp.end();
    delay(4000);
    otaRestoreAP();
    return;
  }

  bool lengthMismatch = hasKnownSize && bytesWritten != static_cast<size_t>(contentLength);
  if (bytesWritten == 0 || lengthMismatch) {
    char progressBuf[32];
    if (hasKnownSize) {
      snprintf(progressBuf, sizeof(progressBuf), "%u/%u Bytes",
               (unsigned)bytesWritten, (unsigned)contentLength);
    } else {
      snprintf(progressBuf, sizeof(progressBuf), "Keine Daten");
    }
    display.showMessage("Update Fehler", "Download unvollst.", progressBuf);
    Update.abort();
    dlHttp.end();
    delay(4000);
    otaRestoreAP();
    return;
  }

  dlHttp.end();

  if (!Update.end(true)) {
    display.showMessage("Update Fehler", "Abschluss fehlg.", Update.errorString());
    delay(4000);
    otaRestoreAP();
    return;
  }
  
  // Update erfolgreich - Reihenfolge wichtig:
  //  1) Tag-Version in NVS speichern (fuer kuenftige Versionsvergleiche)
  //  2) Preferences explizit schliessen -> erzwingt NVS-Commit auf Flash
  //  3) Alten Coredump verwerfen: er gehoert zur alten Firmware (andere
  //     ELF-SHA256), Backtraces waeren sonst irrefuehrend.
  //  4) Reboot.
  prefs.putString("otaVersion", tagName);
  prefs.end();
  esp_core_dump_image_erase();

  display.showMessage("OTA OK", tagName.c_str(), "Neustart...");
  delay(Timing::MESSAGE_DISPLAY_MS);
  ESP.restart();
}

// -----------------------------------------------------------------------------
// WiFi
// -----------------------------------------------------------------------------
static bool startSoftAPWithRetry(IPAddress ip, IPAddress gw, IPAddress mask) {
  if (!WiFi.softAPConfig(ip, gw, mask)) return false;
  for (int i=0;i<3;i++) {
    if (WiFi.softAP(WIFI_SSID, WIFI_PASSWORD, WIFI_CHANNEL, false, 4)) return true;
    delay(200);
  }
  return false;
}

static void stopMdns() {
  if (mdnsRunning) {
    MDNS.end();
    mdnsRunning = false;
  }
  lastMdnsAttempt = 0;
}

static void startMdnsForClient() {
  if (WiFi.status() != WL_CONNECTED || mdnsRunning) return;
  if (lastMdnsAttempt != 0 && millis() - lastMdnsAttempt < 10000) return;
  lastMdnsAttempt = millis();
  if (MDNS.begin(WiFiConfig::MDNS_HOSTNAME)) {
    MDNS.setInstanceName("ConNect Wireless Serial Bridge");
    MDNS.addService("http", "tcp", 80);
    MDNS.addService("telnet", "tcp", 23);
    mdnsRunning = true;
  } else {
    MDNS.end();
  }
}

static bool startAccessPoint(bool fallback) {
  WiFi.mode(WIFI_OFF);
  delay(100);
  WiFi.mode(WIFI_AP);
  delay(100);
  WiFi.setSleep(false);
  WiFi.setTxPower(WIFI_POWER_15dBm);

  IPAddress localIp;
  localIp.fromString(WIFI_IP);
  IPAddress gateway(192, 168, 32, 1);
  IPAddress subnet(255, 255, 255, 0);
  if (!startSoftAPWithRetry(localIp, gateway, subnet)) return false;

  delay(300);
  if (WiFi.softAPIP() == IPAddress(0, 0, 0, 0)) return false;
  wifiFallbackAccessPoint = fallback;
  return true;
}

static bool startWiFiClient() {
  String ssid = prefs.getString("otaSSID", "");
  String password = prefs.getString("otaPass", "");
  ssid.trim();
  if (ssid.length() == 0) return false;

  WiFi.mode(WIFI_OFF);
  delay(100);
  WiFi.mode(WIFI_STA);
  delay(100);
  WiFi.setHostname(WiFiConfig::MDNS_HOSTNAME);
  WiFi.setSleep(WIFI_PS_MIN_MODEM);
  WiFi.setTxPower(WIFI_POWER_15dBm);
  WiFi.setAutoReconnect(true);
  WiFi.begin(ssid.c_str(), password.c_str());

  uint32_t startedAt = millis();
  while (WiFi.status() != WL_CONNECTED &&
         millis() - startedAt < WiFiConfig::CLIENT_CONNECT_TIMEOUT_MS) {
    delay(100);
  }
  if (WiFi.status() != WL_CONNECTED) return false;

  wifiFallbackAccessPoint = false;
  startMdnsForClient();
  return true;
}

static void setupWiFi(){
  stopMdns();
  bool started = configuredWiFiMode == WiFiConfig::Mode::Client
                   ? startWiFiClient()
                   : startAccessPoint(false);
  if (!started && configuredWiFiMode == WiFiConfig::Mode::Client) {
    started = startAccessPoint(true);
    if (started) {
      display.showMessage("WLAN Fehler", "Hotspot aktiv", WIFI_IP, 3000);
    }
  }
  if (!started) {
    display.showMessage("WiFi", "Start fehlgeschl.", "FEHLER", 3000);
    bleNotifyLine("WiFi: Start fehlgeschlagen");
    WiFi.mode(WIFI_OFF);
    return;
  }
  tcpServer.begin();
}

static void serviceWiFi() {
  if (configuredWiFiMode != WiFiConfig::Mode::Client || wifiFallbackAccessPoint) return;

  static uint32_t lastReconnectAttempt = 0;
  if (WiFi.status() == WL_CONNECTED) {
    startMdnsForClient();
    return;
  }

  stopMdns();
  if (millis() - lastReconnectAttempt >= 10000) {
    lastReconnectAttempt = millis();
    WiFi.reconnect();
  }
}

const char* getConfiguredWiFiModeValue() {
  return configuredWiFiMode == WiFiConfig::Mode::Client ? "CLIENT" : "AP";
}

const char* getActiveWiFiModeLabel() {
  if (wifiFallbackAccessPoint) return "Hotspot (Fallback)";
  return configuredWiFiMode == WiFiConfig::Mode::Client ? "WLAN-Client" : "Hotspot";
}

bool isWiFiClientConnected() {
  return configuredWiFiMode == WiFiConfig::Mode::Client &&
         !wifiFallbackAccessPoint && WiFi.status() == WL_CONNECTED;
}

bool isWiFiFallbackAccessPoint() {
  return wifiFallbackAccessPoint;
}

String getWiFiIpAddress() {
  if (configuredWiFiMode == WiFiConfig::Mode::Client && !wifiFallbackAccessPoint) {
    return isWiFiClientConnected() ? WiFi.localIP().toString() : "0.0.0.0";
  }
  return WiFi.softAPIP().toString();
}

String getWiFiHostname() {
  return isWiFiClientConnected() && mdnsRunning ? String(WiFiConfig::MDNS_HOSTNAME) + ".local" : "";
}

// -----------------------------------------------------------------------------
// Commands
// -----------------------------------------------------------------------------
static void printBaudOptions(Print* out){
  String msg = "Available baud rates:\r\n";
  if (out) out->print("Available baud rates:\r\n");
  for (int i = 0; i < BaudConfig::ALL_RATES_COUNT; i++){
    String ln = String("  ") + (i+1) + ": " + String(BaudConfig::ALL_RATES[i]);
    if (out) out->print((ln + "\r\n").c_str());
    msg += ln;
    msg += "\r\n";
  }
  webConsolePushRxBulk(msg.c_str(), msg.length());
  bleSend((const uint8_t*)msg.c_str(), msg.length());
}

static bool isInternalCommand(const String& command) {
  String value = command;
  value.trim();
  value.toLowerCase();
  while (value.startsWith("::")) value.remove(0, 2);
  if (value.startsWith(":")) value.remove(0, 1);

  return value == "usb" || value == "rs232" ||
         value == "echo" || value == "echo on" || value == "echo off" ||
         value == "baud" || value.startsWith("baud ") ||
         value == "pins" || value == "en high" || value == "en low" ||
         value == "loopback" || value == "status" || value == "power" ||
         value == "heap" || value.startsWith("wlan ") ||
         value.startsWith("otaurl ") || value == "ota" ||
         value == "version" || value == "help" ||
         value == "coredump" || value == "coredump erase";
}

static void handleInternalCommand(const String& cmd){
  String s = cmd; s.trim(); s.toLowerCase();
  while (s.startsWith("::")) s.remove(0,2);
  if (s.length() && s.charAt(0)==':') s.remove(0,1);

  Print* out = telnetSessionActive ? &telnetOut : nullptr;
  auto sendLine = [&](const String& line){ if(out){ out->print(line); out->print("\r\n"); } bleNotifyLine(line); };

  if (s=="usb"){ setSerialTargetMode(true); prefs.putBool("usbMode", true); doBeep(); if(out) out->print("Switched to USB\r\n"); bleNotifyLine("Switched to USB"); }
  else if (s=="rs232"){ setSerialTargetMode(false); delay(20); rs232ReinitUart(); setSerialTargetMode(false); prefs.putBool("usbMode", false); doBeep(); if(out) out->print("Switched to RS232\r\n"); bleNotifyLine("Switched to RS232"); }
  else if (s=="echo on")  { setTelnetLocalEcho(true);  prefs.putBool("telnetEcho", true);  if(out) out->print("Local echo: ON\r\n");  bleNotifyLine("Local echo: ON"); }
  else if (s=="echo off") { setTelnetLocalEcho(false); prefs.putBool("telnetEcho", false); if(out) out->print("Local echo: OFF\r\n"); bleNotifyLine("Local echo: OFF"); }
  else if (s=="echo")     { if(out) out->printf("Local echo: %s\r\n", localEcho?"ON":"OFF"); bleNotifyLine(String("Local echo: ")+(localEcho?"ON":"OFF")); }
  else if (s.startsWith("baud")){
    String arg = s.substring(4); arg.trim();
    if (!arg.length()){ printBaudOptions(out); }
    else {
      // Akzeptiere sowohl Index (1..N) als auch echte Baudrate (z.B. 9600).
      uint32_t v = (uint32_t) strtoul(arg.c_str(), nullptr, 10);
      uint32_t b = 0;
      if (v >= 1 && v <= (uint32_t)BaudConfig::ALL_RATES_COUNT) {
        b = BaudConfig::ALL_RATES[v-1];
      } else if (isValidBaud(v)) {
        b = v;
      }
      if (b) {
        setBaudRate(b);
        prefs.putUInt("baudRate", b);  // Speichern in NVS
        if(out) out->printf("Baud rate set to %lu\r\n", (unsigned long)b);
        bleNotifyLine(String("Baud rate set to ")+b);
      }
      else { if(out) out->print("Invalid baud selection\r\n"); bleNotifyLine("Invalid baud selection"); }
    }
  } else if (s=="pins"){
    // Lese GPIO-State direkt aus dem IO-MUX, ohne pinMode zu aendern
    // (digitalRead+pinMode wuerden UART-Pin entfuehren).
    int rxLvl = gpio_get_level((gpio_num_t)UART_RX);
    int txLvl = gpio_get_level((gpio_num_t)UART_TX);
    int enLvl = gpio_get_level((gpio_num_t)MAX3232_EN_PIN);
    bool expectedEnHigh = expectedMax3232EnHigh();
    sendLine("=== GPIO State ===");
    String enNote;
    if (HardwareConfig::MAX3232_ALWAYS_ENABLED) {
      enNote = HardwareConfig::MAX3232_ACTIVE_LEVEL_HIGH ? " (expected HIGH, MAX3232 always active)" : " (expected LOW, MAX3232 always active)";
    } else {
      enNote = isUSBMode ? " (expected LOW for USB)" : " (expected HIGH for RS232)";
    }
    sendLine(String("MAX3232_EN (GPIO") + MAX3232_EN_PIN + "): " + (enLvl?"HIGH":"LOW") + enNote);
    if ((bool)enLvl != expectedEnHigh) {
      sendLine("=> MAX3232_EN readback mismatch: GPIO2 wird extern gehalten oder EN-Logik ist anders verdrahtet");
    }
    sendLine(String("UART_RX (GPIO") + UART_RX + "): " + (rxLvl?"HIGH":"LOW") + " (idle should be HIGH on RS232)");
    sendLine(String("UART_TX (GPIO") + UART_TX + "): " + (txLvl?"HIGH":"LOW") + " (idle should be HIGH when UART aktiv)");
  } else if (s == "en high" || s == "en low") {
    bool high = s.endsWith("high");
    forceMax3232EnLevel(high);
    delay(5);
    int enLvl = gpio_get_level((gpio_num_t)MAX3232_EN_PIN);
    sendLine(String("MAX3232_EN forced ") + (high ? "HIGH" : "LOW") + ", readback=" + (enLvl ? "HIGH" : "LOW"));
    if ((bool)enLvl != high) {
      sendLine("=> GPIO2 folgt dem Firmware-Pegel nicht: Hardware zieht den Pin oder Pin ist falsch verbunden");
    }
  } else if (s=="loopback"){
    sendLine("=== TX Pin Hardware Test ===");
    // 1) UART komplett aus, Pin als manueller GPIO Output toggeln
    RS232Serial.end();
    delay(20);
    gpio_reset_pin((gpio_num_t)UART_TX);
    pinMode(UART_TX, OUTPUT);
    digitalWrite(UART_TX, HIGH);
    delay(2);
    int gpioHigh = digitalRead(UART_TX);
    digitalWrite(UART_TX, LOW);
    delay(2);
    int gpioLow = digitalRead(UART_TX);
    digitalWrite(UART_TX, HIGH);
    delay(2);
    int gpioHigh2 = digitalRead(UART_TX);
    sendLine(String("Manueller GPIO Output: H1=") + (gpioHigh?"HIGH":"LOW")
             + " L=" + (gpioLow?"HIGH":"LOW")
             + " H2=" + (gpioHigh2?"HIGH":"LOW"));
    if (gpioHigh && !gpioLow && gpioHigh2) {
      sendLine("=> GPIO" + String(UART_TX) + " toggelt manuell -> Pin OK");
    } else {
      sendLine("=> GPIO" + String(UART_TX) + " toggelt NICHT korrekt");
    }
    // RX-Pin auch sampeln (sollte bei MAX3232 idle HIGH sein, ohne MAX3232 floating)
    pinMode(UART_RX, INPUT);
    int rxLvl = digitalRead(UART_RX);
    sendLine(String("UART_RX (GPIO") + UART_RX + ") idle: " + (rxLvl?"HIGH":"LOW"));
    // 2) UART neu mit nicht-blockierender Sendung (nur kleine Menge)
    pinMode(UART_TX, INPUT);
    delay(10);
    RS232Serial.setRxBufferSize(SerialBufferConfig::RS232_RX_BYTES);
    RS232Serial.begin(currentBaudRate, SERIAL_8N1, UART_RX, UART_TX);
    uart_set_pin(UART_NUM_1, UART_TX, UART_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    delay(20);
    int txIdle = digitalRead(UART_TX);
    sendLine(String("UART begin -> TX idle: ") + (txIdle?"HIGH":"LOW"));
    // Nur 1 Byte schreiben, mit kurzem Timeout sample
    int avail = RS232Serial.availableForWrite();
    sendLine(String("availableForWrite: ") + avail);
    if (avail > 0) {
      RS232Serial.write((uint8_t)0x55); stat_rs232_tx++;
      // Sample 50ms
      int sawHigh=0, sawLow=0;
      uint32_t end = millis() + 50;
      while (millis() < end) {
        if (digitalRead(UART_TX)) sawHigh++; else sawLow++;
      }
      sendLine(String("Nach 1 Byte: HIGH=") + sawHigh + " LOW=" + sawLow);
    }
    // Cleanup zurueck zum normalen Zustand: gleiche empirisch funktionierende
    // Sequenz wie in setup() (begin/end/manueller Toggle/begin), sonst bleibt
    // UART_TX nach Loopback in high-Z (siehe /memories/repo/uart-init-quirk.md).
    rs232ReinitUart();
  } else if (s=="status"){
    IPAddress ap_ip = WiFi.softAPIP();
    bool wifiAP = (WiFi.getMode() & WIFI_MODE_AP) && (ap_ip != IPAddress(0,0,0,0));
    bool wifiClient = isWiFiClientConnected();
    uint8_t wifiClients = wifiAP ? WiFi.softAPgetStationNum() : 0;
    String target = isUSBMode ? "USB-Host" : "RS232";
    bool usbConnected = false;
    String usbDrv = "";
    if (isUSBMode) {
      usbConnected = usbHostConnected();
      char driverName[16];
      usbHostGetDriverName(driverName, sizeof(driverName));
      usbDrv = driverName;
    }
    int battery = getBatteryLevel();
    String otaSSID = prefs.getString("otaSSID", "");
    String otaUrl = getConfiguredOTAManifestUrl();
    
    sendLine("=== ConNect Status ===");
    sendLine(String("Target: ") + target);
    sendLine(String("Battery: ") + battery + "%");
    if (isUSBMode) {
      sendLine(String("USB connected: ") + (usbConnected ? "yes" : "no") + " (" + usbDrv + ")");
      sendLine(String("USB bytes: rx=") + stat_usb_rx.load() + ", tx=" + stat_usb_tx.load());
      sendLine(String("USB drops: ") + (unsigned long)usbHostDroppedBytes() + ", rx queue=" + (unsigned)usbHostRxQueueCapacity());
    } else {
      sendLine(String("RS232 baud: ") + currentBaudRate);
      sendLine(String("RS232 bytes: rx=") + stat_rs232_rx.load() + ", tx=" + stat_rs232_tx.load());
    }
    sendLine(String("BLE connected: ") + (deviceConnected.load() ? "yes" : "no"));
    sendLine(String("BLE notifications: ") + (bleOutputReady() ? "subscribed" : "off"));
    sendLine(String("Queues: target=") + (unsigned)targetTxBuffer.size() +
         ", telnet=" + (unsigned)telnetTxBuffer.size() +
         ", BLE=" + (unsigned)bleTxBuffer.size());
    sendLine(String("Telnet rejected: ") + (unsigned long)stat_telnet_tx_rejected.load());
    sendLine(String("BLE drops: rx=") + (unsigned long)stat_ble_rx_dropped.load() +
         ", tx=" + (unsigned long)stat_ble_tx_rejected.load());
    sendLine(String("WiFi mode: ") + getActiveWiFiModeLabel());
    if (wifiClient) {
      sendLine(String("WLAN: ") + WiFi.SSID() + ", IP=" + WiFi.localIP().toString());
      sendLine(String("Hostname: ") + getWiFiHostname());
    } else if (wifiAP) {
      sendLine(String("WiFi AP: ") + ap_ip.toString() + ", clients=" + String((unsigned)wifiClients));
    } else {
      sendLine("WiFi: disconnected");
    }
    sendLine(String("Reset: ") + resetReasonToStr(bootResetReason) + ", wake=" + wakeCauseToStr(bootWakeCause) + ", boot=" + bootCount);
    sendLine(String("Firmware: ") + FIRMWARE_VERSION);
    sendLine(String("Gespeichertes WLAN: ") + (otaSSID.length() > 0 ? otaSSID : "(nicht konfiguriert)"));
    sendLine(String("OTA URL: ") + otaUrl);
  } else if (s == "power") {
    sendLine("=== Power / Boot ===");
    sendLine(String("Boot count: ") + bootCount);
    sendLine(String("Reset reason: ") + resetReasonToStr(bootResetReason));
    sendLine(String("Wake cause: ") + wakeCauseToStr(bootWakeCause));
    sendLine(String("Battery: ") + String(batteryVoltageFiltered, 2) + " V, " + String(getBatteryLevel()) + "%");
    sendLine(String("Battery raw: ") + String(batteryVoltageRawLast, 2) + " V, valid=" + (batteryMeasurementValid ? "yes" : "no") + ", samples=" + batteryValidSamples);
    sendLine(String("GPIO0/BOOT: ") + (digitalRead(MODE_BUTTON_PIN) ? "HIGH" : "LOW"));
    sendLine(String("MAX3232_EN: ") + (gpio_get_level((gpio_num_t)MAX3232_EN_PIN) ? "HIGH" : "LOW"));
  } else if (s == "heap") {
    uint32_t freeNow   = (uint32_t)heap_caps_get_free_size(MALLOC_CAP_DEFAULT);
    uint32_t total     = (uint32_t)heap_caps_get_total_size(MALLOC_CAP_DEFAULT);
    uint32_t largest   = (uint32_t)heap_caps_get_largest_free_block(MALLOC_CAP_DEFAULT);
    uint32_t minEver   = (uint32_t)heap_caps_get_minimum_free_size(MALLOC_CAP_DEFAULT);
    uint32_t used      = total - freeNow;
    uint32_t fragPct   = (freeNow > 0) ? (100u - (largest * 100u) / freeNow) : 0u;
    char buf[96];
    snprintf(buf, sizeof(buf), "Heap: used=%u KB / total=%u KB (%u%%)",
             (unsigned)(used/1024), (unsigned)(total/1024),
             (unsigned)((used * 100u) / (total ? total : 1u)));
    sendLine(buf);
    snprintf(buf, sizeof(buf), "Free now=%u KB, largest block=%u KB, frag=%u%%",
             (unsigned)(freeNow/1024), (unsigned)(largest/1024), (unsigned)fragPct);
    sendLine(buf);
    snprintf(buf, sizeof(buf), "Min free ever=%u KB", (unsigned)(minEver/1024));
    sendLine(buf);
  } else if (s.startsWith("wlan ")) {
    // 'wlan SSID PASSWORD - WiFi-Credentials für OTA speichern
    String args = cmd.substring(cmd.indexOf("wlan") + 5);
    args.trim();
    int spaceIdx = args.indexOf(' ');
    if (spaceIdx > 0) {
      String ssid = args.substring(0, spaceIdx);
      String pass = args.substring(spaceIdx + 1);
      pass.trim();
      prefs.putString("otaSSID", ssid);
      prefs.putString("otaPass", pass);
      String response = "WLAN-Zugangsdaten gespeichert: " + ssid;
      if (out) out->print((response + "\r\n").c_str());
      bleNotifyLine(response);
    } else {
      if (out) out->print("Syntax: `wlan SSID PASSWORD\r\n");
      bleNotifyLine("Syntax: `wlan SSID PASSWORD");
    }
  } else if (s.startsWith("otaurl ")) {
    String url = cmd.substring(cmd.indexOf("otaurl") + 7);
    url.trim();
    if (url.length() > 0) {
      prefs.putString("otaUrl", url);
      String response = "OTA URL gespeichert: " + url;
      if (out) out->print((response + "\r\n").c_str());
      bleNotifyLine(response);
    } else {
      if (out) out->print("Syntax: `otaurl https://server/pfad/latest.json\r\n");
      bleNotifyLine("Syntax: `otaurl https://server/pfad/latest.json");
    }
  } else if (s == "ota") {
    // 'ota - OTA Update manuell starten
    if (out) out->print("Starte OTA Update...\r\n");
    bleNotifyLine("Starte OTA Update...");
    delay(100);
    performHTTPOTA();
  } else if (s == "version") {
    // Verwende OTA-Version aus NVS falls vorhanden, sonst GIT_VERSION
    String otaVer = prefs.getString("otaVersion", "");
    String displayVer = otaVer.length() > 0 ? otaVer : String(FIRMWARE_VERSION);
    String ver = String("Firmware: ") + displayVer;
    if (otaVer.length() > 0) {
      ver += " (OTA)";
    } else {
      ver += String(" (") + GIT_COMMIT + ")";
    }
    String build = String("Build: ") + BUILD_TIME;
    if (out) {
      out->print((ver + "\r\n").c_str());
      out->print((build + "\r\n").c_str());
    }
    bleNotifyLine(ver);
    bleNotifyLine(build);
  } else if (s=="help"){
    printHelp(out);
  } else if (s == "coredump") {
    // Zeigt Summary des letzten Core-Dumps (falls vorhanden).
    // Befehle: `coredump         -> Summary ausgeben
    //          `coredump erase   -> Dump verwerfen
    esp_core_dump_summary_t *sum =
        static_cast<esp_core_dump_summary_t*>(malloc(sizeof(esp_core_dump_summary_t)));
    if (!sum) { sendLine("coredump: OOM"); return; }
    esp_err_t err = esp_core_dump_get_summary(sum);
    if (err == ESP_OK) {
      char buf[96];
      snprintf(buf, sizeof(buf), "Task: %s  PC: 0x%08lx",
               sum->exc_task, (unsigned long)sum->exc_pc);
      sendLine(buf);
      snprintf(buf, sizeof(buf), "App ELF SHA256: %s", sum->app_elf_sha256);
      sendLine(buf);
      sendLine("Backtrace (PCs):");
      uint32_t n = sum->exc_bt_info.depth;
      if (n > 16) n = 16;
      for (uint32_t i = 0; i < n; ++i) {
        snprintf(buf, sizeof(buf), "  #%lu 0x%08lx",
                 (unsigned long)i,
                 (unsigned long)sum->exc_bt_info.bt[i]);
        sendLine(buf);
      }
      sendLine("Decode via: xtensa-esp32s3-elf-addr2line -pfiaC -e firmware.elf <pcs>");
    } else if (err == ESP_ERR_INVALID_SIZE) {
      sendLine("coredump: kein Dump gespeichert");
    } else {
      char buf[48];
      snprintf(buf, sizeof(buf), "coredump: Fehler 0x%x", err);
      sendLine(buf);
    }
    free(sum);
  } else if (s == "coredump erase") {
    if (out) out->print("Loesche Coredump...\r\n");
    bleNotifyLine("Loesche Coredump...");
    esp_err_t err = esp_core_dump_image_erase();
    if (err == ESP_OK) {
      if (out) out->print("OK\r\n");
      bleNotifyLine("OK");
    } else {
      char buf[32];
      snprintf(buf, sizeof(buf), "Fehler 0x%x", err);
      if (out) { out->print(buf); out->print("\r\n"); }
      bleNotifyLine(buf);
    }
  } else {
    if(out) out->print("Unknown command. Type `help\r\n");
    bleNotifyLine("Unknown command. Type `help");
  }
}

// -----------------------------------------------------------------------------
// Setup / Loop
// -----------------------------------------------------------------------------
static inline void handleTargetRxByte(char c) {
  if (isUSBMode) {
    uint8_t bits = usbHostDataBits();
    if (bits > 0 && bits < 8) {
      c = static_cast<char>(static_cast<unsigned char>(c) & 0x7F);
    }
  }
  uint8_t value = static_cast<uint8_t>(c);
  if (telnetSessionActive) {
    terminal::enqueueTelnetData(telnetTxBuffer, value);
  }
  bleAppendFromDevice(value);
  webConsolePushRx(c);
  lastReceivedData = c;
  rgbPulseActivity(60);
  lastRxActivity = millis();
  display.resetActivity();
}

static void drainTargetRx(size_t maxBytes = Timing::SERIAL_DRAIN_BUDGET) {
  pumpTelnetTx();
  pumpBleTx();
  size_t processed = 0;
  if (isUSBMode) {
    static uint32_t lastUsbTask = 0;
    uint32_t now = millis();
    if (now - lastUsbTask >= Timing::USB_TASK_INTERVAL) {
      usbHostTask();
      lastUsbTask = now;
    }
    int rb = -1;
    while (processed < maxBytes && terminalOutputsCanAcceptByte() &&
           (rb = usbHostReadByte()) >= 0) {
      stat_usb_rx++;
      handleTargetRxByte((char)rb);
      processed++;
    }
  } else {
    while (processed < maxBytes && terminalOutputsCanAcceptByte() && RS232Serial.available()) {
      char c = RS232Serial.read();
      stat_rs232_rx++;
      handleTargetRxByte(c);
      processed++;
    }
  }
}

static void processBleRx(size_t maxBytes = Timing::BLE_RX_PROCESS_BUDGET) {
  if (!bleRxQueue) return;
  static bool bleAtLineStart = true;
  static bool bleCmdMode = false;
  static uint8_t bleCmdPrefix[2] = {'`', 0};
  static size_t bleCmdPrefixLen = 1;
  static bool pendingAcuteLead = false;
  static uint32_t pendingAcuteSince = 0;
  static terminal::NewlineDecoder newlineDecoder;
  size_t processed = 0;
  uint8_t value = 0;

    if (pendingAcuteLead &&
      targetTxBuffer.freeSpace() > Timing::TARGET_CONTROL_RESERVE &&
      uxQueueMessagesWaiting(bleRxQueue) == 0 &&
      millis() - pendingAcuteSince >= 100) {
    writeToTarget(0xC2);
    pendingAcuteLead = false;
    bleAtLineStart = false;
  }

  while (processed < maxBytes &&
         (bleCmdMode || targetTxBuffer.freeSpace() > Timing::TARGET_CONTROL_RESERVE) &&
         xQueueReceive(bleRxQueue, &value, 0) == pdPASS) {
    processed++;

    if (pendingAcuteLead) {
      pendingAcuteLead = false;
      if (value == 0xB4) {
        bleCmdMode = true;
        bleCmdPrefix[0] = 0xC2;
        bleCmdPrefix[1] = 0xB4;
        bleCmdPrefixLen = 2;
        bleAtLineStart = false;
        continue;
      }
      writeToTarget(0xC2);
      bleAtLineStart = false;
    } else if (!bleCmdMode && bleAtLineStart && value == 0xC2) {
      pendingAcuteLead = true;
      pendingAcuteSince = millis();
      continue;
    }

    uint8_t decoded = 0;
    if (!newlineDecoder.feed(value, decoded)) {
      continue;
    }
    char ch = static_cast<char>(decoded);

    if (ch == '\r') {
      if (bleCmdMode) {
        if (isInternalCommand(bleCmdBuffer)) {
          handleInternalCommand(bleCmdBuffer);
        } else {
          forwardCommandCandidate(bleCmdPrefix, bleCmdPrefixLen, bleCmdBuffer, true);
        }
        bleCmdBuffer.clear();
        bleCmdMode = false;
        bleAtLineStart = true;
        continue;
      }

      writeToTarget('\r');
      lastReceivedData = '\r';
      rgbPulseActivity(10);
      bleAtLineStart = true;
      continue;
    }

    if (!bleCmdMode) {
      if (bleAtLineStart && isCmdPrefix(ch)) {
        bleCmdMode = true;
        bleCmdPrefix[0] = decoded;
        bleCmdPrefixLen = 1;
        bleAtLineStart = false;
        continue;
      }
      writeToTarget(decoded);
      lastReceivedData = ch;
      rgbPulseActivity(20);
      bleAtLineStart = false;
      continue;
    }

    if (decoded == 0x7F || ch == '\b') {
      if (bleCmdBuffer.length() > 0) {
        bleCmdBuffer.remove(bleCmdBuffer.length() - 1);
      }
    } else if (bleCmdBuffer.length() < Timing::TELNET_CMD_BUFFER_MAX) {
      bleCmdBuffer += ch;
    } else {
      forwardCommandCandidate(bleCmdPrefix, bleCmdPrefixLen, bleCmdBuffer, false);
      writeToTarget(decoded);
      bleCmdBuffer.clear();
      bleCmdMode = false;
      bleAtLineStart = false;
    }
    lastReceivedData = ch;
    rgbPulseActivity(10);
  }
}

void setup(){
  // Kein Serial.begin(): USB-CDC ist per Build-Flag deaktiviert
  // (ARDUINO_USB_CDC_ON_BOOT=0). Diagnose laeuft ueber BLE/Telnet/Web.

  // NVS initialisieren. Magic-Check gegen Schema-Bruch:
  // Wenn der gespeicherte Marker nicht zum aktuellen IDF-Major passt,
  // ist der NVS-Inhalt potenziell mit der neuen IDF nicht kompatibel
  // (siehe Bootloop nach Plattform-Upgrade auf IDF 5.x). Dann komplett
  // loeschen, bevor wir weiterarbeiten.
  constexpr uint32_t NVS_SCHEMA_MAGIC =
      0xC011ECD0u + (ESP_IDF_VERSION_MAJOR & 0xFF);
  prefs.begin("connect", false);
  uint32_t storedMagic = prefs.getUInt("schemaMagic", 0);
  if (storedMagic != 0 && storedMagic != NVS_SCHEMA_MAGIC) {
    prefs.end();
    nvs_flash_erase();
    nvs_flash_init();
    prefs.begin("connect", false);
  }
  prefs.putUInt("schemaMagic", NVS_SCHEMA_MAGIC);

  if (prefs.getUChar(OTAConfig::STORAGE_VERSION_KEY, 0) < OTAConfig::STORAGE_VERSION) {
    prefs.putString("otaUrl", OTAConfig::DEFAULT_MANIFEST_URL);
    prefs.putUChar(OTAConfig::STORAGE_VERSION_KEY, OTAConfig::STORAGE_VERSION);
  }

  bootResetReason = esp_reset_reason();
  bootWakeCause = esp_sleep_get_wakeup_cause();
  bootCount = prefs.getUInt("bootCount", 0) + 1;
  prefs.putUInt("bootCount", bootCount);

  currentBaudRate = prefs.getUInt("baudRate", BaudConfig::DEFAULT_BAUD);
  if (!isValidBaud(currentBaudRate)) {
    currentBaudRate = BaudConfig::DEFAULT_BAUD;
    prefs.putUInt("baudRate", currentBaudRate);
  }
  isUSBMode = prefs.getBool("usbMode", true);
  localEcho = prefs.getBool("telnetEcho", false);
  uint8_t savedWiFiMode = prefs.getUChar("wifiMode", static_cast<uint8_t>(WiFiConfig::Mode::AccessPoint));
  configuredWiFiMode = savedWiFiMode == static_cast<uint8_t>(WiFiConfig::Mode::Client)
                         ? WiFiConfig::Mode::Client
                         : WiFiConfig::Mode::AccessPoint;
  bleRxQueue = xQueueCreate(Timing::BLE_RX_QUEUE_LEN, sizeof(uint8_t));
  
  pinMode(MODE_BUTTON_PIN, INPUT_PULLUP);
  setSerialTargetMode(isUSBMode);

  // ADC Konfiguration für Batteriemessung
  analogSetAttenuation(ADC_11db);
  analogReadResolution(12);
  analogSetPinAttenuation(BATTERY_PIN, ADC_11db);
  
  // Batterie-Filter validiert initialisieren. Ein einzelner unplausibler
  // ADC-Startwert (z.B. 0V) darf den Akku nicht auf 0% festnageln.
  batteryVoltageFiltered = readBatteryVoltage();
  getBatteryLevel();

  rgbBegin();
  setupBuzzer(BUZZER_PIN);
  Wire.begin(I2C_SDA, I2C_SCL);

  display.begin();
  display.showLogo();

  // BLE frueh starten, bevor WiFi, USB-Host-Queue und serielle RX-Puffer
  // groessere Heap-Bloecke belegen. NimBLE braucht beim Controller-Init
  // zusammenhaengenden Speicher; spaeteres Init kann mit "BLE_INIT: Malloc
  // failed" scheitern.
  setupBLE();

  // USB-Host muss vor dem WiFi-Treiber installiert werden. Eine nachtraegliche
  // Installation in einen laufenden WiFi-Stack blockiert auf diesem Board den
  // kompletten Netzwerkstack. Im RS232-Modus bleibt USB bewusst uninitialisiert.
  if (isUSBMode) {
    usbHostInit();
    usbHostStarted = usbHostInitialized();
  }

  // WiFi nach BLE und optionalem USB-Host initialisieren, aber noch vor dem
  // dynamischen UART-RX-Puffer.
  setupWiFi();

  // RS232 / UART1 Init: HardwareSerial.begin() in arduino-esp32 3.x /
  // pioarduino 55.x mapped GPIO5/6 nicht zuverlaessig auf UART1.
  // rs232ReinitUart() kapselt die empirisch funktionierende Toggle-Sequenz.
  rs232ReinitUart();
  usbHostSetLineCoding(currentBaudRate);

  // OTA-Update konfigurieren
  ArduinoOTA.setHostname("ConNect");
  // mDNS triggert mit arduino-esp32 3.x / IDF 5.x im SoftAP-Bootpfad einen
  // Abort in der mDNS-Probe. OTA per fester IP (192.168.32.1:3232) bleibt aktiv.
  ArduinoOTA.setMdnsEnabled(false);
  ArduinoOTA.onStart([]() {
    display.showMessage("OTA Update", "gestartet...");
  });
  ArduinoOTA.onEnd([]() {
    display.showMessage("OTA Update", "fertig!", "Neustart...");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    static int lastPct = -1;
    int pct = (progress * 100) / total;
    if (pct != lastPct) {
      char buf[20];
      snprintf(buf, sizeof(buf), "%d%%", pct);
      display.showMessage("OTA Update", buf);
      lastPct = pct;
    }
  });
  ArduinoOTA.onError([](ota_error_t error) {
    const char* errMsg = "Unbekannt";
    if (error == OTA_AUTH_ERROR) errMsg = "Auth Fehler";
    else if (error == OTA_BEGIN_ERROR) errMsg = "Begin Fehler";
    else if (error == OTA_CONNECT_ERROR) errMsg = "Connect Fehler";
    else if (error == OTA_RECEIVE_ERROR) errMsg = "Receive Fehler";
    else if (error == OTA_END_ERROR) errMsg = "End Fehler";
    display.showMessage("OTA Fehler", errMsg);
  });
  ArduinoOTA.begin();
  
  // Web Interface starten
  webServer.begin();

  // Task-Watchdog: arduino-esp32 3.x initialisiert den WDT bereits beim Boot
  // (CONFIG_ESP_TASK_WDT_INIT=y). Ein erneuter esp_task_wdt_init() scheitert
  // mit ESP_ERR_INVALID_STATE - wir nutzen daher reconfigure() und tolerieren
  // einen Fehler beim Hinzufuegen des loopTasks (kann bereits subscribed sein).
  // trigger_panic=true: bei echtem Hang (z.B. WebServer durch parallele
  // Browser-Connections geflutet) soll das Geraet automatisch rebooten.
  // Vorher false -> Geraet blieb tot bis manueller Reset.
  esp_task_wdt_config_t wdtCfg = {
      .timeout_ms = 15000,
      .idle_core_mask = 0,
      .trigger_panic = true,
  };
  esp_err_t wdtErr = esp_task_wdt_reconfigure(&wdtCfg);
  if (wdtErr == ESP_ERR_INVALID_STATE) {
      // WDT noch nicht initialisiert - jetzt tun.
      esp_task_wdt_init(&wdtCfg);
  }
  // loopTask abonnieren; ESP_ERR_INVALID_ARG = bereits registriert -> ok.
  esp_task_wdt_add(nullptr);

  runtimeServicesReady = true;

  doBeep();
}

void loop(){
  // Watchdog fuettern (wird bei jeder loop()-Iteration zurueckgesetzt)
  esp_task_wdt_reset();
  pumpTargetTx();
  drainTargetRx();

  serviceWiFi();

  if (usbModeRestartRequested &&
      millis() - usbModeRestartRequestedAt >= 1200) {
    ESP.restart();
  }

  // OTA-Updates prüfen (lokales Netzwerk)
  ArduinoOTA.handle();
  drainTargetRx();
  
  // Web Interface
  webServer.handleClient();
  drainTargetRx();

  // Web-Konsole TX (Browser -> Zielgerät / interne Commands)
  {
    uint8_t webTx[128];
    static bool webAtLineStart = true;
    static bool webCmdMode = false;
    static uint8_t webCmdPrefix = '`';
    static String webCmdBuffer;
    static terminal::NewlineDecoder webNewlineDecoder;
    size_t webBudget = 0;
    if (webCmdMode) {
      webBudget = sizeof(webTx);
    } else if (targetTxBuffer.freeSpace() > Timing::TARGET_CONTROL_RESERVE) {
      webBudget = std::min<size_t>(sizeof(webTx),
                   targetTxBuffer.freeSpace() - Timing::TARGET_CONTROL_RESERVE);
    }
    size_t webTxLen = webConsoleReadTx(webTx, webBudget);

    for (size_t i = 0; i < webTxLen; i++) {
      uint8_t decoded = 0;
      if (!webNewlineDecoder.feed(webTx[i], decoded)) {
        continue;
      }
      char c = static_cast<char>(decoded);

      if (c == '\r') {
        if (webCmdMode) {
          if (isInternalCommand(webCmdBuffer)) {
            handleInternalCommand(webCmdBuffer);
          } else {
            forwardCommandCandidate(webCmdPrefix, webCmdBuffer, true);
          }
          webCmdBuffer = "";
          webCmdMode = false;
          webAtLineStart = true;
          continue;
        }

        writeToTarget('\r');
        lastTxActivity = millis();
        display.resetActivity();
        rgbPulseActivity(8);
        webAtLineStart = true;
        continue;
      }

      if (!webCmdMode && webAtLineStart && isCmdPrefix(c)) {
        webCmdMode = true;
        webCmdPrefix = decoded;
        webCmdBuffer = "";
        webAtLineStart = false;
        continue;
      }

      if (webCmdMode) {
        if ((uint8_t)c == 0x7F || c == '\b') {
          if (webCmdBuffer.length() > 0) {
            webCmdBuffer.remove(webCmdBuffer.length() - 1);
          }
        } else if (webCmdBuffer.length() < Timing::TELNET_CMD_BUFFER_MAX) {
          webCmdBuffer += c;
        } else {
          forwardCommandCandidate(webCmdPrefix, webCmdBuffer, false);
          writeToTarget(decoded);
          webCmdBuffer = "";
          webCmdMode = false;
          webAtLineStart = false;
        }
        continue;
      }

      writeToTarget(decoded);
      lastTxActivity = millis();
      display.resetActivity();
      rgbPulseActivity(8);
      webAtLineStart = false;
    }
  }
  pumpTargetTx();
  
  processBleRx();
  pumpTargetTx();
  drainTargetRx();
  
  // TX-Aktivitätstracking (basierend auf stat-Änderungen)
  static uint32_t lastStatTx = 0;
  uint32_t currentStatTx = stat_usb_tx.load() + stat_rs232_tx.load();
  if (currentStatTx != lastStatTx) {
    lastTxActivity = millis();
    lastStatTx = currentStatTx;
    display.resetActivity();  // Screensaver Reset bei Aktivität
  }
  
  // ==========================================================================
  // Button State Machine
  // - 1x kurz: USB/RS232 umschalten
  // - 2x kurz (Doppelklick): Baudrate durchschalten
  // - 3x kurz (Dreifachklick): Hotspot/WLAN-Client umschalten
  // - Lang (1-5s): BLE Advertising an/aus
  // - Sehr lang (>5s): OTA Update von GitHub
  // ==========================================================================
  static bool lastButtonState = HIGH;
  static unsigned long buttonPressStart = 0;
  static unsigned long lastReleaseTime = 0;
  static int clickCount = 0;
  static bool longPressHandled = false;
  static bool veryLongPressHandled = false;
  static int lastHoldFeedback = 0;
  static bool buttonArmed = false;
  static unsigned long buttonHighSince = 0;
  static bool otaConfirmationPending = false;
  static bool otaConfirmationPress = false;
  static unsigned long otaConfirmationReadyAt = 0;
  static unsigned long otaConfirmationExpiresAt = 0;
  
  bool cur = digitalRead(MODE_BUTTON_PIN);
  unsigned long now = millis();

  if (otaConfirmationPending &&
      (int32_t)(now - otaConfirmationExpiresAt) >= 0) {
    otaConfirmationPending = false;
    otaConfirmationPress = false;
    otaConfirmationReadyAt = 0;
  }

  // GPIO0 wird auch von Auto-Reset-Schaltungen beeinflusst. Tasteneingaben
  // erst nach einer stabilen Freigabe akzeptieren; ein beim Boot LOW
  // gehaltener Pin kann damit niemals OTA oder andere Aktionen ausloesen.
  if (!buttonArmed) {
    if (cur == HIGH) {
      if (buttonHighSince == 0) buttonHighSince = now;
      if (now - buttonHighSince >= Timing::BUTTON_ARM_STABLE_MS) {
        buttonArmed = true;
        lastButtonState = HIGH;
      }
    } else {
      buttonHighSince = 0;
    }
    buttonPressStart = 0;
    clickCount = 0;
  }
  
  // Button gedrückt
  if (buttonArmed && cur == LOW && lastButtonState == HIGH) {
    buttonPressStart = now;
    longPressHandled = false;
    veryLongPressHandled = false;
    lastHoldFeedback = 0;

    if (otaConfirmationPending) {
      bool ready = otaConfirmationReadyAt != 0 &&
                   (int32_t)(now - otaConfirmationReadyAt) >= 0 &&
                   (int32_t)(otaConfirmationExpiresAt - now) > 0;
      if (ready) {
        otaConfirmationPress = true;
      } else {
        otaConfirmationPending = false;
        buttonPressStart = 0;
      }
    }
  }
  
  // Button wird gehalten - visuelles Feedback mit Countdown
  if (buttonArmed && cur == LOW && buttonPressStart > 0 && !otaConfirmationPress) {
    unsigned long holdTime = now - buttonPressStart;
    
    // Feedback bei 1s: "BLE Toggle..." (nur wenn noch unter 3s)
    if (holdTime >= Timing::BUTTON_FEEDBACK_BLE && holdTime < Timing::BUTTON_FEEDBACK_OTA && lastHoldFeedback < 1) {
      display.showMessage("BLE Toggle", bleAdvertising ? "-> AUS" : "-> AN", "Loslassen!", 2000);
      beep(BUZZER_PIN, 1500, 30);  // Kurzer hoher Ton als Feedback
      lastHoldFeedback = 1;
    }
    // Countdown für OTA: 3s, 4s zeigen verbleibende Zeit
    if (holdTime >= Timing::BUTTON_FEEDBACK_OTA && holdTime < Timing::BUTTON_OTA_MS) {
      int secondsLeft = (Timing::BUTTON_OTA_MS - holdTime) / 1000 + 1;
      int feedbackLevel = 3 + (2 - secondsLeft);  // 3, 4, 5 für 2s, 1s, 0s
      if (lastHoldFeedback < feedbackLevel) {
        char countdownStr[24];
        snprintf(countdownStr, sizeof(countdownStr), "OTA in %d...", secondsLeft);
        display.showMessage(countdownStr, "Weiter halten", "oder loslassen", 1000);
        beep(BUZZER_PIN, 1000 + (3 - secondsLeft) * 200, 50);  // Ton wird höher
        lastHoldFeedback = feedbackLevel;
      }
    }
    // Bei 5s nur die bewusste Zweitbestaetigung vorbereiten. GPIO0 wird auch
    // von USB-Auto-Reset beeinflusst; direktes Starten waere daher unsicher.
    if (holdTime >= Timing::BUTTON_OTA_MS && !veryLongPressHandled) {
      veryLongPressHandled = true;
      longPressHandled = true;  // Verhindert BLE-Toggle beim Loslassen
      otaConfirmationPending = true;
      otaConfirmationReadyAt = 0;
      otaConfirmationExpiresAt = now + Timing::BUTTON_OTA_CONFIRM_WINDOW_MS;
      display.showMessage("OTA bestaetigen", "Loslassen", "dann kurz druecken", 3000);
      beepPattern(2, 1800, 100, 100);
    }
  }
  
  // Button losgelassen
  if (buttonArmed && cur == HIGH && lastButtonState == LOW && buttonPressStart > 0) {
    unsigned long pressDuration = now - buttonPressStart;

    if (otaConfirmationPress) {
      bool confirmed = pressDuration < Timing::BUTTON_SHORT_MAX_MS &&
                       otaConfirmationPending &&
                       (int32_t)(otaConfirmationExpiresAt - now) > 0;
      otaConfirmationPending = false;
      otaConfirmationPress = false;
      otaConfirmationReadyAt = 0;
      clickCount = 0;
      if (confirmed) {
        beepPattern(2, 2000, 80, 80);
        performHTTPOTA();
      }
    } else if (otaConfirmationPending && veryLongPressHandled) {
      otaConfirmationReadyAt = now + Timing::BUTTON_OTA_CONFIRM_DELAY_MS;
      otaConfirmationExpiresAt = now + Timing::BUTTON_OTA_CONFIRM_WINDOW_MS;
      display.showMessage("OTA bestaetigen", "1s warten", "dann kurz druecken", 2500);
      clickCount = 0;
    } else if (pressDuration >= Timing::BUTTON_LONG_MS && pressDuration < Timing::BUTTON_OTA_MS && !longPressHandled) {
      // Langer Druck (1-5s): BLE Toggle
      if (bleAdvertising) {
        NimBLEDevice::stopAdvertising();
        bleAdvertising = false;
        display.showMessage("BLE", "Advertising AUS", nullptr, Timing::MESSAGE_DISPLAY_MS);
      } else {
        if (NimBLEDevice::startAdvertising()) {
          bleAdvertising = true;
          display.showMessage("BLE", "Advertising AN", nullptr, Timing::MESSAGE_DISPLAY_MS);
        } else {
          display.showMessage("BLE", "Adv-Start", "FEHLER", Timing::MESSAGE_DISPLAY_MS);
        }
      }
      longPressHandled = true;
      clickCount = 0;
    } else if (pressDuration < Timing::BUTTON_SHORT_MAX_MS) {
      // Kurzer Klick - zählen für Doppelklick
      if (now - lastReleaseTime < Timing::BUTTON_DOUBLE_GAP_MS) {
        clickCount++;
      } else {
        clickCount = 1;
      }
      lastReleaseTime = now;
    }
    buttonPressStart = 0;
    lastHoldFeedback = 0;
  }
  
  // Doppelklick-Timeout
  if (buttonArmed && clickCount > 0 && cur == HIGH && (now - lastReleaseTime) > 500) {
    if (clickCount == 1) {
      // Einfacher Klick: USB/RS232 umschalten
      setSerialTargetMode(!isUSBMode);
      if (!isUSBMode) {
        delay(20);
        rs232ReinitUart();
        setSerialTargetMode(false);
      }
      prefs.putBool("usbMode", isUSBMode);  // Speichern in NVS
      doBeep();
      bleNotifyLine(isUSBMode ? "Target: USB" : "Target: RS232");
    } else if (clickCount == 2) {
      // Doppelklick: Baudrate durchschalten
      cycleBaudRate();
      char msg[32];
      snprintf(msg, sizeof(msg), "%lu baud", (unsigned long)currentBaudRate);
      display.showMessage("Baudrate", msg, nullptr, Timing::MESSAGE_DISPLAY_MS);
      bleNotifyLine(String("Baud: ") + currentBaudRate);
    } else {
      configuredWiFiMode = configuredWiFiMode == WiFiConfig::Mode::AccessPoint
                             ? WiFiConfig::Mode::Client
                             : WiFiConfig::Mode::AccessPoint;
      prefs.putUChar("wifiMode", static_cast<uint8_t>(configuredWiFiMode));
      const char* label = configuredWiFiMode == WiFiConfig::Mode::Client ? "WLAN Client" : "Hotspot";
      display.showMessage("WiFi Modus", label, "Neustart...", 1500);
      beepPattern(3, 1400, 70, 70);
      delay(1500);
      ESP.restart();
    }
    clickCount = 0;
  }
  
  lastButtonState = cur;
  drainTargetRx();

  // Telnet Server (raw char-mode, echo off)
  if (telnetSessionActive && tcpClient.fd() < 0) {
    closeTelnetSession();
  }

  WiFiClient nc = tcpServer.accept();
  if (nc.fd() >= 0) {
    if (telnetSessionActive) {
      closeTelnetSession();
    }
      tcpClient = nc;
      telnetSessionActive = true;
      tcpClient.setNoDelay(true);
      telnetTxBuffer.clear();
      stat_telnet_tx_rejected.store(0, std::memory_order_relaxed);
      telnetDecoder.reset();
      telnetNewlineDecoder.reset();
      telnetAtLineStart = true;
      telnetCmdMode = false;
      telnetInputError = TelnetInputError::None;
      lastTcpActivity = millis();  // Reset timeout bei neuer Verbindung
      const uint8_t seq[] = {
        terminal::TELNET_IAC, terminal::TELNET_WILL, 0,
        terminal::TELNET_IAC, terminal::TELNET_DO, 0,
        terminal::TELNET_IAC, terminal::TELNET_WILL, 1,
        terminal::TELNET_IAC, terminal::TELNET_WILL, 3,
        terminal::TELNET_IAC, terminal::TELNET_DO, 3,
        terminal::TELNET_IAC, terminal::TELNET_DONT, 34,
      };
      telnetTxBuffer.push(seq, sizeof(seq));
      telnetOut.print("Welcome to ConNect!\r\n");
      telnetOut.printf("Current baud rate: %lu\r\n", (unsigned long)currentBaudRate);
      if (isUSBMode && !usbHostConnected()) {
        telnetOut.print("Target: USB (not connected)\r\n");
      } else {
        telnetOut.printf("Target: %s\r\n", isUSBMode ? "USB" : "RS232");
      }
      telnetOut.printf("Local echo: %s\r\n", localEcho ? "ON" : "OFF");
      printHelp(&telnetOut);
      telnetOut.print("> ");
      pumpTelnetTx();
      telnetCmdBuffer = "";
  }
  if (telnetSessionActive) {
    bool telnetTargetReady = !isUSBMode || usbHostConnected();
    // TCP Idle Timeout Check (15 Minuten)
    if ((millis() - lastTcpActivity) > Timing::TCP_IDLE_TIMEOUT_MS) {
      telnetOut.print("\r\n[Timeout - Verbindung getrennt]\r\n");
      pumpTelnetTx();
      closeTelnetSession(false);
    }
    while (telnetSessionActive && tcpClient.available() &&
           (telnetCmdMode || targetTxBuffer.freeSpace() > Timing::TARGET_CONTROL_RESERVE)) {
      lastTcpActivity = millis();  // Reset timeout bei Aktivität
      uint8_t received = static_cast<uint8_t>(tcpClient.read());
      terminal::TelnetEvent event = telnetDecoder.feed(received);
      if (event.type == terminal::TelnetEventType::Negotiation) {
        handleTelnetNegotiation(event);
        continue;
      }
      if (event.type != terminal::TelnetEventType::Data) continue;

      uint8_t decoded = 0;
      if (!telnetNewlineDecoder.feed(event.value, decoded)) continue;
      char c = static_cast<char>(decoded);

      if (c == '\r') {
        if (telnetCmdMode) {
          bool internalCommand = isInternalCommand(telnetCmdBuffer);
          if (internalCommand) {
            handleInternalCommand(telnetCmdBuffer);
          } else {
            forwardCommandCandidate(telnetCmdPrefix, telnetCmdBuffer, true);
          }
          telnetCmdMode = false;
          telnetCmdBuffer = "";
          telnetAtLineStart = true;
          if (internalCommand) telnetOut.print("\r\n> ");
        } else {
          if (telnetEolMode == 0) {
            sendTelnetTargetByte('\r', telnetTargetReady);
            sendTelnetTargetByte('\n', telnetTargetReady);
          } else if (telnetEolMode == 1) {
            sendTelnetTargetByte('\r', telnetTargetReady);
          } else {
            sendTelnetTargetByte('\n', telnetTargetReady);
          }
          if (localEcho) telnetOut.print("\r\n");
          telnetAtLineStart = true;
        }
        rgbPulseActivity(30);
        continue;
      }

      if (!telnetCmdMode) {
        if (telnetAtLineStart && isCmdPrefix(c)) {
          telnetCmdMode = true;
          telnetCmdPrefix = decoded;
          telnetAtLineStart = false;
          telnetOut.write(decoded);
          continue;
        }
        if ((uint8_t)c == 0x7F || c == '\b') {
          sendTelnetTargetByte('\b', telnetTargetReady);
          if (localEcho) telnetOut.print("\b \b");
        } else {
          sendTelnetTargetByte(decoded, telnetTargetReady);
          if (localEcho) telnetOut.write(decoded);
        }
        telnetAtLineStart = false;
        rgbPulseActivity(10);
      } else {
        if ((uint8_t)c == 0x7F || c == '\b') {
          if (telnetCmdBuffer.length() > 0) {
            telnetCmdBuffer.remove(telnetCmdBuffer.length()-1);
            telnetOut.print("\b \b"); // erase last char visually
          }
        } else {
          // Buffer-Limit um DoS zu verhindern
          if (telnetCmdBuffer.length() < Timing::TELNET_CMD_BUFFER_MAX) {
            telnetCmdBuffer += c;
          } else {
            forwardCommandCandidate(telnetCmdPrefix, telnetCmdBuffer, false);
            writeToTarget(decoded);
            telnetCmdBuffer = "";
            telnetCmdMode = false;
            telnetAtLineStart = false;
          }
          telnetOut.write(decoded); // echo typed char in command mode only
        }
        rgbPulseActivity(5);
      }
    }
    pumpTargetTx();
  }

  // BLE idle flush + welcome (send welcome once after connect)
  bleFlushLineIdle();
  static bool bleWelcomeSent=false;
  static uint32_t bleWelcomeT0=0;
  if (bleOutputReady()) {
    if (bleWelcomeT0==0) bleWelcomeT0=millis();
    if (!bleWelcomeSent && (millis()-bleWelcomeT0) > 5000) {
      bleNotifyLine("Welcome to ConNect (BLE)!");
      if (isWiFiClientConnected()) {
        bleNotifyLine(String("WLAN IP: ") + WiFi.localIP().toString());
        if (mdnsRunning) bleNotifyLine("Web/Telnet: connect.local");
      } else {
        IPAddress ap_ip = WiFi.softAPIP();
        if (ap_ip != IPAddress(0,0,0,0)) {
          bleNotifyLine(String("AP up: ") + ap_ip.toString());
          bleNotifyLine("Telnet on port 23");
        }
      }
      bleNotifyLine("Type `help for commands");
      bleWelcomeSent = true;
    }
  } else {
    bleWelcomeSent=false; bleWelcomeT0=0; // reset for next client
  }

  // USB Connect/Disconnect Töne
  bool currentUsbConnected = usbHostConnected();
  if (isUSBMode && currentUsbConnected != lastUsbConnected) {
    if (currentUsbConnected) {
      beepPattern(3, 1200, 40, 40);  // 3x kurzer Ton bei USB Connect
    } else {
      beepPattern(2, 600, 100, 50);  // 2x tiefer Ton bei USB Disconnect
    }
    lastUsbConnected = currentUsbConnected;
  }

  // Batteriewarnung bei niedrigem Stand
  static uint32_t lastBatteryWarning = 0;
  static bool lowBatteryWarningShown = false;
  uint8_t batLevel = getBatteryLevel();
  bool chargingNow = isBatteryCharging();
  static uint32_t criticalBatterySince = 0;
  if (batLevel <= PowerConfig::CRITICAL_SLEEP_PERCENT && !chargingNow && batteryCanTriggerCriticalSleep()) {
    if (criticalBatterySince == 0) criticalBatterySince = millis();
    if (millis() - criticalBatterySince > PowerConfig::CRITICAL_SLEEP_GRACE_MS) {
      enterCriticalBatterySleep();
    }
  } else {
    criticalBatterySince = 0;
  }
  if (batLevel <= 15 && batLevel > 5) {
    // Bei 15-6%: Warnung alle 5 Minuten
    if (millis() - lastBatteryWarning > 300000 || !lowBatteryWarningShown) {
      char batMsg[16];
      snprintf(batMsg, sizeof(batMsg), "Nur noch %d%%", batLevel);
      display.showMessage("Akku niedrig!", batMsg, "Bitte laden", 3000);
      beepPattern(2, 800, 200, 100);  // 2x tiefer Warnton
      lastBatteryWarning = millis();
      lowBatteryWarningShown = true;
    }
  } else if (batLevel <= 5) {
    // Bei 5% oder weniger: Warnung jede Minute
    if (millis() - lastBatteryWarning > 60000 || !lowBatteryWarningShown) {
      char batMsg[16];
      snprintf(batMsg, sizeof(batMsg), "Nur noch %d%%!", batLevel);
      display.showMessage("AKKU KRITISCH", batMsg, "SOFORT laden!", 3000);
      beepPattern(4, 600, 150, 100);  // 4x tiefer dringender Warnton
      lastBatteryWarning = millis();
      lowBatteryWarningShown = true;
    }
  } else {
    lowBatteryWarningShown = false;  // Reset wenn Akku wieder geladen
  }

  // LED
  rgbUpdate();

  // Buzzer (non-blocking Zustandsautomat)
  buzzerTick();

  // Screensaver Update (schneller als Display-Status)
  static uint32_t lastScreensaver = 0;
  if (millis() - lastScreensaver > 100) {
    display.updateScreensaver();
    lastScreensaver = millis();
  }

  // Display (1 Hz) - nur wenn Screensaver nicht aktiv und keine Message-Lock
  static uint32_t lastDisp=0; 
  if (!display.isScreensaverActive() && !display.isMessageLocked() && millis()-lastDisp > 1000) {
    bool wifiAP = (WiFi.getMode() & WIFI_MODE_AP) && (WiFi.softAPIP() != IPAddress(0,0,0,0));
    bool wifiClient = isWiFiClientConnected();
    uint8_t wifiCli = wifiAP ? WiFi.softAPgetStationNum() : 0;
    
    // RX/TX Aktivität der letzten Sekunde
    uint32_t now = millis();
    bool rxActive = (now - lastRxActivity) < 500;
    bool txActive = (now - lastTxActivity) < 500;
    
    display.updateStatus(isUSBMode?"USB":"RS232", isUSBMode?usbHostConnected():true, batLevel,
               deviceConnected, bleAdvertising, wifiAP || wifiClient, wifiClient, wifiCli,
               currentBaudRate, rxActive, txActive, chargingNow);
    lastDisp=millis();
  }
}
