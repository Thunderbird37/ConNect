#pragma once
#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <atomic>

// Forward declaration - WebServer wird in .cpp definiert
class WebServer;

// Forward declarations für main.cpp Variablen.
// deviceConnected/bleAdvertising werden aus BLE-Callback (anderer Task)
// geschrieben und im loop() + Webserver gelesen -> std::atomic<bool>.
// Die Stats-Zaehler werden aus mehreren Kontexten inkrementiert ->
// std::atomic<uint32_t>.
extern bool isUSBMode;
extern uint32_t currentBaudRate;
extern std::atomic<bool> deviceConnected;
extern std::atomic<bool> bleAdvertising;

// Stats
extern std::atomic<uint32_t> stat_usb_rx, stat_usb_tx;
extern std::atomic<uint32_t> stat_rs232_rx, stat_rs232_tx;

// External functions
extern uint8_t getBatteryLevel();
extern bool usbHostConnected();
extern const char* getConfiguredWiFiModeValue();
extern const char* getActiveWiFiModeLabel();
extern bool isWiFiClientConnected();
extern bool isWiFiFallbackAccessPoint();
extern String getWiFiIpAddress();
extern String getWiFiHostname();

// Preferences für OTA-Einstellungen
#include <Preferences.h>
extern Preferences prefs;

class ConNectWebServer {
public:
    void begin();
    void handleClient();
    void stop();
    bool isRunning() const { return _running; }
    
    // Web-Authentifizierung
    static constexpr const char* AUTH_USER = "admin";
    static constexpr const char* AUTH_PASS = "connect";
    
private:
    WebServer* _server = nullptr;
    bool _running = false;
    
    bool checkAuth();  // Basic Auth prüfen
    void handleRoot();
    void handleStatus();
    void handleConfig();
    void handleSave();
    void handleReboot();
    void handleTerminalRead();
    void handleTerminalSend();
    void handleTerminalClear();
    void handleTerminalDownload();
    void handleNotFound();
    
    String getStatusJSON();
    void streamHTML();
};

extern ConNectWebServer webServer;

void webConsolePushRx(char c);
void webConsolePushRxBulk(const char* buf, size_t len);
size_t webConsoleReadTx(uint8_t* buffer, size_t maxLen);
