#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <U8g2lib.h>
#include <atomic>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C

class Display {
public:
    Display();
    void begin();
    void showLogo();
    void updateStatus(const char* mode, bool connected, int batteryLevel, 
                     bool bleConnected, bool bleAdvertising, bool wifiConnected, bool wifiClientMode, uint8_t wifiClients,
                     uint32_t baudRate, bool rxActivity = false, bool txActivity = false, bool isCharging = false);
    void showMessage(const char* line1, const char* line2 = nullptr, const char* line3 = nullptr);
    void clear();
    void refresh();
    bool isConnected() const { return displayFound; }
    
    // Screensaver
    void setScreensaverTimeout(uint32_t timeoutMs) { screensaverTimeoutMs = timeoutMs; }
    void resetActivity() { lastActivityMs = millis(); screensaverActive = false; }
    bool isScreensaverActive() const { return screensaverActive; }
    void updateScreensaver();
    
    // Message-Lock: Verhindert Überschreiben von showMessage für eine bestimmte Zeit
    void showMessage(const char* line1, const char* line2, const char* line3, uint32_t lockMs);
    bool isMessageLocked() const { return (millis() - messageLockStartMs) < messageLockDurationMs.load(std::memory_order_relaxed); }
    void clearMessageLock() { messageLockDurationMs.store(0, std::memory_order_relaxed); }

private:
    U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2;
    bool useSH1106 = true;
    bool displayFound = false;
    
    // Screensaver
    uint32_t screensaverTimeoutMs = 5 * 60 * 1000;  // 5 Minuten default
    uint32_t lastActivityMs = 0;
    bool screensaverActive = false;
    
    // Message-Lock
    uint32_t messageLockStartMs = 0;
    std::atomic<uint32_t> messageLockDurationMs{0};
    int8_t screensaverX = 0;
    int8_t screensaverY = 0;
    int8_t screensaverDx = 1;
    int8_t screensaverDy = 1;
    
    bool scanI2C(uint8_t address);
    void drawBatteryIcon(int level, bool isCharging);
    void drawHeader(int batteryLevel, bool isCharging);
    void drawPortIcon(int x, int y, const char* mode);
    void drawConnectionDot(int x, int y, bool connected);
    void drawActivityIndicator(int x, int y, bool rx, bool tx);
    String formatBaudRate(uint32_t baud);
};
