#include "display.h"
#include <stdio.h>
#include <cstring>

// Firmware Version (aus git_version.py build flags)
#ifndef GIT_VERSION
#define GIT_VERSION "dev"
#endif

// U8g2 SH1106-only implementation
Display::Display() : u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE), displayFound(false) {
	useSH1106 = true;
}

bool Display::scanI2C(uint8_t address) {
	Wire.beginTransmission(address);
	return (Wire.endTransmission() == 0);
}

void Display::begin() {
	Wire.setClock(400000);
	
	// Prüfe ob Display auf I2C-Adresse 0x3C oder 0x3D antwortet
	displayFound = scanI2C(0x3C) || scanI2C(0x3D);
	
	if (!displayFound) return;
	
	u8g2.begin();
	u8g2.clearBuffer();
	u8g2.setFont(u8g2_font_ncenB08_tr);
	u8g2.sendBuffer();
}

void Display::showLogo() {
	if (!displayFound) return;
	
	u8g2.clearBuffer();
	
	// === "ConNect" Schriftzug (groß, oben zentriert) ===
	u8g2.setFont(u8g2_font_ncenB14_tr);
	u8g2.drawStr(28, 14, "ConNect");
	
	// === Trennlinie ===
	u8g2.drawHLine(4, 18, 120);
	
	// === Mittlerer Bereich: Icons nebeneinander mit Abstand ===
	// Layout: [USB] [232] --> [WiFi] [BLE]
	// Y-Position für Icons: 24-44 (20 Pixel hoch)
	// Y-Position für Text:  50
	
	// --- USB Icon (X: 4-18) ---
	u8g2.drawFrame(4, 24, 14, 18);
	u8g2.drawBox(6, 27, 4, 4);    // Pin links
	u8g2.drawBox(11, 27, 4, 4);   // Pin rechts
	u8g2.drawBox(8, 33, 5, 5);    // Mitte
	u8g2.setFont(u8g2_font_5x7_tr);
	u8g2.drawStr(4, 50, "USB");
	
	// --- RS232 Icon (X: 22-36) ---
	u8g2.drawFrame(22, 24, 14, 18);
	u8g2.drawBox(25, 28, 3, 3);   // Pin oben-links
	u8g2.drawBox(30, 28, 3, 3);   // Pin oben-rechts
	u8g2.drawBox(25, 34, 3, 3);   // Pin unten-links
	u8g2.drawBox(30, 34, 3, 3);   // Pin unten-rechts
	u8g2.drawStr(22, 50, "232");
	
	// --- Pfeile in der Mitte (X: 42-62) ---
	// Doppelpfeil -->
	u8g2.drawHLine(44, 32, 14);
	u8g2.drawPixel(56, 30);
	u8g2.drawPixel(57, 31);
	u8g2.drawPixel(57, 33);
	u8g2.drawPixel(56, 34);
	u8g2.drawHLine(44, 38, 14);
	u8g2.drawPixel(56, 36);
	u8g2.drawPixel(57, 37);
	u8g2.drawPixel(57, 39);
	u8g2.drawPixel(56, 40);
	
	// --- WiFi Icon (X: 68-86) ---
	// Einfaches WiFi: Punkt mit Bögen darüber
	int wifiCx = 77;
	u8g2.drawDisc(wifiCx, 40, 2);  // Punkt unten
	u8g2.drawCircle(wifiCx, 42, 6, U8G2_DRAW_UPPER_LEFT | U8G2_DRAW_UPPER_RIGHT);
	u8g2.drawCircle(wifiCx, 44, 10, U8G2_DRAW_UPPER_LEFT | U8G2_DRAW_UPPER_RIGHT);
	u8g2.drawStr(68, 50, "WiFi");
	
	// --- BLE Icon (X: 94-114) ---
	// Bluetooth "B" Rune
	int bx = 104;
	u8g2.drawVLine(bx, 26, 16);           // Vertikale Linie |
	u8g2.drawLine(bx, 26, bx + 5, 30);    // Oben: \ nach rechts
	u8g2.drawLine(bx + 5, 30, bx, 34);    // Mitte: / zurück
	u8g2.drawLine(bx, 34, bx + 5, 38);    // Mitte: \ nach rechts  
	u8g2.drawLine(bx + 5, 38, bx, 42);    // Unten: / zurück
	u8g2.drawStr(96, 50, "BLE");
	
	// === Untertitel (ganz unten) ===
	u8g2.setFont(u8g2_font_5x8_tr);
	u8g2.drawStr(8, 62, "Wireless Serial Bridge");
	
	u8g2.sendBuffer();
	delay(2000);
}

void Display::updateStatus(const char* mode, bool connected, int batteryLevel,
						   bool bleConnected, bool bleAdvertising, bool wifiConnected, bool wifiClientMode, uint8_t wifiClients,
						   uint32_t baudRate, bool rxActivity, bool txActivity, bool isCharging) {
	if (!displayFound) return;
	
	static bool blink = false;
	static uint32_t lastBlink = 0;
	if (millis() - lastBlink >= 500) { blink = !blink; lastBlink = millis(); }

	u8g2.clearBuffer();
	u8g2.setDrawColor(1);
	drawHeader(batteryLevel, isCharging);

	// === Mittlerer Bereich: Modus + Baudrate + Verbindungsstatus ===
	const int centerX = 64;
	const int midY = 28;
	
	// Port-Icon (USB oder RS232)
	drawPortIcon(centerX - 20, midY - 4, mode);
	
	// Modus-Text rechts neben Icon
	u8g2.setFont(u8g2_font_ncenB08_tr);
	u8g2.drawStr(centerX - 4, midY + 4, mode);
	
	// Baudrate darunter
	String baudStr = formatBaudRate(baudRate);
	int baudW = baudStr.length() * 6;
	u8g2.drawStr(centerX - baudW/2, midY + 16, baudStr.c_str());
	
	// Verbindungs-Indikator (gefüllter/leerer Kreis)
	drawConnectionDot(centerX + 28, midY + 12, connected);
	
	// RX/TX-Aktivitätsanzeige
	drawActivityIndicator(centerX + 36, midY - 2, rxActivity, txActivity);

	// === Unterer Bereich: WiFi und BLE Status (ohne Rahmen) ===
	const int bottomY = 58;
	u8g2.setFont(u8g2_font_ncenB08_tr);
	
	// WiFi Status (links)
	char wifiStr[16];
	if (wifiConnected && wifiClientMode) {
		snprintf(wifiStr, sizeof(wifiStr), "WLAN%c", blink ? '*' : ' ');
	} else if (wifiConnected) {
		snprintf(wifiStr, sizeof(wifiStr), "WiFi %u%c", wifiClients, (wifiClients > 0 && blink) ? '*' : ' ');
	} else {
		snprintf(wifiStr, sizeof(wifiStr), "WiFi Aus");
	}
	u8g2.drawStr(4, bottomY, wifiStr);
	// WiFi aktiv-Punkt
	if (wifiConnected && (wifiClientMode || wifiClients > 0)) {
		u8g2.drawDisc(2, bottomY - 4, 1);
	}
	
	// BLE Status (rechts)
	char bleStr[16];
	if (bleConnected) {
		snprintf(bleStr, sizeof(bleStr), "BLE");
	} else if (bleAdvertising) {
		snprintf(bleStr, sizeof(bleStr), "BLE%c", blink ? '*' : ' ');
	} else {
		snprintf(bleStr, sizeof(bleStr), "BLE Aus");
	}
	int bleW = strlen(bleStr) * 6;
	u8g2.drawStr(128 - bleW - 4, bottomY, bleStr);
	// BLE aktiv-Punkt
	if (bleConnected) {
		u8g2.drawDisc(126, bottomY - 4, 1);
	}

	u8g2.sendBuffer();
}

void Display::drawBatteryIcon(int level, bool isCharging) {
	const int BAT_X = 106; const int BAT_Y = 1; const int BAT_WIDTH = 18; const int BAT_HEIGHT = 10;
	u8g2.setDrawColor(1);
	// outline
	u8g2.drawFrame(BAT_X, BAT_Y, BAT_WIDTH, BAT_HEIGHT);
	// terminal pin (small filled rect)
	u8g2.drawBox(BAT_X + BAT_WIDTH, BAT_Y + (BAT_HEIGHT/2 - 2), 2, 4);
	// inner fill representing charge level
	if (level > 0) {
		int innerW = BAT_WIDTH - 2;
		int fillWidth = (level * innerW) / 100;
		if (fillWidth > 0) u8g2.drawBox(BAT_X + 1, BAT_Y + 1, fillWidth, BAT_HEIGHT - 2);
	}
	
	// Blitz-Symbol wenn Laden (pixelgenau für 8 Pixel Höhe)
	// Das Symbol ist 5x8 Pixel und wird zentriert im Batterie-Icon gezeichnet
	if (isCharging) {
		int bx = BAT_X + BAT_WIDTH/2 - 2;  // Zentriert
		int by = BAT_Y + 1;                 // 1 Pixel vom oberen Rand
		
		// Blitz-Symbol als Pixel-Art (invertiert auf Füllung)
		// Form:  ##
		//       ##
		//      ####
		//       ##
		//      ##
		u8g2.setDrawColor(0);  // Schwarz (invertiert) auf weißer Füllung
		
		// Oberer Teil (schräg von rechts nach links)
		u8g2.drawPixel(bx + 3, by);
		u8g2.drawPixel(bx + 4, by);
		u8g2.drawPixel(bx + 2, by + 1);
		u8g2.drawPixel(bx + 3, by + 1);
		
		// Mitte (breiterer Teil)
		u8g2.drawPixel(bx + 1, by + 2);
		u8g2.drawPixel(bx + 2, by + 2);
		u8g2.drawPixel(bx + 3, by + 2);
		u8g2.drawPixel(bx + 4, by + 2);
		
		// Mittelpunkt
		u8g2.drawPixel(bx + 2, by + 3);
		u8g2.drawPixel(bx + 3, by + 3);
		
		// Unterer Teil (schräg von rechts nach links)
		u8g2.drawPixel(bx + 1, by + 4);
		u8g2.drawPixel(bx + 2, by + 4);
		u8g2.drawPixel(bx + 0, by + 5);
		u8g2.drawPixel(bx + 1, by + 5);
		
		u8g2.setDrawColor(1);
	}
}

void Display::drawHeader(int batteryLevel, bool isCharging) {
	// Einheitliches Design: Weißer Text auf schwarzem Grund
	u8g2.setDrawColor(1);
	u8g2.setFont(u8g2_font_ncenB08_tr);
	u8g2.drawStr(2, 10, "ConNect");

	batteryLevel = constrain(batteryLevel, 0, 100);
	char pct[8];
	if (isCharging) {
		snprintf(pct, sizeof(pct), "%d%%+", batteryLevel);  // + zeigt Laden an
	} else {
		snprintf(pct, sizeof(pct), "%d%%", batteryLevel);
	}
	// Batterie-Icon bei X=106, daher Text mit 6px Abstand
	int pctX = 106 - 6 - (int)(strlen(pct) * 6);
	if (pctX < 50) pctX = 50;
	u8g2.drawStr(pctX, 10, pct);

	drawBatteryIcon(batteryLevel, isCharging);
	
	// Trennlinie unter dem Header
	u8g2.drawHLine(0, 14, 128);
}

void Display::drawPortIcon(int x, int y, const char* mode) {
	// Größeres Icon für bessere Sichtbarkeit
	u8g2.setDrawColor(1);

	if (mode && strcmp(mode, "RS232") == 0) {
		// DB9-Style Icon (größer)
		u8g2.drawFrame(x, y, 18, 12);
		// Trapez-Form andeuten
		u8g2.drawLine(x, y, x + 2, y + 2);
		u8g2.drawLine(x + 17, y, x + 15, y + 2);
		// Pins (3x2 Raster)
		for (int i = 0; i < 3; i++) {
			u8g2.drawBox(x + 4 + i * 4, y + 4, 2, 2);
			u8g2.drawBox(x + 6 + i * 4, y + 7, 2, 2);
		}
	} else {
		// USB-Icon (größer, erkennbarer)
		// Stecker-Körper
		u8g2.drawFrame(x, y + 2, 12, 10);
		u8g2.drawBox(x + 2, y + 4, 8, 6);
		// USB-Anschluss
		u8g2.drawBox(x + 12, y + 5, 4, 4);
		// USB-Symbol (Dreizack)
		u8g2.drawLine(x + 6, y + 2, x + 6, y);
		u8g2.drawPixel(x + 4, y + 1);
		u8g2.drawPixel(x + 8, y + 1);
	}
}

void Display::drawConnectionDot(int x, int y, bool connected) {
	// Gefüllter Kreis = verbunden, leerer Kreis = nicht verbunden
	u8g2.setDrawColor(1);
	if (connected) {
		u8g2.drawDisc(x, y, 3);
	} else {
		u8g2.drawCircle(x, y, 3);
	}
}

String Display::formatBaudRate(uint32_t baud) {
	// Kompakte Darstellung der Baudrate
	if (baud >= 1000000) {
		return String(baud / 1000000) + "." + String((baud % 1000000) / 100000) + "M";
	} else if (baud >= 100000) {
		return String(baud / 1000) + "k";
	} else if (baud >= 10000) {
		return String(baud / 1000) + "." + String((baud % 1000) / 100) + "k";
	} else {
		return String(baud);
	}
}

void Display::clear() { if (displayFound) u8g2.clearBuffer(); }

void Display::refresh() { if (displayFound) u8g2.sendBuffer(); }

void Display::showMessage(const char* line1, const char* line2, const char* line3) {
	// Ohne Lock-Zeit: Standard-Verhalten
	showMessage(line1, line2, line3, 0);
}

void Display::showMessage(const char* line1, const char* line2, const char* line3, uint32_t lockMs) {
	if (!displayFound) return;
	
	// Reset screensaver bei Nachricht
	lastActivityMs = millis();
	screensaverActive = false;
	
	// Message-Lock setzen wenn gewünscht
	if (lockMs > 0) {
		messageLockStartMs = millis();
		messageLockDurationMs.store(lockMs, std::memory_order_relaxed);
	}
	
	u8g2.clearBuffer();
	u8g2.setDrawColor(1);
	u8g2.setFont(u8g2_font_ncenB08_tr);
	
	// Zentrierte Darstellung
	int y = 24;
	if (line1 && line1[0]) {
		int w = strlen(line1) * 6;
		u8g2.drawStr(64 - w/2, y, line1);
		y += 14;
	}
	if (line2 && line2[0]) {
		int w = strlen(line2) * 6;
		u8g2.drawStr(64 - w/2, y, line2);
		y += 14;
	}
	if (line3 && line3[0]) {
		int w = strlen(line3) * 6;
		u8g2.drawStr(64 - w/2, y, line3);
	}
	
	u8g2.sendBuffer();
}

void Display::drawActivityIndicator(int x, int y, bool rx, bool tx) {
	// RX-Pfeil (nach unten) und TX-Pfeil (nach oben)
	u8g2.setDrawColor(1);
	
	// RX (empfangen) - Pfeil nach unten
	if (rx) {
		u8g2.drawTriangle(x, y + 8, x + 4, y + 8, x + 2, y + 12);
		u8g2.drawBox(x + 1, y + 4, 2, 4);
	} else {
		// Nur Umriss wenn inaktiv
		u8g2.drawLine(x, y + 8, x + 2, y + 12);
		u8g2.drawLine(x + 4, y + 8, x + 2, y + 12);
		u8g2.drawPixel(x + 2, y + 6);
	}
	
	// TX (senden) - Pfeil nach oben
	if (tx) {
		u8g2.drawTriangle(x + 8, y + 4, x + 12, y + 4, x + 10, y);
		u8g2.drawBox(x + 9, y + 4, 2, 4);
	} else {
		// Nur Umriss wenn inaktiv
		u8g2.drawLine(x + 8, y + 4, x + 10, y);
		u8g2.drawLine(x + 12, y + 4, x + 10, y);
		u8g2.drawPixel(x + 10, y + 6);
	}
}

void Display::updateScreensaver() {
	if (!displayFound) return;
	
	// Prüfe ob Timeout erreicht
	if (!screensaverActive && (millis() - lastActivityMs) > screensaverTimeoutMs) {
		screensaverActive = true;
		screensaverX = random(0, 80);
		screensaverY = random(10, 40);
	}
	
	if (!screensaverActive) return;
	
	// Bouncing "ConNect" Logo
	u8g2.clearBuffer();
	u8g2.setDrawColor(1);
	u8g2.setFont(u8g2_font_ncenB10_tr);
	u8g2.drawStr(screensaverX, screensaverY, "ConNect");
	u8g2.sendBuffer();
	
	// Bewegung
	screensaverX += screensaverDx;
	screensaverY += screensaverDy;
	
	// Bounce an den Rändern (Text ist ca. 60px breit, 12px hoch)
	if (screensaverX <= 0 || screensaverX >= 68) screensaverDx = -screensaverDx;
	if (screensaverY <= 12 || screensaverY >= 64) screensaverDy = -screensaverDy;
}

