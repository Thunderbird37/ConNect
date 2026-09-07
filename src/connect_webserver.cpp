#include <WebServer.h>
#include "esp_task_wdt.h"
#include "esp_heap_caps.h"
#include "connect_webserver.h"
#include "config_validation.h"
#include "ota_config.h"
#include "usb_host_bridge.h"

// Git version info (from build flags)
#ifndef GIT_VERSION
#define GIT_VERSION "dev"
#endif
#ifndef GIT_COMMIT
#define GIT_COMMIT "unknown"
#endif
#ifndef BUILD_TIME
#define BUILD_TIME "unknown"
#endif

ConNectWebServer webServer;
extern void setSerialTargetMode(bool usbMode);
extern void rs232ReinitUart();

static String getConfiguredOTAManifestUrl() {
    String url = prefs.getString("otaUrl", OTAConfig::DEFAULT_MANIFEST_URL);
    url.trim();
    if (url.length() == 0) {
        return OTAConfig::DEFAULT_MANIFEST_URL;
    }
    return url;
}

// Feste Byte-Ringpuffer-Implementierung (kein Arduino String!):
// Arduino String += c reallokiert bei jedem Byte -> O(n^2) und starke
// Heap-Fragmentierung bei langen Bursts (z.B. `sh run`). Mit einem festen
// char[] sind Pushes amortisiert O(1); Trim wird durch memmove gemacht und
// passiert nur wenn der Puffer voll ist.
static constexpr size_t TERMINAL_RX_MAX = 16384;
static constexpr size_t TERMINAL_TX_MAX = 4096;
static char    terminalRxBuf[TERMINAL_RX_MAX];
static size_t   terminalRxLen  = 0;
static uint64_t terminalRxBase = 0;   // absoluter Offset von terminalRxBuf[0]
                                      // uint64_t: bei 1 MB/s Dauerlast reichen
                                      // size_t (32 bit) nur ~70 min; uint64_t
                                      // ~500 Tausend Jahre.
static String  terminalTxBuffer;

class HtmlChunkWriter {
public:
    explicit HtmlChunkWriter(WebServer& server) : server(server) {}

    HtmlChunkWriter& operator+=(const char* value) {
        if (value) append(value, strlen(value));
        return *this;
    }

    HtmlChunkWriter& operator+=(const String& value) {
        append(value.c_str(), value.length());
        return *this;
    }

    void appendProgmem(PGM_P value, size_t length) {
        while (length > 0 && healthy) {
            size_t writable = min(sizeof(buffer) - used, length);
            memcpy_P(buffer + used, value, writable);
            used += writable;
            value += writable;
            length -= writable;
            if (used == sizeof(buffer)) flush();
        }
    }

    void finish() {
        flush();
        if (healthy && server.client().connected()) {
            server.sendContent("", 0);
        }
    }

private:
    void append(const char* value, size_t length) {
        while (length > 0 && healthy) {
            size_t writable = min(sizeof(buffer) - used, length);
            memcpy(buffer + used, value, writable);
            used += writable;
            value += writable;
            length -= writable;
            if (used == sizeof(buffer)) flush();
        }
    }

    void flush() {
        if (used == 0 || !healthy) return;
        if (!server.client().connected()) {
            healthy = false;
            used = 0;
            return;
        }
        server.sendContent(buffer, used);
        used = 0;
        esp_task_wdt_reset();
        delay(1);
    }

    WebServer& server;
    char buffer[512];
    size_t used = 0;
    bool healthy = true;
};

static String jsonEscape(const String& value) {
    String out;
    out.reserve(value.length() + 8);
    for (size_t i = 0; i < value.length(); ++i) {
        char c = value.charAt(i);
        switch (c) {
        case '"': out += "\\\""; break;
        case '\\': out += "\\\\"; break;
        case '\b': out += "\\b"; break;
        case '\f': out += "\\f"; break;
        case '\n': out += "\\n"; break;
        case '\r': out += "\\r"; break;
        case '\t': out += "\\t"; break;
        default:
            if ((uint8_t)c < 0x20) {
                char esc[7];
                snprintf(esc, sizeof(esc), "\\u%04x", (unsigned)c);
                out += esc;
            } else {
                out += c;
            }
            break;
        }
    }
    return out;
}

static String htmlAttrEscape(const String& value) {
    String out;
    out.reserve(value.length() + 8);
    for (size_t i = 0; i < value.length(); ++i) {
        char c = value.charAt(i);
        switch (c) {
        case '&': out += "&amp;"; break;
        case '"': out += "&quot;"; break;
        case '\'': out += "&#39;"; break;
        case '<': out += "&lt;"; break;
        case '>': out += "&gt;"; break;
        default: out += c; break;
        }
    }
    return out;
}

void webConsolePushRx(char c) {
    if (terminalRxLen >= TERMINAL_RX_MAX) {
        // Puffer voll: aeltestes Viertel verwerfen, damit wir nicht bei jedem
        // Byte memmove-n muessen. Amortisiert bleibt das O(1) pro Byte.
        size_t drop = TERMINAL_RX_MAX / 4;
        memmove(terminalRxBuf, terminalRxBuf + drop, terminalRxLen - drop);
        terminalRxLen  -= drop;
        terminalRxBase += drop;
    }
    terminalRxBuf[terminalRxLen++] = c;
}

void webConsolePushRxBulk(const char* buf, size_t len) {
    if (!buf || len == 0) return;

    // If incoming data is larger than max buffer size, just take the end part
    size_t skippedIncoming = 0;
    if (len >= TERMINAL_RX_MAX) {
        skippedIncoming = len - TERMINAL_RX_MAX;
        buf += (len - TERMINAL_RX_MAX);
        len = TERMINAL_RX_MAX;
    }

    if (terminalRxLen + len > TERMINAL_RX_MAX) {
        size_t drop = TERMINAL_RX_MAX / 4;
        size_t neededDrop = (terminalRxLen + len) - TERMINAL_RX_MAX;
        if (drop < neededDrop) drop = neededDrop;

        // Prevent underflow if drop is greater than current buffer length
        if (drop > terminalRxLen) drop = terminalRxLen;

        memmove(terminalRxBuf, terminalRxBuf + drop, terminalRxLen - drop);
        terminalRxLen  -= drop;
        terminalRxBase += drop;
    }

    terminalRxBase += skippedIncoming;
    memcpy(terminalRxBuf + terminalRxLen, buf, len);
    terminalRxLen += len;
}

size_t webConsoleReadTx(uint8_t* buffer, size_t maxLen) {
    if (!buffer || maxLen == 0 || terminalTxBuffer.length() == 0) return 0;
    size_t len = terminalTxBuffer.length();
    size_t toCopy = len < maxLen ? len : maxLen;
    memcpy(buffer, terminalTxBuffer.c_str(), toCopy);
    terminalTxBuffer.remove(0, toCopy);
    return toCopy;
}

static const char HTML_HEADER[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="de">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>ConNect - Web Interface</title>
    <style>
        :root {
            --bg: #0b1020;
            --bg-soft: #121a33;
            --card: #16213e;
            --accent: #243b6b;
            --text: #e7edff;
            --muted: #9fb3dc;
            --highlight: #3ac8ff;
            --success: #00dd9b;
            --warning: #ffb454;
            --danger: #ff5d73;
            --terminal: #060b15;
        }
        * { box-sizing: border-box; margin: 0; padding: 0; }
        body {
            font-family: 'Segoe UI', system-ui, sans-serif;
            background: radial-gradient(1000px 500px at 10% -10%, #22386f 0%, transparent 60%),
                        radial-gradient(1000px 500px at 90% 0%, #173e53 0%, transparent 55%),
                        var(--bg);
            color: var(--text);
            min-height: 100vh;
            padding: 20px;
        }
        .container { max-width: 980px; margin: 0 auto; }
        .tabs {
            display: flex;
            gap: 10px;
            margin-bottom: 16px;
            flex-wrap: wrap;
        }
        .tab-btn {
            background: #142447;
            color: var(--muted);
            border: 1px solid #2a4b82;
            border-radius: 10px;
            padding: 10px 14px;
            font-weight: 700;
            cursor: pointer;
        }
        .tab-btn.active {
            background: var(--highlight);
            color: #041126;
            border-color: var(--highlight);
        }
        .tab-panel { display: none; }
        .tab-panel.active { display: block; }
        h1 {
            text-align: center;
            color: var(--highlight);
            margin-bottom: 24px;
            font-size: 2em;
            letter-spacing: 0.5px;
        }
        .card {
            background: linear-gradient(180deg, #1a2649 0%, #151f3f 100%);
            border: 1px solid rgba(144, 193, 255, 0.18);
            border-radius: 16px;
            padding: 20px;
            margin-bottom: 20px;
            box-shadow: 0 10px 24px rgba(0,0,0,0.28);
        }
        .card h2 {
            color: var(--highlight);
            margin-bottom: 15px;
            font-size: 1.2em;
            border-bottom: 1px solid var(--accent);
            padding-bottom: 10px;
        }
        .status-grid {
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(150px, 1fr));
            gap: 15px;
        }
        .status-item {
            background: var(--accent);
            padding: 15px;
            border-radius: 12px;
            text-align: center;
            border: 1px solid rgba(144, 193, 255, 0.18);
        }
        .status-item .label {
            font-size: 0.8em;
            color: var(--muted);
            margin-bottom: 5px;
        }
        .status-item .value {
            font-size: 1.4em;
            font-weight: bold;
        }
        .status-item .value.online { color: var(--success); }
        .status-item .value.offline { color: var(--danger); }
        .status-item .value.active { color: var(--highlight); }
        .form-group {
            margin-bottom: 15px;
        }
        .form-group label {
            display: block;
            margin-bottom: 5px;
            color: var(--muted);
        }
        .form-group input, .form-group select {
            width: 100%;
            padding: 10px;
            border: 1px solid var(--accent);
            border-radius: 6px;
            background: var(--bg);
            color: var(--text);
            font-size: 1em;
        }
        .form-group input:focus, .form-group select:focus {
            outline: none;
            border-color: var(--highlight);
        }
        .checkbox-row {
            display: flex !important;
            align-items: center;
            gap: 8px;
        }
        .checkbox-row input {
            width: auto;
        }
        .btn {
            display: inline-block;
            padding: 12px 24px;
            border: none;
            border-radius: 10px;
            cursor: pointer;
            font-size: 1em;
            font-weight: bold;
            transition: all 0.2s;
        }
        .btn-primary {
            background: var(--highlight);
            color: var(--bg);
        }
        .btn-primary:hover { background: #00b8d9; }
        .btn:active { transform: translateY(1px); }
        .btn-danger {
            background: var(--danger);
            color: white;
        }
        .btn-danger:hover { background: #cc3333; }
        .btn-group {
            display: flex;
            gap: 10px;
            flex-wrap: wrap;
        }
        .stats-table {
            width: 100%;
            border-collapse: collapse;
        }
        .stats-table td {
            padding: 8px;
            border-bottom: 1px solid var(--accent);
        }
        .stats-table td:last-child {
            text-align: right;
            font-family: monospace;
        }
        .version {
            text-align: center;
            color: var(--muted);
            font-size: 0.8em;
            margin-top: 20px;
        }
        .terminal-wrap {
            background: var(--terminal);
            border: 1px solid #274070;
            border-radius: 12px;
            padding: 10px;
        }
        #terminal {
            background: #030814;
            border-radius: 10px;
            border: 1px solid #1f355f;
            min-height: 280px;
            max-height: 48vh;
            overflow-y: auto;
            padding: 12px;
            white-space: pre-wrap;
            font-family: ui-monospace, SFMono-Regular, Menlo, Consolas, monospace;
            font-size: 13px;
            line-height: 1.35;
            color: #d7e6ff;
        }
        .term-row {
            display: flex;
            gap: 10px;
            margin-top: 10px;
            align-items: center;
        }
        .term-row input {
            flex: 1;
            min-height: 48px;
            font-size: 1.02em;
            padding: 12px 14px;
        }
        .term-actions {
            display: flex;
            gap: 8px;
            flex-wrap: wrap;
            margin-top: 10px;
        }
        .btn-ghost {
            background: transparent;
            border: 1px solid var(--accent);
            color: var(--text);
            padding: 10px 14px;
        }
        .mobile-keys {
            display: grid;
            grid-template-columns: repeat(6, minmax(48px, 1fr));
            gap: 6px;
            margin-top: 10px;
        }
        .mobile-keys button {
            background: #1a2b51;
            color: #dbe9ff;
            border: 1px solid #2b4b80;
            border-radius: 8px;
            padding: 9px 8px;
            font-size: 0.9em;
        }
        @media (max-width: 600px) {
            body { padding: 10px; }
            h1 { font-size: 1.5em; }
            #terminal { min-height: 220px; max-height: 42vh; }
        }
    </style>
</head>
<body>
    <div class="container">
        <h1>🔌 ConNect</h1>
)rawliteral";

static const char HTML_FOOTER[] PROGMEM = R"rawliteral(
    </div>
    <script>
        let termPos = 0;
        let termAutoScroll = true;

        function switchTab(tabName) {
            document.querySelectorAll('.tab-btn').forEach(btn => btn.classList.remove('active'));
            document.querySelectorAll('.tab-panel').forEach(panel => panel.classList.remove('active'));
            const btn = document.getElementById('tab-btn-' + tabName);
            const panel = document.getElementById('tab-' + tabName);
            if (btn) btn.classList.add('active');
            if (panel) panel.classList.add('active');
        }

        function appendTerminal(text) {
            const term = document.getElementById('terminal');
            if (!term || !text) return;
            const atBottom = term.scrollTop + term.clientHeight >= term.scrollHeight - 8;
            term.textContent += text;
            if (term.textContent.length > 65536) {
                term.textContent = term.textContent.slice(-65536);
            }
            if (termAutoScroll || atBottom) {
                term.scrollTop = term.scrollHeight;
            }
        }

        let pollInFlight = false;
        let statusInFlight = false;
        async function pollTerminal() {
            // Re-entrancy guard: Chromium browsers lassen setInterval gerne
            // parallel feuern wenn Antwort >Intervall braucht. Ohne Guard
            // staut sich das auf bis der ESP32-WebServer (single threaded)
            // haengt.
            if (pollInFlight) return;
            if (document.hidden) return;  // Tab nicht sichtbar -> nicht pollen
            pollInFlight = true;
            try {
                const res = await fetch('/terminal/read?from=' + termPos, { cache: 'no-store' });
                if (!res.ok) return;
                const text = await res.text();
                const nextPos = parseInt(res.headers.get('X-Terminal-Total') || '', 10);
                const len = parseInt(res.headers.get('X-Terminal-Len') || 0, 10);
                appendTerminal(text);
                termPos = Number.isFinite(nextPos) ? nextPos : termPos + len;
            } catch (e) {
                console.error('Terminal read failed:', e);
            } finally {
                pollInFlight = false;
            }
        }

        async function sendTerminalData(data) {
            if (!data) return;
            try {
                await fetch('/terminal/send', {
                    method: 'POST',
                    headers: { 'Content-Type': 'application/x-www-form-urlencoded' },
                    body: 'data=' + encodeURIComponent(data)
                });
            } catch (e) {
                console.error('Terminal send failed:', e);
            }
        }

        function terminalSubmit(event) {
            event.preventDefault();
            const input = document.getElementById('terminalInput');
            if (!input) return;
            const value = input.value;
            if (!value) return;
            sendTerminalData(value + '\r');
            input.value = '';
        }

        function terminalKey(value) {
            sendTerminalData(value);
        }

        function appendConNectBanner() {
            const baudEl = document.getElementById('baud');
            const baud = (baudEl && baudEl.textContent) ? baudEl.textContent : 'unbekannt';
            const lines = [
                'Welcome to ConNect!',
                'Current baud rate: ' + baud,
                '=== ConNect Commands ===',
                "Prefix: ` oder '",
                '`usb / `rs232  - Ziel wechseln',
                '`baud N        - Baudrate',
                'Taste 3x       - Hotspot/WLAN',
                '`status        - Systemstatus',
                '`version       - FW-Version',
                '`wlan SSID PW  - WLAN + OTA',
                '`otaurl URL    - OTA Manifest-Link',
                '`ota           - OTA starten',
                'Type `help for commands',
                ''
            ];
            appendTerminal(lines.join('\n') + '\n');
        }

        async function clearTerminal() {
            await fetch('/terminal/clear', { method: 'POST' });
            const term = document.getElementById('terminal');
            term.textContent = '';
            termPos = 0;
            appendConNectBanner();
        }

        function downloadTerminalLog() {
            window.location.href = '/terminal/download';
        }

        function updateStatus() {
            if (statusInFlight) return;
            statusInFlight = true;
            fetch('/status')
                .finally(() => { statusInFlight = false; })
                .then(r => r.json())
                .then(data => {
                    document.getElementById('mode').textContent = data.mode;
                    document.getElementById('mode').className = 'value active';
                    document.getElementById('baud').textContent = data.baudRate;
                    document.getElementById('usb').textContent = data.usbConnected ? 'Verbunden' : 'Getrennt';
                    document.getElementById('usb').className = 'value ' + (data.usbConnected ? 'online' : 'offline');
                    document.getElementById('ble').textContent = data.bleConnected ? 'Verbunden' : (data.bleAdvertising ? 'Werbung' : 'Aus');
                    document.getElementById('ble').className = 'value ' + (data.bleConnected ? 'online' : 'offline');
                    document.getElementById('battery').textContent = data.battery + '%';
                    document.getElementById('clients').textContent = data.wifiClients;
                    document.getElementById('wifi_mode').textContent = data.wifiActiveMode;
                    document.getElementById('wifi_ip').textContent = data.wifiIP;
                    document.getElementById('wifi_host').textContent = data.wifiHostname || '-';
                    document.getElementById('usb_rx').textContent = data.stats.usb_rx;
                    document.getElementById('usb_tx').textContent = data.stats.usb_tx;
                    document.getElementById('usb_drop').textContent = data.stats.usb_drop;
                    document.getElementById('usb_rx_queue').textContent = data.stats.usb_rx_queue;
                    document.getElementById('rs232_rx').textContent = data.stats.rs232_rx;
                    document.getElementById('rs232_tx').textContent = data.stats.rs232_tx;
                })
                .catch(e => console.error('Status update failed:', e));
        }

        function initWebUi() {
            switchTab('settings');
            const term = document.getElementById('terminal');
            if (term) {
                term.addEventListener('scroll', () => {
                    termAutoScroll = term.scrollTop + term.clientHeight >= term.scrollHeight - 8;
                });
                if (term.textContent.trim().length === 0) {
                    appendConNectBanner();
                }
            }
            const input = document.getElementById('terminalInput');
            if (input) {
                input.addEventListener('keydown', (e) => {
                    if (e.key === 'Enter') {
                        e.preventDefault();
                        terminalSubmit(e);
                    }
                });
            }
        }

        if (document.readyState === 'loading') {
            document.addEventListener('DOMContentLoaded', initWebUi);
        } else {
            initWebUi();
        }

        setInterval(updateStatus, 2000);
        // 250ms statt 100ms: 10 req/s war zu viel fuer den ESP32-WebServer
        // (single thread, Chrome/Edge oeffnen mehrere parallele Connections
        // -> Server haengt). 4 req/s laeuft stabil.
        setInterval(pollTerminal, 250);
        updateStatus();
        pollTerminal();
    </script>
</body>
</html>
)rawliteral";

bool ConNectWebServer::checkAuth() {
    if (!_server->authenticate(AUTH_USER, AUTH_PASS)) {
        _server->requestAuthentication();
        return false;
    }
    return true;
}

void ConNectWebServer::begin() {
    if (_server) delete _server;
    _server = new WebServer(80);
    
    _server->on("/", HTTP_GET, [this]() { handleRoot(); });
    _server->on("/status", HTTP_GET, [this]() { handleStatus(); });
    _server->on("/config", HTTP_GET, [this]() { handleConfig(); });
    _server->on("/save", HTTP_POST, [this]() { handleSave(); });
    _server->on("/reboot", HTTP_POST, [this]() { handleReboot(); });
    _server->on("/terminal/read", HTTP_GET, [this]() { handleTerminalRead(); });
    _server->on("/terminal/send", HTTP_POST, [this]() { handleTerminalSend(); });
    _server->on("/terminal/clear", HTTP_POST, [this]() { handleTerminalClear(); });
    _server->on("/terminal/download", HTTP_GET, [this]() { handleTerminalDownload(); });

    // Favicon: ohne diesen Handler fragt Chrome/Edge bei jedem Request
    // /favicon.ico nach und bekommt einen 404 vom onNotFound-Handler -
    // unnoetige Last auf dem single-threaded WebServer. Wir geben einen
    // leeren 1x1-Pixel ICO direkt mit Caching zurueck.
    _server->on("/favicon.ico", HTTP_GET, [this]() {
        _server->sendHeader("Cache-Control", "public, max-age=86400");
        _server->send(204, "image/x-icon", "");
    });

    // Captive-Portal Detection Endpoints. Chrome/Edge/Windows/macOS
    // probieren beim Verbinden mit einem AP folgende URLs ueber HTTP. Wenn
    // sie nicht beantwortet werden (oder 401 zurueckkommt), markiert das
    // OS das WLAN als "captive" und der Browser blockiert teilweise die
    // Navigation. Wir antworten mit 204 No Content -> System geht von
    // "normales Internet" aus und stoert die ConNect-Seite nicht.
    auto portalNoContent = [this]() { _server->send(204, "text/plain", ""); };
    _server->on("/generate_204",        HTTP_GET, portalNoContent);  // Chrome
    _server->on("/gen_204",             HTTP_GET, portalNoContent);  // Chrome alt
    _server->on("/connecttest.txt",     HTTP_GET, portalNoContent);  // Edge/Win
    _server->on("/ncsi.txt",            HTTP_GET, portalNoContent);  // Win NCSI
    _server->on("/hotspot-detect.html", HTTP_GET, portalNoContent);  // Apple
    _server->on("/library/test/success.html", HTTP_GET, portalNoContent);

    _server->onNotFound([this]() { handleNotFound(); });
    
    _server->begin();
    _running = true;
}

void ConNectWebServer::handleClient() {
    if (_running && _server) {
        _server->handleClient();
    }
}

void ConNectWebServer::stop() {
    if (_server) _server->stop();
    _running = false;
}

String ConNectWebServer::getStatusJSON() {
    // Statischer Buffer um Heap-Fragmentierung zu vermeiden
    static char buf[1024];
    String wifiIp = jsonEscape(getWiFiIpAddress());
    String wifiHostname = jsonEscape(getWiFiHostname());
    uint32_t internalHeapFree = heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    uint32_t internalHeapLargest = heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    snprintf(buf, sizeof(buf),
        "{\"mode\":\"%s\","
        "\"baudRate\":%lu,"
        "\"usbConnected\":%s,"
        "\"bleConnected\":%s,"
        "\"bleAdvertising\":%s,"
        "\"battery\":%d,"
        "\"wifiClients\":%d,"
        "\"wifiMode\":\"%s\","
        "\"wifiActiveMode\":\"%s\","
        "\"wifiConnected\":%s,"
        "\"wifiFallback\":%s,"
        "\"wifiIP\":\"%s\","
        "\"wifiHostname\":\"%s\","
        "\"uptime\":%lu,"
        "\"stats\":{"
        "\"usb_rx\":%lu,\"usb_tx\":%lu,"
        "\"usb_drop\":%lu,\"usb_rx_queue\":%u,"
        "\"rs232_rx\":%lu,\"rs232_tx\":%lu,"
        "\"heap_internal_free\":%lu,\"heap_internal_largest\":%lu},"
        "\"version\":\"" GIT_VERSION "\","
        "\"commit\":\"" GIT_COMMIT "\","
        "\"buildTime\":\"" BUILD_TIME "\"}",
        isUSBMode ? "USB" : "RS232",
        (unsigned long)currentBaudRate,
        usbHostConnected() ? "true" : "false",
        deviceConnected.load() ? "true" : "false",
        bleAdvertising.load() ? "true" : "false",
        getBatteryLevel(),
        WiFi.softAPgetStationNum(),
        getConfiguredWiFiModeValue(),
        getActiveWiFiModeLabel(),
        isWiFiClientConnected() ? "true" : "false",
        isWiFiFallbackAccessPoint() ? "true" : "false",
        wifiIp.c_str(),
        wifiHostname.c_str(),
        (unsigned long)(millis() / 1000),
        (unsigned long)stat_usb_rx.load(),   (unsigned long)stat_usb_tx.load(),
        (unsigned long)usbHostDroppedBytes(), (unsigned)usbHostRxQueueCapacity(),
        (unsigned long)stat_rs232_rx.load(), (unsigned long)stat_rs232_tx.load(),
        (unsigned long)internalHeapFree, (unsigned long)internalHeapLargest
    );
    return String(buf);
}

void ConNectWebServer::handleRoot() {
    if (!checkAuth()) return;

    _server->setContentLength(CONTENT_LENGTH_UNKNOWN);
    _server->sendHeader("Cache-Control", "no-store");
    _server->send(200, "text/html; charset=utf-8", "");
    streamHTML();
}

void ConNectWebServer::handleStatus() {
    // Status ohne Auth für API-Zugriff
    _server->send(200, "application/json", getStatusJSON());
}

void ConNectWebServer::handleConfig() {
    if (!checkAuth()) return;
    // Return current configuration as JSON
    String otaSSID = prefs.getString("otaSSID", "");
    String otaUrl = getConfiguredOTAManifestUrl();
    
    String json = "{";
    json += "\"baudRate\":" + String(currentBaudRate) + ",";
    json += "\"mode\":\"" + String(isUSBMode ? "USB" : "RS232") + "\",";
    json += "\"wifiMode\":\"" + String(getConfiguredWiFiModeValue()) + "\",";
    json += "\"otaSSID\":\"" + jsonEscape(otaSSID) + "\",";
    json += "\"otaUrl\":\"" + jsonEscape(otaUrl) + "\"";
    json += "}";
    _server->send(200, "application/json", json);
}

void ConNectWebServer::handleSave() {
    if (!checkAuth()) return;
    bool restartRequired = false;
    String savedWiFiMode = getConfiguredWiFiModeValue();

    if (_server->hasArg("wifiMode") || _server->hasArg("wifiSSID") ||
        _server->hasArg("otaSSID") || _server->hasArg("wifiPassword") ||
        _server->hasArg("otaPassword")) {
        String requestedMode = _server->hasArg("wifiMode")
                                 ? _server->arg("wifiMode")
                                 : savedWiFiMode;
        if (!config::isValidWiFiMode(requestedMode.c_str())) {
            _server->send(400, "application/json", "{\"success\":false,\"error\":\"Ungültiger WLAN-Modus\"}");
            return;
        }

        String requestedSSID = _server->hasArg("wifiSSID")
                                 ? _server->arg("wifiSSID")
                                 : (_server->hasArg("otaSSID")
                                      ? _server->arg("otaSSID")
                                      : prefs.getString("otaSSID", ""));
        requestedSSID.trim();
        if (!config::isValidSsidLength(requestedSSID.length())) {
            _server->send(400, "application/json", "{\"success\":false,\"error\":\"SSID darf höchstens 32 Bytes haben\"}");
            return;
        }
        if (requestedMode == "CLIENT" && requestedSSID.length() == 0) {
            _server->send(400, "application/json", "{\"success\":false,\"error\":\"Für den WLAN-Client wird eine SSID benötigt\"}");
            return;
        }

        if ((_server->hasArg("wifiPassword") || _server->hasArg("otaPassword")) &&
            !_server->hasArg("clearWifiPassword")) {
            String password = _server->hasArg("wifiPassword")
                                ? _server->arg("wifiPassword")
                                : _server->arg("otaPassword");
            if (!config::isValidWiFiPasswordLength(password.length())) {
                _server->send(400, "application/json", "{\"success\":false,\"error\":\"Passwort muss 8 bis 64 Zeichen haben\"}");
                return;
            }
        }
    }
    if (_server->hasArg("baudRate")) {
        String baudValue = _server->arg("baudRate");
        uint32_t newBaud = baudValue.toInt();
        if (!config::isValidBaudRate(newBaud) || baudValue != String(newBaud)) {
            _server->send(400, "application/json", "{\"success\":false,\"error\":\"Ungültige Baudrate\"}");
            return;
        }
        currentBaudRate = newBaud;
        // Apply to RS232
        extern HardwareSerial RS232Serial;
        RS232Serial.updateBaudRate(currentBaudRate);
        // Apply to USB if connected
        if (usbHostConnected()) {
            usbHostSetLineCoding(currentBaudRate);
        }
    }
    
    if (_server->hasArg("mode")) {
        String mode = _server->arg("mode");
        if (!config::isValidSerialMode(mode.c_str())) {
            _server->send(400, "application/json", "{\"success\":false,\"error\":\"Ungültiger serieller Modus\"}");
            return;
        }
        if (mode == "USB") {
            if (!usbHostInitialized()) restartRequired = true;
            setSerialTargetMode(true);
            prefs.putBool("usbMode", true);
        } else if (mode == "RS232") {
            setSerialTargetMode(false);
            delay(20);
            rs232ReinitUart();
            setSerialTargetMode(false);
            prefs.putBool("usbMode", false);
        }
    }
    
    if (_server->hasArg("wifiMode")) {
        String wifiMode = _server->arg("wifiMode");
        uint8_t modeValue = wifiMode == "CLIENT" ? 1 : 0;
        if (prefs.getUChar("wifiMode", 0) != modeValue) restartRequired = true;
        prefs.putUChar("wifiMode", modeValue);
        savedWiFiMode = modeValue == 1 ? "CLIENT" : "AP";
    }

    // Gemeinsame WLAN-Zugangsdaten fuer Clientbetrieb und HTTP-OTA.
    if (_server->hasArg("wifiSSID") || _server->hasArg("otaSSID")) {
        String ssid = _server->hasArg("wifiSSID") ? _server->arg("wifiSSID") : _server->arg("otaSSID");
        ssid.trim();
        if (prefs.getString("otaSSID", "") != ssid && savedWiFiMode == "CLIENT") restartRequired = true;
        prefs.putString("otaSSID", ssid);
    }
    if (_server->hasArg("clearWifiPassword")) {
        if (prefs.getString("otaPass", "").length() > 0 && savedWiFiMode == "CLIENT") restartRequired = true;
        prefs.remove("otaPass");
    } else if (_server->hasArg("wifiPassword") || _server->hasArg("otaPassword")) {
        String pw = _server->hasArg("wifiPassword") ? _server->arg("wifiPassword") : _server->arg("otaPassword");
        if (pw.length() > 0) {
            if (prefs.getString("otaPass", "") != pw && savedWiFiMode == "CLIENT") restartRequired = true;
            prefs.putString("otaPass", pw);
        }
    }
    
    if (_server->hasArg("otaUrl")) {
        String otaUrl = _server->arg("otaUrl");
        otaUrl.trim();
        if (otaUrl.length() == 0) {
            otaUrl = OTAConfig::DEFAULT_MANIFEST_URL;
        }
        prefs.putString("otaUrl", otaUrl);
    }
    
    _server->send(200, "application/json",
                  String("{\"success\":true,\"restartRequired\":") +
                  (restartRequired ? "true" : "false") +
                  ",\"wifiMode\":\"" + savedWiFiMode + "\"}");
}

void ConNectWebServer::handleReboot() {
    if (!checkAuth()) return;
    _server->send(200, "text/plain", "Rebooting...");
    delay(500);
    ESP.restart();
}

void ConNectWebServer::handleTerminalRead() {
    if (!checkAuth()) return;

    uint64_t bufferEnd = terminalRxBase + (uint64_t)terminalRxLen;

    uint64_t from = terminalRxBase;
    if (_server->hasArg("from")) {
        // toInt() liefert nur 32 bit -> strtoull verwenden fuer 64-bit Offsets.
        String s = _server->arg("from");
        char* endp = nullptr;
        unsigned long long req = strtoull(s.c_str(), &endp, 10);
        if (endp != s.c_str()) from = (uint64_t)req;
    }

    // Client liegt hinter dem aktuellen Ringpuffer-Fenster (Overflow):
    // nach vorne springen, damit der Stream nicht dauerhaft blockiert.
    if (from < terminalRxBase) from = terminalRxBase;
    if (from > bufferEnd)      from = bufferEnd;

    size_t offsetInBuf = (size_t)(from - terminalRxBase);
    size_t available   = terminalRxLen - offsetInBuf;
    size_t maxChunk    = 8192;
    size_t chunkLen    = available > maxChunk ? maxChunk : available;

    // X-Terminal-Total = absoluter Offset NACH dem gesendeten Chunk.
    // So springt der Client bei Mehrfach-Polls sauber weiter, ohne Daten
    // zu überspringen, die noch im Puffer auf den nächsten Poll warten.
    uint64_t nextPos = from + (uint64_t)chunkLen;

    char hdr[32];
    snprintf(hdr, sizeof(hdr), "%llu", (unsigned long long)nextPos);
    _server->sendHeader("X-Terminal-Total", hdr);
    snprintf(hdr, sizeof(hdr), "%llu", (unsigned long long)from);
    _server->sendHeader("X-Terminal-From",  hdr);
    _server->sendHeader("X-Terminal-Len",   String((unsigned)chunkLen));

    // Arduino WebServer::send() braucht String/const char*. Wir bauen die
    // String einmalig aus dem Ringpuffer-Ausschnitt - nur der Chunk
    // (max 8 KB) wird kopiert, nicht der gesamte 64 KB-Puffer.
    String chunk(&terminalRxBuf[offsetInBuf], chunkLen);
    _server->send(200, "text/plain", chunk);
}

void ConNectWebServer::handleTerminalSend() {
    if (!checkAuth()) return;

    String data = _server->arg("data");
    if (data.length() == 0) {
        _server->send(400, "application/json", "{\"success\":false,\"error\":\"no data\"}");
        return;
    }

    if (terminalTxBuffer.length() + data.length() > TERMINAL_TX_MAX) {
        _server->send(503, "application/json", "{\"success\":false,\"error\":\"queue full\"}");
        return;
    }

    terminalTxBuffer += data;
    _server->send(200, "application/json", "{\"success\":true}");
}

void ConNectWebServer::handleTerminalClear() {
    if (!checkAuth()) return;
    terminalRxLen  = 0;
    terminalRxBase = 0;
    _server->send(200, "application/json", "{\"success\":true}");
}

void ConNectWebServer::handleTerminalDownload() {
    if (!checkAuth()) return;

    String filename = "connect-console-log-" + String(millis() / 1000) + ".txt";
    _server->sendHeader("Content-Disposition", "attachment; filename=\"" + filename + "\"");
    _server->sendHeader("Cache-Control", "no-store");

    // Einmalige Kopie fuer den Download (kompletter Ringpuffer-Inhalt).
    String body;
    body.reserve(terminalRxLen);
    for (size_t i = 0; i < terminalRxLen; ++i) body += terminalRxBuf[i];
    _server->send(200, "text/plain; charset=utf-8", body);
}

void ConNectWebServer::handleNotFound() {
    _server->send(404, "text/plain", "Not Found");
}

void ConNectWebServer::streamHTML() {
    HtmlChunkWriter html(*_server);
    html.appendProgmem(HTML_HEADER, sizeof(HTML_HEADER) - 1);
    html += R"rawliteral(
        <div class="tabs">
            <button id="tab-btn-settings" class="tab-btn active" type="button" onclick="switchTab('settings')">Einstellungen</button>
            <button id="tab-btn-terminal" class="tab-btn" type="button" onclick="switchTab('terminal')">Terminal</button>
        </div>
        <div id="tab-settings" class="tab-panel active">
    )rawliteral";
    
    // Status Card
    html += R"rawliteral(
        <div class="card">
            <h2>📊 Status</h2>
            <div class="status-grid">
                <div class="status-item">
                    <div class="label">Modus</div>
                    <div class="value active" id="mode">)rawliteral";
    html += isUSBMode ? "USB" : "RS232";
    html += R"rawliteral(</div>
                </div>
                <div class="status-item">
                    <div class="label">Baudrate</div>
                    <div class="value" id="baud">)rawliteral";
    html += String(currentBaudRate);
    html += R"rawliteral(</div>
                </div>
                <div class="status-item">
                    <div class="label">USB</div>
                    <div class="value )rawliteral";
    html += usbHostConnected() ? "online" : "offline";
    html += R"rawliteral(" id="usb">)rawliteral";
    html += usbHostConnected() ? "Verbunden" : "Getrennt";
    html += R"rawliteral(</div>
                </div>
                <div class="status-item">
                    <div class="label">BLE</div>
                    <div class="value )rawliteral";
    html += deviceConnected.load() ? "online" : "offline";
    html += R"rawliteral(" id="ble">)rawliteral";
    html += deviceConnected.load() ? "Verbunden" : (bleAdvertising.load() ? "Werbung" : "Aus");
    html += R"rawliteral(</div>
                </div>
                <div class="status-item">
                    <div class="label">Batterie</div>
                    <div class="value" id="battery">)rawliteral";
    html += String(getBatteryLevel()) + "%";
    html += R"rawliteral(</div>
                </div>
                <div class="status-item">
                    <div class="label">Hotspot Clients</div>
                    <div class="value" id="clients">)rawliteral";
    html += String(WiFi.softAPgetStationNum());
    html += R"rawliteral(</div>
                </div>
                <div class="status-item">
                    <div class="label">Netzwerkmodus</div>
                    <div class="value active" id="wifi_mode">)rawliteral";
    html += getActiveWiFiModeLabel();
    html += R"rawliteral(</div>
                </div>
                <div class="status-item">
                    <div class="label">Netzwerk-IP</div>
                    <div class="value" id="wifi_ip">)rawliteral";
    html += getWiFiIpAddress();
    html += R"rawliteral(</div>
                </div>
                <div class="status-item">
                    <div class="label">Hostname</div>
                    <div class="value" id="wifi_host">)rawliteral";
    String hostname = getWiFiHostname();
    html += hostname.length() > 0 ? hostname : "-";
    html += R"rawliteral(</div>
                </div>
            </div>
        </div>
    )rawliteral";
    
    // Stats Card
    html += R"rawliteral(
        <div class="card">
            <h2>📈 Statistik</h2>
            <table class="stats-table">
                <tr><td>USB RX</td><td id="usb_rx">)rawliteral";
    html += String(stat_usb_rx.load());
    html += R"rawliteral(</td></tr>
                <tr><td>USB TX</td><td id="usb_tx">)rawliteral";
    html += String(stat_usb_tx.load());
    html += R"rawliteral(</td></tr>
                <tr><td>USB Drops</td><td id="usb_drop">)rawliteral";
    html += String(usbHostDroppedBytes());
    html += R"rawliteral(</td></tr>
                <tr><td>USB RX Queue</td><td id="usb_rx_queue">)rawliteral";
    html += String(usbHostRxQueueCapacity());
    html += R"rawliteral(</td></tr>
                <tr><td>RS232 RX</td><td id="rs232_rx">)rawliteral";
    html += String(stat_rs232_rx.load());
    html += R"rawliteral(</td></tr>
                <tr><td>RS232 TX</td><td id="rs232_tx">)rawliteral";
    html += String(stat_rs232_tx.load());
    html += R"rawliteral(</td></tr>
                <tr><td>Uptime</td><td>)rawliteral";
    uint32_t uptime = millis() / 1000;
    html += String(uptime / 3600) + "h " + String((uptime % 3600) / 60) + "m";
    html += R"rawliteral(</td></tr>
            </table>
        </div>
    )rawliteral";
    
    // Config Card
    html += R"rawliteral(
        <div class="card">
            <h2>⚙️ Konfiguration</h2>
            <form id="configForm" onsubmit="saveConfig(event)">
                <div class="form-group">
                    <label>Modus</label>
                    <select name="mode" id="modeSelect">
                        <option value="USB")rawliteral";
    html += isUSBMode ? " selected" : "";
    html += R"rawliteral(>USB Host</option>
                        <option value="RS232")rawliteral";
    html += !isUSBMode ? " selected" : "";
    html += R"rawliteral(>RS232</option>
                    </select>
                </div>
                <div class="form-group">
                    <label>Baudrate</label>
                    <select name="baudRate" id="baudSelect">)rawliteral";
    
    const uint32_t bauds[] = {300, 1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600};
    for (uint32_t b : bauds) {
        html += "<option value=\"" + String(b) + "\"";
        if (b == currentBaudRate) html += " selected";
        html += ">" + String(b) + "</option>";
    }
    
    html += R"rawliteral(
                    </select>
                </div>
                <div class="btn-group">
                    <button type="submit" class="btn btn-primary">Speichern</button>
                    <button type="button" class="btn btn-danger" onclick="reboot()">Neustart</button>
                </div>
            </form>
        </div>
    )rawliteral";
    
    // WLAN/OTA Config Card
    String otaSSID = prefs.getString("otaSSID", "");
    String otaUrl = getConfiguredOTAManifestUrl();
    
    html += R"rawliteral(
        <div class="card">
            <h2>📡 WLAN &amp; OTA</h2>
            <form id="wifiForm" onsubmit="saveWiFi(event)">
                <div class="form-group">
                    <label>Betriebsart</label>
                    <select name="wifiMode" id="wifiModeSelect">
                        <option value="AP")rawliteral";
    html += String(getConfiguredWiFiModeValue()) == "AP" ? " selected" : "";
    html += R"rawliteral(>Hotspot (ConNect)</option>
                        <option value="CLIENT")rawliteral";
    html += String(getConfiguredWiFiModeValue()) == "CLIENT" ? " selected" : "";
    html += R"rawliteral(>WLAN-Client</option>
                    </select>
                </div>
                <div class="form-group">
                    <label>WLAN SSID</label>
                    <input type="text" name="wifiSSID" id="wifiSSID" maxlength="32" autocomplete="username" placeholder="Home-WiFi" value=")rawliteral";
    html += htmlAttrEscape(otaSSID);
    html += R"rawliteral(">
                </div>
                <div class="form-group">
                    <label>WLAN Passwort</label>
                    <input type="password" name="wifiPassword" id="wifiPassword" minlength="8" maxlength="64" autocomplete="current-password" placeholder="Leer = unverändert">
                </div>
                <div class="form-group">
                    <label class="checkbox-row"><input type="checkbox" name="clearWifiPassword" value="1"> Passwort löschen (offenes WLAN)</label>
                </div>
                <div class="form-group">
                    <label>OTA Manifest</label>
                    <input type="url" name="otaUrl" id="otaUrl" placeholder=")rawliteral";
    html += OTAConfig::DEFAULT_MANIFEST_URL;
    html += R"rawliteral(" value=")rawliteral";
    html += htmlAttrEscape(otaUrl);
    html += R"rawliteral(">
                </div>
                <div class="btn-group">
                    <button type="submit" class="btn btn-primary">WLAN speichern</button>
                </div>
            </form>
            <p style="color:#888;font-size:0.85em;margin-top:15px;">
                Im WLAN-Clientmodus erreichbar unter connect.local. Bei einem Verbindungsfehler startet der Hotspot als Fallback.
            </p>
        </div>
    )rawliteral";

    html += R"rawliteral(
        </div>
        <div id="tab-terminal" class="tab-panel">
        <div class="card">
            <h2>🖥️ Web Konsole</h2>
            <div class="terminal-wrap">
                <div id="terminal"></div>
                <form class="term-row" onsubmit="terminalSubmit(event)">
                    <input type="text" id="terminalInput" placeholder="Befehl eingeben und Enter..." autocomplete="off">
                    <button type="submit" class="btn btn-primary">Senden</button>
                </form>
                <div class="term-actions">
                    <button type="button" class="btn btn-ghost" onclick="terminalKey('\r')">Enter</button>
                    <button type="button" class="btn btn-ghost" onclick="terminalKey('\u0003')">Ctrl+C</button>
                    <button type="button" class="btn btn-ghost" onclick="terminalKey('\u001b')">Esc</button>
                    <button type="button" class="btn btn-ghost" onclick="clearTerminal()">Clear</button>
                    <button type="button" class="btn btn-ghost" onclick="downloadTerminalLog()">Download Log</button>
                </div>
                <div class="mobile-keys">
                    <button type="button" onclick="terminalKey('\u001b[A')">↑</button>
                    <button type="button" onclick="terminalKey('\u001b[B')">↓</button>
                    <button type="button" onclick="terminalKey('\u001b[D')">←</button>
                    <button type="button" onclick="terminalKey('\u001b[C')">→</button>
                    <button type="button" onclick="terminalKey('\t')">Tab</button>
                    <button type="button" onclick="terminalKey(' ')">Space</button>
                </div>
            </div>
            <p style="color:#9fb3dc;font-size:0.85em;margin-top:12px;">
                Live-Ansicht der seriellen Konsole (USB/RS232) direkt im Browser, auch mobil bedienbar.
            </p>
        </div>
        </div>
        <script>
            function saveConfig(e) {
                e.preventDefault();
                const data = new FormData(document.getElementById('configForm'));
                fetch('/save', {
                    method: 'POST',
                    body: new URLSearchParams(data)
                }).then(r => r.json()).then(d => {
                    if(d.success) {
                        alert(d.restartRequired
                            ? 'Gespeichert. Das Gerät startet für den USB-Host neu.'
                            : 'Gespeichert!');
                    }
                });
            }
            function saveWiFi(e) {
                e.preventDefault();
                const form = document.getElementById('wifiForm');
                const data = new FormData(form);
                fetch('/save', {
                    method: 'POST',
                    body: new URLSearchParams(data)
                }).then(r => r.json()).then(d => {
                    if(d.success) {
                        document.getElementById('wifiPassword').value = '';
                        if (d.restartRequired) {
                            const destination = d.wifiMode === 'CLIENT'
                                ? 'Nach dem Neustart: http://connect.local'
                                : 'Danach mit dem Hotspot ConNect verbinden.';
                            alert('WLAN gespeichert. Das Gerät startet neu.\n' + destination);
                            fetch('/reboot', { method: 'POST' });
                        } else {
                            alert('WLAN Einstellungen gespeichert.');
                            location.reload();
                        }
                    } else {
                        alert(d.error || 'WLAN konnte nicht gespeichert werden.');
                    }
                }).catch(() => alert('WLAN konnte nicht gespeichert werden.'));
            }
            function reboot() {
                if(confirm('Wirklich neustarten?')) {
                    fetch('/reboot', {method:'POST'}).then(() => {
                        alert('Neustart...');
                        setTimeout(() => location.reload(), 5000);
                    });
                }
            }
        </script>
    )rawliteral";
    
    // Version Footer
    html += "<div class=\"version\">Version: " GIT_VERSION " (" GIT_COMMIT ")<br>Build: " BUILD_TIME "</div>";
    
    html.appendProgmem(HTML_FOOTER, sizeof(HTML_FOOTER) - 1);
    html.finish();
}
