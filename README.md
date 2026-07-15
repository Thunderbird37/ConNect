# ConNect — Wireless Serial Bridge

<p align="center">
  <img src="https://img.shields.io/badge/Platform-ESP32--S3-blue" alt="Platform">
  <img src="https://img.shields.io/badge/Framework-Arduino-green" alt="Framework">
  <img src="https://img.shields.io/badge/License-PolyForm%20Noncommercial%201.0.0-orange" alt="License: PolyForm Noncommercial 1.0.0">
</p>

**ConNect** is a wireless serial bridge that makes USB and RS232 serial devices accessible via WiFi and Bluetooth Low Energy. Perfect for accessing network equipment consoles (Cisco, Juniper, etc.), embedded systems, or any serial device — wirelessly from your laptop, tablet, or phone.

```text
┌─────────────┐     ┌──────────────────┐     ┌─────────────┐
│  USB/RS232  │────▶│     ConNect      │◀────│   Laptop    │
│   Device    │     │  (ESP32-S3)      │     │  Tablet     │
│             │◀────│  WiFi + BLE      │────▶│  Phone      │
└─────────────┘     └──────────────────┘     └─────────────┘
```

## ✨ Features

- **Dual Serial Input:** USB Host (FTDI, CP210x, CH34x adapters) + RS232 via MAX3232
- **Dual Wireless Output:** WiFi hotspot or WLAN client with Telnet + Bluetooth Low Energy UART
- **Web Interface:** Configure and monitor via browser at <http://192.168.32.1>
- **Web Console:** Live serial terminal directly in browser (desktop + mobile)
- **OLED Display:** Real-time status, battery level, connection info
- **OTA Updates:** Install published firmware directly from GitHub Releases
- **Portable:** Battery-powered with Li-Ion charging support

## 🎯 Use Cases

- **Network Administration:** Access Cisco/Juniper console ports wirelessly
- **Embedded Development:** Debug serial devices without cables
- **Industrial Equipment:** Connect to RS232 PLCs and controllers
- **Retro Computing:** Bridge vintage serial devices to modern systems

## 🔧 Hardware Requirements

### Core Components

| Component | Description |
|-----------|-------------|
| YD-ESP32-S3 with CH343P | Main microcontroller; built with the `esp32-s3-devkitc-1` PlatformIO definition |
| 1.3" OLED (SH1106) | Status display (I2C, 128x64) |
| MAX3232 Module | RS232 level shifter (3.3V!) |
| Li-Ion Battery | 3.7V, any capacity |
| Buzzer | Feedback (optional) |

### Pin Configuration

| Function | GPIO | Notes |
|----------|------|-------|
| Button | 0 | Mode switching (active low) |
| Buzzer | 1 | PWM feedback |
| MAX3232 Enable | 2 | Active LOW / always enabled on current PCB |
| Battery ADC | 3 | Voltage divider 33k/15k |
| RS232 RX | 5 | UART1 |
| RS232 TX | 6 | UART1 |
| I2C SDA | 8 | OLED Display |
| I2C SCL | 9 | OLED Display |
| RGB LED | 48 | WS2812 NeoPixel |

### Cold Boot After Long Storage

This is a hardware power-up problem if the ESP32-S3 does not execute any
firmware until external power is connected. Three independent conditions must be
valid during a cold battery start.

#### Board-Specific Fix: YD-ESP32-S3 Q2

The board used by this project was identified as a YD-ESP32-S3 variant with a
CH343P USB-UART bridge. On this board, the auto-program transistor Q2 can pull
GPIO0 directly to ground. Unlike the official Espressif DevKit schematic, this
path has no removable 0 Ω isolation link such as R13.

Q2 was removed completely on the affected board. The BOOT button and external
10 kΩ GPIO0 pull-up remain connected. This prevents the unpowered CH343P/auto-
program circuit from clamping GPIO0 during a cold battery start.

This modification is specific to the verified YD schematic and reference
designator. Do not remove a component named Q2 on another board without checking
its schematic and measuring that it is the GPIO0 pull-down transistor.

Removing Q2 disables automatic entry into the serial download mode. To upload
through the CH343P USB-UART bridge:

1. Hold BOOT.
2. Press and release RESET.
3. Release BOOT.
4. Run `pio run -e esp32-s3 -t upload`.

Native USB operation and HTTP OTA remain available. The final proof for the
cold-start repair is a successful battery-only boot after at least 30 minutes of
complete power disconnection. If that test still fails, continue with the EN and
battery-path measurements below.

#### 1. GPIO0 Must Be High

GPIO0 is a boot-strapping pin. Populate a 10 kΩ pull-up and connect the button
only between GPIO0 and ground. Do not add a large capacitor to GPIO0.

```text
3.3 V ── 10 kΩ ──┬── GPIO0
                 │
              [Button]
                 │
                GND
```

If GPIO0 is low when `CHIP_PU` rises, the chip waits in download mode instead of
starting the application.

#### 2. Delay CHIP_PU / EN

Espressif requires the 3.3 V rail to stabilize before `CHIP_PU` goes high. The
recommended starting values are 10 kΩ from `CHIP_PU` to 3.3 V and 1 µF from
`CHIP_PU` to ground.

```text
3.3 V ── 10 kΩ ──┬── CHIP_PU / EN
                 │
               1 µF
                 │
                GND
```

For a slowly rising battery regulator, frequent switching, or an unstable rail,
use a voltage supervisor with a threshold around 3.0 V instead of relying only
on the RC delay. Add a reset test pad or button from `CHIP_PU` to ground.

#### 3. Verify the Battery Power Path

The 3.3 V regulator must supply at least 500 mA. Place at least 10 µF plus
0.1 µF close to the ESP32-S3 supply. If the battery/protection-board output is
zero until USB power is attached, the protection circuit is in cutoff or ship
mode; firmware cannot wake it. Check the battery, protection/charger module,
switch placement, regulator enable pin, and total power-off quiescent current.

The 33 kΩ / 15 kΩ battery divider alone draws about 80 µA while connected. Make
sure the power switch also disconnects this divider and other always-on loads,
or switch the divider electronically for long storage periods.

#### Cold-Boot Measurement

Measure before attaching external power:

| Measurement | Interpretation |
|-------------|----------------|
| Battery output missing | Battery protection/ship mode or discharged cell |
| Battery present, 3.3 V missing | Switch, regulator, enable, or UVLO problem |
| 3.3 V stable, EN never rises | EN pull-up/reset circuit problem |
| EN rises with the slow 3.3 V ramp | Add EN RC delay or voltage supervisor |
| EN is high, GPIO0 is low | Boot-strapping problem |
| Pulling EN low briefly makes it boot | Power-up timing is the root cause |

These tests apply to other ESP32-S3 projects with the same cold-start symptom.

## 🚀 Getting Started

### Prerequisites

- [VS Code](https://code.visualstudio.com/) with [PlatformIO Extension](https://platformio.org/install/ide?install=vscode)
- USB cable for initial flash

### Build & Upload

```bash
# Clone the repository
git clone https://github.com/Thunderbird37/ConNect.git
cd ConNect

# Build and upload
pio run -e esp32-s3 -t upload

# Monitor serial output (optional)
pio device monitor -b 115200
```

### Automatic Versioning

Firmware version is automatically generated from Git tags via `git describe`.
Tagged commits show `v1.0.0`, untagged show `v1.0.0-5-g1a2b3c4`.

### First Connection

1. Power on ConNect
2. Connect to WiFi: **SSID:** `ConNect` / **Password:** `12345678`
3. Open <http://192.168.32.1> in your browser
4. Web Interface Login: **User:** `admin` / **Password:** `connect`
5. Or connect via Telnet: `telnet 192.168.32.1`

## 📱 Connectivity

### WiFi Access Point

| Setting | Value |
|---------|-------|
| SSID | ConNect |
| Password | 12345678 |
| IP Address | 192.168.32.1 |
| Telnet Port | 23 |
| Web Interface | Port 80 |

### WLAN Client

In `WLAN-Client` mode, ConNect joins the configured WLAN instead of opening its
own hotspot. Your computer stays connected to its normal network and therefore
keeps internet access while using the serial bridge.

| Service | Address |
|---------|---------|
| Web Interface | <http://connect.local> |
| Telnet | `telnet connect.local` |
| Direct access | DHCP address shown in the WebUI or by `` `status `` |

The selected mode and credentials are stored in NVS and restored at boot. If
client startup fails or no credentials exist, ConNect opens its `ConNect`
hotspot at `192.168.32.1` as a recovery fallback. mDNS is intentionally enabled
only in client mode, so the stable SoftAP boot path remains unchanged.

### Bluetooth Low Energy

Uses Nordic UART Service (NUS) — compatible with most BLE terminal apps:

| UUID | Function |
|------|----------|
| 6E400001-B5A3-F393-E0A9-E50E24DCCA9E | Service |
| 6E400002-B5A3-F393-E0A9-E50E24DCCA9E | RX (Write) |
| 6E400003-B5A3-F393-E0A9-E50E24DCCA9E | TX (Notify) |

**Recommended Apps:**

- iOS: [nRF Connect](https://apps.apple.com/app/nrf-connect-for-mobile/id1054362403), [Serial Bluetooth Terminal](https://apps.apple.com/app/serial-bluetooth-terminal/id1595838279)
- Android: [Serial Bluetooth Terminal](https://play.google.com/store/apps/details?id=de.kai_morich.serial_bluetooth_terminal)

### Terminal Behavior

- Telnet negotiates binary mode and escapes IAC (`0xFF`) according to RFC 854.
- Telnet local echo is off by default because serial consoles usually echo input
  themselves. ConNect still negotiates `WILL ECHO` so the Telnet client never
  adds a second local copy. Use `` `echo on `` for targets without remote echo;
  the selection is stored persistently.
- BLE RX accepts acknowledged writes and Write Without Response for broad NUS
  terminal compatibility; BLE TX uses notifications with an MTU up to 247 bytes.
- Cursor, navigation, function-key, Tab, Ctrl, Backspace, DEL, and other escape
  sequences are forwarded byte-for-byte.
- CRLF and CR-NUL are treated as one Enter. A standalone CR or LF emits one CR;
  consecutive LF bytes remain consecutive Enter presses.
- Fixed queues absorb bursts in both directions. When a connected client is
  temporarily slow, ConNect stops draining the preceding queue instead of
  deleting its oldest bytes.
- In USB mode without an attached USB serial device, Telnet reports that input
  was not sent instead of silently retaining stale keystrokes.
- `` `status `` reports queue fill levels, USB drops, and BLE rejects. Non-zero
  counters indicate that a physical transport limit was hit.

RS232 has no RTS/CTS flow control on the documented wiring. The 8 KB UART RX
buffer absorbs temporary stalls, but no device can guarantee an unlimited stream
if every wireless receiver remains blocked indefinitely. For sustained high-rate
captures, keep the terminal connected and ensure all counters remain zero.

## 🎮 Controls

### Button Actions

| Action | Duration | Function |
|--------|----------|----------|
| Single Press | < 500ms | Toggle USB ↔ RS232; first USB activation may restart |
| Double Press | < 500ms gap | Cycle baud rate |
| Triple Press | < 500ms gap | Toggle hotspot ↔ WLAN client, then restart |
| Long Press | 1-5s | Toggle BLE Advertising |
| Very Long Press | > 5s | Arm OTA; release, wait 1s, then press briefly to confirm |

### In-Band Commands

Send commands via Telnet or BLE (prefix with `` ` `` or `'`):

| Command | Description |
|---------|-------------|
| `` `usb `` | Switch to USB mode; starts USB safely before WiFi after a restart |
| `` `rs232 `` | Switch to RS232 mode |
| `` `baud 115200 `` | Set baud rate |
| `` `status `` | Show system status |
| `` `version `` | Show firmware version |
| `` `wlan SSID PASSWORD `` | Store WLAN credentials |
| `` `ota `` | Start OTA update |
| `` `help `` | Show all commands |

### Baud Rate Presets

Cycle through: 9600 → 19200 → 38400 → 57600 → 115200 → 9600...

## 🖥️ Display Information

The OLED shows:

- **Mode:** USB or RS232 (with interface icon)
- **Wireless:** WiFi/BLE status and client count
- **Baud:** Current baud rate
- **Battery:** Charge level with charging indicator ⚡

### LED Indicators

| Color | Pattern | Meaning |
|-------|---------|---------|
| Cyan | Blinking | WiFi active, no clients |
| Cyan | Solid | WiFi with connected client |
| Magenta | Blinking | BLE advertising |
| Magenta | Solid | BLE connected |
| White | Flash | Data activity |

## 🔌 RS232 Wiring

> ⚠️ **Important:** Use MAX3232 (3.3V), NOT MAX232 (5V)!

### Basic Wiring

| Signal | GPIO | MAX3232 Pin |
|--------|------|-------------|
| VCC | 3.3V | VCC |
| GND | GND | GND |
| Enable | GPIO2 | EN/SHDN, active LOW on current PCB |
| TX | GPIO6 | T1IN |
| RX | GPIO5 | R1OUT |

### RJ45 for Cisco Console

| RJ45 Pin | Signal | MAX3232 |
|----------|--------|---------|
| 3 | TXD | T1OUT |
| 6 | RXD | R1IN |
| 4, 5 | GND | GND |

> **Note:** Use a rollover cable for Cisco equipment.

### DB9 Male (DTE)

| DB9 Pin | Signal | MAX3232 |
|---------|--------|---------|
| 2 | RXD | R1IN |
| 3 | TXD | T1OUT |
| 5 | GND | GND |

**Required Jumpers (no hardware flow control):**

| Bridge | Pins | Purpose |
|--------|------|---------|
| DTR ↔ DSR | 4 ↔ 6 | Device ready signals |
| RTS ↔ CTS | 7 ↔ 8 | Flow control bypass |

> **Note:** Use a null-modem cable for PC-to-PC connections.

## 📡 OTA Updates

### Method 1: GitHub Releases (Recommended)

```bash
# Configure WiFi once via Telnet/BLE:
`wlan YourWiFiSSID YourWiFiPassword

# Then trigger update:
`ota
# Or hold button > 5 seconds
```

The default manifest is always resolved through the latest public GitHub
release. Firmware upgrading from an earlier build migrates its stored OTA
configuration to this endpoint once on first boot. A custom manifest can still
be selected with `` `otaurl URL `` afterwards.

Expected manifest example:

```json
{
  "version": "v1.2.3",
  "firmware": "https://github.com/Thunderbird37/ConNect/releases/download/v1.2.3/firmware.bin",
  "md5": "0123456789abcdef0123456789abcdef"
}
```

GitHub Actions calculates the checksum automatically. The device rejects a
download whose MD5 does not match the manifest.

### Method 2: Local OTA (Development)

```bash
# Connect to ConNect WiFi, then:
pio run -e esp32-s3-ota -t upload
```

### Creating a Release

```bash
./release.sh v1.2.3
```

The script validates the repository state, identity, tests, and local firmware
build before pushing the tag. GitHub Actions then builds the tagged source and
publishes the binaries, manifest, ELF, and checksums as release assets.

For the migration, keep the previous OTA service available for one bridge
release or redirect its manifest request to the latest GitHub manifest. On first
boot, that firmware stores GitHub Releases as its OTA source.

### Terminal Transport Test

```bash
c++ -std=c++17 -Wall -Wextra -Werror -Isrc \
  test/terminal_transport_test.cpp -o /tmp/connect-terminal-test
/tmp/connect-terminal-test
```

The test covers ring-buffer wraparound and atomic overflow rejection, Telnet IAC
negotiation, CR/LF handling, cursor keys, Home/End, Page Up/Down, Insert/Delete,
F1, Tab, Ctrl+C, Backspace, and DEL.

## 🐛 Troubleshooting

| Problem | Solution |
|---------|----------|
| Can see output but can't type | Check baud rate, try `` `baud 9600 `` |
| No serial communication | Verify TX/RX wiring, check MAX3232_EN pin |
| USB device not detected | Ensure device is CDC-compatible (FTDI/CP210x/CH34x) |
| BLE not visible | Press button 1-5s to enable advertising |
| OTA fails | Check WiFi credentials, OTA URL and uploaded manifest/files |

### Loopback Test

To verify RS232 wiring without a device:

- DB9: Bridge pins 2-3
- RJ45: Bridge pins 3-6

Type in Telnet — you should see your input echoed.

## 📦 Dependencies

| Library | Purpose | License |
|---------|---------|---------|
| [NimBLE-Arduino](https://github.com/h2zero/NimBLE-Arduino) | BLE stack | Apache 2.0 |
| [U8g2](https://github.com/olikraus/u8g2) | OLED display | BSD 2-Clause; fonts vary |
| [Adafruit NeoPixel](https://github.com/adafruit/Adafruit_NeoPixel) | RGB LED | LGPL 3.0 |

## 🏗️ Project Structure

```text
ConNect/
├── src/
│   ├── main.cpp              # Main application logic
│   ├── display.cpp/h         # OLED display handling
│   ├── buzzer.cpp/h          # Audio feedback
│   ├── usb_host_bridge.cpp/h # USB Host CDC
│   ├── connect_webserver.cpp/h # Web interface
│   └── ota_config.h          # OTA release endpoint
├── lib/
│   ├── usb_host_cdc_acm/     # USB CDC driver
│   ├── usb_host_ftdi_vcp/    # FTDI support
│   ├── usb_host_cp210x_vcp/  # CP210x support
│   └── usb_host_ch34x_vcp/   # CH34x support
├── include/
│   └── display.h
├── .github/workflows/
│   └── firmware.yml          # CI and GitHub Releases
├── platformio.ini            # Build configuration
├── release.sh                # Release automation
└── git_version.py            # Version from git tags
```

## 📄 License

Code and documentation owned by Thunderbird37 are available under the
[PolyForm Noncommercial License 1.0.0](LICENSE). You may study, modify, and
redistribute them for permitted noncommercial purposes. No commercial rights
are granted; commercial use of ConNect is reserved to Thunderbird37.

This is a source-available license, not an OSI-approved open-source license.
Third-party components retain their own licenses and are not relicensed under
PolyForm. See [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md) for details.

Versions obtained under an earlier license remain governed by that version's
original license terms.

## 🤝 Contributing

Noncommercial forks, experiments, and modifications are welcome under the
project license. Before submitting code or documentation for inclusion, read
[CONTRIBUTING.md](CONTRIBUTING.md). A separate contributor agreement is required
before a pull request can be merged so that commercial rights remain centralized.

Bug reports and feature proposals are welcome as GitHub issues.

---

<p align="center">
  Made with ❤️ for the networking community
</p>
