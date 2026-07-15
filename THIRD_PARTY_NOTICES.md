# Third-Party Notices

The project license in [LICENSE](LICENSE) applies only to material for which
Thunderbird37 is the licensor. It does not replace or restrict the licenses of
third-party components.

## Vendored USB Host Code

Files below the following directories carry Espressif copyright notices and the
SPDX identifier `Apache-2.0`:

- `lib/usb_host_cdc_acm/`
- `lib/usb_host_ch34x_vcp/`
- `lib/usb_host_cp210x_vcp/`
- `lib/usb_host_ftdi_vcp/`
- `lib/usb_host_vcp/`

Those files remain licensed under the
[Apache License 2.0](LICENSES/Apache-2.0.txt).

## Build Dependencies

PlatformIO downloads additional components during the build. They retain their
upstream terms, including:

| Component | License |
|-----------|---------|
| [NimBLE-Arduino](https://github.com/h2zero/NimBLE-Arduino) | Apache-2.0 |
| [U8g2](https://github.com/olikraus/u8g2) | BSD-2-Clause for library code; individual fonts may differ |
| [Adafruit NeoPixel](https://github.com/adafruit/Adafruit_NeoPixel) | LGPL-3.0-only |
| [Arduino-ESP32](https://github.com/espressif/arduino-esp32) and its framework components | Upstream component licenses |

The upstream packages provide their complete copyright and license notices.
Permissions granted for those components do not grant commercial rights to the
ConNect-owned portions of this project.
