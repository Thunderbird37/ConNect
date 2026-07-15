#pragma once

namespace OTAConfig {
inline constexpr const char* DEFAULT_MANIFEST_URL =
    "https://github.com/Thunderbird37/ConNect/releases/latest/download/latest.json";
inline constexpr const char* FIRMWARE_FILENAME = "firmware.bin";
inline constexpr const char* STORAGE_VERSION_KEY = "otaCfgVer";
inline constexpr unsigned char STORAGE_VERSION = 1;
}