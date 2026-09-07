#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <string_view>

namespace config {

constexpr std::array<uint32_t, 12> VALID_BAUD_RATES = {
    300, 1200, 2400, 4800, 9600, 19200,
    38400, 57600, 115200, 230400, 460800, 921600,
};

constexpr bool isValidBaudRate(uint32_t baudRate) {
    for (uint32_t candidate : VALID_BAUD_RATES) {
        if (candidate == baudRate) return true;
    }
    return false;
}

constexpr bool isValidSerialMode(std::string_view mode) {
    return mode == "USB" || mode == "RS232";
}

constexpr bool isValidWiFiMode(std::string_view mode) {
    return mode == "AP" || mode == "CLIENT";
}

constexpr bool isValidSsidLength(size_t length) {
    return length <= 32;
}

constexpr bool isValidWiFiPasswordLength(size_t length) {
    return length == 0 || (length >= 8 && length <= 64);
}

} // namespace config
