#pragma once

#include <cstddef>
#include <cstdint>

#include "config_validation.h"

namespace BaudConfig {
// Einzige Quelle fuer gueltige Raten ist config::VALID_BAUD_RATES.
inline constexpr const auto& ALL_RATES = config::VALID_BAUD_RATES;
inline constexpr int ALL_RATES_COUNT = static_cast<int>(config::VALID_BAUD_RATES.size());

// Rotation fuer Doppelklick (haeufig verwendete Raten)
inline constexpr uint32_t CYCLE_RATES[] = {9600, 19200, 38400, 57600, 115200, 230400};
inline constexpr int CYCLE_RATES_COUNT = sizeof(CYCLE_RATES) / sizeof(CYCLE_RATES[0]);

inline constexpr uint32_t DEFAULT_BAUD = 115200;

constexpr bool isValid(uint32_t baud) {
    return config::isValidBaudRate(baud);
}
} // namespace BaudConfig
