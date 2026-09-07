#include "terminal_transport.h"
#include "config_validation.h"

#include <array>
#include <cassert>
#include <cstdint>
#include <vector>

using terminal::ByteRingBuffer;
using terminal::NewlineDecoder;
using terminal::TelnetDecoder;
using terminal::TelnetEventType;

static std::vector<uint8_t> drain(ByteRingBuffer<8>& buffer) {
    std::vector<uint8_t> output;
    while (!buffer.empty()) {
        size_t length = buffer.contiguousSize();
        output.insert(output.end(), buffer.frontData(), buffer.frontData() + length);
        buffer.consume(length);
    }
    return output;
}

static void testRingWraparound() {
    ByteRingBuffer<8> buffer;
    const uint8_t first[] = {1, 2, 3, 4, 5, 6};
    assert(buffer.push(first, sizeof(first)));
    buffer.consume(5);
    const uint8_t second[] = {7, 8, 9, 10, 11, 12};
    assert(buffer.push(second, sizeof(second)));
    assert((drain(buffer) == std::vector<uint8_t>{6, 7, 8, 9, 10, 11, 12}));
}

static void testRingRejectsAtomically() {
    ByteRingBuffer<8> buffer;
    const uint8_t initial[] = {1, 2, 3, 4, 5, 6, 7};
    const uint8_t tooLarge[] = {8, 9};
    assert(buffer.push(initial, sizeof(initial)));
    assert(!buffer.push(tooLarge, sizeof(tooLarge)));
    assert((drain(buffer) == std::vector<uint8_t>{1, 2, 3, 4, 5, 6, 7}));
}

static void testTelnetDataAndNegotiation() {
    TelnetDecoder decoder;
    const uint8_t input[] = {
        0x1B, '[', 'A',
        terminal::TELNET_IAC, terminal::TELNET_WILL, 3,
        terminal::TELNET_IAC, terminal::TELNET_IAC,
        terminal::TELNET_IAC, terminal::TELNET_SB, 31, 0, 80,
        terminal::TELNET_IAC, terminal::TELNET_SE,
        'Z',
    };
    std::vector<uint8_t> data;
    size_t negotiations = 0;
    for (uint8_t value : input) {
        auto event = decoder.feed(value);
        if (event.type == TelnetEventType::Data) {
            data.push_back(event.value);
        } else if (event.type == TelnetEventType::Negotiation) {
            ++negotiations;
            assert(event.command == terminal::TELNET_WILL);
            assert(event.option == 3);
        }
    }
    assert(negotiations == 1);
    assert((data == std::vector<uint8_t>{0x1B, '[', 'A', 0xFF, 'Z'}));
}

static void testTelnetEncoding() {
    ByteRingBuffer<8> buffer;
    assert(terminal::enqueueTelnetData(buffer, 'A'));
    assert(terminal::enqueueTelnetData(buffer, 0xFF));
    assert(terminal::enqueueTelnetData(buffer, 'B'));
    assert((drain(buffer) == std::vector<uint8_t>{'A', 0xFF, 0xFF, 'B'}));
}

static void testNewlinesAndControlSequences() {
    NewlineDecoder decoder;
    const uint8_t input[] = {'A', '\r', '\n', '\n', '\n', 0x1B, '[', 'D', '\r', '\0', 'B'};
    std::vector<uint8_t> output;
    for (uint8_t value : input) {
        uint8_t decoded = 0;
        if (decoder.feed(value, decoded)) {
            output.push_back(decoded);
        }
    }
    assert((output == std::vector<uint8_t>{'A', '\r', '\r', '\r', 0x1B, '[', 'D', '\r', 'B'}));
}

static void testTerminalKeysRemainByteExact() {
    const std::vector<uint8_t> input = {
        0x1B, '[', 'A', 0x1B, '[', 'B', 0x1B, '[', 'C', 0x1B, '[', 'D',
        0x1B, '[', 'H', 0x1B, '[', 'F',
        0x1B, '[', '5', '~', 0x1B, '[', '6', '~',
        0x1B, '[', '2', '~', 0x1B, '[', '3', '~',
        0x1B, 'O', 'P', '\t', 0x03, '\b', 0x7F,
    };

    TelnetDecoder telnetDecoder;
    NewlineDecoder newlineDecoder;
    std::vector<uint8_t> output;
    for (uint8_t value : input) {
        auto event = telnetDecoder.feed(value);
        if (event.type != TelnetEventType::Data) continue;
        uint8_t decoded = 0;
        if (newlineDecoder.feed(event.value, decoded)) {
            output.push_back(decoded);
        }
    }
    assert(output == input);
}

static void testConfigurationValidation() {
    assert(config::isValidBaudRate(300));
    assert(config::isValidBaudRate(921600));
    assert(!config::isValidBaudRate(0));
    assert(!config::isValidBaudRate(115201));

    assert(config::isValidSerialMode("USB"));
    assert(config::isValidSerialMode("RS232"));
    assert(!config::isValidSerialMode("usb"));
    assert(config::isValidWiFiMode("AP"));
    assert(config::isValidWiFiMode("CLIENT"));
    assert(!config::isValidWiFiMode("STA"));

    assert(config::isValidSsidLength(0));
    assert(config::isValidSsidLength(32));
    assert(!config::isValidSsidLength(33));
    assert(config::isValidWiFiPasswordLength(0));
    assert(config::isValidWiFiPasswordLength(8));
    assert(config::isValidWiFiPasswordLength(64));
    assert(!config::isValidWiFiPasswordLength(7));
    assert(!config::isValidWiFiPasswordLength(65));
}

int main() {
    testRingWraparound();
    testRingRejectsAtomically();
    testTelnetDataAndNegotiation();
    testTelnetEncoding();
    testNewlinesAndControlSequences();
    testTerminalKeysRemainByteExact();
    testConfigurationValidation();
    return 0;
}
