#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>

namespace terminal {

constexpr uint8_t TELNET_SE = 240;
constexpr uint8_t TELNET_SB = 250;
constexpr uint8_t TELNET_WILL = 251;
constexpr uint8_t TELNET_WONT = 252;
constexpr uint8_t TELNET_DO = 253;
constexpr uint8_t TELNET_DONT = 254;
constexpr uint8_t TELNET_IAC = 255;

template <size_t Capacity>
class ByteRingBuffer {
public:
    static_assert(Capacity > 0, "ByteRingBuffer capacity must be positive");

    constexpr size_t capacity() const { return Capacity; }
    size_t size() const { return size_; }
    size_t freeSpace() const { return Capacity - size_; }
    bool empty() const { return size_ == 0; }

    bool push(uint8_t value) {
        if (size_ == Capacity) {
            return false;
        }
        storage_[(head_ + size_) % Capacity] = value;
        ++size_;
        return true;
    }

    bool push(const uint8_t* data, size_t length) {
        if ((!data && length > 0) || length > freeSpace()) {
            return false;
        }
        for (size_t index = 0; index < length; ++index) {
            push(data[index]);
        }
        return true;
    }

    const uint8_t* frontData() const {
        return empty() ? nullptr : storage_.data() + head_;
    }

    size_t contiguousSize() const {
        return std::min(size_, Capacity - head_);
    }

    void consume(size_t length) {
        size_t consumed = std::min(length, size_);
        head_ = (head_ + consumed) % Capacity;
        size_ -= consumed;
        if (size_ == 0) {
            head_ = 0;
        }
    }

    void clear() {
        head_ = 0;
        size_ = 0;
    }

private:
    std::array<uint8_t, Capacity> storage_{};
    size_t head_ = 0;
    size_t size_ = 0;
};

enum class TelnetEventType {
    None,
    Data,
    Negotiation,
};

struct TelnetEvent {
    TelnetEventType type = TelnetEventType::None;
    uint8_t value = 0;
    uint8_t command = 0;
    uint8_t option = 0;
};

class TelnetDecoder {
public:
    TelnetEvent feed(uint8_t value) {
        switch (state_) {
        case State::Data:
            if (value == TELNET_IAC) {
                state_ = State::Iac;
                return {};
            }
            return {TelnetEventType::Data, value, 0, 0};

        case State::Iac:
            if (value == TELNET_IAC) {
                state_ = State::Data;
                return {TelnetEventType::Data, TELNET_IAC, 0, 0};
            }
            if (value >= TELNET_WILL && value <= TELNET_DONT) {
                pendingCommand_ = value;
                state_ = State::Negotiation;
                return {};
            }
            state_ = value == TELNET_SB ? State::Subnegotiation : State::Data;
            return {};

        case State::Negotiation: {
            uint8_t command = pendingCommand_;
            state_ = State::Data;
            return {TelnetEventType::Negotiation, 0, command, value};
        }

        case State::Subnegotiation:
            if (value == TELNET_IAC) {
                state_ = State::SubnegotiationIac;
            }
            return {};

        case State::SubnegotiationIac:
            state_ = value == TELNET_SE ? State::Data : State::Subnegotiation;
            return {};
        }
        return {};
    }

    void reset() {
        state_ = State::Data;
        pendingCommand_ = 0;
    }

private:
    enum class State {
        Data,
        Iac,
        Negotiation,
        Subnegotiation,
        SubnegotiationIac,
    };

    State state_ = State::Data;
    uint8_t pendingCommand_ = 0;
};

class NewlineDecoder {
public:
    bool feed(uint8_t input, uint8_t& output) {
        if (previousWasCR_) {
            previousWasCR_ = false;
            if (input == '\n' || input == '\0') {
                return false;
            }
        }

        if (input == '\r') {
            output = '\r';
            previousWasCR_ = true;
            return true;
        }
        if (input == '\n') {
            output = '\r';
            return true;
        }

        output = input;
        return true;
    }

    void reset() { previousWasCR_ = false; }

private:
    bool previousWasCR_ = false;
};

template <size_t Capacity>
bool enqueueTelnetData(ByteRingBuffer<Capacity>& buffer, uint8_t value) {
    size_t required = value == TELNET_IAC ? 2 : 1;
    if (buffer.freeSpace() < required) {
        return false;
    }
    buffer.push(value);
    if (value == TELNET_IAC) {
        buffer.push(value);
    }
    return true;
}

} // namespace terminal