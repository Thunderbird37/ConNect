#include <Arduino.h>
#include "buzzer.h"

// arduino-esp32 3.x: ledc-API arbeitet jetzt pin-basiert statt kanalbasiert.
// ledcSetup/ledcAttachPin -> ledcAttach(pin, freq, resolution).
// ledcWriteTone(channel, freq) -> ledcWriteTone(pin, freq).
//
// Non-blocking Zustandsautomat: beep()/beepPattern() reihen Toene nur in
// eine Queue ein und kehren sofort zurueck. buzzerTick() wird regelmaessig
// aus loop() gepumpt und schaltet die Toene nach millis()-Zeitplan.
//
// Thread-safety: beep()/beepPattern() werden auch aus BLE-Callbacks
// aufgerufen (anderer FreeRTOS-Task). Queue-Zugriffe sind mit einem
// portMUX_TYPE Spinlock abgesichert.

static int s_buzzerPin = -1;

struct ToneStep {
    uint16_t freqHz;   // 0 = Pause
    uint16_t durMs;
};

static constexpr size_t QUEUE_MAX = 16;
static ToneStep s_queue[QUEUE_MAX];
static size_t   s_qHead = 0;   // next to play
static size_t   s_qTail = 0;   // next free slot
static bool     s_playing = false;
static uint32_t s_stepEndMs = 0;

static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;

static inline size_t qCountLocked() {
    return (s_qTail + QUEUE_MAX - s_qHead) % QUEUE_MAX;
}

static bool qPush(uint16_t freqHz, uint16_t durMs) {
    if (durMs == 0) return true;
    bool ok = false;
    portENTER_CRITICAL(&s_mux);
    if (qCountLocked() < QUEUE_MAX - 1) {
        s_queue[s_qTail] = {freqHz, durMs};
        s_qTail = (s_qTail + 1) % QUEUE_MAX;
        ok = true;
    }
    portEXIT_CRITICAL(&s_mux);
    return ok;
}

static bool qPop(ToneStep& out) {
    bool ok = false;
    portENTER_CRITICAL(&s_mux);
    if (s_qHead != s_qTail) {
        out = s_queue[s_qHead];
        s_qHead = (s_qHead + 1) % QUEUE_MAX;
        ok = true;
    }
    portEXIT_CRITICAL(&s_mux);
    return ok;
}

void setupBuzzer(int pin) {
    s_buzzerPin = pin;
    ledcAttach(pin, 1000, 8);  // 1000 Hz, 8-bit Aufloesung
    ledcWriteTone(pin, 0);
}

void buzzerTick() {
    if (s_buzzerPin < 0) return;

    if (s_playing) {
        if ((int32_t)(millis() - s_stepEndMs) < 0) return; // noch laufen lassen
        ledcWriteTone(s_buzzerPin, 0);
        s_playing = false;
    }

    ToneStep st;
    if (qPop(st)) {
        ledcWriteTone(s_buzzerPin, st.freqHz);   // freqHz==0 schaltet ab
        s_stepEndMs = millis() + st.durMs;
        s_playing   = true;
    }
}

void beep(int pin) {
    if (s_buzzerPin < 0) s_buzzerPin = pin;
    qPush(1000, 100);
}

void beep(int pin, int freqHz, int durationMs) {
    if (s_buzzerPin < 0) s_buzzerPin = pin;
    if (durationMs <= 0) return;
    qPush((uint16_t)freqHz, (uint16_t)durationMs);
}

void beepPattern(int count, int freqHz, int durationMs, int pauseMs) {
    if (s_buzzerPin < 0 || count <= 0 || durationMs <= 0) return;
    for (int i = 0; i < count; i++) {
        qPush((uint16_t)freqHz, (uint16_t)durationMs);
        if (i < count - 1 && pauseMs > 0) qPush(0, (uint16_t)pauseMs);
    }
}
