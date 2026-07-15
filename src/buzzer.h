#ifndef BUZZER_H
#define BUZZER_H

#include <Arduino.h>

void setupBuzzer(int pin);
void beep(int pin);
void beep(int pin, int freqHz, int durationMs);  // Mit Frequenz und Dauer
void beepPattern(int count, int freqHz = 1000, int durationMs = 80, int pauseMs = 80);

// Muss in loop() regelmaessig aufgerufen werden, treibt den Buzzer-
// Zustandsautomaten ohne blockierende delay()-Aufrufe.
void buzzerTick();

#endif
