#include "buzzer.h"
#include <Arduino.h>

#define BUZZ_PIN 2

void initBuzzer() {
    pinMode(BUZZ_PIN, OUTPUT);
}

void clickSound() {
    tone(BUZZ_PIN, 1000, 20);  
    delay(10);
    noTone(BUZZ_PIN);
}

void alertSound() {
    tone(BUZZ_PIN, 1000, 200);  // 경고음
    delay(210);
    noTone(BUZZ_PIN);
}

void modeSound() {
    tone(BUZZ_PIN, 800, 50);  // 짧은 부저음 (모드 변경 알림)
    delay(60);
    tone(BUZZ_PIN, 1200, 50);
    delay(60);
    noTone(BUZZ_PIN);
}

void stopwatchClickSound() {
    tone(BUZZ_PIN, 2000, 20);
    delay(60);
    noTone(BUZZ_PIN);
}
