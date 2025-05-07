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

void startbeep() {
  tone(BUZZ_PIN, 880, 100);  delay(6);  // A5
  tone(BUZZ_PIN, 1046, 100); delay(6);  // C6
  tone(BUZZ_PIN, 1318, 300); delay(100);  // E6
  noTone(BUZZ_PIN);
}

void countdownBeep(int secondsLeft) {
  int freq = 0;
  int duration = 0;

  switch (secondsLeft) {
    case 3:
      freq = 1000; duration = 400; break;
    case 2:
      freq = 1000; duration = 400; break;
    case 1:
      freq = 1000; duration = 400; break;
    default:
      return;
  }

  tone(BUZZ_PIN, freq, duration);
  delay(duration);
  noTone(BUZZ_PIN);
}