// buzzer.cpp

#include "buzzer.h"
#include <Arduino.h> // Arduino 함수 사용을 위해 포함

void initBuzzer() {
  pinMode(BUZZ_PIN, OUTPUT);
  noTone(BUZZ_PIN); // 초기 상태는 소리 없음
}

void playTone(int frequency, int duration) {
  if (frequency > 0) {
    tone(BUZZ_PIN, frequency, duration);
  } else {
    delay(duration);
  }
}

void clickSound() {
  tone(BUZZ_PIN, 2000, 50);
  delay(50);
  noTone(BUZZ_PIN);
}

void alertSound() {
  tone(BUZZ_PIN, 1500, 100);
  delay(100);
  noTone(BUZZ_PIN);
}

void modeSound() {
  tone(BUZZ_PIN, 1000, 80);
  delay(50);
  tone(BUZZ_PIN, 1200, 80);
  delay(80);
  noTone(BUZZ_PIN);
}

void stopwatchClickSound() {
  tone(BUZZ_PIN, 2000, 20);
  delay(60);
  noTone(BUZZ_PIN);
}

void startMeasureSound() {
  tone(BUZZ_PIN, 3000, 150);
  delay(150);
  noTone(BUZZ_PIN);
}

void stopMeasureSound() {
  tone(BUZZ_PIN, 500, 200);
  delay(200);
  noTone(BUZZ_PIN);
}

void successSound() {
  tone(BUZZ_PIN, 2500, 80);
  delay(50);
  tone(BUZZ_PIN, 3000, 80);
  delay(80);
  noTone(BUZZ_PIN);
}

void successUpwardTone() {
  tone(BUZZ_PIN, 1500, 100);
  tone(BUZZ_PIN, 2000, 100);
  tone(BUZZ_PIN, 2500, 100);
  delay(100);
  noTone(BUZZ_PIN);
}

void errorSound() {
  tone(BUZZ_PIN, 500, 100);
  delay(50);
  tone(BUZZ_PIN, 400, 100);
  delay(150);
  noTone(BUZZ_PIN);
}

void errorDownwardTone() {
  tone(BUZZ_PIN, 2500, 100);
  tone(BUZZ_PIN, 2000, 100);
  tone(BUZZ_PIN, 1500, 100);
  delay(100);
  noTone(BUZZ_PIN);
}

void longErrorSound() {
  tone(BUZZ_PIN, 300, 300);
  delay(300);
  noTone(BUZZ_PIN);
}

void warningSound() {
  tone(BUZZ_PIN, 1000, 50);
  delay(50);
  tone(BUZZ_PIN, 1000, 50);
  delay(150);
  noTone(BUZZ_PIN);
}

void highAlertSound() {
  tone(BUZZ_PIN, 3500, 120);
  delay(120);
  noTone(BUZZ_PIN);
}

void startReadySound() {
  tone(BUZZ_PIN, 800, 100);
  tone(BUZZ_PIN, 1500, 100);
  delay(100);
  noTone(BUZZ_PIN);
}

void endSound() {
  tone(BUZZ_PIN, 1500, 100);
  tone(BUZZ_PIN, 800, 100);
  delay(100);
  noTone(BUZZ_PIN);
}

void longEndSound() {
  tone(BUZZ_PIN, 600, 400);
  delay(400);
  noTone(BUZZ_PIN);
}

void mouseClickSound() {
  tone(BUZZ_PIN, 3000, 30);
  delay(30);
  noTone(BUZZ_PIN);
}

void mouseClickSound2() {
  tone(BUZZ_PIN, 3500, 25);
  delay(25);
  noTone(BUZZ_PIN);
}

void mouseClickSound3() {
  tone(BUZZ_PIN, 2800, 40);
  delay(40);
  noTone(BUZZ_PIN);
}

void realMouseClickSound1() {
    tone(BUZZ_PIN, 110, 10);  // 낮은 주파수, 짧은 시간(10ms)
    delay(10);
    noTone(BUZZ_PIN);
}

void realMouseClickSound2() {
  for (int freq = 3000; freq <= 3500; freq += 50) {
    tone(BUZZ_PIN, freq, 1);
  }
  tone(BUZZ_PIN, 3500, 20);
  delay(20);
  noTone(BUZZ_PIN);
}

void realMouseClickSound3() {
  tone(BUZZ_PIN, 3300, 1);
  for (int freq = 3300; freq >= 2800; freq -= 50) {
    tone(BUZZ_PIN, freq, 1);
  }
  tone(BUZZ_PIN, 2800, 25);
  delay(25);
  noTone(BUZZ_PIN);
}

void mechanicalKeyboardClick1() {
  tone(BUZZ_PIN, 4000, 20);
  delay(20);
  noTone(BUZZ_PIN);
}

void mechanicalKeyboardClick2() {
  tone(BUZZ_PIN, 3800, 25);
  delay(25);
  noTone(BUZZ_PIN);
}

void mechanicalKeyboardClick3() {
  tone(BUZZ_PIN, 3500, 15);
  delay(3);
  tone(BUZZ_PIN, 4200, 20);
  delay(20);
  noTone(BUZZ_PIN);
}

void mechanicalKeyboardClick4() {
  for (int freq = 3800; freq <= 4200; freq += 100) {
    tone(BUZZ_PIN, freq, 1);
  }
  tone(BUZZ_PIN, 4200, 15);
  delay(15);
  noTone(BUZZ_PIN);
}

void mechanicalKeyboardClick5() {
  tone(BUZZ_PIN, 4100, 15);
  for (int freq = 4100; freq >= 3700; freq -= 100) {
    tone(BUZZ_PIN, freq, 1);
  }
  delay(15);
  noTone(BUZZ_PIN);
}

void mechanicalKeyboardClick6() {
  tone(BUZZ_PIN, 4000, 30);
  delay(30);
  noTone(BUZZ_PIN);
}

void countdownBeep() {
  // 짧고 날카로운 '삐' 소리
  tone(BUZZ_PIN, 2000, 80);  // 주파수: 2000Hz, 지속시간: 80ms
  delay(90);  // 살짝 여유 (소리 겹치지 않게)
}