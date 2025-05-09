// buzzer.h

#ifndef BUZZER_H
#define BUZZER_H

// 부저 핀 정의 (본인의 핀 번호에 맞게 수정)
#define BUZZ_PIN 2

// 함수 선언 (구현은 .cpp 파일에)
void initBuzzer(); //초기화
void playTone(int frequency, int duration);
void clickSound();
void alertSound();
void modeSound();
void stopwatchClickSound();
void startMeasureSound();
void stopMeasureSound();
void successSound();
void successUpwardTone();
void errorSound();
void errorDownwardTone();
void longErrorSound();
void warningSound();
void highAlertSound();
void startReadySound();
void endSound();
void longEndSound();
void mouseClickSound();
void mouseClickSound2();
void mouseClickSound3();
void realMouseClickSound1();
void realMouseClickSound2();
void realMouseClickSound3();
void mechanicalKeyboardClick1();
void mechanicalKeyboardClick2();
void mechanicalKeyboardClick3();
void mechanicalKeyboardClick4();
void mechanicalKeyboardClick5();
void mechanicalKeyboardClick6();
void countdownBeep();

#endif