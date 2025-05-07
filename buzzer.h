#ifndef BUZZER_H
#define BUZZER_H


// 부저 초기화 함수
void initBuzzer();

// 클릭음 함수
void clickSound();

// 경고음 함수
void alertSound();

// 모드 변경음 함수
void modeSound();

// 스톱워치 클릭음 함수
void stopwatchClickSound();

// 카운트다운 비프음
void countdownBeep(int secondsLeft);

// 스타트 비프음
void startbeep();

#endif