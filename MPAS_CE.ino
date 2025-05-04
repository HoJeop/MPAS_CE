#include <M5StickCPlus2.h>
#include <arduinoFFT.h>
#include <math.h>
#include <time.h>
#include "buzzer.h"

// 색상 정의 (16비트 컬러) =====================================================
#define WHITE       0xFFFF      // 흰색
#define BLUE        0x001F      // 파란색
#define ORANGE      0xFD20      // 주황색
#define YELLOW      0xFFE0      // 노란색
#define RED         0xF800      // 빨간색
#define PURPLE      0x780F      // 보라색
#define DARKGRAY    0x7BEF      // 어두운 회색
#define DARKGREEN   0x03E0      // 어두운 녹색
#define DARKRED     0x7800      // 어두운 빨간색
#define LIGHTGRAY   0xBDF7      // 밝은 회색

// M5StickC Plus2의 화면을 더블 버퍼링 방식으로 사용하기 위한 스프라이트 객체
M5Canvas sprite(&M5.Display);

// [전역] 버튼 처리 관련 변수 ==================================================
unsigned long btnAPressStart = 0;     // 버튼 A가 눌린 시작 시간
bool longPressTriggered = false;      // 버튼 A가 길게 눌렸는지 여부
unsigned long btnBPressStart = 0;     // 버튼 B가 눌린 시작 시간
bool bLongPressTriggered = false;     // 버튼 B가 길게 눌렸는지 여부

// FFT 설정===================================================================
#define SAMPLES             512     // FFT 분석을 위한 샘플 개수 (2의 제곱수)
#define SAMPLING_FREQUENCY  2453     // 샘플링 주파수 (Hz), 마이크에서 데이터를 읽어오는 속도

// FFT 및 마이크 처리 관련 변수
float vReal[SAMPLES];               // 실수부 데이터를 저장할 배열
float vImag[SAMPLES];               // 허수부 데이터를 저장할 배열 (FFT 결과)
int16_t micBuffer[SAMPLES];         // 마이크로부터 읽어온 데이터를 저장할 버퍼
ArduinoFFT<float> FFT = ArduinoFFT<float>(); // FFT 객체 생성

float findMajorPeakInFrequencyRange(float *magnitude, int samples, float sampleRate, float minFreq, float maxFreq) {
    int startIndex = round(minFreq * samples / sampleRate);
    int endIndex = round(maxFreq * samples / sampleRate);
    float maxMagnitude = 0;
    int maxIndex = -1;

    for (int i = startIndex; i <= endIndex; i++) {
        if (magnitude[i] > maxMagnitude) {
            maxMagnitude = magnitude[i];
            maxIndex = i;
        }
    }

    if (maxIndex != -1) {
        return (float)maxIndex * sampleRate / samples;
    } else {
        return 0.0; // 해당 범위 내에 유효한 피크가 없을 경우
    }
}

//=============================================================================
// 모드 번호 정의 (0~3)
// 0: 센서 모드 (마이크를 이용한 속도 측정)
// 1: 스톱워치 모드
// 2: 경과시간 시계 모드
// 3: QR코드 순환출력 모드
int mode = 0;                       // 현재 활성화된 모드
int prevMode = -1;                  // 이전 모드를 저장하여 모드 전환 시 화면을 초기화하는 데 사용

// [모드 0] 센서 모드 관련 변수 =================================================
float wheelSizes[] = {24.0, 26.0, 31.0}; // 휠 크기 옵션 (단위: mm)
int numWheelSizes = sizeof(wheelSizes) / sizeof(wheelSizes[0]); // 휠 크기 옵션 개수
int currentWheelIndex = 0;          // 현재 선택된 휠 크기 인덱스
float gearRatios[] = {3.7, 3.5};     // 기어비 옵션
int numGearRatios = sizeof(gearRatios) / sizeof(gearRatios[0]); // 기어비 옵션 개수
int currentGearIndex = 0;           // 현재 선택된 기어비 인덱스
uint16_t gearColors[] = {GREEN, CYAN}; // 기어비에 따른 색상

#define AVG_WINDOW_SIZE 15 // 이동평균 필터
float frequencyBuffer[AVG_WINDOW_SIZE];
int bufferIndex = 0;

// [모드 1] 스톱워치 모드 관련 변수 =============================================
bool stopwatchRunning = false;      // 스톱워치가 실행 중인지 여부
unsigned long stopwatchStartTime = 0, stopwatchElapsed = 0, lastLapTime = 0;
                                    // 스톱워치 시작 시간, 경과 시간, 마지막 랩 타임
int lapCount = 0, currentLapTargetIndex = 0;
                                    // 현재 랩 수, 랩 목표 인덱스
const int MAX_LAPS = 10;            // 최대 랩 저장 개수
unsigned long lapTimes[MAX_LAPS];   // 랩 타임을 저장할 배열
int allowedLapTargets[] = {3};      // 허용된 랩 목표 (현재는 3랩으로 고정)
int splitTarget = allowedLapTargets[currentLapTargetIndex]; // 현재 랩 목표
// 깜박임 효과를 위한 변수
static unsigned long lastBlinkTime = 0; // 마지막 깜박임 시간
static bool blinkState = false;         // 현재 깜박임 상태 (true: 보임, false: 숨김)
const unsigned long blinkInterval = 300; // 깜박임 간격 (ms)

// [모드 2] 경과시간 시계 관련 변수 =============================================
unsigned long clockStartTime = 0;   // 시계 시작 시간
bool clockPaused = false;           // 시계가 일시 정지되었는지 여부
unsigned long pausedTime = 0;       // 일시 정지된 총 시간
unsigned long lastDisplaySecond = 0; // 마지막으로 화면에 표시된 초

// [모드 3] QR코드 순환출력 =====================================================
const int numQRs = 3;               // 표시할 QR 코드 개수
int currentQRIndex = 0;             // 현재 표시 중인 QR 코드 인덱스
const char* qrCodes[numQRs] = {     // QR 코드 내용
    "https://www.helloabt.com/mini4wd/",
    "https://map.naver.com/p/favorite/myPlace/folder/029fd0e8addc4d899ea722f0f5a7a613",
    "https://map.kakao.com/?target=other&folderid=16126480"
};
const char* qrLabels[numQRs] = {    // QR 코드 레이블 (설명)
    "Racer's Manner",
    "Naver Map",
    "Kakao Map"
};

// 배터리 잔량 계산 함수 ========================================================
float getBatteryPercent() {
    const uint32_t BAT_LOW = 3300;  // 배터리 부족 전압 (mV)
    const uint32_t BAT_HIGH = 4190; // 배터리 완충 전압 (mV)
    uint32_t voltage = M5.Power.getBatteryVoltage(); // 현재 배터리 전압 읽기
    if(voltage < BAT_LOW)
        voltage = BAT_LOW;          // 최저 전압 이하로 내려가지 않도록 보정
    if(voltage > BAT_HIGH)
        voltage = BAT_HIGH;         // 최고 전압 이상으로 올라가지 않도록 보정
    return ((float)(voltage - BAT_LOW) / (BAT_HIGH - BAT_LOW)) * 100.0f; // 백분율로 변환하여 반환
}

// setup() 함수 ===============================================================
void setup(){
    M5.begin();                     // M5StickC Plus2 초기화
    // 화면 설정
    M5.Display.setRotation(1);      // 화면 회전 설정 (가로 방향)
    M5.Display.fillScreen(BLACK);   // 화면 전체를 검은색으로 채워 초기화

    // 스프라이트 생성 (디스플레이 크기: 240x135)
    sprite.createSprite(240, 135);
    sprite.setTextSize(5);          // 텍스트 크기 설정 (기본 크기)
    sprite.setTextColor(WHITE);     // 텍스트 색상 설정 (흰색)
    sprite.setTextDatum(TL_DATUM);  // 텍스트 정렬 기준 설정 (Top-Left)
    M5.Power.begin();               // 전원 관리 초기화
    M5.Mic.begin();                 // 마이크 초기화
    M5.Imu.begin();                 // IMU 센서 초기화
    initBuzzer();                   // 부저 초기화 (buzzer.h에 정의된 함수)
    delay(100);                     // 초기화 완료 대기
}

// loop() 함수 ================================================================
void loop(){
    M5.update();                    // 버튼 입력 및 센서 값 갱신 (필수 호출)
    // 공용 처리: 배터리 잔량 측정 및 LED 깜빡임
    float batteryPct = getBatteryPercent(); // 배터리 잔량 백분율 계산
    if(batteryPct <= 10.0){         // 배터리 잔량이 10% 이하인 경우
        if(millis() % 2000 < 1000)   // 2초마다 1초씩
            M5.Power.setLed(1);      // LED 켜기 (빨간색)
        else
            M5.Power.setLed(0);      // LED 끄기
    } else {                        // 배터리 잔량이 충분한 경우
        M5.Power.setLed(0);          // LED 끄기
    }
    // 모드 전환 시: 전체 화면 BLACK 및 기본 텍스트 정렬 복구
    if(prevMode != mode){           // 현재 모드가 이전 모드와 다른 경우 (모드 전환 발생)
        sprite.fillScreen(BLACK);   // 화면을 검은색으로 채우고
        sprite.setTextDatum(TL_DATUM); // 텍스트 정렬을 기본값 (Top-Left)으로 설정
        prevMode = mode;            // 이전 모드를 현재 모드로 업데이트
    }
    // 글로벌 버튼 B 처리 (모드 0~3 모두 적용)
    if (M5.BtnB.isPressed()) {      // 버튼 B가 눌린 상태라면
        if (btnBPressStart == 0)     // 버튼 B가 처음 눌린 순간이라면
            btnBPressStart = millis(); // 눌린 시작 시간 기록
       }
       else if (M5.BtnB.wasReleased()) { // 버튼 B가 떼어진 순간이라면
        unsigned long duration = millis() - btnBPressStart; // 버튼 B가 눌린 총 시간 계산

        // 3초 이상 누른 경우: 센서 모드와 스톱워치 모드에서 값 초기화 (모드는 변경하지 않음)
        if (duration >= 3000) {
            if (mode == 1) {         // 현재 모드가 스톱워치 모드라면
                // 스톱워치 리셋
                stopwatchRunning = false; // 스톱워치 정지
                stopwatchElapsed = 0;   // 경과 시간 초기화
                stopwatchStartTime = 0; // 시작 시간 초기화
                lapCount = 0;           // 랩 수 초기화
                lastLapTime = 0;        // 마지막 랩 타임 초기화
            }
        }
        // 1초 이상 2초 미만 누른 경우: 이전 모드로 이동
        else if (duration >= 1000 && duration < 2000) {
            mode = (mode - 1 + 4) % 4;  // 현재 모드를 이전 모드로 변경 (음수 인덱스 방지)
        }
        // 1초 미만 (짧게 클릭): 다음 모드로 이동
        else if (duration < 1000) {
            mode = (mode + 1) % 4;      // 현재 모드를 다음 모드로 변경
            clickSound();               // 짧게 클릭 시 효과음 재생 (buzzer.h에 정의)
        }
        // 버튼 상태 초기화
        btnBPressStart = 0;
        bLongPressTriggered = false;
        // 화면 초기화 후 잠시 지연 (250ms)
        sprite.fillScreen(BLACK);
        delay(250);
        return;                     // 현재 루프 종료 후 다음 루프로 진행
    }

    // ────────────────────────────────────────────
    // [모드 0] 센서 모드
    if(mode == 0){
        // A 버튼 처리
        //  - 누른 시간 < 1000ms (릴리즈): 기어비 변경 처리
        //  - 누른 시간 1000ms 이상 2000ms 미만 (릴리즈): 휠 크기 변경 처리
        unsigned long aDuration = 0;
        if (M5.BtnA.isPressed()) {      // 버튼 A가 눌린 상태라면
            if (btnAPressStart == 0)     // 버튼 A가 처음 눌린 순간이라면
            btnAPressStart = millis(); // 눌린 시작 시간 기록
            aDuration = millis() - btnAPressStart; // 눌린 시간 계산
        }
        if (M5.BtnA.wasReleased()) {    // 버튼 A가 떼어진 순간이라면
            aDuration = millis() - btnAPressStart; // 버튼 A가 눌린 총 시간 계산
            // 짧게 눌렀을 경우 (< 1000ms): 기어비 변경 처리
            if (aDuration < 1000) {
                currentGearIndex = (currentGearIndex + 1) % numGearRatios; // 다음 기어비 인덱스로 변경 (순환)
                alertSound();               // 짧게 클릭 시 효과음 재생 (buzzer.h에 정의)
            }
            // 1초 이상 2초 미만 눌렀다 떼면: 휠 크기 변경 처리
            else if (aDuration >= 1000 && aDuration < 2000) {
                currentWheelIndex = (currentWheelIndex + 1) % numWheelSizes; // 다음 휠 크기 인덱스로 변경 (순환)
                modeSound();                // 중간 길이 클릭 시 효과음 재생 (buzzer.h에 정의)
            }
            // 릴리즈 후 변수 모두 재설정
            btnAPressStart = 0;
            longPressTriggered = false;
        }
        // 센서 데이터 처리 (마이크 FFT 측정)
         float currentFrequencyRaw = 0.0; // 추가: currentFrequencyRaw 초기화
        float currentFrequency = 0.0;     // 추가: currentFrequency 선언 및 초기화
        if(M5.Mic.record(micBuffer, SAMPLES, SAMPLING_FREQUENCY)){ // 마이크로부터 데이터를 읽어왔다면
            for(int i = 0; i < SAMPLES; i++){
                vReal[i] = (float)micBuffer[i]; // 읽어온 마이크 데이터를 실수부 배열에 저장
                vImag[i] = 0.0;                 // 허수부 배열은 0으로 초기화
            }
            FFT.windowing(vReal, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD, vReal, true);
            // FFT 분석을 위한 윈도잉 함수 적용 (Hamming 윈도우)
            FFT.compute(vReal, vImag, SAMPLES, FFT_FORWARD);
            // 실제 FFT 계산 수행
            FFT.complexToMagnitude(vReal, vImag, SAMPLES);
            // 복소수 결과를 크기로 변환
            float currentFrequencyRaw = currentFrequency; // currentFrequency 값을 currentFrequencyRaw에 할당
             // 특정 주파수 범위 내에서 주요 피크 찾기
            float minFrequency = 250.0;
            float maxFrequency = 1000.0;
            float currentFrequency = findMajorPeakInFrequencyRange(vReal, SAMPLES, SAMPLING_FREQUENCY, minFrequency, maxFrequency);

             // --- 이동 평균 필터 코드 ---
            frequencyBuffer[bufferIndex] = currentFrequencyRaw;
            bufferIndex = (bufferIndex + 1) % AVG_WINDOW_SIZE;

            float currentFrequencyFiltered = 0;
                for (int i = 0; i < AVG_WINDOW_SIZE; i++) {
                    currentFrequencyFiltered += frequencyBuffer[i];
                }
            currentFrequencyFiltered /= AVG_WINDOW_SIZE;
            // --- 이동 평균 필터 코드 끝 ---

            if(currentFrequency < 20)
                currentFrequency = 0;       // 노이즈로 인한 낮은 주파수 값은 0으로 처리
            int rpmCurrent = (int)round(currentFrequency * 60);
            // 추출된 주파수를 RPM (분당 회전수)으로 변환
            float currentWheelDiameter = wheelSizes[currentWheelIndex] / 1000.0;
            // 선택된 휠 크기를 미터 단위로 변환
            float selectedGear = gearRatios[currentGearIndex];
            // 선택된 기어비
            float estimatedSpeed = (rpmCurrent / selectedGear) * (PI * currentWheelDiameter * 0.06);
            // 추정 속도 계산 (RPM / 기어비 * 휠 둘레 * 단위 변환 계수)

            // 휠 사이즈 출력 (좌측 상단)
            sprite.fillRect(10, 5, 90, 15, BLACK); // 해당 영역을 검은색으로 채워 지움
            sprite.setTextSize(2);
            sprite.setCursor(10,5);
            sprite.setTextColor(gearColors[currentGearIndex]); // 현재 기어비에 해당하는 색상으로 설정
            sprite.print(wheelSizes[currentWheelIndex], 1); // 휠 크기 출력 (소수점 1자리)
            sprite.setCursor(60,5);
            sprite.setTextColor(WHITE);
            sprite.print("mm");

            // 배터리 잔량 텍스트 (우측 상단)
            float batPct = getBatteryPercent(); // 현재 배터리 잔량 백분율 계산
            uint16_t batColor = GREEN;      // 기본 배터리 색상을 녹색으로 설정
            if(batPct <= 10.0){             // 배터리 잔량이 10% 이하인 경우
                if(millis() % 2000 < 1000)   // 2초마다 1초씩
                    batColor = RED;         // 빨간색으로 변경
                else
                    batColor = BLACK;       // 검은색으로 변경 (깜빡임 효과)
            }
            sprite.setTextSize(2);
            float batteryPct = getBatteryPercent();
            int roundedBattery = round(batteryPct); // 배터리 잔량 반올림
            sprite.fillRect(sprite.width()-60, 5, 60, 20, BLACK); // 해당 영역을 검은색으로 채워 지움
            sprite.setTextSize(2);
            sprite.setTextColor(batColor);
            sprite.setCursor(sprite.width()-59, 5);
            char formattedBattery[10];
            sprintf(formattedBattery, "%3d%", roundedBattery); // 배터리 잔량 문자열 포맷팅
            sprite.print(formattedBattery);
            sprite.setCursor(sprite.width()-20, 5);
            sprite.setTextColor(WHITE);
            sprite.print("%");

            // 배터리 잔량 10% 이하 도달시 LED 점등 (이미 상단에서 처리됨)

            // 현재 주파수
            sprite.fillRect(60, 33, 180, 30, BLACK); // 해당 영역을 검은색으로 채워 지움
            sprite.setTextSize(4);
            sprite.setCursor(10, 33);
            sprite.setTextColor(WHITE);
            sprite.print("P");
            int currFreqInt = (int)round(currentFrequency); // 현재 주파수 반올림
            sprite.setTextSize(3);
            sprite.setCursor(63, 33);
            sprite.setTextColor(CYAN);
            sprite.print(currFreqInt);
            sprite.setTextColor(WHITE);

            // 추정속도
            sprite.fillRect(50, 63, 190, 25, BLACK); // 해당 영역을 검은색으로 채워 지움
            sprite.setTextSize(2.3);
            sprite.setCursor(63, 63);
            sprite.setTextColor(ORANGE);
            sprite.print(estimatedSpeed, 1); // 추정 속도 출력 (소수점 1자리)
            sprite.setTextColor(WHITE);
            sprite.print(" km/h");

            // RPM
            sprite.fillRect(60, 88, 200, 25, BLACK); // 해당 영역을 검은색으로 채워 지움
            sprite.setTextSize(2.2);
            sprite.setCursor(63, 88);
            sprite.setTextColor(MAGENTA);
            sprite.print(rpmCurrent);
            sprite.setTextColor(WHITE);
            sprite.print(" rpm");

            //version 정보 (컴파일 시점의 시스템 시간에 기초)
            String dateStr = String(__DATE__).substring(4, 6); // 월 정보 추출
            String timeStr = String(__TIME__).substring(0, 2) + // 시
                             String(__TIME__).substring(3, 5) + // 분
                             String(__TIME__).substring(6, 8); // 초
            String mpas_version = "MPAS-CE-v0.1_" + dateStr + timeStr; // 버전 문자열 생성
            sprite.setTextSize(1.7);
            sprite.setTextColor(DARKGRAY);
            sprite.drawCentreString(mpas_version, sprite.width()/2, 116, 1.5); // 화면 중앙 하단에 버전 정보 출력
        }
        sprite.pushSprite(0, 0); // 스프라이트 내용을 실제 화면에 출력
        delay(100);
        return;
    } //mode 0 end

    //────────────────────────────────────────────
    // [모드 1] 스톱워치 모드
    if (mode == 1) {

        // A 버튼 입력 처리
        if (M5.BtnA.isPressed()) {
            if (btnAPressStart == 0)
                btnAPressStart = millis();  // 버튼 누른 시점 기록

            // 스톱워치가 멈춘 상태에서 1초 이상 버튼을 누르면 랩 타겟 변경 방지
            if (!stopwatchRunning && (millis() - btnAPressStart >= 1000)) {
                splitTarget = 3;            // 랩 타겟을 항상 3으로 고정
            }
        }

        // A 버튼 릴리즈 처리
        if (M5.BtnA.wasReleased()) {
            // 길게 눌렀지만 스톱워치가 멈춘 상태라면 동작 중지
            if (longPressTriggered && !stopwatchRunning) {
                btnAPressStart = 0;
                longPressTriggered = false;
                return;
            } else {
                // 스톱워치가 정지 상태라면 시작
                if (!stopwatchRunning) {
                    stopwatchRunning = true;
                    stopwatchStartTime = millis();
                    stopwatchElapsed = 0;
                    lapCount = 0;
                    lastLapTime = 0;
                } else {
                    // 현재 경과 시간 계산
                    unsigned long currentTimeLocal = stopwatchElapsed + (millis() - stopwatchStartTime);

                    // 랩 기록 저장
                    unsigned long lapInterval = currentTimeLocal - lastLapTime;
                    if (lapCount < MAX_LAPS)
                        lapTimes[lapCount] = lapInterval;

                    lapCount++;
                    lastLapTime = currentTimeLocal;

                    // 랩 카운트가 설정된 타겟에 도달하면 자동 정지
                    if (lapCount >= splitTarget) {
                        stopwatchRunning = false;
                        stopwatchElapsed = currentTimeLocal;
                    }
                }
            }

            // 버튼 입력 초기화
            btnAPressStart = 0;
            longPressTriggered = false;
            stopwatchClickSound();       // 스톱워치 버튼 클릭 시 효과음 재생 (buzzer.h에 정의)
        }

        // 스톱워치의 현재 시간 계산 (6시간 제한 적용)
        unsigned long currentTimeDisplay = stopwatchElapsed;
        if (stopwatchRunning)
            currentTimeDisplay += (millis() - stopwatchStartTime);
        if (currentTimeDisplay > 3600000UL) // 6시간 초과 방지
            currentTimeDisplay = 3600000UL;

        // 분, 초, 센티초 계산
        int minute = currentTimeDisplay / 60000;
        int second = (currentTimeDisplay % 60000) / 1000;
        int centisecond = (currentTimeDisplay % 1000) / 10;

        // 시간 문자열 포맷팅
        char timeStr[16];
        sprintf(timeStr, "%02d : %02d . %02d", minute, second, centisecond);

        // 상단 타이머 영역(높이 40픽셀)만 지워서 다시 업데이트
        sprite.fillRect(0, 0, sprite.width(), 40, BLACK);

        // 스톱워치가 정지되어 랩 타겟에 도달한 경우 타이머 숫자만 깜박임
        if (!stopwatchRunning && lapCount >= splitTarget) {
            if (millis() - lastBlinkTime >= blinkInterval) {
                blinkState = !blinkState;
                lastBlinkTime = millis();
            }
            uint16_t displayColor = blinkState ? WHITE : BLACK;

            sprite.setTextSize(2.8);
            for (int dx = -1; dx <= 1; dx++) {
                for (int dy = -1; dy <= 1; dy++) {
                    sprite.setTextColor(displayColor);
                    sprite.drawCentreString(timeStr, 120 + dx, 5 + dy, 2);
                }
            }
        } else {
            // 스톱워치가 진행 중이면 일반적으로 출력
            sprite.setTextSize(2.8);
            for (int dx = -1; dx <= 1; dx++) {
                for (int dy = -1; dy <= 1; dy++) {
                    sprite.setTextColor(WHITE);
                    sprite.drawCentreString(timeStr, 120 + dx, 5 + dy, 2);
                }
            }
        }

        // 랩 기록 출력 영역 업데이트 (타이머 아래쪽)
        sprite.fillRect(0, 40, sprite.width(), sprite.height() - 40, BLACK);

        if (lapCount > 0) {
            int startLap = (lapCount > 5) ? lapCount - 5 : 0; // 최근 5개 랩만 표시
            int yPos = 45;
            char lapLabel[10], lapSplitStr[20], cumStr[20];

            for (int i = startLap; i < lapCount; i++) {
                unsigned long lapTime = lapTimes[i];
                int lapMin = lapTime / 60000;
                int lapSec = (lapTime % 60000) / 1000;
                int lapCs = (lapTime % 1000) / 10;

                // 누적 시간 계산
                unsigned long cumulative = 0;
                for (int j = 0; j <= i; j++) {
                    cumulative += lapTimes[j];
                }
                int cumMin = cumulative / 60000;
                int cumSec = (cumulative % 60000) / 1000;
                int cumCs = (cumulative % 1000) / 10;

                // 랩 정보 출력
                sprintf(lapLabel, "L%d ", i + 1);
                sprintf(lapSplitStr, "%02d:%02d.%02d", lapMin, lapSec, lapCs);
                sprintf(cumStr, " (%02d:%02d.%02d)", cumMin, cumSec, cumCs);

                int xPos = 10;
                sprite.setTextSize(1.7);
                sprite.setTextColor(WHITE);
                sprite.drawString(lapLabel, xPos, yPos);
                xPos += sprite.textWidth(lapLabel);
                sprite.setTextColor(CYAN);
                sprite.drawString(lapSplitStr, xPos, yPos);
                xPos += sprite.textWidth(lapSplitStr);
                sprite.setTextColor(YELLOW);
                sprite.drawString(cumStr, xPos, yPos);
                yPos += 15;
            }
        }
        sprite.pushSprite(0, 0);
        delay(10);
        return;
    }   // mode 1 end


    //────────────────────────────────────────────
    // [모드 2] 경과시간 시계 모드
    if(mode == 2){
        if(M5.BtnA.isPressed()){
            if(btnAPressStart == 0)
                btnAPressStart = millis();
            if(millis() - btnAPressStart >= 1000 && !longPressTriggered){
                longPressTriggered = true;
                clockStartTime = millis(); // 길게 누르면 시작 시간 재설정
                pausedTime = 0;
                clockPaused = false;
            }
        }
        else if(M5.BtnA.wasReleased()){
            if(!longPressTriggered){
                if(!clockPaused){
                    clockPaused = true;
                    pausedTime = millis() - clockStartTime; // 일시 정지 시점 기록
                }
                else{
                    clockPaused = false;
                    clockStartTime = millis() - pausedTime; // 재개 시 시작 시간 업데이트
                }
            }
            btnAPressStart = 0;
            longPressTriggered = false;
            stopwatchClickSound();       // 버튼 클릭 시 효과음 재생
        }
        unsigned long elapsed = clockPaused ? pausedTime : (millis() - clockStartTime); // 경과 시간 계산
        unsigned int hours = elapsed / 3600000UL;
        unsigned int minutes = (elapsed % 3600000UL) / 60000;
        unsigned int seconds = (elapsed % 60000UL) / 1000;
        char timeStr[9];
        sprintf(timeStr, "%02u:%02u:%02u", hours, minutes, seconds); // 시간 문자열 포맷팅
        if(seconds != lastDisplaySecond){ // 초가 변경되었을 때만 화면 업데이트
            lastDisplaySecond = seconds;
            int posY = (sprite.height() - 20) / 2;
            sprite.fillRect(0, posY, sprite.width(), 20, BLACK); // 해당 영역 지우기
            sprite.setTextSize(4);
            sprite.setTextColor(YELLOW, BLACK);
            sprite.setTextDatum(MC_DATUM); // 중앙 정렬
            int centerX = sprite.width() / 2;
            sprite.drawString(timeStr, centerX, posY); // 시간 출력
        }
        sprite.pushSprite(0, 0);
        delay(50);
        return;
    } //mode 2 end


    //────────────────────────────────────────────
    // [모드 3] QR모드
    if(mode == 3){
        bool qrDisplayed = false;   // QR 코드를 한 번만 출력하기 위한 플래그
        if (!qrDisplayed) {
            int qrSize = 100;
            int centerX = (sprite.width() - qrSize) / 2;
            int centerY = (sprite.height() - qrSize) / 2;
            // QR 코드 출력
            sprite.qrcode(qrCodes[currentQRIndex], centerX, centerY-7, qrSize);
            // QR 코드 아래에 레이블 출력
            sprite.setTextSize(1.9);
            sprite.setTextColor(WHITE, BLACK);
            int labelX = (sprite.width() - sprite.textWidth(qrLabels[currentQRIndex])) / 2;
            int labelY = centerY + qrSize + 1;
            sprite.fillRect(0, labelY - 5, sprite.width(), 20, BLACK); // 레이블 영역 지우기
            sprite.drawString(qrLabels[currentQRIndex], labelX, labelY);

            qrDisplayed = true;
        }

        // A 버튼을 짧게 누르면 QR 코드 및 레이블 변경
        if (M5.BtnA.wasReleased()) {
            currentQRIndex = (currentQRIndex + 1) % numQRs;  // 다음 QR 코드 인덱스로 변경 (순환)
            qrDisplayed = false;    // 다음 루프에서 QR 코드 다시 출력
        }
        // B 버튼을 눌렀을 때 모드 변경
        if (M5.BtnB.wasReleased()) {
            mode++;
            qrDisplayed = false;    // 다음 모드로 변경되면 QR 코드 출력을 다시 허용
        }
        sprite.pushSprite(0, 0);
    } //mode3 end

} //loop end