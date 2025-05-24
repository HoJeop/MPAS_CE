#include <M5StickCPlus2.h>
#include <arduinoFFT.h>
#include <math.h>
#include <cmath>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "image_display_utils.h"
#include "version_info.h"
#include "buzzer.h"
#define BUZZ_PIN 2

// 추가 색상 정의 (16비트 컬러) =====================================================
#define WHITE 0xFFFF
#define BLUE 0x001F
#define ORANGE 0xFD20
#define YELLOW 0xFFE0
#define RED 0xF800
#define PURPLE 0x780F
#define DARKGRAY 0x7BEF
#define DARKGREEN 0x03E0
#define DARKRED 0x7800
#define LIGHTGRAY 0xBDF7
#define DARKMAGENTA 0x8B008B
#define DARKORANGE 0xFF8C00
#define BLACK 0x0000
#define MAGENTA 0xF81F
#define CYAN 0x07FF
#define GREEN 0x07E0

// M5StickC Plus2의 화면을 더블 버퍼링 방식으로 사용하기 위한 스프라이트 객체
M5Canvas sprite(&M5.Display);

// 화면 회전 축을 나타내는 열거형 정의
enum RotationAxis {
  AXIS_X,
  AXIS_Y,
  AXIS_Z
};

// [전역] 버튼 처리 관련 변수 ==================================================
unsigned long btnAPressStart = 0;
bool longPressTriggered = false;
unsigned long btnBPressStart = 0;
bool bLongPressTriggered = false;

//=============================================================================
// 모드 번호 정의
int mode = 0;
int prevMode = -1;       // 모드 전환 시 이전 모드 (텍스트 정렬 복구용)
int totalmode = 8;       // 총 모드 개수

// FFT 설정===================================================================
#define SAMPLES 1024
#define SAMPLING_FREQUENCY 2453
// FFT 및 마이크 처리 관련 변수
float vReal[SAMPLES];
float vImag[SAMPLES];
int16_t micBuffer[SAMPLES];
ArduinoFFT<float> FFT = ArduinoFFT<float>();

// [모드 0] 센서 모드 관련 변수 =================================================
float wheelSizes[] = { 20, 22.5, 23.0, 23.5, 23.6, 23.8, 24.0, 24.2, 26.0, 31.0 };  // 단위 mm
int numWheelSizes = sizeof(wheelSizes) / sizeof(wheelSizes[0]);
int currentWheelIndex = 8;
float gearRatios[] = { 5.0, 4.2, 4.0, 3.7, 3.5 };
int numGearRatios = sizeof(gearRatios) / sizeof(gearRatios[0]);
int currentGearIndex = 4;
uint16_t gearColors[] = { BLUE, RED, WHITE, GREEN, CYAN };

// 코어 간 통신을 위한 전역 변수 및 뮤텍스
static float g_currentFrequency = 0.0;
SemaphoreHandle_t xFrequencyMutex;

#define AVG_WINDOW_SIZE 10  // 이동평균 필터
float frequencyBuffer[AVG_WINDOW_SIZE];
int bufferIndex = 0;

// 1차 저역 통과 필터 변수
float lowPassFilteredValue = 0.0;
float previousInputValueLPF = 0.0;
float previousOutputValueLPF = 0.0;

float findMajorPeakInFrequencyRange(float* magnitude, int samples, float sampleRate, float minFreq,
                                    float maxFreq) {
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
    return 0.0;
  }
}

// 1차 저역 통과 필터 함수
float applyLowPassFilter(float inputValue, float cutoffFrequency, float sampleRate) {
  float RC = 1.0 / (2.0 * M_PI * cutoffFrequency);
  float dt = 1.0 / sampleRate;
  float alpha = dt / (RC + dt);
  float outputValue = alpha * inputValue + (1 - alpha) * previousOutputValueLPF;

  previousInputValueLPF = inputValue;
  previousOutputValueLPF = outputValue;
  return outputValue;
}

float calculateFrequency() {
  float currentFrequencyRaw = 0.0;  // 초기화
  float currentFrequency = 0.0;     // 초기화

  if (M5.Mic.record(micBuffer, SAMPLES, SAMPLING_FREQUENCY)) {
    for (int i = 0; i < SAMPLES; i++) {
      // 저역 통과 필터 적용
      // 저역 통과 필터의 차단 주파수 (300 Hz)
      vReal[i] = applyLowPassFilter((float)micBuffer[i], 300, SAMPLING_FREQUENCY);
      vImag[i] = 0.0;
    }
    FFT.windowing(vReal, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD, vReal, true);
    FFT.compute(vReal, vImag, SAMPLES, FFT_FORWARD);
    FFT.complexToMagnitude(vReal, vImag, SAMPLES);

    // 주요 피크 찾기 (이 결과를 raw 값으로 사용)
    currentFrequencyRaw =
      findMajorPeakInFrequencyRange(vReal, SAMPLES, SAMPLING_FREQUENCY, 0.0,
                                    (float)SAMPLING_FREQUENCY / 2.0);  // 전체 범위에서 탐색

    // --- 이동 평균 필터 코드 ---
    frequencyBuffer[bufferIndex] = currentFrequencyRaw;
    bufferIndex = (bufferIndex + 1) % AVG_WINDOW_SIZE;

    float currentFrequencyFiltered = 0;
    for (int i = 0; i < AVG_WINDOW_SIZE; i++) {
      currentFrequencyFiltered += frequencyBuffer[i];
    }
    currentFrequencyFiltered /= AVG_WINDOW_SIZE;
    // --- 이동 평균 필터 코드 끝 ---

    currentFrequency = currentFrequencyFiltered;  // 필터링된 값을 최종 주파수로 사용

    //마이크로 부터 모터의 진동음을 받아 주파수를 추정하는 과정에서
    if (currentFrequency < 30)  // 필터링까지 거친 값이 30 미만으로 잡히면
    {
      currentFrequency = 0;  // 0 으로 처리. (= 노이즈로 간주하고 측정값으로 인정하지 않음)
    }
  } else  // 위와 다르게 "M5.Mic.record(...) == false" 즉, 마이크 데이터 자체를 못 받은 경우
  {
    currentFrequency = 0;  // 0 으로 처리. (측정되지 않았으므로)
  }
  return currentFrequency;
}

// Core 0에서 실행될 FreeRTOS 태스크
void frequencyCalculationTask(void* pvParameters) {
  for (;;) {
    float calculatedFreq = calculateFrequency();  // 주파수 계산 함수 호출

    // 뮤텍스를 사용하여 전역 변수 안전하게 업데이트
    if (xSemaphoreTake(xFrequencyMutex, portMAX_DELAY) == pdTRUE) {
      g_currentFrequency = calculatedFreq;
      xSemaphoreGive(xFrequencyMutex);
    }
    vTaskDelay(pdMS_TO_TICKS(50));  // 다른 태스크에 CPU 시간을 양보 (50ms 지연)
  }
}

// 배터리 잔량 계산 함수
float getBatteryPercent() {
  const uint32_t BAT_LOW = 3150;                    // 배터리 부족 전압 (mV)
  const uint32_t BAT_HIGH = 4100;                   // 배터리 완충 전압 (mV)
  uint32_t voltage = M5.Power.getBatteryVoltage();  // 현재 배터리 전압 읽기
  if (voltage < BAT_LOW)
    voltage = BAT_LOW;  // 최저 전압 이하로 내려가지 않도록 보정
  if (voltage > BAT_HIGH)
    voltage = BAT_HIGH;                                                 // 최고 전압 이상으로 올라가지 않도록 보정
  return ((float)(voltage - BAT_LOW) / (BAT_HIGH - BAT_LOW)) * 100.0f;  // 백분율로 변환하여 반환
}

// [모드 1] 특정 모터 레벨측정 모드 관련 변수 ==================================================
struct Motor {
  const char* name;
  int maxThreshold;
  uint16_t color;
};
Motor motors[] = {
  { "FA-130", 400, WHITE }, { "Rev.T", 430, BLUE }, { "Torque.T", 420, ORANGE }, { "Atomic.T", 420, DARKGRAY }, { "Light.D", 470, YELLOW }, { "Hyper.D", 550, RED }, { "Power.D", 630, DARKGREEN }, { "Mach.D", 630, DARKRED }, { "Sprint.D", 650, LIGHTGRAY }, { "Ultra.D", 700, PURPLE }
};

int numMotors = sizeof(motors) / sizeof(motors[0]);
int currentMotorIndex = 0;

// [모드 2] 스톱워치 모드 관련 변수 =============================================
bool stopwatchRunning = false;
unsigned long stopwatchStartTime = 0, stopwatchElapsed = 0, lastLapTime = 0;
int lapCount = 0, currentLapTargetIndex = 0;
const int MAX_LAPS = 10;
unsigned long lapTimes[MAX_LAPS];
int allowedLapTargets[] = { 2, 3, 5 };
int splitTarget = allowedLapTargets[currentLapTargetIndex];

// 깜박임 효과를 위한 변수
static unsigned long lastBlinkTime = 0;
static bool blinkState = false;
const unsigned long blinkInterval = 300;  // 깜박임 간격 (ms)

// 전역 변수 필요
bool countdownReady = false;
unsigned long countdownStartTime = 0;
bool countdownInProgress = false;
bool isCountingDown = false;
int previousCountdownValue = -1;

// [모드 3] 경과시간 시계 관련 변수 =============================================
unsigned long clockStartTime = 0;
bool clockPaused = false;
unsigned long pausedTime = 0;
unsigned long bootTime = 0;
unsigned long lastDisplaySecond = 0;
unsigned long lastUpdate = 0;

// [모드 4] 각도기 모드 + 화면회전 변수 ===============================================
// 가속도 센서 데이터 변수
float accX = 0, accY = 0, accZ = 0;

// 화면회전 관련 변수
float threshold = 45;  // 회전 감지 임계값
bool isUpsideDown = false;

// 각도 관련 변수 - 버튼 A 글자가 똑바로 보이게 세웠을 때를 기준
double psi = 0, last_psi = 0;      // Yaw   (회전 - 오른쪽 왼쪽 돌기)
double phi = 0, last_phi = 0;      // Pitch (앞뒤 기울기 - 눕혔다 세웠다)
double theta = 0, last_theta = 0;  // Roll  (좌우 기울기 - 오른쪽 왼쪽 기울기)

// 보정값 저장 변수
double offset_psi = 0, offset_phi = 0;

// 저역통과 필터 계수 (데이터 평활화)
double alpha = 0.1;

// 버튼 상태 변수
bool usePhi = false;             // 현재 표시 모드 (Pitch / Yaw)
bool ignoreNextRelease = false;  // 버튼 릴리즈 무시 여부

// [모드 5] QR코드 순환출력 =====================================================
const int numQRs = 3;    // QR 코드 총 개수
int currentQRIndex = 0;  // 현재 표시된 QR 코드 인덱스

const char* qrCodes[numQRs] = {
  "https://www.helloabt.com/mini4wd/",
  "https://map.naver.com/p/favorite/myPlace/folder/029fd0e8addc4d899ea722f0f5a7a613",
  "https://map.kakao.com/?target=other&folderid=16126480"
};

const char* qrLabels[numQRs] = { "Racer's Manner", "Naver Map", "Kakao Map" };


// setup() 함수 ===============================================================
void setup() {
  M5.begin();                     // 버튼 함수 등 인식
  sprite.setRotation(1);          // 화면 회전 (A 버튼이 오른쪽) "우왕!!!"
  sprite.fillScreen(BLACK);       // 화면 초기화 (검은색 배경)
  sprite.createSprite(135, 240);  // 스프라이트 생성 (디스플레이 크기: 240x135)
  sprite.setSwapBytes(true);      // GIMP 데이터와 일치하도록 바이트 스왑 설정
  sprite.setTextSize(5);          // 텍스트 크기 설정
  sprite.setTextColor(WHITE);     // 텍스트 색상 설정
  sprite.setTextDatum(TL_DATUM);
  M5.Power.begin();  // 배터리 관련 초기화
  M5.Mic.begin();    // 마이크 초기화
  M5.Imu.begin();    // 가속도 센서 초기화
  initBuzzer();      // 부저 초기화
  Serial.begin(115200);
  // 뮤텍스 생성
  xFrequencyMutex = xSemaphoreCreateMutex();
  if (xFrequencyMutex == NULL) {
    // 뮤텍스 생성 실패 처리 (예: 시리얼 모니터에 에러 메시지 출력)
    Serial.println("Error: Mutex creation failed!");
  }

  // 주파수 계산 태스크를 Core 0에 할당하여 생성
  // 스택 크기는 필요에 따라 조정할 수 있습니다. (예: 8192 바이트)
  xTaskCreatePinnedToCore(
    frequencyCalculationTask,  // 태스크 함수
    "FreqCalcTask",            // 태스크 이름
    8192,                      // 스택 크기 (바이트)
    NULL,                      // 태스크 매개변수
    2,                         // 태스크 우선순위 (높을수록 우선순위 높음)
    NULL,                      // 태스크 핸들 (필요 없으면 NULL)
    0                          // 실행할 코어 번호 (Core 0)
  );

  bootTime = millis();
  clockStartTime = bootTime;
}

// 그라데이션 색상 계산 함수
uint16_t getGradientColor(uint16_t startColor, uint16_t endColor, float ratio) {
  uint8_t r1 = (startColor >> 11) & 0x1F;
  uint8_t g1 = (startColor >> 5) & 0x3F;
  uint8_t b1 = startColor & 0x1F;

  uint8_t r2 = (endColor >> 11) & 0x1F;
  uint8_t g2 = (endColor >> 5) & 0x3F;
  uint8_t b2 = endColor & 0x1F;

  uint8_t r = r1 + (r2 - r1) * ratio;
  uint8_t g = g1 + (g2 - g1) * ratio;
  uint8_t b = b1 + (b2 - b1) * ratio;

  return (r << 11) | (g << 5) | b;
}

// IMU 센서 보정값 초기화 함수
void resetSensor() {
  offset_psi = psi;  // 현재 Yaw(회전) 값을 기준점으로 설정
  offset_phi = phi;  // 현재 Pitch(기울기) 값을 기준점으로 설정
}

// 화면회전 함수
void handleScreenRotation(RotationAxis axis) {
  float accX, accY, accZ, psi;
  M5.Imu.getAccelData(&accX, &accY, &accZ);

  if (axis == AXIS_Y) {
    if (accY != 0) {
      psi = atan2(accX, accY) * 57.295;
    }
  } else if (axis == AXIS_X) {
    if (accX != 0) {
      psi = atan2(accY, accX) * 57.295;
    }
  } else if (axis == AXIS_Z) { // Z축 로직 추가
    if (accZ != 0) {
      // accX와 accZ를 사용하여 Z축 기준의 기울기를 계산
      // M5Stack을 앞뒤로 기울였을 때의 회전을 감지합니다.
      psi = atan2(accX, accZ) * 57.295;
    }
  }

  if (psi > threshold && !isUpsideDown) {
    sprite.setRotation(1);
    sprite.fillScreen(BLACK);
    isUpsideDown = true;
  } else if (psi < -threshold && isUpsideDown) {
    sprite.setRotation(3);
    sprite.fillScreen(BLACK);
    isUpsideDown = false;
  }
}

// loop() 함수 ================================================================
void loop() {
  M5.update();
  M5.Imu.getAccelData(&accX, &accY, &accZ);
  psi = atan2(accX, accY) * 57.295;

  // 글로벌 버튼 B 처리 (수정된 부분)
  if (M5.BtnB.isPressed()) {
    if (btnBPressStart == 0)
      btnBPressStart = millis();
  } else if (M5.BtnB.wasReleased()) {
    unsigned long duration = millis() - btnBPressStart;

    if (duration >= 2000) {  // 2초 이상 길게 눌렀다 떼면 (스톱워치 랩 타겟 변경 또는 잠금화면 해제)
      if (mode == 2)         // 스톱워치 모드에서
      {
        currentLapTargetIndex = (currentLapTargetIndex + 1) % 3;
        splitTarget = allowedLapTargets[currentLapTargetIndex];
        realMouseClickSound1();
      }
    } else if (duration >= 650 && duration < 950)  // 0.8초 이상 1.5초 미만 눌렀다 떼면 (이전 모드로 이동)
    {
      if (mode == 2 && psi < -threshold) {
        alertSound();
      } else if (mode == 2 && psi > threshold) {
        mode = 0;
        realMouseClickSound2();
        return;
      } else {
        mode = (mode - 1 + totalmode) % totalmode;  // 이전 모드로 순환
      }
      realMouseClickSound2();
    } else if (duration < 600)  // 짧게 누르면 (다음 모드로 이동)
    {
      if (mode == 0 || mode == 1)  // 모드 0 혹은 모드 1일 경우
      {
        mode = 2;  // 모드 2로 전환
      } else {
        mode = (mode + 1) % totalmode;  // 다음 모드로 순환
      }
      realMouseClickSound1();
    }

    btnBPressStart = 0;           // 버튼 입력 플래그 초기화
    bLongPressTriggered = false;  // 길게 누름 플래그 초기화

    sprite.fillScreen(BLACK);  // 모드 전환 시 화면 초기화
    return;
  }

  // Yaw 값 기반 모드 전환 (센서/그래프 전환)
  if (mode == 0 || mode == 1) {
    if (mode == 0 && psi < -threshold) {
      mode = 1;
      sprite.setRotation(3);
      sprite.fillScreen(BLACK);
    } else if (mode == 1 && psi > threshold) {
      mode = 0;
      sprite.setRotation(1);
      sprite.fillScreen(BLACK);
    }
  }

  // 배터리 관련
  float batteryPct = getBatteryPercent();  // 배터리 전압 퍼센트로 받아옴
  uint16_t batColor = GREEN;               // 배터리 글자색
  bool ledState = 0;                       // led 점등상태

  if (batteryPct <= 10.0) {                       // 전압이 10% 이하일 경우
    ledState = (millis() % 2000 < 1000) ? 1 : 0;  // LED 점등상태는 1과 0을 반복
    batColor = ledState ? RED : BLACK;            // LED 점등 여부에 따라 배터리 색깔 변경
  }

  M5.Power.setLed(ledState);  // 위의 조건에 따라 LED를 켜고 끈다.

  // 배터리 잔량 텍스트 포멧화
  int roundedBattery = round(batteryPct);
  char formattedBattery[10];
  sprintf(formattedBattery, "%3d", roundedBattery);

  // 모드 전환 시 전체 화면 BLACK 및 기본 텍스트 정렬 복구
  if (prevMode != mode) {
    sprite.fillScreen(BLACK);
    sprite.setTextDatum(TL_DATUM);
    prevMode = mode;
  }

  // ─────────────────
  // [모드 0] 센서 모드
  if (mode == 0) {
    // A 버튼 처리
    //  - 누른 시간 < 1000ms (릴리즈): 기어비 변경 처리
    //  - 누른 시간 1000ms 이상 2000ms 미만 (릴리즈): 휠 크기 변경 처리
    unsigned long aDuration = 0;  // 버튼 눌린시간 기록하는 변수
    if (M5.BtnA.isPressed())      // 버튼 A가 눌린 상태라면
    {
      if (btnAPressStart == 0)                // 버튼 A가 처음 눌린 순간이라면
        btnAPressStart = millis();            // 눌린 시작 시간 기록
      aDuration = millis() - btnAPressStart;  // 눌린 시간 계산
    }
    if (M5.BtnA.wasReleased()) {              // 버튼 A가 떼어진 순간이라면
      aDuration = millis() - btnAPressStart;  // 버튼 A가 눌린 총 시간 계산
      if (aDuration < 500)                    // 짧게 눌렀을 경우 기어비 변경 처리
      {
        currentGearIndex =
          (currentGearIndex + 1) % numGearRatios;       // 다음 기어비 인덱스로 변경 (순환)
      } else if (aDuration >= 800 && aDuration < 1500)  // 1초 이상 2초 미만 눌렀다 떼면
      {
        currentWheelIndex =
          (currentWheelIndex + 1) % numWheelSizes;  // 다음 휠 크기 인덱스로 변경 (순환)
      }
      // 버튼에서 완전히 손을 때면
      btnAPressStart = 0;          // 버튼 눌린시간 초기화
      longPressTriggered = false;  // 길게 누름 플래그 초기화
    }

    // Core 0에서 계산된 주파수 값 가져오기
    float currentFrequency = 0.0;
    if (xSemaphoreTake(xFrequencyMutex, portMAX_DELAY) == pdTRUE) {
      currentFrequency = g_currentFrequency;
      xSemaphoreGive(xFrequencyMutex);
    }

    int rpmCurrent =
      (int)round(currentFrequency * 60);  // 추출된 주파수를 RPM (분당 회전수)으로 변환

    float currentWheelDiameter =
      wheelSizes[currentWheelIndex] / 1000.0;  // 선택된 휠 크기(mm)를 미터 단위(m)로 변환 하고

    float selectedGear = gearRatios[currentGearIndex];  // 선택된 기어비를 불러와

    float estimatedSpeed =
      (rpmCurrent / selectedGear) * (PI * currentWheelDiameter * 0.06);  // 추정 속도를 계산한다. (RPM / 기어비 * 휠 둘레 * 단위 변환 계수)

    //좌측상단 경과시간
    static unsigned long clockStartTime = 0;
    static unsigned long pausedTime = 0;
    static bool clockPaused = false;
    static int lastDisplaySecond = -1;
    static char lastTimeStr[9] = "";  // 이전 출력 문자열 캐싱

    unsigned long elapsed = clockPaused ? pausedTime : (millis() - clockStartTime);
    unsigned int hours = elapsed / 3600000UL;
    unsigned int minutes = (elapsed % 3600000UL) / 60000;
    unsigned int seconds = (elapsed % 60000UL) / 1000;

    char timeStr[9];
    sprintf(timeStr, "%02u:%02u:%02u", hours, minutes, seconds);

    // 초가 바뀔 때만 텍스트 업데이트
    if (seconds != lastDisplaySecond) {
      lastDisplaySecond = seconds;
      //시간 표시
      if (strcmp(timeStr, lastTimeStr) != 0) {
        strcpy(lastTimeStr, timeStr);
        sprite.setTextSize(2);
        sprite.setTextColor(YELLOW, BLACK);  // 배경 지정으로 자동 덮어쓰기
        sprite.setTextDatum(TL_DATUM);
        sprite.drawString(timeStr, 12, 9);
      }
    }

    // 배터리 잔량 표시 (우측 상단)
    sprite.setTextSize(2);
    sprite.setTextColor(batColor, BLACK);
    sprite.setCursor(sprite.width() - 56, 9);
    sprite.print(formattedBattery);  // 루프 내부 상단에 미리 정의한 포맷화된 수치
    //sprite.print(M5.Power.getBatteryVoltage());
    sprite.setTextColor(WHITE, BLACK);
    sprite.setCursor(sprite.width() - 19, 9);
    sprite.print("%");

    // 현재 주파수
    sprite.fillRect(12, 33, 100, 33, BLACK);  // 해당 영역을 검은색으로 채워 지움
    //sprite.fillRect(12, 33, 100, 33, WHITE);
    sprite.setTextSize(3.9);
    sprite.setCursor(12, 33);
    int currFreqInt = (int)round(currentFrequency);  // 현재 주파수 반올림
    sprite.setTextColor(CYAN);
    //sprite.print("256");
    sprite.print(currFreqInt);
    sprite.setTextColor(WHITE);
    sprite.setTextSize(3.3);
    sprite.setCursor(82, 37);
    sprite.print(" Hz");

    // 휠 사이즈 출력
    sprite.fillRect(156, 35, 90, 18, BLACK);  // 휠 숫자 영역만 검은색으로 채워 초기화
    sprite.setTextSize(2.1);
    sprite.setCursor(156, 35);
    sprite.setTextColor(DARKGREEN);
    sprite.print(wheelSizes[currentWheelIndex], 1);  // 휠 크기 출력 (소수점 1자리)
    sprite.setTextSize(1.8);
    sprite.setCursor(209, 37);
    sprite.print("mm");

    sprite.setTextColor(LIGHTGRAY);
    sprite.setTextSize(2.1);
    sprite.setCursor(157, 57);
    sprite.print("Ratios");
    sprite.fillRect(160, 80, 75, 33, BLACK);
    sprite.setTextSize(3.9);
    sprite.setCursor(160, 80);
    sprite.setTextColor(gearColors[currentGearIndex]);  // 현재 기어비에 해당하는 색상으로 설정
    sprite.print(gearRatios[currentGearIndex], 1);

    // RPM
    sprite.fillRect(13, 69, 100, 25, BLACK);
    //sprite.fillRect(13, 69, 100, 25, RED);
    sprite.setTextSize(2.2);
    sprite.setCursor(13, 69);
    sprite.setTextColor(MAGENTA);
    //sprite.print("600000");
    sprite.print(rpmCurrent);
    sprite.setTextColor(WHITE);
    sprite.setTextSize(2.2);
    sprite.setCursor(75, 67);
    sprite.print("  rpm");

    // 추정속도
    sprite.fillRect(13, 92, 100, 25, BLACK);
    //sprite.fillRect(13, 92, 100, 25, YELLOW);
    sprite.setTextSize(2.2);
    sprite.setCursor(13, 92);
    sprite.setTextColor(ORANGE);
    //sprite.print("100.0");
    sprite.print(estimatedSpeed, 1);  // 추정 속도 출력 (소수점 1자리)
    sprite.setCursor(62, 92);
    sprite.setTextColor(WHITE);
    sprite.print("  km/h");

    sprite.setCursor(5, 120);
    displayBuildInfo();

    sprite.pushSprite(0, 0);  // 스프라이트 내용을 실제 화면에 출력

  }  // mode 0 end

  // ───────────────────────────────────────────────────────────────
  //  [모드 1] 특정 모터 주파수 레벨 측정
  if (mode == 1) {
    static bool firstTimeMode1 = true;  // 모드 1에 처음 진입했는지 확인하는 플래그

    if (firstTimeMode1) {
      currentMotorIndex = 5;   // Hyper 모터의 인덱스 (0부터 시작)
      firstTimeMode1 = false;  // 다시 초기화되지 않도록 플래그를 false로 설정
    }

    if (M5.BtnA.wasReleased()) {
      currentMotorIndex = (currentMotorIndex + 1) % numMotors;
    }

    float currentFrequency = 0.0;  // Core 0에서 계산된 주파수 값 가져오기
    if (xSemaphoreTake(xFrequencyMutex, portMAX_DELAY) == pdTRUE) {
      currentFrequency = g_currentFrequency;
      xSemaphoreGive(xFrequencyMutex);
    }

    float ratio = currentFrequency / (float)motors[currentMotorIndex].maxThreshold;
    if (ratio > 1.0) {
      ratio = 1.0;
    } else if (ratio < 0.0) {
      ratio = 0.0;
    }

    static float smoothRatio = 0.0;
    float alpha = 0.15;  // 지수 이동 평균 계수 (조절 가능)
    smoothRatio = alpha * ratio + (1 - alpha) * smoothRatio;
    int brickWidth = 12;
    int barHeight = 78;                                       // 프로그래스 바 전체 높이
    int barWidth = sprite.width() - 3;                        // 프로그래스 바 전체 너비 (양쪽 여백)
    int startX = (sprite.width() - 10) - brickWidth;          // 오른쪽 끝에서 시작
    int startY = (sprite.height() - 30) / 2 - barHeight / 2;  // 중앙 y 좌표
                                                              // 일반 벽돌 너비
    int thinBrickWidth = brickWidth / 3;                      // 얇은 벽돌 너비
    int brickSpacing = 3;                                     // 벽돌 사이 간격
    // 그라데이션 색상 정의 (시작 색상 -> 끝 색상)
    uint16_t startColor = DARKGRAY;
    uint16_t endColor = motors[currentMotorIndex].color;
    uint16_t targetColor = RED;  // 목표점 색상

    // 그래프 초기화
    sprite.fillRect(0, 0, sprite.width(), sprite.height() - 30, BLACK);

    int numBricks = barWidth / (brickWidth + brickSpacing);  // 가로 방향 벽돌 개수
    int filledBricks = (int)(smoothRatio * numBricks);

    for (int i = numBricks - 1; i >= 0; i--)  // 오른쪽에서 왼쪽으로 루프
    {
      int brickX = startX - i * (brickWidth + brickSpacing);
      int brickY = startY;
      int currentBrickWidth = brickWidth;  // 기본 벽돌 너비

      //if (i >= numBricks - filledBricks) // 좌왕!!!
      if (i < filledBricks) {  //우왕!!
        // 채워진 벽돌에 그라데이션 적용
        //float gradientRatio = (float)(numBricks - 1 - i) / (filledBricks > 0 ? filledBricks : 1); // 좌왕!!!
        float gradientRatio = (float)i / (filledBricks > 0 ? filledBricks : 1);  // 우왕!!
        uint16_t currentColor = getGradientColor(startColor, endColor, gradientRatio);
        sprite.fillRect(brickX, brickY, currentBrickWidth, barHeight, currentColor);
      } else {
        // 채워지지 않은 벽돌은 검정색으로 가림
        sprite.fillRect(brickX, brickY, currentBrickWidth, barHeight, BLACK);
      }

      //if (i == 0) // 좌왕!!! (타겟블럭-빨간색)
      if (i == numBricks - 1) {  //우왕!!
        sprite.fillRect(brickX, brickY, thinBrickWidth, barHeight, targetColor);
      }
    }

    // 텍스트 영역 배경색 채우기
    sprite.fillRect(0, sprite.height() - 30, sprite.width(), 30, BLACK);
    sprite.setTextSize(2);
    sprite.setTextColor(motors[currentMotorIndex].color);
    char motorStr[30];
    sprintf(motorStr, "%s : %d / %d", motors[currentMotorIndex].name,
            motors[currentMotorIndex].maxThreshold, (int)currentFrequency);
    int x = (sprite.width() - sprite.textWidth(motorStr)) / 2;
    int y = sprite.height() - 30 + (30 - 16) / 2;
    sprite.drawString(motorStr, x, y);

    sprite.pushSprite(0, 0);
  }  // mode 1 end

  // ─────────────────────
  //  [모드 2] 스톱워치 모드
  if (mode == 2) {
    // A 버튼 처리
    if (M5.BtnA.isPressed()) {
      if (btnAPressStart == 0)
        btnAPressStart = millis();

      if (!stopwatchRunning && (millis() - btnAPressStart >= 1000) && !longPressTriggered) {
        longPressTriggered = true;
        countdownReady = true;  // 5초 카운트다운 준비 상태
      }
    }

    if (M5.BtnA.wasReleased()) {
      if (countdownReady) {
        // 5초 카운트다운 시작
        countdownStartTime = millis();
        countdownReady = false;
        isCountingDown = true;
      } else if (!longPressTriggered) {
        // 짧게 눌렀을 경우: 스톱워치 토글
        if (!stopwatchRunning) {
          stopwatchRunning = true;
          stopwatchStartTime = millis();
          stopwatchElapsed = 0;
          lapCount = 0;
          lastLapTime = 0;
        } else {
          unsigned long currentTimeLocal = stopwatchElapsed + (millis() - stopwatchStartTime);
          if (lapCount < (splitTarget - 1)) {
            unsigned long lapInterval = currentTimeLocal - lastLapTime;
            if (lapCount < MAX_LAPS)
              lapTimes[lapCount] = lapInterval;
            lapCount++;
            lastLapTime = currentTimeLocal;
          } else {
            unsigned long lapInterval = currentTimeLocal - lastLapTime;
            if (lapCount < MAX_LAPS)
              lapTimes[lapCount] = lapInterval;
            lapCount++;
            lastLapTime = currentTimeLocal;
            stopwatchRunning = false;
            stopwatchElapsed = currentTimeLocal;
          }
        }
        stopwatchClickSound();
      }
      btnAPressStart = 0;
      longPressTriggered = false;
    }

    if (isCountingDown) {
      unsigned long countdownElapsed = millis() - countdownStartTime;
      int remaining = 5 - (countdownElapsed / 1000);

      if (remaining >= 1) {
        // 3초 이하 & 숫자 변화 시 비프음 재생
        if (remaining <= 3 && remaining != previousCountdownValue) {
          countdownBeep();  // 기존 비프음 함수
        }
        previousCountdownValue = remaining;

        handleScreenRotation(AXIS_Y);

        // 카운트다운 화면 출력
        sprite.fillScreen(BLACK);
        sprite.setTextColor(RED);
        sprite.setTextSize(2.8);
        sprite.drawCentreString("Ready??", sprite.width() / 2, 10, 2);
        sprite.setTextColor(GREEN);
        sprite.setTextSize(4);
        sprite.drawCentreString(String(remaining), sprite.width() / 2, 54, 2);
        sprite.pushSprite(0, 0);
        delay(10);
        return;
      } else if (remaining == 0) {
        // 변수에 값을 먼저 기록하여 측정을 우선시함.
        // 스톱워치 정확한 시간 기준으로 시작
        stopwatchRunning = true;
        stopwatchStartTime = millis();
        stopwatchElapsed = 0;
        lapCount = 0;
        lastLapTime = 0;

        // 상태 초기화
        isCountingDown = false;
        previousCountdownValue = -1;

        // 화면 출력은 플래그 설정 등이 먼저 이뤄진 다음에 진행되므로 실제 측정에 딜레이는 없음.
        //  "Go!" 메시지 출력 및 스타트 비프음
        sprite.fillScreen(BLACK);
        sprite.setTextColor(GREENYELLOW);
        sprite.setTextSize(6.5);
        sprite.drawCentreString("Go!!", sprite.width() / 2, 5, 2);
        sprite.pushSprite(0, 0);
        tone(BUZZ_PIN, 4000, 200);
        delay(500);  // 사용자에게 보여지는 시간 (측정에는 영향 없음)

        return;
      }
    }

    // 스톱워치 현재 시간 계산
    unsigned long currentTimeDisplay = stopwatchElapsed;
    if (stopwatchRunning)
      currentTimeDisplay += (millis() - stopwatchStartTime);
    if (currentTimeDisplay > 3600000UL)
      currentTimeDisplay = 3600000UL;

    int minute = currentTimeDisplay / 60000;
    int second = (currentTimeDisplay % 60000) / 1000;
    int centisecond = (currentTimeDisplay % 1000) / 10;
    char timeStr[16];
    sprintf(timeStr, "%02d : %02d . %02d", minute, second, centisecond);

    // 랩 타겟에 따른 색상
    uint16_t stopwatchColor = WHITE;
    if (splitTarget == 2)
      stopwatchColor = GREEN;
    else if (splitTarget == 3)
      stopwatchColor = BLUE;
    else if (splitTarget == 5)
      stopwatchColor = ORANGE;

    sprite.fillRect(0, 0, sprite.width(), 40, BLACK);
    M5.Imu.getAccelData(&accX, &accY, &accZ);
    if (accY != 0)
      psi = atan2(accX, accY) * 57.295;
    if (psi > threshold && !isUpsideDown) {
      sprite.setRotation(1);
      isUpsideDown = true;
    } else if (psi < -threshold && isUpsideDown) {
      sprite.setRotation(3);
      isUpsideDown = false;
    }

    if (!stopwatchRunning && lapCount >= splitTarget) {
      if (millis() - lastBlinkTime >= blinkInterval) {
        blinkState = !blinkState;
        lastBlinkTime = millis();
      }
      uint16_t displayColor = blinkState ? stopwatchColor : BLACK;
      sprite.setTextSize(2.8);
      for (int dx = -1; dx <= 1; dx++) {
        for (int dy = -1; dy <= 1; dy++) {
          sprite.setTextColor(displayColor);
          sprite.drawCentreString(timeStr, 120 + dx, 5 + dy, 2);
        }
      }
    } else {
      sprite.setTextSize(2.8);
      for (int dx = -1; dx <= 1; dx++) {
        for (int dy = -1; dy <= 1; dy++) {
          sprite.setTextColor(stopwatchColor);
          sprite.drawCentreString(timeStr, 120 + dx, 5 + dy, 2);
        }
      }
    }

    sprite.fillRect(0, 40, sprite.width(), sprite.height() - 40, BLACK);
    if (lapCount > 0) {
      int startLap = (lapCount > 5) ? lapCount - 5 : 0;
      int yPos = 45;
      char lapLabel[10], lapSplitStr[20], cumStr[20];
      for (int i = startLap; i < lapCount; i++) {
        unsigned long lapTime = lapTimes[i];
        int lapMin = lapTime / 60000;
        int lapSec = (lapTime % 60000) / 1000;
        int lapCs = (lapTime % 1000) / 10;
        unsigned long cumulative = 0;
        for (int j = 0; j <= i; j++) cumulative += lapTimes[j];
        int cumMin = cumulative / 60000;
        int cumSec = (cumulative % 60000) / 1000;
        int cumCs = (cumulative % 1000) / 10;

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
  }  // mode 2 end

  // ────────────────────────────────────────────
  //  [모드 3] 경과시간 시계 모드
  if (mode == 3) {
    static unsigned long btnAPressStart = 0;
    static bool longPressTriggered = false;
    static unsigned long clockStartTime = 0;
    static unsigned long pausedTime = 0;
    static bool clockPaused = false;
    static int lastDisplaySecond = -1;
    static char lastTimeStr[9] = "";  // 이전 출력 문자열 캐싱

    if (M5.BtnA.isPressed()) {
      if (btnAPressStart == 0)
        btnAPressStart = millis();
      if (millis() - btnAPressStart >= 1000 && !longPressTriggered) {
        longPressTriggered = true;
        clockStartTime = millis();
        pausedTime = 0;
        clockPaused = false;
      }
    } else if (M5.BtnA.wasReleased()) {
      if (!longPressTriggered) {
        if (!clockPaused) {
          clockPaused = true;
          pausedTime = millis() - clockStartTime;
        } else {
          clockPaused = false;
          clockStartTime = millis() - pausedTime;
        }
      }
      stopwatchClickSound();
      btnAPressStart = 0;
      longPressTriggered = false;
    }

    unsigned long elapsed = clockPaused ? pausedTime : (millis() - clockStartTime);
    unsigned int hours = elapsed / 3600000UL;
    unsigned int minutes = (elapsed % 3600000UL) / 60000;
    unsigned int seconds = (elapsed % 60000UL) / 1000;

    // 화면 회전 대응
    handleScreenRotation(AXIS_Y);

    char timeStr[9];
    sprintf(timeStr, "%02u:%02u:%02u", hours, minutes, seconds);

    // 초가 바뀔 때만 텍스트 업데이트
    if (seconds != lastDisplaySecond) {
      lastDisplaySecond = seconds;

      // 중앙 시간 텍스트
      if (strcmp(timeStr, lastTimeStr) != 0) {
        strcpy(lastTimeStr, timeStr);
        int posY = (sprite.height() - 10) / 2;
        sprite.setTextSize(4);
        sprite.setTextColor(YELLOW, BLACK);  // 배경 지정으로 자동 덮어쓰기
        sprite.setTextDatum(MC_DATUM);
        int centerX = sprite.width() / 2;
        sprite.drawString(timeStr, centerX, posY);
      }
      sprite.pushSprite(0, 0);
    }

    delay(50);
    return;
  }  // mode 3 end

  // ────────────────────────────────────────────
  //  [모드 4] 수평계 모드 (보완 필터 + LPF + 칼만 필터 적용)
  if (mode == 4) {
    M5.Imu.getAccelData(&accX, &accY, &accZ);  // 가속도 센서 값 가져오기

    // 센서 데이터를 사용하여 기기의 각도 계산
    if (accY != 0)
      psi = atan2(accX, accY) * 57.295;  // Yaw 각도 계산
    if (accZ != 0)
      phi = atan2(accY, accZ) * 57.295;  // Pitch 각도 계산
    if ((accX < 1) && (accX > -1))
      theta = asin(-accX) * 57.295;  // Roll 계산

    // 기울기 값 보정
    phi = abs(phi);
    psi = psi >= 0 ? 90 - psi : 90 + abs(psi);
    theta = constrain(theta, -90, 90);  // 값을 -90 ~ 90도 범위로 제한

    // 저역통과 필터 적용 (노이즈 감소)
    psi = alpha * psi + (1 - alpha) * last_psi;
    phi = alpha * phi + (1 - alpha) * last_phi;
    theta = alpha * theta + (1 - alpha) * last_theta;

    // A 버튼을 1.5초 이상 누르면 센서 값 초기화
    if (M5.BtnA.pressedFor(1500)) {
      // 초기화 사운드
      resetSensor();             // 센서 보정 초기화
      ignoreNextRelease = true;  // 버튼 릴리즈를 다음 루프에서 무시
    }

    // 버튼이 짧게 눌렸다가 떼어졌을 경우 표시 모드 변경
    if (M5.BtnA.wasReleased()) {
      if (!ignoreNextRelease) {
        usePhi = !usePhi;        // Pitch(Y 기울기) ↔ Yaw(회전) 모드 변경
        realMouseClickSound1();  // 모드 변경 사운드
      }
      ignoreNextRelease = false;
    }

    // 보정된 각도 계산
    double adj_psi = psi - offset_psi;  // 보정된 Yaw 값
    double adj_phi = phi - offset_phi;  // 보정된 Pitch 값

    // 스프라이트를 사용하여 화면을 갱신 (더블 버퍼링 방식)
    sprite.fillScreen(BLACK);  // 화면 초기화
    sprite.setTextColor(WHITE);
    sprite.setTextSize(5);
    sprite.setCursor(5, 49);
    sprite.printf("%6.1f", usePhi ? adj_phi : adj_psi);  // 현재 표시 중인 각도 출력

    // 현재 모드(usePhi)에 따라 그래픽 요소 표시
    if (usePhi) {
      // Pitch 표시 모드: 세로 바 그래프
      int barPos = constrain(map(adj_phi, 0, 90, 235, 5), 5, 235);
      sprite.drawLine(barPos, 0, barPos, 150, BLUE);  // 파란색 수직선 표시

      int thetaBarPos = constrain(map(theta, -90, 90, 135, 5), 5, 135);
      sprite.drawLine(0, thetaBarPos, 240, thetaBarPos, YELLOW);  // 노란색 수평선 표시
    } else {
      // Yaw 표시 모드: 중앙 회전선 그래픽
      int center_x = 120, center_y = 67, length = 240;
      int x_start = center_x + (length / 2) * cos(-adj_psi * PI / 180);
      int y_start = center_y + (length / 2) * sin(-adj_psi * PI / 180);
      int x_end = center_x - (length / 2) * cos(-adj_psi * PI / 180);
      int y_end = center_y - (length / 2) * sin(-adj_psi * PI / 180);
      sprite.drawLine(x_start, y_start, x_end, y_end, RED);  // 빨간색 회전선 표시
    }

    // 스프라이트를 디스플레이에 출력 (화면 깜빡임 방지)
    sprite.pushSprite(0, 0);

    // 이전 상태 저장 (다음 루프에서 필터링을 위해 사용)
    last_psi = psi;
    last_phi = phi;
    last_theta = theta;

    delay(100);  // 루프 주기 설정 (100ms마다 갱신)

  }  // mode 4 end

  // ────────────────────────────────────────────
  //  [모드 5] QR모드
  if (mode == 5) {

    bool qrDisplayed = false;  // QR 코드 한 번만 출력하도록 설정
    if (!qrDisplayed) {
      int qrSize = 100;
      int centerX = (sprite.width() - qrSize) / 2;
      int centerY = (sprite.height() - qrSize) / 2;

      // QR 코드 출력
      sprite.qrcode(qrCodes[currentQRIndex], centerX, centerY - 7, qrSize);

      // QR 코드 아래에 레이블 출력
      sprite.setTextSize(1.9);
      sprite.setTextColor(WHITE, BLACK);
      int labelX = (sprite.width() - sprite.textWidth(qrLabels[currentQRIndex])) / 2;
      int labelY = centerY + qrSize + 1;
      sprite.fillRect(0, labelY - 5, sprite.width(), 20, BLACK);  // 레이블 영역 지우기
      sprite.drawString(qrLabels[currentQRIndex], labelX, labelY);

      qrDisplayed = true;
    }

    // A 버튼을 짧게 누르면 QR 코드 및 레이블 변경
    if (M5.BtnA.wasReleased()) {
      currentQRIndex = (currentQRIndex + 1) % numQRs;  // QR 코드 순환
      qrDisplayed = false;                             // 다음 QR 코드 출력 허용
    }

    // B 버튼을 눌렀을 때 모드 변경
    if (M5.BtnB.wasReleased()) {
      mode++;
      qrDisplayed = false;  // 다음 모드로 변경되면 다시 QR 코드 출력 허용
    }

    sprite.pushSprite(0, 0);
  }  // mode 5 end

  // ────────────────────────────────────────────
  //  [모드 6] 컴파일 시점의 버전정보 생성
  if (mode == 6) {
    // 배터리 잔량 표시 (우측 상단)
    sprite.fillScreen(BLACK);
    float batteryvolt = M5.Power.getBatteryVoltage();
    sprite.setTextSize(2);
    sprite.setTextColor(WHITE);
    sprite.setCursor(sprite.width() - 56, 9);
    sprite.print(batteryvolt, 0);

    sprite.setTextSize(4);
    sprite.setCursor(10, 20);
    sprite.print("Stuck?");
    sprite.setTextSize(2);
    sprite.setCursor(10, 60);
    displayDeviceSerialNumber();
    sprite.setCursor(10, 75);
    displayBuildInfo();
    sprite.pushSprite(0, 0);

  }  // mode 6 end

  // ────────────────────────────────────────────
  //  [모드 7] 이미지 표시 (테스트)
  if (mode == 7) {
    handleScreenRotation(AXIS_Z);
    if (isUpsideDown == true) {
      sprite.fillScreen(BLACK);
      drawSpecificImage(&hugging_mpas);
    } else if (isUpsideDown == false) {
      sprite.fillScreen(BLACK);
      drawSpecificImage(&hugging_neochamp);
    }
    sprite.pushSprite(0, 0);

  }  // mode 7 end

}  // loop end
