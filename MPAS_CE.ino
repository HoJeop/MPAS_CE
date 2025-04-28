#include <M5StickCPlus2.h>
#include <arduinoFFT.h>
#include <math.h>
#include <time.h>

// 색상 정의 (16비트 컬러) =====================================================
#define WHITE      0xFFFF
#define BLUE       0x001F
#define ORANGE     0xFD20
#define YELLOW     0xFFE0
#define RED        0xF800
#define PURPLE     0x780F
#define DARKGRAY   0x7BEF   
#define DARKGREEN  0x03E0    
#define DARKRED    0x7800    
#define LIGHTGRAY  0xBDF7

// [전역] 버튼 처리 관련 변수 ==================================================
unsigned long btnAPressStart = 0;
bool longPressTriggered = false;
unsigned long btnBPressStart = 0;
bool bLongPressTriggered = false;

// FFT 설정===================================================================
#define SAMPLES             512
#define SAMPLING_FREQUENCY  850
// FFT 및 마이크 처리 관련 변수
float vReal[SAMPLES];
float vImag[SAMPLES];
int16_t micBuffer[SAMPLES];
ArduinoFFT<float> FFT = ArduinoFFT<float>();

//=============================================================================
// 모드 번호 정의 (0~5)
// 0: 센서 모드
// 1: 스톱워치 모드
// 2: 경과시간 시계 모드
// 3: QR모드
int mode = 0;
int prevMode = -1;

// [모드 0] 센서 모드 관련 변수 =================================================
float wheelSizes[] = {24.0, 26.0, 31.0}; // 단위 mm
int numWheelSizes = sizeof(wheelSizes) / sizeof(wheelSizes[0]);
int currentWheelIndex = 0;
float gearRatios[] = {3.7, 3.5};
int numGearRatios = sizeof(gearRatios) / sizeof(gearRatios[0]);
int currentGearIndex = 0;
uint16_t gearColors[] = {GREEN, CYAN};

// [모드 1] 스톱워치 모드 관련 변수 =============================================
bool stopwatchRunning = false;
unsigned long stopwatchStartTime = 0, stopwatchElapsed = 0, lastLapTime = 0;
int lapCount = 0, currentLapTargetIndex = 0;
const int MAX_LAPS = 10;
unsigned long lapTimes[MAX_LAPS];
int allowedLapTargets[] = {2, 3, 5};  
int splitTarget = allowedLapTargets[currentLapTargetIndex];
// 깜박임 효과를 위한 변수
static unsigned long lastBlinkTime = 0;
static bool blinkState = false;
const unsigned long blinkInterval = 300;

// [모드 2] 경과시간 시계 관련 변수 =============================================
unsigned long clockStartTime = 0;
bool clockPaused = false;
unsigned long pausedTime = 0;
unsigned long bootTime = 0;
unsigned long lastDisplaySecond = 0;

// [모드 3] QR코드 순환출력 =====================================================
const int numQRs = 3;
int currentQRIndex = 0;
const char* qrCodes[numQRs] = {
    "https://www.helloabt.com/mini4wd/",
    "https://map.naver.com/p/favorite/myPlace/folder/029fd0e8addc4d899ea722f0f5a7a613",
    "https://map.kakao.com/?target=other&folderid=16126480"
  };
const char* qrLabels[numQRs] = {
    "Racer's Manner",
    "Naver Map",
    "Kakao Map"
  };

// 배터리 잔량 계산 함수 ========================================================
float getBatteryPercent() {
    const uint32_t BAT_LOW = 3300; //최저전압
    const uint32_t BAT_HIGH = 4190; //완충전압
    uint32_t voltage = M5.Power.getBatteryVoltage();
    if(voltage < BAT_LOW)
      voltage = BAT_LOW;
    if(voltage > BAT_HIGH)
      voltage = BAT_HIGH;
    return ((float)(voltage - BAT_LOW) / (BAT_HIGH - BAT_LOW)) * 100.0f;
  }

// 텍스트 관련 함수 ===========================================================
bool hasPrinted = false;

// setup() 함수 ===============================================================
void setup(){
  M5.begin();
  M5.Lcd.setRotation(1);
  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setTextDatum(TL_DATUM);
  M5.Lcd.setTextColor(WHITE);
  M5.Lcd.setTextSize(4);
  M5.Power.begin();
  M5.Mic.begin();
  M5.Imu.begin();
  delay(100);

  bootTime = millis(); //부팅 후 경과시간을 기록
  clockStartTime = bootTime;
}

// loop() 함수 ================================================================
void loop(){
  M5.update();
  // 공용 처리: 배터리 잔량 측정 및 LED 깜빡임
  float batteryPct = getBatteryPercent();
  if(batteryPct <= 10.0){
    if(millis() % 2000 < 1000)
      M5.Power.setLed(1);
    else
      M5.Power.setLed(0);
  } else {
    M5.Power.setLed(0);
  }
  // 모드 전환 시: 전체 화면 BLACK 및 기본 텍스트 정렬 복구
  if(prevMode != mode){
    M5.Lcd.fillScreen(BLACK);
    M5.Lcd.setTextDatum(TL_DATUM);
    prevMode = mode;
  }
  // 글로벌 버튼 B 처리 (모드 0~5 모두 적용)
  if (M5.BtnB.isPressed()) {
    if (btnBPressStart == 0)
      btnBPressStart = millis();
   } 
    else if (M5.BtnB.wasReleased()) {
    unsigned long duration = millis() - btnBPressStart;
    // 3초 이상 누른 경우: 센서 모드와 스톱워치 모드에서 값 초기화 (모드는 변경하지 않음)
    if (duration >= 3000) {
      if (mode == 1) {
        // 스톱워치 리셋
        stopwatchRunning = false;
        stopwatchElapsed = 0;
        stopwatchStartTime = 0;
        lapCount = 0;
        lastLapTime = 0;
      }
    }
    // 1초 이상 2초 미만 누른 경우: 이전 모드로 이동
    else if (duration >= 1000 && duration < 2000) {
     mode = (mode - 1 + 4) % 4;  // 음수 방지하며 이전 모드로 순환
    }
    // 1초 미만 (짧게 클릭): 다음 모드로 이동
    else if (duration < 1000) {
     mode = (mode + 1) % 4;  // 다음 모드로 순환
    }
    // 버튼 상태 초기화
    btnBPressStart = 0;
    bLongPressTriggered = false;
    // 화면 초기화 후 잠시 지연 (250ms)
    M5.Lcd.fillScreen(BLACK);
    delay(250);
    return;
  }

  // ────────────────────────────────────────────
  // [모드 0] 센서 모드
  if(mode == 0){
    // A 버튼 처리
    //  - 누른 시간 < 1000ms (릴리즈): 기어비 변경 처리
    //  - 누른 시간 1000ms 이상 2000ms 미만 (릴리즈): 휠 크기 변경 처리
    unsigned long aDuration = 0;
     if (M5.BtnA.isPressed()) {
        if (btnAPressStart == 0)
        btnAPressStart = millis();
        aDuration = millis() - btnAPressStart;
      }
      if (M5.BtnA.wasReleased()) {
        aDuration = millis() - btnAPressStart;
        // 짧게 눌렀을 경우 (< 1000ms): 기어비 변경 처리
        if (aDuration < 1000) {
          currentGearIndex = (currentGearIndex + 1) % numGearRatios;
        }
          // 1초 이상 2초 미만 눌렀다 떼면: 휠 크기 변경 처리
          else if (aDuration >= 1000 && aDuration < 2000) {
            currentWheelIndex = (currentWheelIndex + 1) % numWheelSizes;
          }
        // 릴리즈 후 변수 모두 재설정
        btnAPressStart = 0;
        longPressTriggered = false;
    }
    // 센서 데이터 처리 (마이크 FFT 측정)
    if(M5.Mic.record(micBuffer, SAMPLES, SAMPLING_FREQUENCY)){
      for(int i = 0; i < SAMPLES; i++){
        vReal[i] = (float)micBuffer[i];
        vImag[i] = 0.0;
      }
      FFT.windowing(vReal, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD, vReal, true);
      FFT.compute(vReal, vImag, SAMPLES, FFT_FORWARD);
      FFT.complexToMagnitude(vReal, vImag, SAMPLES);
      float currentFrequency = FFT.majorPeak(vReal, SAMPLES, SAMPLING_FREQUENCY);
      if(currentFrequency < 20)
        currentFrequency = 0;
      int rpmCurrent = (int)round(currentFrequency * 60);
      float currentWheelDiameter = wheelSizes[currentWheelIndex] / 1000.0;
      float selectedGear = gearRatios[currentGearIndex];
      float estimatedSpeed = (rpmCurrent / selectedGear) * (PI * currentWheelDiameter * 0.06);

      M5.Lcd.startWrite();
      // 휠 사이즈 출력 (좌측 상단)
      M5.Lcd.fillRect(10, 5, 90, 15, BLACK);
      M5.Lcd.setTextSize(2);
      M5.Lcd.setCursor(10,5);
      M5.Lcd.setTextColor(gearColors[currentGearIndex]);
      M5.Lcd.print(wheelSizes[currentWheelIndex], 1);
      M5.Lcd.setCursor(60,5);
      M5.Lcd.setTextColor(WHITE);
      M5.Lcd.print("mm");
      // 배터리 잔량 텍스트 (우측 상단)
      M5.Lcd.fillRect(M5.Lcd.width()-59, 5, 50, 20, BLACK);
      float batPct = getBatteryPercent();
      uint16_t batColor = GREEN;
      if(batPct <= 10.0){
        if(millis() % 2000 < 1000)
          batColor = RED;
        else
          batColor = BLACK;
      }
      M5.Lcd.setTextSize(2);
      float batteryPct = getBatteryPercent();
      int roundedBattery = round(batteryPct);
      M5.Lcd.fillRect(M5.Lcd.width()-60, 5, 60, 20, BLACK);
      M5.Lcd.setTextSize(2);
      M5.Lcd.setTextColor(batColor);
      M5.Lcd.setCursor(M5.Lcd.width()-59, 5);
      char formattedBattery[10];
      sprintf(formattedBattery, "%3d%", roundedBattery);
      M5.Lcd.print(formattedBattery);
      M5.Lcd.setCursor(M5.Lcd.width()-20, 5);
      M5.Lcd.setTextColor(WHITE);
      M5.Lcd.print("%");
      // 배터리 잔량 10% 이하 도달시 LED 점등
      if(batPct <= 10.0){
        if(millis() % 2000 < 1000)
          M5.Power.setLed(1);
        else
          M5.Power.setLed(0);
      }
      else {
        M5.Power.setLed(0);
      }
      // 현재 주파수
      M5.Lcd.fillRect(60, 33, 180, 30, BLACK);
      M5.Lcd.setTextSize(4);
      M5.Lcd.setCursor(10, 33);
      M5.Lcd.setTextColor(WHITE);
      M5.Lcd.print("P");
      int currFreqInt = (int)round(currentFrequency);
      M5.Lcd.setTextSize(3);
      M5.Lcd.setCursor(63, 33);
      M5.Lcd.setTextColor(CYAN);
      M5.Lcd.print(currFreqInt);
      M5.Lcd.setTextColor(WHITE);
      // 추정속도
      M5.Lcd.fillRect(50, 63, 190, 25, BLACK);
      M5.Lcd.setTextSize(2.3);
      M5.Lcd.setCursor(63, 63);
      M5.Lcd.setTextColor(ORANGE);
      M5.Lcd.print(estimatedSpeed, 1);
      M5.Lcd.setTextColor(WHITE);
      M5.Lcd.print(" km/h");
      // RPM
      M5.Lcd.fillRect(60, 88, 200, 25, BLACK);
      M5.Lcd.setTextSize(2.2);
      M5.Lcd.setCursor(63, 88);
      M5.Lcd.setTextColor(MAGENTA);
      M5.Lcd.print(rpmCurrent);
      M5.Lcd.setTextColor(WHITE);
      M5.Lcd.print(" rpm");
      //version 정보 (컴파일 시점의 시스템 시간에 기초)
      String dateStr = String(__DATE__).substring(4, 6);
      String timeStr = String(__TIME__).substring(0, 2) +
                      String(__TIME__).substring(3, 5) +
                      String(__TIME__).substring(6, 8);
      String mpas_version = "MPAS-CE-v0.1_" + dateStr + timeStr;
      M5.Lcd.setTextSize(1.7);
      M5.Lcd.setTextColor(DARKGRAY);
      M5.Lcd.drawCentreString(mpas_version, M5.Lcd.width()/2, 116, 1.5);
    }
    delay(100);
    return;
  } //mode 0 end

  //────────────────────────────────────────────
  // [모드 1] 스톱워치 모드
  if(mode == 1){
      if (M5.BtnA.isPressed()) {
    if (btnAPressStart == 0)
      btnAPressStart = millis();
    // 스톱워치 정지 상태에서 A 버튼 1초 이상 눌러도 랩 타겟 변경하지 않음
    if (!stopwatchRunning && (millis() - btnAPressStart >= 1000)) {
      splitTarget = 3;  // 랩 타겟을 항상 3으로 고정
    }
  }
  if (M5.BtnA.wasReleased()) {
    if (longPressTriggered && !stopwatchRunning) {
      btnAPressStart = 0;
      longPressTriggered = false;
      return;
    } else {
      if (!stopwatchRunning) {
        // 스톱워치 시작
        stopwatchRunning = true;
        stopwatchStartTime = millis();
        stopwatchElapsed = 0;
        lapCount = 0;
        lastLapTime = 0;
      } else {
        unsigned long currentTimeLocal = stopwatchElapsed + (millis() - stopwatchStartTime);

        // lapCount가 2 미만이면 일반 랩 기록
        if (lapCount < 2) {
          unsigned long lapInterval = currentTimeLocal - lastLapTime;
          if (lapCount < MAX_LAPS)
            lapTimes[lapCount] = lapInterval;
            lapCount++;
            lastLapTime = currentTimeLocal;
        }
        // lapCount가 (splitTarget - 1)와 같으면 마지막 랩 기록 후 자동 정지
        else {
          unsigned long lapInterval = currentTimeLocal - lastLapTime;
          if (lapCount < MAX_LAPS)
            lapTimes[lapCount] = lapInterval;
          lapCount++;
          lastLapTime = currentTimeLocal;
          stopwatchRunning = false;
          stopwatchElapsed = currentTimeLocal;
        }
      }
    }
    btnAPressStart = 0;
    longPressTriggered = false;
  }
  // 스톱워치의 현재 시간 계산 (6시간 제한 적용)
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
  // 상단 타이머 영역(높이 40픽셀)만 지워서 다시 업데이트
  M5.Lcd.fillRect(0, 0, M5.Lcd.width(), 40, BLACK);
  // 스톱워치가 정지되어 랩 타겟 (예, 2랩)에 도달한 경우 타이머 숫자만 깜박임
  if (!stopwatchRunning && lapCount >= splitTarget) {
    if (millis() - lastBlinkTime >= blinkInterval) {
      blinkState = !blinkState;
      lastBlinkTime = millis();
    }
    uint16_t displayColor = blinkState ? WHITE : BLACK;
    M5.Lcd.setTextSize(2.8);
    // 굵은 효과를 위해 3x3 오프셋으로 그리기
    for (int dx = -1; dx <= 1; dx++) {
      for (int dy = -1; dy <= 1; dy++) {
        M5.Lcd.setTextColor(displayColor);
        M5.Lcd.drawCentreString(timeStr, 120 + dx, 5 + dy, 2);
      }
    }
  } else {
    // 스톱워치가 진행중이면 일반적으로 타이머 숫자(굵게) 출력
    M5.Lcd.setTextSize(2.8);
    for (int dx = -1; dx <= 1; dx++) {
      for (int dy = -1; dy <= 1; dy++) {
        M5.Lcd.setTextColor(WHITE);
        M5.Lcd.drawCentreString(timeStr, 120 + dx, 5 + dy, 2);
      }
    }
  }
  // 랩 기록 출력 영역 업데이트 (타이머 아래쪽)
  M5.Lcd.fillRect(0, 40, M5.Lcd.width(), M5.Lcd.height() - 40, BLACK);
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
      for (int j = 0; j <= i; j++) {
        cumulative += lapTimes[j];
      }
      int cumMin = cumulative / 60000;
      int cumSec = (cumulative % 60000) / 1000;
      int cumCs = (cumulative % 1000) / 10;   
      sprintf(lapLabel, "L%d ", i + 1);
      sprintf(lapSplitStr, "%02d:%02d.%02d", lapMin, lapSec, lapCs);
      sprintf(cumStr, " (%02d:%02d.%02d)", cumMin, cumSec, cumCs);
      int xPos = 10;
      M5.Lcd.setTextSize(1.7);
      M5.Lcd.setTextColor(WHITE);
      M5.Lcd.drawString(lapLabel, xPos, yPos);
      xPos += M5.Lcd.textWidth(lapLabel);
      M5.Lcd.setTextColor(CYAN);
      M5.Lcd.drawString(lapSplitStr, xPos, yPos);
      xPos += M5.Lcd.textWidth(lapSplitStr);
      M5.Lcd.setTextColor(YELLOW);
      M5.Lcd.drawString(cumStr, xPos, yPos);
      yPos += 15;
    }
  }
  delay(10);
  return;
  } //mode 1 end

  //────────────────────────────────────────────
  // [모드 2] 경과시간 시계 모드
  if(mode == 2){
    if(M5.BtnA.isPressed()){
      if(btnAPressStart == 0)
        btnAPressStart = millis();
      if(millis() - btnAPressStart >= 1000 && !longPressTriggered){
        longPressTriggered = true;
        clockStartTime = millis();
        pausedTime = 0;
        clockPaused = false;
      }
    }
    else if(M5.BtnA.wasReleased()){
      if(!longPressTriggered){
        if(!clockPaused){
          clockPaused = true;
          pausedTime = millis() - clockStartTime;
        }
        else{
          clockPaused = false;
          clockStartTime = millis() - pausedTime;
        }
      }
      btnAPressStart = 0;
      longPressTriggered = false;
    }
    unsigned long elapsed = clockPaused ? pausedTime : (millis() - clockStartTime);
    unsigned int hours = elapsed / 3600000UL;
    unsigned int minutes = (elapsed % 3600000UL) / 60000;
    unsigned int seconds = (elapsed % 60000UL) / 1000;
    char timeStr[9];
    sprintf(timeStr, "%02u:%02u:%02u", hours, minutes, seconds);
    if(seconds != lastDisplaySecond){
      lastDisplaySecond = seconds;
      int posY = (M5.Lcd.height() - 20) / 2;
      M5.Lcd.fillRect(0, posY, M5.Lcd.width(), 20, BLACK);
      M5.Lcd.setTextSize(4);
      M5.Lcd.setTextColor(YELLOW, BLACK);
      M5.Lcd.setTextDatum(MC_DATUM);
      int centerX = M5.Lcd.width() / 2;
      M5.Lcd.drawString(timeStr, centerX, posY);
    }
    delay(50);
    return;
  } //mode 2 end

  //────────────────────────────────────────────
  // [모드 3] QR모드
  if(mode == 3){
      bool qrDisplayed = false;  // QR 코드 한 번만 출력하도록 설정
      if (!qrDisplayed) {  
          int qrSize = 100;
          int centerX = (M5.Lcd.width() - qrSize) / 2;
          int centerY = (M5.Lcd.height() - qrSize) / 2;
          // QR 코드 출력
          M5.Lcd.qrcode(qrCodes[currentQRIndex], centerX, centerY-7, qrSize);
          // QR 코드 아래에 레이블 출력
          M5.Lcd.setTextSize(1.9);
          M5.Lcd.setTextColor(WHITE, BLACK);
          int labelX = (M5.Lcd.width() - M5.Lcd.textWidth(qrLabels[currentQRIndex])) / 2;
          int labelY = centerY + qrSize + 1;
          M5.Lcd.fillRect(0, labelY - 5, M5.Lcd.width(), 20, BLACK);  // 레이블 영역 지우기
          M5.Lcd.drawString(qrLabels[currentQRIndex], labelX, labelY);

          qrDisplayed = true;
      }
      // A 버튼을 짧게 누르면 QR 코드 및 레이블 변경
      if (M5.BtnA.wasReleased()) {
         currentQRIndex = (currentQRIndex + 1) % numQRs;  // QR 코드 순환
          qrDisplayed = false;  // 다음 QR 코드 출력 허용
      }
      // B 버튼을 눌렀을 때 모드 변경
      if (M5.BtnB.wasReleased()) {
          mode++;
          qrDisplayed = false;  // 다음 모드로 변경되면 다시 QR 코드 출력 허용
      }
  } //mode3 end
} //loop end
