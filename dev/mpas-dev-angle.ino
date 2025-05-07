#include <M5StickCPlus2.h>  // M5StickC Plus2용 라이브러리 포함

#define BUZZ_PIN 2  // 부저(Buzzer) 핀 번호 정의

// 가속도 센서 데이터를 저장할 변수
float accX = 0, accY = 0, accZ = 0;

// 각도 관련 변수 (기울기 및 방향)
double psi = 0, last_psi = 0;  // Yaw (회전)
double phi = 0, last_phi = 0;  // Pitch (앞뒤 기울기)
double theta = 0, last_theta = 0;  // Roll (좌우 기울기)

// 보정값 저장 변수
double offset_psi = 0, offset_phi = 0;

// 저역통과 필터 계수 (데이터 평활화)
double alpha = 0.1;

// 버튼 상태 변수
bool usePhi = false;  // 현재 표시 모드 (Pitch / Yaw)
bool ignoreNextRelease = false;  // 버튼 릴리즈 무시 여부

// M5StickC Plus2의 화면을 더블 버퍼링 방식으로 사용하기 위한 스프라이트 객체
M5Canvas sprite(&M5.Display);

void setup() {
    auto cfg = M5.config();  // M5 장치 기본 설정 로드
    M5.begin(cfg);  // M5StickC Plus2 초기화

    // 화면 설정
    M5.Display.setRotation(1);  // 화면 회전
    M5.Display.fillScreen(BLACK);  // 화면 초기화 (검은색 배경)

    // 스프라이트 생성 (디스플레이 크기: 240x135)
    sprite.createSprite(240, 135);
    sprite.setTextSize(5);  // 텍스트 크기 설정
    sprite.setTextColor(WHITE);  // 텍스트 색상 설정

    // 부저 핀을 출력 모드로 설정
    pinMode(BUZZ_PIN, OUTPUT);
}

// 센서 보정값 초기화 함수
void resetSensor() {
    offset_psi = psi;  // 현재 Yaw(회전) 값을 기준점으로 설정
    offset_phi = phi;  // 현재 Pitch(기울기) 값을 기준점으로 설정
}

void loop() {
    M5.update();  // 버튼 상태 및 센서 값 갱신
    M5.Imu.getAccelData(&accX, &accY, &accZ);  // 가속도 센서 값 가져오기

    // 센서 데이터를 사용하여 기기의 각도 계산
    if (accY != 0) psi = atan2(accX, accY) * 57.295;  // Yaw 각도 계산
    if (accZ != 0) phi = atan2(accY, accZ) * 57.295;  // Pitch 각도 계산
    if ((accX < 1) && (accX > -1)) theta = asin(-accX) * 57.295;  // Roll 계산

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
        tone(BUZZ_PIN, 500, 200);  // 부저음 발생 (500Hz, 200ms)
        delay(210);
        noTone(BUZZ_PIN);
        resetSensor();  // 센서 보정 초기화
        ignoreNextRelease = true;  // 버튼 릴리즈를 다음 루프에서 무시
    }

    // 버튼이 짧게 눌렸다가 떼어졌을 경우 표시 모드 변경
    if (M5.BtnA.wasReleased()) {
        if (!ignoreNextRelease) {
            usePhi = !usePhi;  // Pitch(Y 기울기) ↔ Yaw(회전) 모드 변경
            tone(BUZZ_PIN, 800, 50);  // 짧은 부저음 (모드 변경 알림)
            delay(60);
            tone(BUZZ_PIN, 1200, 50);
            delay(60);
            noTone(BUZZ_PIN);
        }
        ignoreNextRelease = false;
    }

    // 보정된 각도 계산
    double adj_psi = psi - offset_psi;  // 보정된 Yaw 값
    double adj_phi = phi - offset_phi;  // 보정된 Pitch 값

    // 스프라이트를 사용하여 화면을 갱신 (더블 버퍼링 방식)
    sprite.fillScreen(BLACK);  // 화면 초기화
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
}
