#include <M5StickCPlus2.h>

float threshold = 45; // 회전 감지 임계값
bool isUpsideDown = false;
// M5StickC Plus2의 화면을 더블 버퍼링 방식으로 사용하기 위한 스프라이트 객체
M5Canvas sprite(&M5.Display);

// 가속도 센서 데이터를 저장할 변수
float accX = 0, accY = 0, accZ = 0;
// 각도 관련 변수 (회전)
double psi = 0;

void setup() {
    M5.begin();
    M5.Imu.init();
    M5.Display.setRotation(1);  // 초기 화면 회전 설정 (세로)
    M5.Display.fillScreen(BLACK);  // 화면 초기화 (검은색 배경)
    sprite.createSprite(240, 135);  // 스프라이트 생성 (디스플레이 크기: 240x135)
    M5.Display.setRotation(1); // 초기 화면 방향 설정
}

void loop() {
    M5.Imu.getAccelData(&accX, &accY, &accZ);  // 가속도 센서 값 가져오기

    // 센서 데이터를 사용하여 기기의 각도 계산
    if (accY != 0) psi = atan2(accX, accY) * 57.295;  // Yaw 각도 계산

    // Yaw 값이 임계값을 넘으면 화면을 반전 (상하 반전)
    if (psi > threshold && !isUpsideDown) {
        M5.Display.setRotation(1); // 180도 회전 (상하 반전)
        isUpsideDown = true;
    } else if (psi < -threshold && isUpsideDown) {
        M5.Display.setRotation(3); // 원래 방향으로 회전
        isUpsideDown = false;
    }

    // 스프라이트를 사용하여 화면을 갱신 (더블 버퍼링 방식)
    sprite.fillScreen(BLACK);  // 화면 초기화
    sprite.setTextColor(WHITE);
    sprite.setTextSize(5);
    sprite.setCursor(5, 49);
    sprite.printf("%6.1f", psi);

    // 스프라이트를 디스플레이에 출력 (화면 깜빡임 방지)
    sprite.pushSprite(0, 0);
    delay(100);  // 루프 주기 설정 (100ms마다 갱신)
}