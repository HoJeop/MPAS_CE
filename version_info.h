#ifndef VERSION_INFO_H
#define VERSION_INFO_H

#include <M5StickCPlus2.h>
#include <WiFi.h>        // WiFi.macAddress()를 사용하기 위해 추가 (이전 코드 호환성을 위해 유지)
#include <esp_system.h>  // ESP.getEfuseMac()을 사용하기 위해 추가

#define DARKGRAY 0x7BEF

// 버전 정보를 표시할 스프라이트 객체 (extern으로 선언하여 main.cpp에서 정의된 객체를 사용하도록 함)
extern M5Canvas sprite;

/**
 * @brief 컴파일 시점의 빌드 정보를 화면에 표시합니다.
 * "MPAS-FE-v0.1_DDhhmmss" 형식으로 날짜와 시간을 포함합니다.
 */
inline void displayBuildInfo() {
  // __DATE__ 예: "Apr 27 2025" → substring(7, 11)는 "2025" (연도)
  // __DATE__ 예: "Apr 27 2025" → substring(4, 6)는 "27" (날짜)
  // __TIME__ 예: "22:45:12" → 결합하면 "224512" (시간)

  String yearStr = String(__DATE__).substring(7, 11);  // 연도 추출
  String monthStr;

  // __DATE__의 첫 3글자("Apr" 같은 월 정보)를 숫자로 변환
  String monthMap = "JanFebMarAprMayJunJulAugSepOctNovDec";
  int monthIndex = monthMap.indexOf(String(__DATE__).substring(0, 3)) / 3 + 1;
  monthStr = (monthIndex < 10 ? "0" : "") + String(monthIndex);

  // [컴파일 시점의 날짜와 시간을 YYMMDDhhmmss 형식으로 변환]
  // - __DATE__ → "Apr 27 2025" 형식의 문자열 (월, 날짜, 연도 포함)
  // - __TIME__ → "22:45:12" 형식의 문자열 (시, 분, 초 포함)

  String dayStr = String(__DATE__).substring(4, 6);  // 날짜 부분 추출
  // - __DATE__의 4~6번째 문자 추출 → "27" (예: "Apr 27 2025"에서 날짜 부분)
  // - 이 값을 이용해 YYMMDD 형식의 'DD' 부분 설정 가능

  String timeStr = String(__TIME__).substring(0, 2) +  // 시간 부분 추출
                   String(__TIME__).substring(3, 5) +  // 분 부분 추출
                   String(__TIME__).substring(6, 8);   // 초 부분 추출
  // - __TIME__의 특정 위치 문자열을 결합하여 "hhmmss" 형식 생성
  //   예) "22:45:12" → "224512"

  String mpas_version = "MPAS-CE-v0.2_" + /*yearStr.substring(2, 4) +*/ monthStr + dayStr + timeStr;

  // 텍스트 설정 및 화면 중앙에 빌드 정보 출력
  sprite.setTextSize(1.7);
  sprite.setTextColor(DARKGRAY);
  sprite.print(mpas_version);  // 빌드 정보 출력
}

/**
 * @brief 장치의 시리얼 넘버 (MAC 주소 기반)를 화면에 표시합니다.
 * WiFi 초기화 여부와 상관없이 항상 고유한 MAC 주소를 반환하는 ESP.getEfuseMac()을 사용합니다.
 */
inline void displayDeviceSerialNumber() {
  // ESP.getEfuseMac()을 사용하여 MAC 주소를 uint64_t 형태로 가져옵니다.
  uint64_t chipid = ESP.getEfuseMac();

  // MAC 주소를 사람이 읽을 수 있는 16진수 문자열 형식으로 변환합니다.
  // 예: "SN:XXXXXXXXXXXX"
  char serialNumberChar[17];  // 12자리 MAC 주소 + "SN:" + 널 종료 문자
  sprintf(serialNumberChar, "SN:%04X%08X", (uint16_t)(chipid >> 32), (uint32_t)chipid);
  String serialNumber = String(serialNumberChar);

  // 텍스트 설정 및 화면 중앙에 시리얼 넘버 출력
  sprite.setTextSize(1.7);
  sprite.setTextColor(DARKGRAY);
  sprite.print(serialNumber);
}

#endif
