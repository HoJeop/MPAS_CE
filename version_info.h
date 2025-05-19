#ifndef VERSION_INFO_H
#define VERSION_INFO_H

#include <M5StickCPlus2.h>
#include <WiFi.h>
#include "esp_efuse.h"
#include "esp_efuse_table.h"

// 버전 정보를 표시할 스프라이트 객체 (extern으로 선언하여 main.cpp에서 정의된 객체를 사용하도록 함)
extern M5Canvas sprite;

// 버전 정보를 화면에 표시하는 함수 정의 (inline 또는 static으로 선언 권장)
inline void displayVersionInfo() {
  // 시리얼 번호 (MAC 주소 기반) 가져오기
  uint8_t mac[6];
  esp_efuse_mac_get_default(mac);
  char serialNumber[18];
  sprintf(serialNumber, "%02X%02X%02X%02X%02X%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  sprite.printf("Serial:%s\n", serialNumber);

/*
  // ESP-IDF 버전 정보 가져오기
  esp_chip_info_t chip_info;
  esp_chip_info(&chip_info);
  sprite.print("ESP-IDF_");
  sprite.printf("%s\n", esp_get_idf_version());


  // SDK 버전 정보 가져오기
  sprite.print("SDK_");
  sprite.printf("%s\n", ESP.getSdkVersion());
  */
}

#endif