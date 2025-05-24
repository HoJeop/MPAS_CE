#ifndef IMAGE_DISPLAY_UTILS_H
#define IMAGE_DISPLAY_UTILS_H

#include <M5StickCPlus2.h>
#include "image.h"   // GimpImage 구조체와 이미지 데이터를 사용하므로 필요

// M5Canvas 객체를 외부에 선언 (다른 파일에서 사용하기 위함)
// extern 키워드를 사용하여 이 변수가 다른 곳에서 정의되었음을 알립니다.
extern M5Canvas sprite;

// 함수 선언
void drawSpecificImage(const GimpImage* img);

#endif