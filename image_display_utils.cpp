#include <M5StickCPlus2.h>
#include "image_display_utils.h"  // 이 헤더에서 선언된 함수들을 정의
#include "image.h"                // 이미지 데이터를 사용

// 특정 이미지를 화면에 그리는 헬퍼 함수 정의
void drawSpecificImage(const GimpImage* img) {
  sprite.fillScreen(BLACK);
  sprite.pushImage(0, 0, img->width, img->height, (uint16_t*)img->pixel_data);
  sprite.pushSprite(0, 0);
}

