#ifndef IMAGE_H
#define IMAGE_H

#include <stdint.h> // unsigned int, unsigned char를 사용하므로 필요

// GimpImage 구조체 정의
struct GimpImage {
  unsigned int    width;
  unsigned int    height;
  unsigned int    bytes_per_pixel; /* 2:RGB16, 3:RGB, 4:RGBA */ 
  char            *comment;
  unsigned char   pixel_data[240 * 135 * 2 + 1]; // 이미지 크기에 맞게 조정
};

// 이미지 데이터를 담을 변수들의 'extern' 선언
// 이 변수들은 다른 .cpp 파일에서 정의될 것임을 컴파일러에 알립니다.
extern const GimpImage hugging_mpas;
extern const GimpImage hugging_neochamp;
extern const GimpImage neogulman;

#endif