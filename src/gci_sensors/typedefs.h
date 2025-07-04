////////////////////////////////////////////////
//  The MIT License (MIT)
//  Copyright (c) 2023 Kevin Walchko
//  see LICENSE for full details
////////////////////////////////////////////////
#pragma once

#include <stdint.h>

#if defined __cplusplus
extern "C" {
#endif

// constexpr float M_PIf   = 3.14159265358979323846f;

#ifndef pin_t

typedef uint32_t pin_t;

#endif

typedef struct {
  float x, y, z;
} svec_t;

typedef struct {
  int16_t x, y, z;
} svec_raw_t;

// typedef union vector2_u {
//   float v[2];
//   struct {
//     float x, y;
//   };
// } vector2_t;

// typedef union vector3_u {
//   float v[3];
//   struct {
//     float x, y, z;
//   };
// } vector3_t;

#if defined __cplusplus
}
#endif
