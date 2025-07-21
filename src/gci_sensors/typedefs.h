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

#ifndef __PIN__
  #define __PIN__
typedef uint32_t pin_t;
#endif

#ifndef __VEC3F__
  #define __VEC3F__
typedef struct {
  float x, y, z;
} vec3f_t;
#endif

#ifndef __VEC3S__
  #define __VEC3S__
typedef struct {
  int16_t x, y, z;
} vec3s_t;
#endif

// typedef struct {
//   float x, y, z;
// } svec_t;

// typedef struct {
//   int16_t x, y, z;
// } vec3s_t;

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
