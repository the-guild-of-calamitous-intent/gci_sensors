////////////////////////////////////////////////
//  The MIT License (MIT)
//  Copyright (c) 2022 Kevin Walchko
//  see LICENSE for full details
////////////////////////////////////////////////
#pragma once

#include <stdint.h>

#if defined __cplusplus
extern "C" {
#endif

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

#ifndef __PT__
  #define __PT__
typedef struct {
  float pressure;
  float temperature;
} pt_t;
#endif

#ifndef __IMU__
  #define __IMU__
typedef struct {
  vec3f_t a;
  vec3f_t g;
  float temperature;
} imuf_t;
#endif

#if defined __cplusplus
}
#endif

// constexpr float M_PIf   = 3.14159265358979323846f;

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
