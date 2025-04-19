/**************************************\
 * The MIT License (MIT)
 * Copyright (c) 2022 Kevin Walchko
 * see LICENSE for full details
\**************************************/
#pragma once

// #if defined(__USE_SENSOR_LIS3MDL__)

#include <picolibc.h>
#include <stdbool.h>
#include <stdint.h>

#if defined __cplusplus
extern "C" {
#endif

constexpr uint8_t LIS3MDL_ADDRESS     = 0x1C;
constexpr uint8_t LIS3MDL_ADDRESS_ALT = 0x1E;

constexpr uint8_t RANGE_4GAUSS        = 0x00; // default
constexpr uint8_t RANGE_8GAUSS        = 0x20;
constexpr uint8_t RANGE_12GAUSS       = 0x40;
constexpr uint8_t RANGE_16GAUSS       = 0x60;

constexpr uint8_t LIS3MDL_LP          = 0x00;
constexpr uint8_t LIS3MDL_MP          = 0x01;
constexpr uint8_t LIS3MDL_HIP         = 0x02;
constexpr uint8_t LIS3MDL_UHP         = 0x03;

constexpr uint8_t ODR_155HZ           = LIS3MDL_UHP; // 3
constexpr uint8_t ODR_300HZ           = LIS3MDL_HIP; // 2
constexpr uint8_t ODR_560HZ           = LIS3MDL_MP;  // 1
constexpr uint8_t ODR_1000HZ          = LIS3MDL_LP;  // 0

// typedef enum : uint8_t {
//   RANGE_4GAUSS  = 0x00, // default
//   RANGE_8GAUSS  = 0x20,
//   RANGE_12GAUSS = 0x40,
//   RANGE_16GAUSS = 0x60
// } Range;

// typedef enum : uint8_t {
//   ODR_155HZ  = LIS3MDL_UHP, // 3
//   ODR_300HZ  = LIS3MDL_HIP, // 2
//   ODR_560HZ  = LIS3MDL_MP,  // 1
//   ODR_1000HZ = LIS3MDL_LP   // 0
// } Odr;

// enum mdl_error : uint8_t {
//   NO_ERROR,
//   ERROR_WHOAMI,
//   ERROR_REG1,
//   ERROR_REG2,
//   ERROR_REG3,
//   ERROR_REG4,
//   ERROR_REG5
// };

typedef struct {
  float x, y, z;
  bool ok; // error?
} lis3mdl_t;

typedef union {
  struct {
    int16_t x, y, z, temp;
  } s;
  uint8_t b[8]; // bytes
} buff_t;

typedef struct {
  i2c_inst_t *i2c;
  uint8_t addr;
  buff_t buff;
  float sm[12];
} lis3mdl_i2c_t;

lis3mdl_i2c_t *lis3mdl_i2c_init(uint32_t port, uint8_t addr, uint8_t range,
                                uint8_t odr);
bool lis3mdl_reboot(lis3mdl_i2c_t *hw);
const lis3mdl_t lis3mdl_read(lis3mdl_i2c_t *hw);
bool lis3mdl_ready(lis3mdl_i2c_t *hw);

#if defined __cplusplus
}
#endif