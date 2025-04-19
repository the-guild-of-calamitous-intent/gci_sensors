/**************************************\
 * The MIT License (MIT)
 * Copyright (c) 2022 Kevin Walchko
 * see LICENSE for full details
\**************************************/
#pragma once


// #if defined(__USE_SENSOR_LIS3MDL__)

#include <stdint.h>
#include <stdbool.h>
#include <string.h> // memcpy
#include <picolibc.h>

#if defined __cplusplus
extern "C" {
#endif



constexpr uint8_t REG_WHO_AM_I   = 0x0F;
constexpr uint8_t REG_CTRL_REG1  = 0x20;
constexpr uint8_t REG_CTRL_REG2  = 0x21;
constexpr uint8_t REG_CTRL_REG3  = 0x22;
constexpr uint8_t REG_CTRL_REG4  = 0x23;
constexpr uint8_t REG_CTRL_REG5  = 0x24;
constexpr uint8_t REG_STATUS_REG = 0x27;
constexpr uint8_t REG_OUT_X_L    = 0x28;
// constexpr uint8_t REG_OUT_X_H    = 0x29;
// constexpr uint8_t REG_OUT_Y_L    = 0x2A;
// constexpr uint8_t REG_OUT_Y_H    = 0x2B;
// constexpr uint8_t REG_OUT_Z_L    = 0x2C;
// constexpr uint8_t REG_OUT_Z_H    = 0x2D;
// constexpr uint8_t REG_TEMP_OUT_L = 0x2E;
// constexpr uint8_t REG_TEMP_OUT_H = 0x2F;
// constexpr uint8_t REG_INT_CFG    = 0x30;
// constexpr uint8_t REG_INT_SRC    = 0x31;
// constexpr uint8_t REG_INT_THS_L  = 0x32;
// constexpr uint8_t REG_INT_THS_H  = 0x33;

constexpr uint8_t WHO_AM_I            = 0x3D;
constexpr uint8_t STATUS_ZYXDA        = 0x08; // 0b00001000;
constexpr uint8_t LIS3MDL_TEMP_EN     = 0x80; // chip default off
constexpr uint8_t LIS3MDL_FAST_ODR_EN = 0x02;
constexpr uint8_t LIS3MDL_BDU_EN      = 0x40; // chip default off

constexpr uint8_t LIS3MDL_LP          = 0x00;
constexpr uint8_t LIS3MDL_MP          = 0x01;
constexpr uint8_t LIS3MDL_HIP         = 0x02;
constexpr uint8_t LIS3MDL_UHP         = 0x03;

constexpr uint8_t LIS3MDL_ADDRESS        = 0x1C;
constexpr uint8_t LIS3MDL_ADDRESS_ALT    = 0x1E;

typedef enum: uint8_t {
  RANGE_4GAUSS  = 0x00, // default
  RANGE_8GAUSS  = 0x20,
  RANGE_12GAUSS = 0x40,
  RANGE_16GAUSS = 0x60
} Range;

typedef enum: uint8_t {
  ODR_155HZ  = LIS3MDL_UHP, // 3
  ODR_300HZ  = LIS3MDL_HIP, // 2
  ODR_560HZ  = LIS3MDL_MP,  // 1
  ODR_1000HZ = LIS3MDL_LP   // 0
} Odr;

enum mdl_error : uint8_t {
  NO_ERROR,
  ERROR_WHOAMI,
  ERROR_REG1,
  ERROR_REG2,
  ERROR_REG3,
  ERROR_REG4,
  ERROR_REG5
};

typedef struct {
  float x, y, z;
  bool ok;  // error?
} lis3mdl_t;

// struct lis3mdl_raw_t {
//   int16_t x, y, z;
//   bool ok;  // error?

//   int16_t operator[](size_t i) {
//     return (i == 0) ? x : (i == 1) ? y : z;
//   }
// };
typedef union {
  struct {
    int16_t x, y, z, temp;
  } s;
  // int16_t s[4]; // signed short
  uint8_t b[8]; // bytes
} buff_t;

typedef struct {
  i2c_inst_t *i2c;
  uint8_t addr;
  buff_t buff;
  float sm[12];
} lis3mdl_i2c_t;

lis3mdl_i2c_t* lis3mdl_i2c_init(uint32_t port, uint8_t addr, Range range, Odr odr);
bool lis3mdl_reboot(lis3mdl_i2c_t *hw);
const lis3mdl_t lis3mdl_read(lis3mdl_i2c_t *hw);
bool lis3mdl_ready(lis3mdl_i2c_t *hw);

#if defined __cplusplus
}
#endif