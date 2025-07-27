////////////////////////////////////////////////
//  The MIT License (MIT)
//  Copyright (c) 2022 Kevin Walchko
//  see LICENSE for full details
////////////////////////////////////////////////
#pragma once

#include "gci_sensors/io.h"
#include <stdbool.h>
#include <stdint.h>

#if defined __cplusplus
extern "C" {
#endif

#define MAG_BUFFER_SIZE 6

#define LIS3MDL_ADDRESS 0x1C
#define LIS3MDL_ADDRESS_ALT 0x1E
typedef enum : uint8_t {
  LIS3MDL_RANGE_4GAUSS  = 0x00, // default
  LIS3MDL_RANGE_8GAUSS  = 0x20,
  LIS3MDL_RANGE_12GAUSS = 0x40,
  LIS3MDL_RANGE_16GAUSS = 0x60
} lis3mdl_range_t;

typedef enum : uint8_t {
  LIS3MDL_ODR_155HZ  = 3, // LIS3MDL_UHP, // 3
  LIS3MDL_ODR_300HZ  = 2, // LIS3MDL_HIP, // 2
  LIS3MDL_ODR_560HZ  = 1, // LIS3MDL_MP,  // 1
  LIS3MDL_ODR_1000HZ = 0, // LIS3MDL_LP   // 0
} lis3mdl_odr_t;

typedef struct {
  float x, y, z;
} lis3mdl_t;

typedef union {
  struct {
    int16_t x, y, z, temp;
  } s;
  uint8_t b[MAG_BUFFER_SIZE]; // bytes
} buff_t;

typedef struct {
  comm_interface_t *comm;
  buff_t buff;
  float sm[12];
  float scale;
  bool ok;
} lis3mdl_io_t;

lis3mdl_io_t *lis3mdl_spi_init(uint8_t port, pin_t cs, lis3mdl_range_t range,
                               lis3mdl_odr_t odr);
lis3mdl_io_t *lis3mdl_i2c_init(uint8_t port, uint8_t addr,
                               lis3mdl_range_t range, lis3mdl_odr_t odr);
const lis3mdl_t lis3mdl_read(lis3mdl_io_t *hw);
bool lis3mdl_reboot(lis3mdl_io_t *hw);
bool lis3mdl_ready(lis3mdl_io_t *hw);

#if defined __cplusplus
}
#endif

// typedef enum {
//   LIS3MDL_LP  = 0x00,
//   LIS3MDL_MP  = 0x01,
//   LIS3MDL_HIP = 0x02,
//   LIS3MDL_UHP = 0x03
// } lis3mdl_pwr_t;

// typedef struct {
//   i2c_inst_t *i2c;
//   uint8_t addr;
//   buff_t buff;
//   float sm[12];
//   float scale;
// } lis3mdl_i2c_t;

// lis3mdl_i2c_t *lis3mdl_i2c_init(uint32_t port, uint8_t addr, uint8_t range,
//                                 uint8_t odr);
// bool lis3mdl_reboot(lis3mdl_i2c_t *hw);
// const lis3mdl_t lis3mdl_read(lis3mdl_i2c_t *hw);
// bool lis3mdl_ready(lis3mdl_i2c_t *hw);

// constexpr uint8_t RANGE_4GAUSS        = 0x00; // default
// constexpr uint8_t RANGE_8GAUSS        = 0x20;
// constexpr uint8_t RANGE_12GAUSS       = 0x40;
// constexpr uint8_t RANGE_16GAUSS       = 0x60;

// constexpr uint8_t ODR_155HZ           = LIS3MDL_UHP; // 3
// constexpr uint8_t ODR_300HZ           = LIS3MDL_HIP; // 2
// constexpr uint8_t ODR_560HZ           = LIS3MDL_MP;  // 1
// constexpr uint8_t ODR_1000HZ          = LIS3MDL_LP;  // 0
