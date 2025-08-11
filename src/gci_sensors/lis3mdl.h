////////////////////////////////////////////////
//  The MIT License (MIT)
//  Copyright (c) 2022 Kevin Walchko
//  see LICENSE for full details
////////////////////////////////////////////////
#pragma once
#include <stdbool.h>
#include <stdint.h>

#include "gci_sensors/io.h"
#include "gci_sensors/typedefs.h"

#if defined __cplusplus
extern "C" {
#endif

// Pins:
// - INT: configurable to different things - not useful to me
// - DRDY: goes low when data is ready

#define LIS3MDL_BUFFER_SIZE 6
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

// typedef struct {
//   float x, y, z;
// } lis3mdl_t;

typedef struct {
  comm_interface_t *comm;
  uint8_t buffer[LIS3MDL_BUFFER_SIZE];
  float sm[12];
  float scale;
} lis3mdl_io_t;

lis3mdl_io_t *lis3mdl_create(interface_t type, uint8_t port, uint8_t addr_cs);
int lis3mdl_init(lis3mdl_io_t *hw, lis3mdl_range_t range, lis3mdl_odr_t odr);
int lis3mdl_read(lis3mdl_io_t *hw, vec3f_t *mag);
bool lis3mdl_reboot(lis3mdl_io_t *hw);
bool lis3mdl_ready(lis3mdl_io_t *hw);

#if defined __cplusplus
}
#endif