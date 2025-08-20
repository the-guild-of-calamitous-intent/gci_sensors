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

#define LIS2MDL_ADDRESS 0x1E
#define LIS2MDL_BUFFER_SIZE 8 // 6 mag, 2 temp

typedef enum {
  LIS2MDL_ODR_10  = 0,
  LIS2MDL_ODR_20  = (1 << 2),
  LIS2MDL_ODR_50  = (2 << 2),
  LIS2MDL_ODR_100 = (3 << 3)
} lis2mdl_odr_t;

typedef struct {
  comm_interface_t *comm;
  uint8_t buffer[LIS2MDL_BUFFER_SIZE];
  float mcal[12];
  lis2mdl_odr_t odr;
} lis2mdl_io_t;

lis2mdl_io_t *lis2mdl_create(interface_t type, uint8_t port, uint8_t addr_cs);
int lis2mdl_init(lis2mdl_io_t *hw);
int lis2mdl_read(lis2mdl_io_t *hw, vec3f_t *mag);
int lis2mdl_dump(lis2mdl_io_t *hw);
int lis2mdl_calibrate(lis2mdl_io_t *hw, uint16_t num_pts);

#if defined __cplusplus
}
#endif