////////////////////////////////////////////////
//  The MIT License (MIT)
//  Copyright (c) 2022 Kevin Walchko
//  see LICENSE for full details
////////////////////////////////////////////////
#pragma once

#include "gci_sensors/io.h"
#include "gci_sensors/typedefs.h"

#include <stdbool.h>
#include <stdint.h>

#if defined __cplusplus
extern "C" {
#endif

#define LSM6DSOX_ADDRESS 0x6A
#define LSM6DSOX_ADDRESS_ALT 0x6B
#define LSM6DSOX_BUFFER_SIZE 14

typedef enum : uint8_t {
  LSM6DSOX_ODR_SHUTDOWN = 0x00,
  LSM6DSOX_ODR_104_HZ   = 0x40,
  LSM6DSOX_ODR_208_HZ   = 0x50,
  LSM6DSOX_ODR_416_HZ   = 0x60, // needs XL_HM_MODE = 0
  LSM6DSOX_ODR_833_HZ   = 0x70, // needs XL_HM_MODE = 0
  LSM6DSOX_ODR_1660_HZ  = 0x80, // needs XL_HM_MODE = 0
  LSM6DSOX_ODR_3330_HZ  = 0x90, // needs XL_HM_MODE = 0
  LSM6DSOX_ODR_6660_HZ  = 0xA0  // needs XL_HM_MODE = 0
} lsm6dsox_odr_t;

typedef enum {
  LSM6DSOX_XL_2_G  = 0x00,
  LSM6DSOX_XL_16_G = 0x04, // CTRL8_XL: XL_FS_MODE = 0
  LSM6DSOX_XL_4_G  = 0x08,
  LSM6DSOX_XL_8_G  = 0x0C
} lsm6dsox_xl_range_t;

typedef enum { // CTRL2_G
  LSM6DSOX_G_125_DPS  = 0x02,
  LSM6DSOX_G_250_DPS  = 0x00,
  LSM6DSOX_G_500_DPS  = 0x04,
  LSM6DSOX_G_1000_DPS = 0x08,
  LSM6DSOX_G_2000_DPS = 0x0C
} lsm6dsox_g_range_t;

typedef struct {
  comm_interface_t *comm;
  float g_scale;  // int -> float
  float a_scale;  // int -> float
  float acal[12]; // accel scale/bias
  float gcal[12]; // gyro scale/bias
  uint8_t buff[LSM6DSOX_BUFFER_SIZE];
} lsm6dsox_io_t;

lsm6dsox_io_t *lsm6dsox_create(interface_t type, uint8_t port, uint8_t addr_cs);
int lsm6dsox_init(lsm6dsox_io_t *hw, lsm6dsox_xl_range_t accel_range,
                  lsm6dsox_g_range_t gyro_range, lsm6dsox_odr_t odr);
int lsm6dsox_read(lsm6dsox_io_t *hw, imuf_t *imu);
// int lsm6dsox_read_calibrated(lsm6dsox_io_t *hw, imuf_t *imu);
// void lsm6dsox_calibrate(lsm6dsox_io_t *hw, imuf_t *imu);
// void lsm6dsox_set_cal(lsm6dsox_io_t *hw, float a[12], float g[12]);
// bool lsm6dsox_reboot(lsm6dsox_io_t *hw);
void lsm6dsox_dump(lsm6dsox_io_t *hw);
// bool lsm6dsox_ready(lsm6dsox_io_t *hw);
// int32_t lsm6dsox_available(lsm6dsox_io_t *hw);

#if defined __cplusplus
}
#endif
