/**************************************\
 * The MIT License (MIT)
 * Copyright (c) 2022 Kevin Walchko
 * see LICENSE for full details
\**************************************/
#pragma once

// #include "sensor.hpp"
#include <stdbool.h>
#include <stdint.h>
#include <string.h> // memcpy
#include <picolibc.h>

#if defined __cplusplus
extern "C" {
#endif

constexpr int LSM6DSOX_ADDRESS     = 0x6A;
constexpr int LSM6DSOX_ADDRESS_ALT = 0x6B;

constexpr uint8_t REG_FIFO_CTRL4   = 0x0A;
constexpr uint8_t REG_INT1_CTRL    = 0x0D;
constexpr uint8_t REG_INT2_CTRL    = 0x0E;
constexpr uint8_t REG_WHO_AM_I     = 0x0F;
constexpr uint8_t REG_CTRL1_XL     = 0x10; // Accel settings
constexpr uint8_t REG_CTRL2_G      = 0x11; // Gyro settings hz and dps
constexpr uint8_t REG_CTRL3_C      = 0x12; // interrupt stuff
constexpr uint8_t REG_CTRL4_C      = 0x13;
constexpr uint8_t REG_CTRL5_C      = 0x14;
constexpr uint8_t REG_CTRL6_C      = 0x15; // Accel perf mode and Gyro LPF
constexpr uint8_t REG_CTRL7_G      = 0x16; // Gyro filtering
constexpr uint8_t REG_CTRL8_XL     = 0x17; // Accel filtering
constexpr uint8_t REG_CTRL9_XL     = 0x18; // Accel filtering
constexpr uint8_t REG_CTRL10_C     = 0x19; // tiimestamp

constexpr uint8_t REG_STATUS       = 0x1E;

constexpr uint8_t REG_OUT_TEMP_L   = 0x20; // termperature
constexpr uint8_t REG_OUTX_L_G     = 0x22; // gyro
constexpr uint8_t REG_OUTX_L_A     = 0x28; // accel
constexpr uint8_t REG_TIMESTAMP0   = 0x40; // 4B timestamp

// Betaflight values
// Accel:
//   833Hz ODR, 16G, use LPF1 output
//   high performance mode
// Gyro:
//   6664Hz ODR, 2000dps
//   LPF1 cutoff 335.5Hz
//
// latch LSB/MSB at reads, pins high, pins push/pull, auto-increment reads
// disable i3c interface

// The accelerometer/gyroscope data rate
enum ODR : uint8_t {
  RATE_SHUTDOWN = 0x00,
  RATE_104_HZ   = 0x40,
  RATE_208_HZ   = 0x50,
  RATE_416_HZ   = 0x60,
  RATE_833_HZ   = 0x70,
  RATE_1_66_KHZ = 0x80,
  RATE_3_33_KHZ = 0x90,
  RATE_6_66_KHZ = 0xA0,
};

// The accelerometer data range
enum accel_range : uint8_t {
  ACCEL_RANGE_2_G  = (0x00 << 2),
  ACCEL_RANGE_16_G = (0x01 << 2),
  ACCEL_RANGE_4_G  = (0x02 << 2),
  ACCEL_RANGE_8_G  = (0x03 << 2)
};

// The gyro data range
enum gyro_range : uint8_t {
  GYRO_RANGE_125_DPS  = (0x01 << 1),
  GYRO_RANGE_250_DPS  = (0x00 << 1),
  GYRO_RANGE_500_DPS  = (0x02 << 1),
  GYRO_RANGE_1000_DPS = (0x04 << 1),
  GYRO_RANGE_2000_DPS = (0x06 << 1)
};

constexpr uint8_t WHO_AM_I     = 0x6C; // 01101100
constexpr uint8_t IF_INC       = 0x04;
constexpr uint8_t XL_FS_MODE   = 0x02; // new mode, default 0
constexpr uint8_t TIMESTAMP_EN = 0x20;
constexpr uint8_t LPF2_XL_EN   = 0x02;  // output from LPF2 second
                                        // filtering stage selected
                                        // (not default)
constexpr uint8_t INT_DRDY_XL   = 0x01; // accel data ready INT pin
constexpr uint8_t INT_DRDY_G    = 0x02; // gyro data ready INT pin
constexpr uint8_t INT_DRDY_TEMP = 0x04; // temperature data ready INT pin
// constexpr uint8_t H_LACTIVE    = 0x20; // 0-high, 1-low - don't set this

constexpr float LSM6DSOX_TIMESTEP_RES = 25e-6f;
constexpr float TEMP_SCALE            = 1.0f / 256.0f;

// TDA: temperature
// GDA: gyro
// XLDA: accel
//                             4   2    1
// STATUS_REG: MSB 0 0 0 0 0 TDA GDA XLDA LSB
enum sox_available_t : uint8_t {
  SOX_NONE            = 0,
  SOX_ACCEL           = 1,
  SOX_GYRO            = 2,
  SOX_ACCEL_GYRO      = 3,
  SOX_TEMP            = 4,
  SOX_ACCEL_GYRO_TEMP = 7,
};

enum sox_error : uint8_t {
  ERROR_NONE             = 0,
  ERROR_WHOAMI           = 1,
  ERROR_GYRO_RANGE       = 2,
  ERROR_ACCEL_RANGE      = 3,
  ERROR_ENABLE_INT_ACCEL = 9,
  ERROR_ENABLE_INT_GYRO  = 10,
  ERROR_CTRL1_XL         = 11,
  ERROR_CTRL2_G          = 12,
  ERROR_CTRL3_C          = 13,
  ERROR_CTRL4_C          = 14,
  ERROR_CTRL5_C          = 15,
  ERROR_CTRL6_C          = 16,
  ERROR_CTRL7_G          = 17,
  ERROR_CTRL8_XL         = 18,
  ERROR_CTRL9_XL         = 19,
  ERROR_CTRL10_C         = 20
};

// constexpr int MAX_CHECK = 10;

typedef struct {
  vec_t a, g;
  float temperature;
  bool ok;
  uint64_t timestamp_us;
} lsm6dsox_t;

typedef union {
  struct regs_t {
    int16_t temperature; // 2b, -40C to 80C
    vec_raw_t g;         // 2*3 = 6b
    vec_raw_t a;         // 2*3 = 6b
  } regs;                // 14b
  uint32_t timestamp;
  uint8_t b[14];
} block_t;

typedef struct {
  i2c_inst_t *i2c;
  uint8_t addr;
  float g_scale, a_scale;

  // accel scale/bias
  float acal[12]; // = {1., 0., 0., 0., 0., 1., 0., 0., 0., 0., 1., 0.};
  // gyro scale/bias
  float gcal[12]; // = {1., 0., 0., 0., 0., 1., 0., 0., 0., 0., 1., 0.};

  // union block_t {
  //   struct regs_t {
  //     int16_t temperature; // 2b, -40C to 80C
  //     vec_raw_t g;         // 2*3 = 6b
  //     vec_raw_t a;         // 2*3 = 6b
  //   } regs;                // 14b
  //   uint32_t timestamp;
  //   uint8_t b[14];
  // } block;
  block_t block;
} lsm6dsox_i2c_t;

// ACCEL_RANGE_4_G, GYRO_RANGE_2000_DPS, RATE_104_HZ
lsm6dsox_i2c_t *lsm6dsox_i2c_init(uint8_t port, uint8_t addr,
                                  uint8_t accel_range, uint8_t gyro_range,
                                  uint8_t odr);
lsm6dsox_i2c_t *lsm6dsox_i2c_init_defaults(uint8_t port);
bool lsm6dsox_reboot(lsm6dsox_i2c_t *hw);
lsm6dsox_t lsm6dsox_read(lsm6dsox_i2c_t *hw);
bool lsm6dsox_ready(lsm6dsox_i2c_t *hw);
int32_t lsm6dsox_available(lsm6dsox_i2c_t *hw);

// sox_available_t available();

#if defined __cplusplus
}
#endif
