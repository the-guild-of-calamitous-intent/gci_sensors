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

typedef enum {
  LSM6DSOX_ERROR_NONE         = 0,
  LSM6DSOX_ERROR_WHOAMI       = -1,
  LSM6DSOX_ERROR_G            = -2,
  LSM6DSOX_ERROR_XL           = -4,
  LSM6DSOX_ERROR_ODR          = -8,
  LSM6DSOX_ERROR_COMM_IF_FAIL = -16,
  LSM6DSOX_ERROR_INIT_BLK0    = -32,
  LSM6DSOX_ERROR_INIT_BLK1    = -64,
  LSM6DSOX_ERROR_READ         = -128,
  LSM6DSOX_ERROR_WRITE        = -256
} sox6dsox_error_t;

// temperature pg 9 datasheet:
// - accel: +/- 0.1 mg/C
// - gyro: +/- 0.01 dps/C
// So I can't find the details of this, but I assume
// these deltas are from 25C
typedef struct {
  vec3f_t a;
  vec3f_t g;
  float temperature; // do I use this?
} lsm6dsox_t;

typedef struct {
  comm_interface_t *comm;
  float g_scale;  // int -> float
  float a_scale;  // int -> float
  float acal[12]; // accel scale/bias
  float gcal[12]; // gyro scale/bias
  // block_t block;
  uint8_t buff[LSM6DSOX_BUFFER_SIZE];
  bool ok;
  // change to err_init ... only place it is useful: whoami, gyrorange,
  // accelrange, good
  sox6dsox_error_t errnum; // 0: ok, <0: error
} lsm6dsox_io_t;

lsm6dsox_io_t *lsm6dsox_i2c_init(uint8_t port, uint8_t addr,
                                 lsm6dsox_xl_range_t accel_range,
                                 lsm6dsox_g_range_t gyro_range,
                                 lsm6dsox_odr_t odr);

bool lsm6dsox_reboot(lsm6dsox_io_t *hw);
lsm6dsox_t lsm6dsox_read(lsm6dsox_io_t *hw);
lsm6dsox_t lsm6dsox_read_calibrated(lsm6dsox_io_t *hw);
void lsm6dsox_dump(lsm6dsox_io_t *hw);
lsm6dsox_t lsm6dsox_calibrate(lsm6dsox_io_t *hw, lsm6dsox_t data);
void lsm6dsox_set_cal(lsm6dsox_io_t *hw, float a[12], float g[12]);
bool lsm6dsox_ready(lsm6dsox_io_t *hw);
int32_t lsm6dsox_available(lsm6dsox_io_t *hw);

// lsm6dsox_io_t *lsm6dsox_spi_init_alt(uint8_t port, pin_t cs,
//                                      uint8_t accel_range, uint8_t gyro_range,
//                                      uint8_t odr);
lsm6dsox_io_t *lsm6dsox_spi_init(uint8_t port, pin_t cs,
                                 lsm6dsox_xl_range_t accel_range,
                                 lsm6dsox_g_range_t gyro_range,
                                 lsm6dsox_odr_t odr);
lsm6dsox_io_t *lsm6dsox_spi_int_init(uint8_t port, pin_t cs, pin_t interrupt,
                                     void (*func)(void), lsm6dsox_odr_t odr);

#if defined __cplusplus
}
#endif

// typedef union {
//   // struct regs_t {
//   //   int16_t temperature; // 2b, -40C to 80C
//   //   vec3s_t g;         // 2*3 = 6b
//   //   vec3s_t a;         // 2*3 = 6b
//   // } regs;                // 14b
//   struct {
//     int16_t temperature; // 2b, -40C to 80C
//     vec3s_t g;           // 2*3 = 6b
//     vec3s_t a;           // 2*3 = 6b
//   }; // 14b
//   // uint32_t timestamp;
//   uint8_t b[LSM6DSOX_BUFFER_SIZE];
// } block_t;

// The accelerometer/gyroscope data rate
// constexpr uint8_t ODR_SHUTDOWN = 0x00;
// constexpr uint8_t ODR_104_HZ   = 0x40;
// constexpr uint8_t ODR_208_HZ   = 0x50;
// constexpr uint8_t ODR_416_HZ   = 0x60;
// constexpr uint8_t ODR_833_HZ   = 0x70;
// constexpr uint8_t ODR_1660_HZ  = 0x80;
// constexpr uint8_t ODR_3330_HZ  = 0x90;
// constexpr uint8_t ODR_6660_HZ  = 0xA0;

// The accelerometer data range
// constexpr uint8_t XL_2_G  = (0x00 << 2);
// constexpr uint8_t XL_16_G = (0x01 << 2); // CTRL8_XL: XL_FS_MODE = 0
// constexpr uint8_t XL_4_G  = (0x02 << 2);
// constexpr uint8_t XL_8_G  = (0x03 << 2);

// The gyro data range
// constexpr uint8_t G_125_DPS  = (0x01 << 1);
// constexpr uint8_t G_250_DPS  = (0x00 << 1);
// constexpr uint8_t G_500_DPS  = (0x02 << 1);
// constexpr uint8_t G_1000_DPS = (0x04 << 1);
// constexpr uint8_t G_2000_DPS = (0x06 << 1);

// #define LSM6DSOX_INT1_CRTL 0x02 // DRDY Gyro
// #define LSM6DSOX_INT2_CRTL 0x00 // disable INT2

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

// // The accelerometer/gyroscope data rate
// enum ODR : uint8_t {
//   ODR_SHUTDOWN = 0x00,
//   ODR_104_HZ   = 0x40,
//   ODR_208_HZ   = 0x50,
//   ODR_416_HZ   = 0x60,
//   ODR_833_HZ   = 0x70,
//   ODR_1_66_KHZ = 0x80,
//   ODR_3_33_KHZ = 0x90,
//   ODR_6_66_KHZ = 0xA0,
// };

// // The accelerometer data range
// enum accel_range : uint8_t {
//   XL_2_G  = (0x00 << 2),
//   XL_16_G = (0x01 << 2),
//   XL_4_G  = (0x02 << 2),
//   XL_8_G  = (0x03 << 2)
// };

// // The gyro data range
// enum gyro_range : uint8_t {
//   G_125_DPS  = (0x01 << 1),
//   G_250_DPS  = (0x00 << 1),
//   G_500_DPS  = (0x02 << 1),
//   G_1000_DPS = (0x04 << 1),
//   G_2000_DPS = (0x06 << 1)
// };

// TDA: temperature
// GDA: gyro
// XLDA: accel
//                             4   2    1
// STATUS_REG: MSB 0 0 0 0 0 TDA GDA XLDA LSB
// enum sox_available_t : uint8_t {
//   SOX_NONE            = 0,
//   SOX_ACCEL           = 1,
//   SOX_GYRO            = 2,
//   SOX_ACCEL_GYRO      = 3,
//   SOX_TEMP            = 4,
//   SOX_ACCEL_GYRO_TEMP = 7,
// };

// enum sox_error : uint8_t {
//   ERROR_NONE             = 0,
//   ERROR_WHOAMI           = 1,
//   ERROR_G       = 2,
//   ERROR_XL      = 3,
//   ERROR_ENABLE_INT_ACCEL = 9,
//   ERROR_ENABLE_INT_GYRO  = 10,
//   ERROR_CTRL1_XL         = 11,
//   ERROR_CTRL2_G          = 12,
//   ERROR_CTRL3_C          = 13,
//   ERROR_CTRL4_C          = 14,
//   ERROR_CTRL5_C          = 15,
//   ERROR_CTRL6_C          = 16,
//   ERROR_CTRL7_G          = 17,
//   ERROR_CTRL8_XL         = 18,
//   ERROR_CTRL9_XL         = 19,
//   ERROR_CTRL10_C         = 20
// };

// constexpr int MAX_CHECK = 10;

// typedef struct {
//   i2c_inst_t *i2c;
//   uint8_t addr;
//   float g_scale;  // int -> float
//   float a_scale;  // int -> float
//   float acal[12]; // accel scale/bias
//   float gcal[12]; // gyro scale/bias
//   block_t block;
// } lsm6dsox_i2c_t;

// typedef struct {
//   spi_inst_t *spi;
//   pin_t miso, mosi, sck, cs;
//   // uint8_t addr;
//   float g_scale;  // int -> float
//   float a_scale;  // int -> float
//   float acal[12]; // accel scale/bias
//   float gcal[12]; // gyro scale/bias
//   block_t block;
// }

// XL_4_G, G_2000_DPS, ODR_104_HZ
// lsm6dsox_i2c_t *lsm6dsox_i2c_init(uint8_t port, uint8_t addr,
//                                   uint8_t accel_range, uint8_t gyro_range,
//                                   uint8_t odr);
// lsm6dsox_i2c_t *lsm6dsox_i2c_init_defaults(uint8_t port);
// bool lsm6dsox_reboot(lsm6dsox_i2c_t *hw);
// lsm6dsox_t lsm6dsox_read(lsm6dsox_i2c_t *hw);
// bool lsm6dsox_ready(lsm6dsox_i2c_t *hw);
// int32_t lsm6dsox_available(lsm6dsox_i2c_t *hw);

// sox_available_t available();

// lsm6dsox_spi_t *lsm6dsox_spi_init_defaults(uint8_t port);