////////////////////////////////////////////////
//  The MIT License (MIT)
//  Copyright (c) 2022 Kevin Walchko
//  see LICENSE for full details
////////////////////////////////////////////////
// https://www.mide.com/air-pressure-at-altitude-calculator

/*

Table 10, datasheet
Use Case   | Mode | Res     | P   | T  | IIR | ODR | Noise RMS [cm] |
---------------------------------------------------------------------
Indoor Nav | Norm | Ultr Hi | x16 | x2 | 4   | 25  | 5
Drone      | Norm | Std res | x8  | x1 | 2   | 50  | 11

Table 23, datasheet
Oversamp | P   | T  | Hz Typ |
------------------------------
Low Pwr  | x2  | x1 | 146 | << can set ODR 100Hz
Std Res  | x4  | x1 | 92  | << max ODR is 50Hz
Hi Res   | x8  | x1 | 53  |
Ultr Hi  | x16 | x2 | 27  |
*/
#pragma once

#include "gci_sensors/io.h"
#include <stdbool.h>
#include <stdint.h>

#if defined __cplusplus
extern "C" {
#endif

#define BMP390_ADDRESS 0x77
#define BMP390_ADDRESS_ALT 0x76
#define BMP390_DATA_LEN 6

typedef enum : uint8_t {
  BMP390_IIR_DISABLE   = 0x00, // 000 0   1 step lag
  BMP390_IIR_COEFF_OFF = 0x00, // 000 0   1 step lag
  BMP390_IIR_COEFF_1   = 0x02, // 001 0  10 step lag
  BMP390_IIR_COEFF_3   = 0x04, // 010 0  20 step lag
  BMP390_IIR_COEFF_7   = 0x06, // 011 0  40 step lag
  BMP390_IIR_COEFF_15  = 0x08, // 100 0
  BMP390_IIR_COEFF_31  = 0x0A, // 101 0
  BMP390_IIR_COEFF_63  = 0x0C, // 110 0
  BMP390_IIR_COEFF_127 = 0x0E  // 111 0
} bmp390_iir_t;

typedef enum : uint8_t {
  BMP390_ODR_200_HZ  = 0x00,
  BMP390_ODR_100_HZ  = 0x01,
  BMP390_ODR_50_HZ   = 0x02,
  BMP390_ODR_25_HZ   = 0x03,
  BMP390_ODR_12_5_HZ = 0x04
} bmp390_odr_t;

typedef struct {
  float par_t1;
  float par_t2;
  float par_t3;

  float par_p1;
  float par_p2;
  float par_p3;
  float par_p4;
  float par_p5;
  float par_p6;
  float par_p7;
  float par_p8;
  float par_p9;
  float par_p10;
  float par_p11;

  float t_lin; // was int64_t??
} bmp3_reg_calib_data;

typedef struct {
  float pressure;
  float temperature;
} bmp390_t;

typedef struct {
  comm_interface_t *comm;
  bmp3_reg_calib_data calib;
  uint8_t buffer[BMP390_DATA_LEN];
  bool ok;
} bmp390_io_t;

bmp390_io_t *bmp390_i2c_init(uint8_t port, uint8_t addr, bmp390_odr_t odr,
                             bmp390_iir_t iir);
bmp390_io_t *bmp390_spi_init(uint8_t port, pin_t cs, bmp390_odr_t odr,
                             bmp390_iir_t iir);

const bmp390_t bmp390_read(bmp390_io_t *hw);
// bool bmp390_ready(bmp390_io_t *hw);

#if defined __cplusplus
}
#endif

// typedef struct {
//   int32_t pressure, temperature;
//   bool ok;
// } bmp390_raw_t;

// enum bmp_error : uint8_t {
//   NO_ERROR,
//   ERROR_WHOAMI,
//   ERROR_RESET,
//   ERROR_CAL_DATA,
//   ERROR_ODR,
//   ERROR_BMP390_IIR,
//   ERROR_INT_PIN,
//   ERROR_PWR_MODE
// };

// typedef struct {
//   i2c_inst_t *i2c;
//   uint8_t addr;
//   bmp3_reg_calib_data calib;
//   uint8_t buffer[BMP390_DATA_LEN];
// } bmp390_i2c_t;

// #define BMP390_IIR_DISABLE   = 0x00; // 000 0   1 step lag
// #define BMP390_IIR_COEFF_OFF = 0x00; // 000 0   1 step lag
// #define BMP390_IIR_COEFF_1   = 0x02; // 001 0  10 step lag
// #define BMP390_IIR_COEFF_3   = 0x04; // 010 0  20 step lag
// #define BMP390_IIR_COEFF_7   = 0x06; // 011 0  40 step lag
// #define BMP390_IIR_COEFF_15  = 0x08; // 100 0
// #define BMP390_IIR_COEFF_31  = 0x0A; // 101 0
// #define BMP390_IIR_COEFF_63  = 0x0C; // 110 0
// #define BMP390_IIR_COEFF_127 = 0x0E; // 111 0

// // datasheet Table 45, p 38 oversample hz
// #define BMP390_ODR_200_HZ   = 0x00;
// #define BMP390_ODR_100_HZ   = 0x01;
// #define BMP390_ODR_50_HZ    = 0x02;
// #define BMP390_ODR_25_HZ    = 0x03;
// #define BMP390_ODR_12_5_HZ  = 0x04;