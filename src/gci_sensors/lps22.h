/**************************************\
 * The MIT License (MIT)
 * Copyright (c) 2022 Kevin Walchko
 * see LICENSE for full details
\**************************************/
/*
References:
- https://www.st.com/resource/en/datasheet/dm00140895.pdf
*/

#pragma once

#include "gci_sensors/io.h"
#include <stdint.h>

#if defined __cplusplus
extern "C" {
#endif

// Registers and fields
// CTRL_REG1 (10)
#define LPS22_ODR_PWR_DOWN 0
#define LPS22_ODR_1HZ (1 << 4)
#define LPS22_ODR_10HZ (2 << 4)
#define LPS22_ODR_25HZ (3 << 4)
#define LPS22_ODR_50HZ (4 << 4)
#define LPS22_ODR_75HZ (5 << 4)

// #define LPS22_EN_LPFP (1 << 3) // enable LPF on pressure
// #define LPS22_LPFP_CFG (1 << 2) // 0 - ODR/9, 1 - ODR/20
// #define LPS22_BDU 0x02 // output not updated until previous read, don't do
// #define LPS22_SIM 0x01 // 0 - SPI 4wire, 1 - SPI 3wire

// // CTRL_REG2 (11)
// #define LPS22_IF_ADD_INC 0x10 // auto increment
// #define LPS22_I2C_DIS 0x80 // disable I2C

// // CTRL_REG3 (12)
// #define LPS22_INT_DRDY 0x04 // data ready sent to INT pin

// typedef enum {
//   LPS22_OK,
//   LPS22_ERROR_WH
// } lps22_errors_t;

typedef struct {
  float pressure;
  float temperature;
} lps22_t;

typedef struct {
  comm_interface_t *comm;
  uint8_t sensor_data[5];
  // int errflag;
  bool ok;
} lps22_io_t;

lps22_io_t *lps22_spi_init(uint8_t port, pin_t cs, uint8_t odr);
lps22_t lps22_read(lps22_io_t *hw);

#if defined __cplusplus
}
#endif