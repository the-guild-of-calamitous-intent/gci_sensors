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

// #include <math.h>
#include "io.h"
#include <stdint.h>
// #include "sensor.hpp"

#if defined __cplusplus
extern "C" {
#endif

// #if defined(__USE_SENSOR_LPS22HB__)

// 10 MHz max SPI frequency
// constexpr uint32_t LPS22_MAX_SPI_CLK_HZ = 10000000;

// Registers and fields
#define LPS22_ODR_PWR_DOWN 0
#define LPS22_ODR_1HZ (1 << 4)
#define LPS22_ODR_10HZ (2 << 4)
#define LPS22_ODR_25HZ (3 << 4)
#define LPS22_ODR_50HZ (4 << 4)
#define LPS22_ODR_75HZ (5 << 4)

typedef struct {
  // int32_t p, t;
  float pressure, temperature;
} lps22_t;

typedef struct {
  comm_interface_t *comm;
  uint8_t sensor_data[5];
} lps22_io_t;

lps22_io_t *lps22_spi_init(uint8_t port, pin_t cs, uint8_t odr);
lps22_t lps22_read(lps22_io_t *hw);

#if defined __cplusplus
}
#endif