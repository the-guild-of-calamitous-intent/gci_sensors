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

#include <stdint.h>
#include <stdbool.h>
#include "gci_sensors/io.h"

#if defined __cplusplus
extern "C" {
#endif

#define LPS22_ADDRESS 0x5C
#define LPS22_ADDRESS_ALT 0x5D

typedef enum : uint8_t {
  LPS22_ODR_PWR_DOWN = 0,
  LPS22_ODR_1HZ      = (1 << 4),
  LPS22_ODR_10HZ     = (2 << 4),
  LPS22_ODR_25HZ     = (3 << 4),
  LPS22_ODR_50HZ     = (4 << 4),
  LPS22_ODR_75HZ     = (5 << 4)
} lps22_odr_t;

typedef struct {
  float pressure;
  float temperature;
} lps22_t;

typedef struct {
  comm_interface_t *comm;
  uint8_t sensor_data[5];
  bool ok;
} lps22_io_t;

lps22_io_t *lps22_spi_init(uint8_t port, pin_t cs, lps22_odr_t odr);
lps22_t lps22_read(lps22_io_t *hw);

#if defined __cplusplus
}
#endif
