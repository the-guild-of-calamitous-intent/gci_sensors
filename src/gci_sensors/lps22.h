////////////////////////////////////////////////
//  The MIT License (MIT)
//  Copyright (c) 2022 Kevin Walchko
//  see LICENSE for full details
////////////////////////////////////////////////
/*
References:
- https://www.st.com/resource/en/datasheet/dm00140895.pdf
*/

#pragma once

#include "gci_sensors/io.h"
#include "gci_sensors/typedefs.h"

#include <stdbool.h>
#include <stdint.h>

#if defined __cplusplus
extern "C" {
#endif

#define LPS22_ADDRESS 0x5C
#define LPS22_ADDRESS_ALT 0x5D
#define LPS22_DATA_LEN 5

typedef enum : uint8_t {
  LPS22_ODR_PWR_DOWN = 0,
  LPS22_ODR_1HZ      = (1 << 4),
  LPS22_ODR_10HZ     = (2 << 4),
  LPS22_ODR_25HZ     = (3 << 4),
  LPS22_ODR_50HZ     = (4 << 4),
  LPS22_ODR_75HZ     = (5 << 4)
} lps22_odr_t;

typedef struct {
  comm_interface_t *comm;
  uint8_t sensor_data[LPS22_DATA_LEN];
} lps22_io_t;

lps22_io_t *lps22_create(uint8_t port, pin_t cs);
int lps22_spi_init(lps22_io_t *hw, lps22_odr_t odr);
int lps22_read(lps22_io_t *hw, pt_t *ret);

#if defined __cplusplus
}
#endif
