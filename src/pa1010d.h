/**************************************\
 * The MIT License (MIT)
 * Copyright (c) 2022 Kevin Walchko
 * see LICENSE for full details
\**************************************/
#pragma once

#include <picolibc.h>
#include <stdbool.h>
#include <stdint.h>

#if defined __cplusplus
extern "C" {
#endif

constexpr uint8_t PA1010D_ADDRESS = 0x10;
constexpr size_t I2C_BUFFER_SIZE  = 32;

typedef struct {
  i2c_inst_t *i2c;
  uint8_t addr;
  uint8_t buffer[I2C_BUFFER_SIZE];
} pa1010d_i2c_t;

pa1010d_i2c_t *pa1010d_i2c_init(uint32_t port, uint8_t addr);
int32_t pa1010d_write(pa1010d_i2c_t *hw, const uint8_t *command,
                      size_t cmd_size);
uint32_t pa1010d_read(pa1010d_i2c_t *hw, char buff[], const size_t buff_size);

#if defined __cplusplus
}
#endif