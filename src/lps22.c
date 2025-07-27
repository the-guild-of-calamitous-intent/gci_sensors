/**************************************\
 * The MIT License (MIT)
 * Copyright (c) 2022 Kevin Walchko
 * see LICENSE for full details
\**************************************/
/*
References:
- https://www.st.com/resource/en/datasheet/dm00140895.pdf
*/

// #pragma once

#include "gci_sensors/lps22.h"
#include <math.h>
#include <stdio.h>  // printf
#include <stdlib.h> // calloc

// Registers and fields
#define LPS22_WHO_AM_I 0xB1

#define LPS22_CTRL_REG1_BDU_EN (1 << 1)
#define LPS22_CTRL_REG1_LPFP_DISABLE (1 << 2)
#define LPS22_CTRL_REG1_LPFP_DIV_9 (2 << 2)
#define LPS22_CTRL_REG1_LPFP_DIV_20 (3 << 2)
#define LPS22_CTRL_REG2_ONESHOT 0x01
#define LPS22_CTRL_REG2_SWRESET (1 << 2)
#define LPS22_CTRL_REG2_I2C_DIS (1 << 3)
#define LPS22_CTRL_REG2_BOOT (1 << 7)
#define LPS22_IF_ADD_INC 0x10 // auto increment

#define LPS22_CTRL_REG3_INT_S_DRDY 0x00
#define LPS22_CTRL_REG3_DRDY_EN (1 << 2)
#define LPS22_CTRL_REG3_PUSH_PULL 0x00 // (0 << 6)
#define LPS22_CTRL_REG3_OPEN_DRAIN (1 << 6)
#define LPS22_CTRL_REG3_ACTIVE_HI 0x00 // (0 << 7)
#define LPS22_CTRL_REG3_ACTIVE_LO (1 << 7)

#define LPS22_INTERRUPT_CFG 0x0B
#define LPS22_WHO_AM_I_REG 0x0F
#define LPS22_CTRL_REG1 0x10
#define LPS22_CTRL_REG2 0x11
#define LPS22_CTRL_REG3 0x12
// RESERVED                           0x13
// FIFO                               0x14
#define LPS22_REF_P_XL 0x15
#define LPS22_REF_P_L 0x16
#define LPS22_REF_P_H 0x17
#define LPS22_RPDS_L 0x18
#define LPS22_RPDS_H 0x19
#define LPS22_RES_CONF 0x1A
#define LPS22_INT_SOURCE 0x25
#define LPS22_STATUS 0x27
// FIFO RES                           0x26
#define LPS22_PRESSURE_OUT_XL 0x28
#define LPS22_PRESSURE_OUT_L 0x29
#define LPS22_PRESSURE_OUT_H 0x2A
#define LPS22_TEMP_OUT_L 0x2B
#define LPS22_TEMP_OUT_H 0x2C
#define LPS22_LPFP_RES 0x33

lps22_io_t *lps22_spi_init(uint8_t port, pin_t cs, lps22_odr_t ODR) {
  lps22_io_t *hw = (lps22_io_t *)calloc(1, sizeof(lps22_io_t));
  if (hw == NULL) return NULL;

  comm_interface_t *comm = comm_interface_init(port, cs, SPI_INTERFACE);
  if (comm == NULL) return NULL;

  hw->comm = comm;

  uint8_t reg = 0;
  int ok      = comm->read(comm->config, LPS22_WHO_AM_I_REG, &reg, 1);
  if ((reg != LPS22_WHO_AM_I) || (ok < 0)) return NULL;

  // SWRESET - reset regs to default
  reg = LPS22_CTRL_REG2_SWRESET;
  comm->write(comm->config, LPS22_CTRL_REG2, &reg, 1);

  uint8_t regs[3] = {
      ODR | LPS22_CTRL_REG1_LPFP_DIV_9,           // REG 1
      LPS22_CTRL_REG2_I2C_DIS | LPS22_IF_ADD_INC, // REG 2
      LPS22_CTRL_REG3_DRDY_EN                     // REG 3
  };
  comm->write(comm->config, LPS22_CTRL_REG1, regs, 3);

  return hw;
}

lps22_t lps22_read(lps22_io_t *hw) {
  comm_interface_t *comm = hw->comm;
  uint8_t *sensor_data   = hw->sensor_data;

  int ok = comm->read(comm->config, LPS22_PRESSURE_OUT_XL, sensor_data, 5);
  if (ok < 0) {
    hw->ok      = false;
    lps22_t ret = {0.0f, 0.0f};
    return ret;
  }

  int32_t p = (int32_t)sensor_data[0] | ((int32_t)sensor_data[1] << 8) | ((int32_t)sensor_data[2] << 16);
  // 2's complement fix?
  // if(up & 0x00800000) up |= 0xFF000000;
  // int32_t p = (int32_t) up;

  int32_t t = ((int32_t)sensor_data[3]) | ((int32_t)sensor_data[4] << 8);

  lps22_t ret = {
      (float)(p) / 4096.0f, // hPa
      (float)(t) / 100.0f   // C
  };
  hw->ok = true;
  return ret;
}

// lps22_io_t *lps22_spi_init(uint8_t port, pin_t cs, uint8_t ODR) {
//   lps22_io_t *hw = (lps22_io_t *)calloc(1, sizeof(lps22_io_t));
//   if (hw == NULL) return NULL;

//   comm_interface_t *comm = comm_interface_init(port, cs, SPI_INTERFACE);
//   if (comm == NULL) return NULL;

//   hw->comm = comm;

//   // printf("> read whoami\n");
//   uint8_t reg = 0;
//   int ok = comm->read(comm->config, LPS22_WHO_AM_I_REG, &reg, 1);
//   // printf("> ok (read bytes): %d id: %d\n", ok, (int)reg); // 0xB1 == 177
//   if ((reg != LPS22_WHO_AM_I) || (ok < 0)) return NULL;
//   // printf("> CORRECT whoami\n");

//   // SWRESET - reset regs to default
//   reg = LPS22_CTRL_REG2_SWRESET;
//   comm->write(comm->config, LPS22_CTRL_REG2, &reg, 1);

//   // Enable continous, 75Hz, LPF (ODR/9 = 8Hz)
//   reg = ODR;
//   reg |= LPS22_CTRL_REG1_LPFP_DIV_9;
//   comm->write(comm->config, LPS22_CTRL_REG1, &reg, 1);

//   // disable I2C
//   reg = LPS22_CTRL_REG2_I2C_DIS;
//   reg |= LPS22_IF_ADD_INC;
//   comm->write(comm->config, LPS22_CTRL_REG2, &reg, 1);

//   // Enable DRDY on pin when data ready
//   // CTRL_REG3, INT_H_L, defaults to 0 -> active high
//   //            PP_OD, defaults to 0 -> push-pull on the pin
//   reg = LPS22_CTRL_REG3_DRDY_EN;
//   // reg |= LPS22_CTRL_REG3_INT_S_DRDY;
//   // reg |= LPS22_CTRL_REG3_OPEN_DRAIN;
//   comm->write(comm->config, LPS22_CTRL_REG3, &reg, 1);

//   return hw;
// }

// #define LPS22_INT_SOURCE_PH 0x01
// #define LPS22_INT_SOURCE_PL 0x02
// #define LPS22_INT_SOURCE_IA 0x04
// #define LPS22_INT_SOURCE_BOOT_ON 0x80

// #define LPS22_STATUS_P_DA 0x01
// #define LPS22_STATUS_T_DA 0x02
// #define LPS22_STATUS_P_OR 0x10
// #define LPS22_STATUS_T_OR 0x20
