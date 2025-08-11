/**************************************\
 * The MIT License (MIT)
 * Copyright (c) 2022 Kevin Walchko
 * see LICENSE for full details
\**************************************/
/*
References:
- https://www.st.com/resource/en/datasheet/dm00140895.pdf
*/

#include <math.h>
#include <stdio.h>  // printf
#include <stdlib.h> // calloc

#include "gci_sensors/lps22.h"
#include "gci_sensors/io.h" // cov_bb2f


// Registers and fields
#define LPS22_WHO_AM_I 0xB1

#define LPS22_CTRL_REG1_BDU_EN       (1 << 1)
#define LPS22_CTRL_REG1_LPFP_DISABLE (1 << 2)
#define LPS22_CTRL_REG1_LPFP_DIV_9   (2 << 2)
#define LPS22_CTRL_REG1_LPFP_DIV_20  (3 << 2)
#define LPS22_CTRL_REG2_ONESHOT      0x01
#define LPS22_CTRL_REG2_SWRESET      (1 << 2)
#define LPS22_CTRL_REG2_I2C_DIS      (1 << 3)
#define LPS22_CTRL_REG2_BOOT         (1 << 7)
#define LPS22_IF_ADD_INC             0x10 // auto increment

#define LPS22_CTRL_REG3_INT_S_DRDY 0x00
#define LPS22_CTRL_REG3_DRDY_EN    (1 << 2)
#define LPS22_CTRL_REG3_PUSH_PULL  0x00 // (0 << 6)
#define LPS22_CTRL_REG3_OPEN_DRAIN (1 << 6)
#define LPS22_CTRL_REG3_ACTIVE_HI  0x00 // (0 << 7)
#define LPS22_CTRL_REG3_ACTIVE_LO  (1 << 7)

#define LPS22_INTERRUPT_CFG 0x0B
#define LPS22_WHO_AM_I_REG  0x0F
#define LPS22_CTRL_REG1     0x10
#define LPS22_CTRL_REG2     0x11
#define LPS22_CTRL_REG3     0x12
// RESERVED                           0x13
// FIFO                               0x14
#define LPS22_REF_P_XL   0x15
#define LPS22_REF_P_L    0x16
#define LPS22_REF_P_H    0x17
#define LPS22_RPDS_L     0x18
#define LPS22_RPDS_H     0x19
#define LPS22_RES_CONF   0x1A
#define LPS22_INT_SOURCE 0x25
#define LPS22_STATUS     0x27
// FIFO RES                           0x26
#define LPS22_PRESSURE_OUT_XL 0x28
#define LPS22_PRESSURE_OUT_L  0x29
#define LPS22_PRESSURE_OUT_H  0x2A
#define LPS22_TEMP_OUT_L      0x2B
#define LPS22_TEMP_OUT_H      0x2C
#define LPS22_LPFP_RES        0x33


lps22_io_t *lps22_create(uint8_t port, pin_t cs) {
  lps22_io_t *hw = (lps22_io_t *)calloc(1, sizeof(lps22_io_t));
  if (hw == NULL) return NULL;

  comm_interface_t *comm = comm_interface_init(port, cs, SPI_INTERFACE);
  if (comm == NULL) {
    free(hw);
    return NULL;
  }

  hw->comm = comm;
  return hw;
}

int lps22_spi_init(lps22_io_t *hw, lps22_odr_t ODR) {
  if (hw == NULL) return GCIS_ERROR_IO_NULL;
  comm_interface_t *comm = hw->comm;

  uint8_t reg;
  if(comm->read(comm->config, LPS22_WHO_AM_I_REG, &reg, 1) < 0) return -1;
  if (reg != LPS22_WHO_AM_I) return GCIS_ERROR_WHOAMI;

  // SWRESET - reset regs to default
  reg = LPS22_CTRL_REG2_SWRESET;
  comm->write(comm->config, LPS22_CTRL_REG2, &reg, 1);
  sleep_ms(10);

  uint8_t regs[3] = {
      ODR | LPS22_CTRL_REG1_LPFP_DIV_9,           // REG 1
      LPS22_CTRL_REG2_I2C_DIS | LPS22_IF_ADD_INC, // REG 2
      LPS22_CTRL_REG3_DRDY_EN                     // REG 3
  };
  if (comm->write(comm->config, LPS22_CTRL_REG1, regs, sizeof(regs)) < 0) return -1;

  return 0;
}

int lps22_read(lps22_io_t *hw, pt_t *ret) {
  if (hw == NULL) return GCIS_ERROR_IO_NULL;
  comm_interface_t *comm = hw->comm;
  uint8_t *buff   = hw->sensor_data;

  if (comm->read(comm->config, LPS22_PRESSURE_OUT_XL, buff, LPS22_DATA_LEN) < 0) {
    return -1;
  }

  ret->pressure = cov_bbb2f(buff[0],buff[1],buff[2]) / 4096.0f; // hPa
  ret->temperature = cov_bb2f(buff[3],buff[4]) / 100.0f;   // C
  
  return 0;
}
