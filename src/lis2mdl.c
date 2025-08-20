#include <stdio.h>
#include <string.h>

#include "gci_sensors/lis2mdl.h"

// LIS2MDL register addresses
#define LIS3MDL_OFFSET_REG 0x45
#define LIS2MDL_WHOAMI_REG 0x4F
#define LIS2MDL_CFG_REG_A  0x60
// #define LIS2MDL_CFG_REG_B  0x61
// #define LIS2MDL_CFG_REG_C  0x62
#define LIS2MDL_STATUS_REG 0x67
#define LIS2MDL_OUTX_L_REG 0x68
// #define LIS2MDL_OUTX_H_REG 0x69
// #define LIS2MDL_OUTY_L_REG 0x6A
// #define LIS2MDL_OUTY_H_REG 0x6B
// #define LIS2MDL_OUTZ_L_REG 0x6C
// #define LIS2MDL_OUTZ_H_REG 0x6D

// LIS2MDL configuration values
#define LIS2MDL_WHOAMI 0x40
// CFG_REG_A -----------------
#define COMP_TEMP_EN 0x80
#define SOFT_RESET   0x20
#define CONT_MODE_EN 0x00
// CFG_REG_B -----------------
#define LPF_EN 0x01 // LFP BW, off: ODR/2 on: ODR/4
// CFG_REG_C -----------------
#define SPI4_EN 0x04
#define BDU_EN  0x10
#define I2C_DIS 0x20

// Default
// --------------------
// CFG_REG_A  0x03  Idle mode, 10Hz, Hi-Res, temp comp disabled
// CFG_REG_B  0x00  LPF disabled
// CFG_REG_C  0x00  I2C enabled, BDU off, SPI-4W (SDO) disabled

// static
// int fail(lis2mdl_io_t *hw) {
//   if (hw->comm != NULL) comm_interface_free(hw->comm);
//   if (hw != NULL) free(hw);
//   printf("*** fail ***\n");
//   return -1;
// }

#define SPI_ACTIVATE   0
#define SPI_DEACTIVATE 1

// THIS WORKS!!!!!!!!!!!!!!!!
// static
// int lis2mdl_spi_read(void *config, uint8_t reg, uint8_t *data, size_t len) {
//   spi_config_t *cfg = (spi_config_t *)config;
//   int ret;
//   reg |= 0x80; // set bit 7, read bit for SPI

//   // uint8_t buff[len+1];
//   // memset(data, 0, len);
//   // buff[0] = reg | 0x80;
//   // data[0] = reg | 0x80;
//   // uint8_t buff[2] = {}

//   gpio_put(cfg->cs_pin, SPI_ACTIVATE); // CS low
//   // ret = spi_write_read_blocking(cfg->spi, data, data, len);
//   ret = spi_write_blocking(cfg->spi, &reg, 1);
//   printf(">> spi_read_blocking reg ret: %d\n", ret);
//   // sleep_us(1);
//   ret = spi_read_blocking(cfg->spi, 0x00, data, len);
//   printf(">> spi_read_blocking data ret: %d\n", ret);
//   gpio_put(cfg->cs_pin, SPI_DEACTIVATE); // CS high
//   // memcpy(data, buff, len);

//   // printf(">> spi_read reg: 0x%02X\n", reg);
//   for (int i=0; i< len; ++i) printf(">> spi_read data: 0x%02X\n", data[i]);
//   return ret;
// }

lis2mdl_io_t *lis2mdl_create(interface_t type, uint8_t port, uint8_t addr_cs) {
  // printf(">> Create LIS2MDL start\n");
  lis2mdl_io_t *hw = (lis2mdl_io_t *)calloc(1, sizeof(lis2mdl_io_t));
  if (hw == NULL) return NULL;

  comm_interface_t *comm = comm_interface_init(port, addr_cs, type);
  if (comm == NULL) {
    free(hw);
    return NULL;
  }

  hw->comm = comm;
  hw->odr = LIS2MDL_ODR_50;

  return hw;
}

int lis2mdl_init(lis2mdl_io_t *hw) {
  if (hw == NULL) return GCIS_ERROR_IO_NULL;

  const comm_interface_t *comm = hw->comm;
  const lis2mdl_odr_t odr = hw->odr;

  float sm[12] = {1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0};
  memcpy(hw->sm, sm, 12 * sizeof(float));


  //
  // FUCK, FUCK, FUCK!!!!!!!!!!!!!!!!
  // OK, SDO is disabled until I enable 4wire SPI, so
  // you CANNOT READ WHOAMI until you SET IT FIRST!!!
  //
  const uint8_t spi_en = (comm->type == SPI_INTERFACE) ? SPI4_EN | I2C_DIS : 0x00;
  uint8_t config[3]    = {
      CONT_MODE_EN | odr | COMP_TEMP_EN, // CFG_REG_A
      LPF_EN,                            // CFG_REG_B
      spi_en | BDU_EN,                   // CFG_REG_C
  };
  // for (int i=0; i < 3; ++i) printf(">> REG[0x%02X]: 0x%02X\n", 0x60 + i, config[i]);

  if (comm->write(comm->config, LIS2MDL_CFG_REG_A, config, sizeof(config)) < 0) return -1;

  uint8_t id;
  if (comm->read(comm->config, LIS2MDL_WHOAMI_REG, &id, 1) < 0) return -1;
  if (id != LIS2MDL_WHOAMI) {
    // printf("Error: WHO_AM_I 0x%02X, expected 0x%02X\n", id, LIS2MDL_WHOAMI);
    return GCIS_ERROR_WHOAMI;
  }
  // printf("LIS2MDL WHO_AM_I verified: 0x%02X\n", id);

  return 0;
}

int lis2mdl_dump(lis2mdl_io_t *hw) {
  if (hw == NULL) return GCIS_ERROR_IO_NULL;
  const comm_interface_t *comm = hw->comm;
  uint8_t config[3];

  if (comm->read(comm->config, LIS2MDL_CFG_REG_A, config, sizeof(config)) < 0) return -1;
  for (int i = 0; i < sizeof(config); ++i)
    printf(">> REG[0x%02X]: 0x%02X\n", 0x60 + i, config[i]);
  return 0;
}

int lis2mdl_read(lis2mdl_io_t *hw, vec3f_t *mag) {
  if (hw == NULL || mag == NULL) return GCIS_ERROR_IO_NULL;
  comm_interface_t *comm = hw->comm;
  const float scale = 1.5; // pg 4, Table 2, mgauss/LSB
  uint8_t *buf = hw->buffer;

  if (comm->read(comm->config, LIS2MDL_OUTX_L_REG, buf, LIS2MDL_BUFFER_SIZE) < 0) return -1;

  mag->x      = scale * cov_bb2f(buf[0], buf[1]); // mGauss
  mag->y      = scale * cov_bb2f(buf[2], buf[3]);
  mag->z      = scale * cov_bb2f(buf[4], buf[5]);

  if (LIS2MDL_BUFFER_SIZE == 8) {
    // Temperature sensor data
    // These registers contain temperature values from the internal temperature sensor. The output value is expressed
    // as a signed 16-bit byte in two's complement. The four most significant bits contain a copy of the sign bit.
    // The nominal sensitivity is 8 LSB/C.
    float temp = cov_bb2f(buf[6], buf[7]) / 8.0f + 25.0f;
    // printf("temperature: %.1f C\n", temp);
    // correct for temperature
  }

  return 0;
}

// this is for the calibration below
static
inline int16_t to_int16(uint8_t lsb, uint8_t msb) {
  return (int16_t)((uint16_t)msb << 8) | lsb;
}

int lis2mdl_calibrate(lis2mdl_io_t *hw, uint16_t num_pts) {
  if (hw == NULL) return GCIS_ERROR_IO_NULL;
  int i, j;
  comm_interface_t *comm;
  comm              = hw->comm;
  uint8_t status    = 0;
  uint8_t *buf      = hw->buffer;
  int16_t magMin[3] = {0, 0, 0};
  int16_t magMax[3] = {0, 0, 0};
  int16_t bias[3]   = {0, 0, 0};

  for (i = 0; i < num_pts; i++) {
    do {
      sleep_ms(1);
      comm->read(comm->config, LIS2MDL_STATUS_REG, &status, 1);
    } while ((status & 0x08) == 0);

    comm->read(comm->config, LIS2MDL_OUTX_L_REG, buf, LIS2MDL_BUFFER_SIZE);

    int16_t magTemp[3] = {
        to_int16(buf[0], buf[1]), // x
        to_int16(buf[2], buf[3]), // y
        to_int16(buf[4], buf[5])  // z
    };

    for (j = 0; j < 3; j++) {
      if (magTemp[j] > magMax[j]) magMax[j] = magTemp[j];
      if (magTemp[j] < magMin[j]) magMin[j] = magTemp[j];
    }
  }

  int16_t m;
  uint8_t config[6] = {0};
  for (j = 0; j < 3; j++) {
    m                 = (magMax[j] + magMin[j]) / 2;
    config[2 * j]     = (uint8_t)(m & 0xFF);
    config[2 * j + 1] = (uint8_t)(m >> 8);
  }
  comm->write(comm->config, LIS3MDL_OFFSET_REG, config, sizeof(config));

  return 0;
}