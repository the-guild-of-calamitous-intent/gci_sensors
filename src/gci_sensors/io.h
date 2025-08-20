////////////////////////////////////////////////
//  The MIT License (MIT)
//  Copyright (c) 2022 Kevin Walchko
//  see LICENSE for full details
////////////////////////////////////////////////
#pragma once

#include <hardware/i2c.h>
#include <hardware/spi.h>
#include <pico/stdlib.h>
#include <stdint.h>
#include <stdlib.h>

// A lot of pico IO functions return PICO_ERROR_GENERIC on
// error:
//
// PICO_ERROR_GENERIC -1

#ifndef __PIN__
  #define __PIN__
typedef uint32_t pin_t;
#endif

#define PICO_I2C_0 0
#define PICO_I2C_1 1
#define PICO_SPI_0 0
#define PICO_SPI_1 1

#define GCIS_SPI_20MHZ 20000000 // maximum
#define GCIS_SPI_10MHZ 10000000
#define GCIS_SPI_5MHZ 5000000
#define GCIS_SPI_1MHZ 1000000
#define GCIS_SPI_100KHZ 100000

#define GCIS_I2C_100KHZ (100 * 1000UL)
#define GCIS_I2C_400KHZ (400 * 1000UL)
#define GCIS_I2C_1000KHZ (1000 * 1000UL)
#define GCIS_I2C_1700KHZ (1700 * 1000UL)
#define GCIS_I2C_3400KHZ (3400 * 1000UL)

typedef enum {
  GCIS_ERROR_NONE       =  0,
  GCIS_ERROR_RW         = -1,
  GCIS_ERROR_WHOAMI     = -2,
  GCIS_ERROR_IO_NULL    = -4,
  GCIS_ERROR_PARAM      = -8,
  GCIS_ERROR_SDATA_PIN  = -16,
  GCIS_ERROR_SCK_PIN    = -32,
  GCIS_ERROR_PORT       = -64,
} gcis_error_t;

// --- Interrupts -----------------------------------------
void gcis_interrupt(pin_t pin, void (*f)(uint, uint32_t), uint32_t mask);

// --- Communication Interface ----------------------------
typedef enum { I2C_INTERFACE, SPI_INTERFACE } interface_t;

// --- Abstract Interface ---------------------------------
typedef struct {
  int (*write)(void *config, uint8_t reg, const uint8_t *data, size_t len);
  int (*read)(void *config, uint8_t reg, uint8_t *data, size_t len);
  void *config; // protocol specific interface
  interface_t type;
} comm_interface_t;

comm_interface_t *comm_interface_init(uint8_t port, uint8_t addr_cs,
                                      interface_t type);
void comm_interface_free(comm_interface_t *comm);

// --- I2C Implementation ---------------------------------
typedef struct {
  i2c_inst_t *i2c;
  uint8_t addr;
} i2c_config_t;

// return:
//     >0: number bytes read/written
//     <0: error
int i2c_write(void *config, uint8_t reg, const uint8_t *data, size_t len);
int i2c_read(void *config, uint8_t reg, uint8_t *data, size_t len);

int32_t gcis_i2c_bus_init(uint32_t port, uint32_t baud, pin_t sda, pin_t scl);
void gcis_i2c_free(uint8_t port);

// --- SPI Implementation ---------------------------------
// typedef enum {
//   SPI_INVALID_SDI_PIN = -1,
//   SPI_INVALID_SDO_PIN = -2,
//   SPI_INVALID_SCK_PIN = -3,
//   SPI_INVALID_PORT    = -4,
//   SPI_PTR_NULL        = -5,
//   SPI_UNINITIALIZED   = -99
// } spi_errors_t;

typedef struct {
  spi_inst_t *spi;
  uint8_t cs_pin;
} spi_config_t;

int32_t gcis_spi_bus_init(uint8_t port, uint32_t baud, pin_t sdi, pin_t sdo,
                          pin_t sck);

int32_t gcis_spi0_init(uint32_t baud, pin_t sdi, pin_t sdo, pin_t sck);
int32_t gcis_spi1_init(uint32_t baud, pin_t sdi, pin_t sdo, pin_t sck);

// void gcis_spi_free(uint8_t port);

// return:
//     >0: number bytes read/written
//     <0: error
int spi_write(void *config, uint8_t reg, const uint8_t *data, size_t len);
int spi_read(void *config, uint8_t reg, uint8_t *data, size_t len);
int spi_write_status(void *config, uint8_t reg, const uint8_t *data,
                     size_t len);
int spi_read_status(void *config, uint8_t reg, uint8_t *data, size_t len);

// --- Useful Functions -------------------
inline float cov_bb2f(uint8_t lsb, uint8_t msb) {
  return (float)((int16_t)((uint16_t)msb << 8) | lsb);
}

inline float cov_bbb2f(uint8_t lsb, uint8_t xsb, uint8_t msb) {
  return (float)(((uint32_t)msb << 16) | ((uint32_t)xsb << 8) | (uint32_t)lsb);
}

