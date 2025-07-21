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

#ifndef pin_t
typedef uint32_t pin_t;
#endif

#define PICO_I2C_0 0
#define PICO_I2C_1 1
#define PICO_SPI_0 0
#define PICO_SPI_1 1

constexpr uint32_t GCIS_SPI_20MHZ        = 20000000; // maximum
constexpr uint32_t GCIS_SPI_10MHZ        = 10000000;
constexpr uint32_t GCIS_SPI_1MHZ         = 1000000;
constexpr uint32_t GCIS_SPI_100KHZ       = 100000;
constexpr uint32_t GCIS_SPI_DEFAULT_BAUD = 48000; // default rate

constexpr uint32_t GCIS_I2C_100KHZ       = 100 * 1000UL;
constexpr uint32_t GCIS_I2C_400KHZ       = 400 * 1000UL;
constexpr uint32_t GCIS_I2C_1000KHZ      = 1000 * 1000UL;
constexpr uint32_t GCIS_I2C_1700KHZ      = 1700 * 1000UL;
constexpr uint32_t GCIS_I2C_3400KHZ      = 3400 * 1000UL;

// --- Communication Interface ----------------------------

typedef enum { I2C_INTERFACE, SPI_INTERFACE } interface_t;

// abstract interface
typedef struct {
  int (*write)(void *config, uint8_t reg, const uint8_t *data, size_t len);
  int (*read)(void *config, uint8_t reg, uint8_t *data, size_t len);
  void *config; // protocol specific interface
} comm_interface_t;

comm_interface_t *comm_interface_init(uint8_t port, uint8_t addr_cs,
                                      interface_t type);

// --- I2C Implementation ---------------------------------
// typedef enum {
//   I2C_INVALID_SCL_PIN   = -1,
//   I2C_INVALID_SDA_PIN   = -2,
//   I2C_INVALID_PORT  = -3,
//   I2C_PTR_NULL      = -4,
//   I2C_UNINITIALIZED = -99
// } i2c_errors_t;

typedef struct {
  i2c_inst_t *i2c;
  uint8_t addr;
} i2c_config_t;

// return:
//     >0: number bytes read/written
//     <0: error
int i2c_write(void *config, uint8_t reg, const uint8_t *data, size_t len);
int i2c_read(void *config, uint8_t reg, uint8_t *data, size_t len);

int32_t gcis_i2c_bus_init(uint32_t port, uint32_t baud, pin_t pin_sda,
                          pin_t pin_scl);

// --- SPI Implementation ---------------------------------
typedef enum {
  SPI_INVALID_SDI_PIN = -1,
  SPI_INVALID_SDO_PIN = -2,
  SPI_INVALID_SCK_PIN = -3,
  SPI_INVALID_PORT    = -4,
  SPI_PTR_NULL        = -5,
  SPI_UNINITIALIZED   = -99
} spi_errors_t;

typedef struct {
  spi_inst_t *spi;
  uint8_t cs_pin;
} spi_config_t;

int32_t gcis_spi_bus_init(uint8_t port, uint32_t baud, pin_t sdi, pin_t sdo,
                          pin_t sck);
int32_t gcis_spi0_init(uint32_t baud, pin_t sdi, pin_t sdo, pin_t sck);
int32_t gcis_spi1_init(uint32_t baud, pin_t sdi, pin_t sdo, pin_t sck);

typedef enum { SPI_CS_NONE, SPI_CS_PULLUP, SPI_CS_PULLDOWN } spi_cs_t;

void gcis_spi_init_cs(pin_t cs, spi_cs_t opt);

// return:
//     >0: number bytes read/written
//     <0: error
int spi_write(void *config, uint8_t reg, const uint8_t *data, size_t len);
int spi_read(void *config, uint8_t reg, uint8_t *data, size_t len);