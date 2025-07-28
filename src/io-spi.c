#include "gci_sensors/io.h"

#include <stdio.h>
#include <string.h> // memcpy

#define SPI_ACTIVATE 0
#define SPI_DEACTIVATE 1

int32_t gcis_spi_bus_init(
    uint8_t port, uint32_t baud,
    pin_t sdi, pin_t sdo, pin_t sck) {

  uint32_t valid_sdi;
  uint32_t valid_sck;
  uint32_t valid_sdo;
  spi_inst_t *spi = NULL;

  if (port == 0) {
    valid_sdi = (1 << 0) | (1 << 4) | (1 << 16) | (1 << 20);
    valid_sck = (1 << 2) | (1 << 6) | (1 << 18);
    valid_sdo = (1 << 3) | (1 << 7) | (1 << 19);
    spi = spi0;
  }
  else if (port == 1) {
    valid_sdi = (1 << 8) | (1 << 12) | (1 << 28);
    valid_sck = (1 << 10) | (1 << 14) | (1 << 26);
    valid_sdo = (1 << 11) | (1 << 15) | (1 << 27);
    spi = spi1;
  }
  else return SPI_INVALID_PORT;

  if (((1 << sdi) & valid_sdi) == 0) return SPI_INVALID_SDI_PIN;
  if (((1 << sdo) & valid_sdo) == 0) return SPI_INVALID_SDO_PIN;
  if (((1 << sck) & valid_sck) == 0) return SPI_INVALID_SCK_PIN;

  baud = spi_init(spi, baud);

  // Map SPI signals to GPIO ports
  gpio_set_function(sdi, GPIO_FUNC_SPI);
  gpio_set_function(sdo, GPIO_FUNC_SPI);
  gpio_set_function(sck, GPIO_FUNC_SPI);

  return baud;
}

int32_t gcis_spi0_init(uint32_t baud, pin_t sdi, pin_t sdo, pin_t sck) {
  return gcis_spi_bus_init(0, baud, sdi, sdo, sck);
}

int32_t gcis_spi1_init(uint32_t baud, pin_t sdi, pin_t sdo, pin_t sck) {
  return gcis_spi_bus_init(1, baud, sdi, sdo, sck);
}

void gcis_spi_free(uint8_t port) {
  // spi_config_t *cfg = (spi_config_t *)config;
  spi_inst_t *spi = NULL;
  spi             = (port == 0) ? spi0 : spi1;
  spi_deinit(spi);
}

int spi_write(void *config, uint8_t reg, const uint8_t *data, size_t len) {
  int ret;
  spi_config_t *cfg = (spi_config_t *)config;

  gpio_put(cfg->cs_pin, SPI_ACTIVATE); // CS low (active)
  uint8_t buffer[len + 1];
  buffer[0] = reg;
  memcpy(&buffer[1], data, len);
  ret = spi_write_blocking(cfg->spi, buffer, len + 1);
  gpio_put(cfg->cs_pin, SPI_DEACTIVATE); // CS high

  // this doesn't always work ... why?
  // gpio_put(cfg->cs_pin, SPI_ACTIVATE); // CS low (active)
  // spi_write_blocking(cfg->spi, &reg, 1);
  // ret = spi_write_blocking(cfg->spi, data, len);
  // gpio_put(cfg->cs_pin, SPI_DEACTIVATE); // CS high

  return ret;
}

int spi_read(void *config, uint8_t reg, uint8_t *data, size_t len) {
  // https://github.com/betaflight/betaflight/blob/c545435f2e1b2561085bbda6c387424db4c383b7/src/main/drivers/bus.c#L123
  // BF ORs with 0x80 also
  reg |= 0x80;
  int ret;
  spi_config_t *cfg = (spi_config_t *)config;
  gpio_put(cfg->cs_pin, SPI_ACTIVATE); // CS low
  spi_write_blocking(cfg->spi, &reg, 1);
  sleep_us(1);
  // if (spi_is_readable(cfg->spi) == false) sleep_us(100); // doesn't help
  ret = spi_read_blocking(cfg->spi, 0, data, len);
  gpio_put(cfg->cs_pin, SPI_DEACTIVATE); // CS high
  return ret;
}


// typedef struct {
//   pin_t sdi;
//   pin_t csn;
//   pin_t sck;
//   pin_t sdo;
// } valid_spi_pins_t;

// constexpr valid_spi_pins_t spi0_valid = {
//     .sdi = (1 << 0) | (1 << 4) | (1 << 16) | (1 << 20),
//     .csn = (1 << 1) | (1 << 5) | (1 << 17) | (1 << 21),
//     .sck = (1 << 2) | (1 << 6) | (1 << 18),
//     .sdo = (1 << 3) | (1 << 7) | (1 << 19)};

// constexpr valid_spi_pins_t spi1_valid = {
//     .sdi = (1 << 8) | (1 << 12) | (1 << 28),
//     .csn = (1 << 9) | (1 << 13),
//     .sck = (1 << 10) | (1 << 14) | (1 << 26),
//     .sdo = (1 << 11) | (1 << 15) | (1 << 27)};

// void gcis_spi_init_cs(pin_t cs, spi_cs_t opt) {
//   // Chip select is active-low, so we'll initialise it to a driven-high state
//   gpio_init(cs);
//   gpio_set_dir(cs, GPIO_OUT);
//   gpio_put(cs, 1);

//   if (opt == SPI_CS_PULLDOWN) gpio_pull_down(cs);
//   else if (opt == SPI_CS_PULLUP) gpio_pull_up(cs);
//   else gpio_disable_pulls(cs);
// }
