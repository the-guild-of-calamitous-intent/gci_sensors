#include "gci_sensors/io.h"

#include <stdio.h>
#include <string.h> // memcpy

#define SPI_ACTIVATE 0
#define SPI_DEACTIVATE 1

typedef struct {
  uint32_t miso;
  uint32_t mosi;
  uint32_t sck;
} valid_spi_pins_t;

void gcis_spi_init_cs(pin_t cs) {
  // Chip select is active-low, so we'll initialise it to a driven-high state
  gpio_init(cs);
  gpio_set_dir(cs, GPIO_OUT);
  gpio_put(cs, 1);
}

uint32_t gcis_spi_bus_init(uint8_t port, uint32_t baud, pin_t miso,
                           pin_t mosi, pin_t sck) {
  spi_inst_t *spi = NULL;

  if (port == 0) {
    spi = spi0;
  }
  else if (port == 1) {
    spi = spi1;
  }
  else return SPI_INVALID_PORT;

  baud = spi_init(spi, baud);
  // Format (channel, data bits per transfer, polarity, phase, order)
  const spi_order_t order = SPI_MSB_FIRST; // this can ONLY be MSB first
  spi_set_format(spi, 16, (spi_cpol_t)0, (spi_cpha_t)0, order);
  spi_set_slave(spi, false); // set to Master (default)

  // Map SPI signals to GPIO ports
  gpio_set_function(miso, GPIO_FUNC_SPI);
  gpio_set_function(mosi, GPIO_FUNC_SPI);
  gpio_set_function(sck, GPIO_FUNC_SPI);
  // gpio_set_function(cs, GPIO_FUNC_SPI); // doesn't this work?

  // // Chip select is active-low, so we'll initialise it to a driven-high state
  // gpio_init(cs);
  // gpio_set_dir(cs, GPIO_OUT);
  // gpio_put(cs, 1);

  return baud;
}

uint32_t gcis_spi0_init(uint32_t baud, pin_t miso, pin_t mosi, pin_t sck) {
  return gcis_spi_bus_init(0, baud, miso, mosi, sck);
}

uint32_t gcis_spi1_init(uint32_t baud, pin_t miso, pin_t mosi, pin_t sck) {
  return gcis_spi_bus_init(1, baud, miso, mosi, sck);
}

int spi_write(void *config, uint8_t reg, const uint8_t *data, size_t len) {
  spi_config_t *cfg = (spi_config_t *)config;
  // uint8_t buffer[len + 1];
  // buffer[0] = reg;
  // memcpy(&buffer[1], data, len);
  gpio_put(cfg->cs_pin, SPI_ACTIVATE); // CS low (active)
  // int ret = spi_write_blocking(cfg->spi, buffer, len + 1);
  spi_write_blocking(cfg->spi, &reg, 1);
  int ret = spi_write_blocking(cfg->spi, data, len);
  gpio_put(cfg->cs_pin, SPI_DEACTIVATE); // CS high
  return ret;
}

int spi_read(void *config, uint8_t reg, uint8_t *data, size_t len) {
  spi_config_t *cfg = (spi_config_t *)config;
  gpio_put(cfg->cs_pin, SPI_ACTIVATE); // CS low
  spi_write_blocking(cfg->spi, &reg, 1);
  int ret = spi_read_blocking(cfg->spi, 0, data, len);
  gpio_put(cfg->cs_pin, SPI_DEACTIVATE); // CS high
  return ret;
}

// // --- Driver Structure ---

// typedef struct {
//     comm_interface_t *comm; // Pointer to communication interface
// } device_driver_t;

// // --- Driver Functions ---

// bool device_init(device_driver_t *driver, comm_interface_t *comm) {
//     driver->comm = comm;
//     return driver->comm->init(driver->comm->config);
// }

// int device_write_register(device_driver_t *driver, uint8_t reg, const uint8_t *data, size_t len) {
//     return driver->comm->write(driver->comm->config, reg, data, len);
// }

// int device_read_register(device_driver_t *driver, uint8_t reg, uint8_t *data, size_t len) {
//     return driver->comm->read(driver->comm->config, reg, data, len);
// }

// // --- Example Usage ---

// int main() {
//     stdio_init_all();

//     // Example: Initialize I2C driver
//     i2c_config_t i2c_cfg = {
//         .i2c = i2c0,
//         .baudrate = 100000, // 100 kHz
//         .sda_pin = 4,
//         .scl_pin = 5,
//         .addr = 0x68 // Example device address
//     };
//     comm_interface_t i2c_interface = {
//         .init = i2c_init,
//         .write = i2c_write,
//         .read = i2c_read,
//         .config = &i2c_cfg
//     };

//     // Example: Initialize SPI driver
//     spi_config_t spi_cfg = {
//         .spi = spi0,
//         .baudrate = 1000000, // 1 MHz
//         .sck_pin = 2,
//         .mosi_pin = 3,
//         .miso_pin = 4,
//         .cs_pin = 5
//     };
//     comm_interface_t spi_interface = {
//         .init = spi_init,
//         .write = spi_write,
//         .read = spi_read,
//         .config = &spi_cfg
//     };

//     // Initialize driver with I2C (or SPI by swapping interface)
//     device_driver_t driver;
//     device_init(&driver, &i2c_interface); // Use i2c_interface or spi_interface

//     // Example: Write and read from a device register
//     uint8_t data_write = 0x42;
//     uint8_t data_read;
//     device_write_register(&driver, 0x01, &data_write, 1);
//     device_read_register(&driver, 0x01, &data_read, 1);

//     printf("Read data: 0x%02X\n", data_read);
//     return 0;
// }

// #define I2C_BUS_HOLD true
// #define I2C_BUS_RELEASE false

// --- Communication Interface ---

// bool i2c_init(void *config) {
//     i2c_config_t *cfg = (i2c_config_t *)config;
//     // i2c_init(cfg->i2c, cfg->baudrate);
//     gpio_set_function(cfg->sda_pin, GPIO_FUNC_I2C);
//     gpio_set_function(cfg->scl_pin, GPIO_FUNC_I2C);
//     gpio_pull_up(cfg->sda_pin);
//     gpio_pull_up(cfg->scl_pin);
//     return true;
// }

// int i2c_write(void *config, uint8_t reg, const uint8_t *data, size_t len) {
//   i2c_config_t *cfg = (i2c_config_t *)config;

//   // uint8_t buffer[len + 1];
//   // buffer[0] = reg;
//   // memcpy(&buffer[1], data, len);
//   // return i2c_write_blocking(cfg->i2c, cfg->addr, buffer, len + 1, false);

//   i2c_write_blocking(cfg->i2c, cfg->addr, &reg, 1, I2C_BUS_HOLD);
//   return i2c_write_blocking(cfg->i2c, cfg->addr, data, len, I2C_BUS_RELEASE);
// }

// int i2c_read(void *config, uint8_t reg, uint8_t *data, size_t len) {
//   // printf("read\n");
//   i2c_config_t *cfg = (i2c_config_t *)config;
//   i2c_write_blocking(cfg->i2c, cfg->addr, &reg, 1, I2C_BUS_HOLD);
//   return i2c_read_blocking(cfg->i2c, cfg->addr, data, len, I2C_BUS_RELEASE);
// }

// --- SPI Implementation ---

// bool spi_init(void *config) {
//     spi_config_t *cfg = (spi_config_t *)config;
//     // spi_init(cfg->spi, cfg->baudrate);
//     gpio_set_function(cfg->sck_pin, GPIO_FUNC_SPI);
//     gpio_set_function(cfg->mosi_pin, GPIO_FUNC_SPI);
//     gpio_set_function(cfg->miso_pin, GPIO_FUNC_SPI);
//     gpio_init(cfg->cs_pin);
//     gpio_set_dir(cfg->cs_pin, GPIO_OUT);
//     gpio_put(cfg->cs_pin, 1); // CS high (inactive)
//     return true;
// }
