#include "gci_sensors/io.h"

#include <stdio.h>
#include <string.h> // memcpy

#define I2C_BUS_HOLD true
#define I2C_BUS_RELEASE false

// FIXME: make global when picolib_c gone
typedef enum {
  I2C_INVALID_SCL_PIN = -1,
  I2C_INVALID_SDA_PIN = -2,
  I2C_INVALID_PORT    = -3,
  I2C_PTR_NULL        = -4,
  I2C_UNINITIALIZED   = -99
} i2c_errors_t;
int32_t gcis_i2c_bus_init(uint32_t port, uint32_t baud, pin_t pin_sda, pin_t pin_scl) {
  pin_t sda, scl;
  i2c_inst_t *i2c = NULL;

  if (port == 0) {
    sda = (1 << 0) | (1 << 4) | (1 << 8) | (1 << 12) | (1 << 16) | (1 << 20) | (1 << 24) | (1 << 28);
    scl = (1 << 1) | (1 << 5) | (1 << 9) | (1 << 13) | (1 << 17) | (1 << 21) | (1 << 25) | (1 << 29);
    i2c = i2c0;
  }
  else if (port == 1) {
    sda = (1 << 2) | (1 << 6) | (1 << 10) | (1 << 14) | (1 << 18) | (1 << 22) | (1 << 26);
    scl = (1 << 3) | (1 << 7) | (1 << 11) | (1 << 15) | (1 << 19) | (1 << 23) | (1 << 27);
    i2c = i2c1;
  }
  else return I2C_INVALID_PORT;

  // check if valid pins
  if (((1 << pin_sda) & sda) == 0) return I2C_INVALID_SDA_PIN;
  if (((1 << pin_scl) & scl) == 0) return I2C_INVALID_SCL_PIN;

  int32_t speed = i2c_init(i2c, baud);

  gpio_set_function(pin_sda, GPIO_FUNC_I2C);
  gpio_set_function(pin_scl, GPIO_FUNC_I2C);
  gpio_pull_up(pin_sda);
  gpio_pull_up(pin_scl);
  // Make the I2C pins available to picotool
  // THIS  WON'T WORK, cannot use variables, only constants
  // bi_decl(bi_2pins_with_func(pin_sda, pin_scl, GPIO_FUNC_I2C));

  // hw->baudrate = i2c_init(hw->i2c, baud);
  // return hw;
  return speed;
}

void gcis_i2c_free(uint8_t port) {
  i2c_deinit((port == 0) ? i2c0 : i2c1);
}

int i2c_write(void *config, uint8_t reg, const uint8_t *data, size_t len) {
  i2c_config_t *cfg = (i2c_config_t *)config;

  uint8_t buffer[len + 1];
  buffer[0] = reg;
  memcpy(&buffer[1], data, len);
  return i2c_write_blocking(cfg->i2c, cfg->addr, buffer, len + 1, I2C_BUS_RELEASE);

  // this doesn't always work ... why?
  // i2c_write_blocking(cfg->i2c, cfg->addr, &reg, 1, I2C_BUS_HOLD);
  // // sleep_ms(10);
  // sleep_us(1000);
  // return i2c_write_blocking(cfg->i2c, cfg->addr, data, len, I2C_BUS_RELEASE);
}

int i2c_read(void *config, uint8_t reg, uint8_t *data, size_t len) {
  // printf("read\n");
  i2c_config_t *cfg = (i2c_config_t *)config;
  i2c_write_blocking(cfg->i2c, cfg->addr, &reg, 1, I2C_BUS_HOLD);
  return i2c_read_blocking(cfg->i2c, cfg->addr, data, len, I2C_BUS_RELEASE);
}

// typedef struct {
//   uint32_t sda;
//   uint32_t scl;
// } valid_i2c_pins_t;

// constexpr valid_i2c_pins_t i2c0_valid = {
//     .sda = (1 << 0) | (1 << 4) | (1 << 8) | (1 << 12) | (1 << 16) | (1 << 20) | (1 << 24) | (1 << 28),
//     .scl = (1 << 1) | (1 << 5) | (1 << 9) | (1 << 13) | (1 << 17) | (1 << 21) | (1 << 25) | (1 << 29)};

// constexpr valid_i2c_pins_t i2c1_valid = {
//     .sda = (1 << 2) | (1 << 6) | (1 << 10) | (1 << 14) | (1 << 18) | (1 << 22) | (1 << 26),
//     .scl = (1 << 3) | (1 << 7) | (1 << 11) | (1 << 15) | (1 << 19) | (1 << 23) | (1 << 27)};


// int32_t gcis_i2c0_bus_init(uint32_t baud, pin_t pin_sda, pin_t pin_scl) {
//   return gcis_i2c_bus_init(0, baud, pin_sda, pin_scl);
// }

// int32_t gci_i2c1_bus_init(uint32_t baud, pin_t pin_sda, pin_t pin_scl) {
//   return gci_i2c_bus_init(1, baud, pin_sda, pin_scl);
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

// void kspi_init_cs(pin_t cs) {
//   // Chip select is active-low, so we'll initialise it to a driven-high state
//   gpio_init(cs);
//   gpio_set_dir(cs, GPIO_OUT);
//   gpio_put(cs, 1);
// }

// static uint32_t kspix_init(spi_inst_t *spi, uint32_t baud, pin_t miso,
//                           pin_t mosi, pin_t sck) {
//   // Initialize SPI channel (channel, baud rate set to 20MHz)
//   baud = spi_init(spi, baud);
//   // Format (channel, data bits per transfer, polarity, phase, order)
//   const spi_order_t order = SPI_MSB_FIRST; // this can ONLY be MSB first
//   spi_set_format(spi, 16, (spi_cpol_t)0, (spi_cpha_t)0, order);
//   spi_set_slave(spi, false); // set to Master (default)

//   // Map SPI signals to GPIO ports
//   gpio_set_function(miso, GPIO_FUNC_SPI);
//   gpio_set_function(mosi, GPIO_FUNC_SPI);
//   gpio_set_function(sck, GPIO_FUNC_SPI);
//   // gpio_set_function(cs, GPIO_FUNC_SPI); // doesn't this work?

//   // // Chip select is active-low, so we'll initialise it to a driven-high state
//   // gpio_init(cs);
//   // gpio_set_dir(cs, GPIO_OUT);
//   // gpio_put(cs, 1);

//   return baud;
// }

// uint32_t kspi0_init(uint32_t baud, pin_t miso, pin_t mosi, pin_t sck) {
//   return kspix_init(spi0, baud, miso, mosi, sck);
// }

// uint32_t kspi1_init(uint32_t baud, pin_t miso, pin_t mosi, pin_t sck) {
//   return kspix_init(spi1, baud, miso, mosi, sck);
// }

// int spi_write(void *config, uint8_t reg, const uint8_t *data, size_t len) {
//     spi_config_t *cfg = (spi_config_t *)config;
//     // uint8_t buffer[len + 1];
//     // buffer[0] = reg;
//     // memcpy(&buffer[1], data, len);
//     gpio_put(cfg->cs_pin, SPI_ACTIVATE); // CS low (active)
//     // int ret = spi_write_blocking(cfg->spi, buffer, len + 1);
//     spi_write_blocking(cfg->spi, &reg, 1);
//     int ret = spi_write_blocking(cfg->spi, data, len);
//     gpio_put(cfg->cs_pin, SPI_DEACTIVATE); // CS high
//     return ret;
// }

// int spi_read(void *config, uint8_t reg, uint8_t *data, size_t len) {
//     spi_config_t *cfg = (spi_config_t *)config;
//     gpio_put(cfg->cs_pin, SPI_ACTIVATE); // CS low
//     spi_write_blocking(cfg->spi, &reg, 1);
//     int ret = spi_read_blocking(cfg->spi, 0, data, len);
//     gpio_put(cfg->cs_pin, SPI_DEACTIVATE); // CS high
//     return ret;
// }

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