#include "gci_sensors/io.h"
#include <hardware/gpio.h>

comm_interface_t *comm_interface_init(uint8_t port, uint8_t addr_cs, interface_t type) {

  comm_interface_t *comm = (comm_interface_t *)calloc(1, sizeof(comm_interface_t));
  if (comm == NULL) return NULL;

  if (type == I2C_INTERFACE) {
    i2c_config_t *config = (i2c_config_t *)calloc(1, sizeof(i2c_config_t));
    if (config == NULL) return NULL;
    config->i2c  = (port == 0) ? i2c0 : i2c1;
    config->addr = addr_cs;
    comm->read   = i2c_read;
    comm->write  = i2c_write;
    comm->config = config;
  }
  else {
    spi_config_t *config = (spi_config_t *)calloc(1, sizeof(spi_config_t));
    if (config == NULL) return NULL;
    config->spi    = (port == 0) ? spi0 : spi1;
    config->cs_pin = addr_cs;
    comm->read     = spi_read;
    comm->write    = spi_write;
    comm->config   = config;

    // Chip select is active-low, initialise it to a driven-high state
    gpio_init(addr_cs);
    gpio_set_dir(addr_cs, GPIO_OUT);
    gpio_put(addr_cs, 1);
  }

  return comm;
}
