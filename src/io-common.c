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
    const pin_t cs       = addr_cs;
    spi_config_t *config = (spi_config_t *)calloc(1, sizeof(spi_config_t));
    if (config == NULL) return NULL;
    config->spi    = (port == 0) ? spi0 : spi1;
    config->cs_pin = cs;
    comm->read     = spi_read;
    comm->write    = spi_write;
    comm->config   = config;

    // Chip select is active-low, initialise it to a driven-high state
    gpio_init(cs);
    gpio_set_dir(cs, GPIO_OUT);
    gpio_put(cs, 1);
  }
  comm->type = type;

  return comm;
}

void comm_interface_free(comm_interface_t *comm) {
  if (comm->config != NULL) free(comm->config);
  if (comm != NULL) free(comm);
}

// Interrupt assumes active HIGH and will trigger on RISING
void gcis_interrupt(pin_t pin, void (*f)(uint, uint32_t), uint32_t mask) {
  gpio_init(pin);
  gpio_set_dir(pin, GPIO_IN);
  gpio_pull_down(pin);
  gpio_set_irq_enabled_with_callback(pin, mask, true, f);
}