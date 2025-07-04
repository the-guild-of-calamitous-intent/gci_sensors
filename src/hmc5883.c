
#include "gci_sensors/hmc5883.h"
#include <stdint.h>
#include <stdlib.h> // calloc

// HMC5883L I2C address and register definitions
#define HMC5883L_ADDR 0x1E

#define HMC5883L_CONFIG_A_REG 0x00
#define HMC5883L_CONFIG_B_REG 0x01
#define HMC5883L_MODE_REG 0x02
#define HMC5883L_DATA_X_MSB 0x03
#define HMC5883L_STATUS_REG 0x09
#define HMC5883L_WHO_AM_I_REG 0x10 // ID REG A really, holds ASCII H

// #define HMC5883_BUFFER_SIZE 6

#define HMC5883L_WHO_AM_I 0x68 // ASCII H

hmc5883_io_t *hmc5883_init(interface_t type, uint8_t port) {
  int32_t ok;
  uint8_t cmd;
  hmc5883_io_t *hw = (hmc5883_io_t *)calloc(1, sizeof(hmc5883_io_t));
  if (hw == NULL) return NULL;

  comm_interface_t *comm = comm_interface_init(port, HMC5883L_ADDR, I2C_INTERFACE);
  if (comm == NULL) return NULL;

  hw->comm = comm;

  hw->calibrated = false;
  hw->scale      = 1.0f;

  ok = comm->read(comm->config, HMC5883L_WHO_AM_I_REG, &cmd, 1);
  if ((cmd != HMC5883L_WHO_AM_I) || (ok < 0)) return NULL;

  // Config A: 2 (0x01) samples averaged, 75 Hz (0x06) output rate, normal measurement
  cmd = (0x01 << 5) | (0x06 << 2);
  ok  = comm->write(comm->config, HMC5883L_CONFIG_A_REG, &cmd, 1);
  if (ok < 0) return NULL;

  // Config B: Set gain to 1.3 Ga (1090 LSB/Gauss)
  cmd = 0x20;
  ok  = comm->write(comm->config, HMC5883L_CONFIG_B_REG, &cmd, 1);
  if (ok < 0) return NULL;

  // Mode: Continuous measurement mode
  cmd = 0x00;
  ok  = comm->write(comm->config, HMC5883L_MODE_REG, &cmd, 1);
  if (ok < 0) return NULL;

  sleep_ms(100);

  return hw;
}

hmc5883_io_t *hmc5883_i2c_init(uint8_t port) {
  return hmc5883_init(I2C_INTERFACE, port);
}

hmc5883_io_t *hmc5883_spi_init(uint8_t port) {
  return hmc5883_init(SPI_INTERFACE, port);
}

hmc5883_t hmc5883_read(hmc5883_io_t *hw) {
  hmc5883_t ret;
  hw->ok                 = false;
  comm_interface_t *comm = hw->comm;
  uint8_t *buf           = hw->buf;

  // uint8_t reg = HMC5883L_DATA_X_MSB;

  // // Set the register to read from
  // i2c_write_blocking(I2C_PORT, HMC5883L_ADDR, &reg, 1, true);
  // // Read 6 bytes (X MSB, X LSB, Z MSB, Z LSB, Y MSB, Y LSB)
  // i2c_read_blocking(I2C_PORT, HMC5883L_ADDR, buf, 6, false);

  // ok  = comm->write(comm->config, REG_CTRL8_XL, &reg, 1);
  // if (ok < 0) return NULL;

  bool ok = comm->read(comm->config, HMC5883L_DATA_X_MSB, buf, HMC5883_BUFFER_SIZE);
  if (ok < 0) return ret;

  // Combine MSB and LSB
  ret.x = hw->scale * (float)((int16_t)((buf[0] << 8) | buf[1]));
  ret.z = hw->scale * (float)((int16_t)((buf[2] << 8) | buf[3])); // Z comes before Y in HMC5883L
  ret.y = hw->scale * (float)((int16_t)((buf[4] << 8) | buf[5]));

  hw->ok = true;

  return ret;
}