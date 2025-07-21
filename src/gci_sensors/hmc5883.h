#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "gci_sensors/io.h"

#if defined __cplusplus
extern "C" {
#endif

// // HMC5883L I2C address and register definitions
// #define HMC5883L_ADDR 0x1E

// #define HMC5883L_CONFIG_A 0x00
// #define HMC5883L_CONFIG_B 0x01
// #define HMC5883L_MODE 0x02
// #define HMC5883L_DATA_X_MSB 0x03
// #define HMC5883L_STATUS 0x09
// #define HMC5883L_WHO_AM_I 0x0F

#define HMC5883_BUFFER_SIZE 6

// typedef enum: uint8_t {
//   HMC5883_0_88 = 0,
//   HMC5883_1_3 = 1,
//   HMC5883_1_9 = 2,
//   HMC5883_2_5 = 3,
//   HMC5883_4_0 = 4,
//   HMC5883_4_7 = 5,
//   HMC5883_5_6 = 6,
//   HMC5883_8_1 = 7,
// } hmc5883_range_t;

typedef enum: uint8_t {
  HMC5883_2GAUSS = (0 << 4),
  HMC5883_8GAUSS = (1 << 4),
} hmc5883_range_t;

typedef struct {
  comm_interface_t *comm;
  float scale;                      // int -> float
  float mcal[12];                   // scale/bias
  uint8_t buf[HMC5883_BUFFER_SIZE]; // used for reading
  bool calibrated;
  bool ok;
} hmc5883_io_t;

typedef struct {
  float x, y, z;
  float temperature;
} hmc5883_t;

hmc5883_io_t *hmc5883_i2c_init(uint8_t port, hmc5883_range_t range);
hmc5883_io_t *hmc5883_spi_init(uint8_t port, hmc5883_range_t range);
hmc5883_t hmc5883_read(hmc5883_io_t *hw);

#if defined __cplusplus
}
#endif
