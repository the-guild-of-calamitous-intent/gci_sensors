#pragma once

#include "gci_sensors/io.h"
#include <stdint.h>

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

typedef struct {
  comm_interface_t *comm;
  float scale;                      // int -> float
  float mcal[12];                   // scale/bias
  uint8_t buf[HMC5883_BUFFER_SIZE]; // used for reading
  bool calibrated;
  // bool use_imu_timestamp; // whose timestamp to use
  bool ok;
} hmc5883_io_t;

typedef struct {
  float x, y, z;
  // bool ok;
} hmc5883_t;

hmc5883_io_t *hmc5883_i2c_init(uint8_t port);
hmc5883_t hmc5883_read(hmc5883_io_t *hw);

#if defined __cplusplus
}
#endif
