#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "gci_sensors/io.h"

#if defined __cplusplus
extern "C" {
#endif

#define QMC5883_BUFFER_SIZE 6

typedef enum: uint8_t {
  QMC5883_2GAUSS = (0 << 4),
  QMC5883_8GAUSS = (1 << 4),
} qmc5883_range_t;

typedef struct {
  comm_interface_t *comm;
  float scale;                      // int -> float
  float mcal[12];                   // scale/bias
  uint8_t buf[QMC5883_BUFFER_SIZE]; // used for reading
  bool calibrated;
  bool ok;
} qmc5883_io_t;

typedef struct {
  float x, y, z;
  float temperature;
} qmc5883_t;

qmc5883_io_t *qmc5883_i2c_init(uint8_t port, qmc5883_range_t range);
qmc5883_io_t *qmc5883_spi_init(uint8_t port, qmc5883_range_t range);
qmc5883_t qmc5883_read(qmc5883_io_t *hw);

#if defined __cplusplus
}
#endif


// // QMC5883L I2C address and register definitions
// #define QMC5883L_ADDR 0x1E

// #define QMC5883L_CONFIG_A 0x00
// #define QMC5883L_CONFIG_B 0x01
// #define QMC5883L_MODE 0x02
// #define QMC5883L_DATA_X_MSB 0x03
// #define QMC5883L_STATUS 0x09
// #define QMC5883L_WHO_AM_I 0x0F

// typedef enum: uint8_t {
//   QMC5883_0_88 = 0,
//   QMC5883_1_3 = 1,
//   QMC5883_1_9 = 2,
//   QMC5883_2_5 = 3,
//   QMC5883_4_0 = 4,
//   QMC5883_4_7 = 5,
//   QMC5883_5_6 = 6,
//   QMC5883_8_1 = 7,
// } qmc5883_range_t;