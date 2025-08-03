#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "gci_sensors/io.h"
#include "gci_sensors/typedefs.h"

#if defined __cplusplus
extern "C" {
#endif

#define QMC5883_BUFFER_SIZE 6

typedef enum : uint8_t {
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

// typedef struct {
//   float x, y, z;
//   float temperature;
// } vec3f_t;

qmc5883_io_t *qmc5883_i2c_init(uint8_t port, qmc5883_range_t range);
qmc5883_io_t *qmc5883_spi_init(uint8_t port, qmc5883_range_t range);
vec3f_t qmc5883_read(qmc5883_io_t *hw);

#if defined __cplusplus
}
#endif
