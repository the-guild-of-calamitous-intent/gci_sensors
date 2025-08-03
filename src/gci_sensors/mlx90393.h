#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "gci_sensors/io.h"
#include "gci_sensors/typedefs.h"

#if defined __cplusplus
extern "C" {
#endif

#define MLX90393_BUFFER_SIZE 6
#define MLX90393_DEFAULT_ADDR 0x0C // Can also be 0x18, depending on IC

typedef struct {
  comm_interface_t *comm;
  uint32_t gain, res;
  uint8_t buffer[MLX90393_BUFFER_SIZE];
  float sm[12];
  float scale;
  bool ok;
} mlx90393_io_t;

mlx90393_io_t *mlx90393_init(interface_t type, uint8_t port, uint8_t addr);
mlx90393_io_t *mlx90393_spi_init(uint8_t port, pin_t cs);
vec3f_t mlx90393_read(mlx90393_io_t *hw);

#if defined __cplusplus
}
#endif
