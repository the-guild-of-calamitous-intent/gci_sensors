#include <hardware/gpio.h>
#include <pico/binary_info.h>
#include <pico/stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <tusb.h> // wait for USB

#include "gci_sensors/lis3mdl.h"

#define PORT 0
#define SCL  1
#define SDA  0

int main() {
  stdio_init_all();
  while (!tud_cdc_connected()) {
    sleep_ms(100);
  }

  int32_t speed = gcis_i2c_bus_init(PORT, GCIS_I2C_400KHZ, SDA, SCL);

  printf(">> i2c instance: %u baud: %u\n", PORT, speed);
  printf(">> i2c SDA: %u SCL: %u\n", SDA, SCL);
  bi_decl(bi_2pins_with_func(SDA, SCL, GPIO_FUNC_I2C)); // compile info

  printf("/// MAGNETOMETER START ///\n");

  lis3mdl_io_t *mag = lis3mdl_create(I2C_INTERFACE, PORT, LIS3MDL_ADDRESS);
  while (true) {
    if (lis3mdl_init(mag, LIS3MDL_RANGE_4GAUSS, LIS3MDL_ODR_155HZ) == 0) break;
    printf("*** mag error ***\n");
    sleep_ms(1000);
  }

  while (true) {
    sleep_ms(5); // ~200 Hz

    vec3f_t m;
    if (lis3mdl_read(mag, &m) < 0) {
      printf("*** bad reading ***\n");
      continue;
    }

    printf("Mags: %f %f %f\n", m.x, m.y, m.z);
  }
}