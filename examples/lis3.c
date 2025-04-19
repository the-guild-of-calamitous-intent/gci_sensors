#include <hardware/gpio.h>
#include <pico/binary_info.h>
#include <pico/stdlib.h>
#include <stdint.h>
#include <stdio.h>

#include "lis3mdl.h"
#include <picolibc.h>

constexpr pin_t i2c_scl = 1;
constexpr pin_t i2c_sda = 0;

int main() {
  stdio_init_all();
  wait_for_usb();

  int32_t speed = gci_i2c0_bus_init(I2C_400KHZ, i2c_sda, i2c_scl);

  printf(">> i2c instance: %u baud: %u\n", 0, speed);
  printf(">> i2c SDA: %u SCL: %u\n", i2c_sda, i2c_scl);
  bi_decl(bi_2pins_with_func(i2c_sda, i2c_scl, GPIO_FUNC_I2C)); // compile info

  printf("/// MAGNETOMETER START ///\n");

  lis3mdl_i2c_t *mag = NULL;
  while (true) {
    mag = lis3mdl_i2c_init(0, LIS3MDL_ADDRESS, RANGE_4GAUSS, ODR_155HZ);
    if (mag != NULL) break;
    printf("mag error\n");
    sleep_ms(1000);
  }

  while (true) {
    lis3mdl_t m = lis3mdl_read(mag);
    if (m.ok == false) continue;

    // printf("-----------------------------\n");
    printf("Mags: %f %f %f (normalized)\n", m.x, m.y, m.z);
  }
}