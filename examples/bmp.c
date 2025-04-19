#include <hardware/gpio.h>
#include <pico/binary_info.h>
#include <pico/stdlib.h>
#include <picolibc.h>
#include <stdint.h>
#include <stdio.h>

#include "algorithms.h"
#include "bmp390.h"

constexpr pin_t i2c_scl = 1;
constexpr pin_t i2c_sda = 0;

int main() {
  stdio_init_all();
  wait_for_usb();

  int32_t speed = gci_i2c0_bus_init(I2C_400KHZ, i2c_sda, i2c_scl);

  printf(">> i2c instance: %u baud: %u\n", 0, speed);
  printf(">> i2c SDA: %u SCL: %u\n", i2c_sda, i2c_scl);
  bi_decl(bi_2pins_with_func(i2c_sda, i2c_scl, GPIO_FUNC_I2C)); // compile info

  printf("/// Accel/Gyros START ///\n");

  bmp390_i2c_t *pt = NULL;
  while (true) {
    pt = bmp390_i2c_init(0, BMP390_ADDRESS, ODR_50_HZ, IIR_FILTER_COEFF_3);
    if (pt != NULL) break;
    printf("bmp390 error\n");
    sleep_ms(1000);
  }

  while (true) {
    bmp390_t i = bmp390_read(pt);
    if (i.ok == false) continue;

    float alt = pressure_altitude(i.pressure);

    // printf("-----------------------------\n");
    printf("Press: %8.1f Pa  Temp: %5.2f C  Altitude: %.3f m\n",
           i.pressure,
           i.temperature,
           alt);
  }
}