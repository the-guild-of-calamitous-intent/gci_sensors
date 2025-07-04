#include <hardware/gpio.h>
#include <pico/binary_info.h>
#include <pico/stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <tusb.h> // wait for USB

#include "gci_sensors/algorithms.h"
#include "gci_sensors/bmp390.h"

constexpr pin_t i2c_scl = 1;
constexpr pin_t i2c_sda = 0;

int main() {
  stdio_init_all();
  while (!tud_cdc_connected()) {
    sleep_ms(100);
  }

  int32_t speed = gcis_i2c_bus_init(0, GCIS_I2C_400KHZ, i2c_sda, i2c_scl);

  printf(">> i2c instance: %u baud: %u\n", 0, speed);
  printf(">> i2c SDA: %u SCL: %u\n", i2c_sda, i2c_scl);
  bi_decl(bi_2pins_with_func(i2c_sda, i2c_scl, GPIO_FUNC_I2C)); // compile info

  bmp390_io_t *pt = NULL;
  while (true) {
    pt = bmp390_i2c_init(0, BMP390_ADDRESS, ODR_50_HZ, IIR_FILTER_COEFF_3);
    if (pt != NULL) break;
    printf("bmp390 error\n");
    sleep_ms(1000);
  }

  printf("/// PRESSURE/TEMPERATURE START ///\n");

  while (true) {
    sleep_ms(500);

    bmp390_t i = bmp390_read(pt);
    if (pt->ok == false) {
      printf("*** Bad reading ***\n");
      continue;
    }

    float alt = pressure_altitude(i.pressure);

    // printf("-----------------------------\n");
    printf("Press: %8.1f Pa  Temp: %5.2f C  Altitude: %.3f m\n",
           i.pressure,
           i.temperature,
           alt);
  }
}