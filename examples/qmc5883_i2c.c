#include <hardware/gpio.h>
#include <pico/binary_info.h>
#include <pico/stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <tusb.h> // wait for USB

#include "gci_sensors/qmc5883.h"

#define PORT 0
#define SDA  4
#define SCL  5

int main() {
  stdio_init_all();
  while (!tud_cdc_connected()) {
    sleep_ms(100);
  }

  int32_t speed = gcis_i2c_bus_init(PORT, GCIS_I2C_100KHZ, SDA, SCL);

  printf(">> i2c%u baud: %u\n", PORT, speed);
  printf(">> i2c SDA: %u SCL: %u\n", SDA, SCL);
  bi_decl(bi_2pins_with_func(SDA, SCL, GPIO_FUNC_I2C)); // compile info

  printf("/// MAGNETOMETER START ///\n");

  qmc5883_io_t *mag = NULL;
  while (true) {
    if (qmc5883_i2c_init(mag, 0, QMC5883_8GAUSS) == 0) break;
    printf("*** Couldn't initalize QMC5883 ***\n");
    sleep_ms(1000);
  }

  while (true) {
    sleep_ms(2000);

    vec3f_t m;
    if (qmc5883_read(mag, &m) < 0) {
      printf("*** bad reading ***\n");
      continue;
    }

    printf("Mags: %f %f %f\n", m.x, m.y, m.z);
    printf("Temp: %f C\n", m.temperature);
  }
}