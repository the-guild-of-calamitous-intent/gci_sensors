#include <hardware/gpio.h>
#include <pico/binary_info.h>
#include <pico/stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <tusb.h> // wait for USB

#include "gci_sensors/algorithms.h"
#include "gci_sensors/bmp390.h"

#define PORT 0
#define SCL  17
#define SDA  16

int main() {
  stdio_init_all();
  while (!tud_cdc_connected()) {
    sleep_ms(100);
  }

  int32_t speed = gcis_i2c_bus_init(PORT, GCIS_I2C_400KHZ, SDA, SCL);

  printf(">> i2c%u @ %u bps\n", PORT, speed);
  printf(">> i2c SDA: %u SCL: %u\n", SDA, SCL);
  bi_decl(bi_2pins_with_func(SDA, SCL, GPIO_FUNC_I2C)); // compile info

  bmp390_io_t *pt = NULL;
  while (true) {
    pt = bmp390_i2c_init(PORT, BMP390_ADDRESS, BMP390_ODR_50_HZ, BMP390_IIR_COEFF_3);
    if (pt != NULL) break;
    printf("bmp390 error\n");
    sleep_ms(1000);
  }

  printf("/// I2C PRESSURE/TEMPERATURE START ///\n");

  while (true) {
    sleep_ms(1000);

    pt_t i = bmp390_read(pt);
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