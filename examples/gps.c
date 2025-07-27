
#include "gci_sensors/pa1010d.h"
#include "gci_sensors/pmtk.h"
#include <hardware/gpio.h>
#include <pico/binary_info.h>
#include <pico/stdlib.h>
#include <stdio.h>
#include <tusb.h> // wait for USB

#define i2c_scl 1
#define i2c_sda 0

int main() {
  stdio_init_all();
  while (!tud_cdc_connected()) {
    sleep_ms(100);
  }

  int32_t speed = gcis_i2c_bus_init(0, GCIS_I2C_400KHZ, i2c_sda, i2c_scl);

  pa1010d_io_t *gps = pa1010d_i2c_init(0);

  printf(">> i2c instance: %u baud: %u\n", 0, speed);
  printf(">> i2c SDA: %u SCL: %u\n", i2c_sda, i2c_scl);
  bi_decl(bi_2pins_with_func(i2c_sda, i2c_scl, GPIO_FUNC_I2C)); // compile info

  printf("/// GPS PA1010D START ///\n");

  pa1010d_write(gps, PMTK_FULL_POWER, sizeof(PMTK_FULL_POWER));
  pa1010d_write(gps, PMTK_RMCGGAGSA, sizeof(PMTK_RMCGGAGSA));

  char nema[250];

  while (1) {
    int32_t num = pa1010d_read(gps, nema, sizeof(nema));
    if (num > 0) printf("GPS[%d]: %s\n", num, nema);
    else printf("*** Bad read ***\n");
    sleep_ms(1000);
  }
}