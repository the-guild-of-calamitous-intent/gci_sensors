#include "pa1010d.h"
#include "pmtk.h"
#include <hardware/gpio.h>
#include <pico/binary_info.h>
#include <pico/stdlib.h>
#include <picolibc.h>
#include <stdio.h>

constexpr pin_t i2c_scl = 1;
constexpr pin_t i2c_sda = 0;

int main() {
  stdio_init_all();
  wait_for_usb();

  int32_t speed = gci_i2c0_bus_init(I2C_400KHZ, i2c_sda, i2c_scl);

  pa1010d_i2c_t *gps = pa1010d_i2c_init(0, PA1010D_ADDRESS);

  printf(">> i2c instance: %u baud: %u\n", 0, speed);
  printf(">> i2c SDA: %u SCL: %u\n", i2c_sda, i2c_scl);
  bi_decl(bi_2pins_with_func(i2c_sda, i2c_scl, GPIO_FUNC_I2C)); // compile info

  printf("/// GPS PA1010D START ///\n");

  pa1010d_write(gps, PMTK_FULL_POWER, sizeof(PMTK_FULL_POWER));
  pa1010d_write(gps, PMTK_RMCGGAGSA, sizeof(PMTK_RMCGGAGSA));

  char nema[250];

  while (1) {
    uint32_t num = pa1010d_read(gps, nema, sizeof(nema));
    if (num > 0) printf("GPS[%ld]: %s\n", num, nema);
    else printf("*** Bad read ***\n");
    sleep_ms(100);
  }
}