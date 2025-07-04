// #include "hardware/spi.h"
#include <pico/binary_info.h>
#include <pico/stdlib.h>
#include <tusb.h>
#include <stdio.h>
#include "gci_sensors/algorithms.h"
#include "gci_sensors/lps22.h"

// screen /dev/tty.usbmodemXXXXXX
// ctrl-a-d

constexpr pin_t RX             = 0;
constexpr pin_t CS             = 1;
constexpr pin_t SCK            = 2;
constexpr pin_t TX             = 3;

int main() {
  stdio_init_all();
  while (!tud_cdc_connected()) {
    sleep_ms(100);
  }

  uint32_t speed = gcis_spi_bus_init(0, GCIS_SPI_1MHZ, RX, TX, SCK);
  gcis_spi_init_cs(CS);

  printf(">> spi instance: %u baudrate: %u\n", 0, speed);
  printf(">> spi MISO: %u MOSI: %u SCK: %u CS: %u\n", TX, RX, SCK, CS);

  bi_decl(bi_3pins_with_func(RX, TX, SCK, GPIO_FUNC_SPI));
  bi_decl(bi_1pin_with_name(CS, "SPI CS"));

  lps22_io_t *press = NULL;
  while (true) {
    press = lps22_spi_init(0, CS, LPS22_ODR_1HZ);
    if (press != NULL) break;
    printf("lp22 error\n");
    sleep_ms(1000);
  }

  printf("/// PRES/TEMP START ///\n");

  // Sealevel: 1,013.25 hPa
  while (1) {
    lps22_t pt = lps22_read(press);
    float alt  = pressure_altitude(pt.pressure);
    printf("\nPress: %10.3f Temp: %10.3f Alt: %10.3f %lu\n",
           pt.pressure,
           pt.temperature);

    sleep_ms(100);
  }

  return 0;
}