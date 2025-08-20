// #include "hardware/spi.h"
#include "gci_sensors/algorithms.h"
#include "gci_sensors/lps22.h"
#include <hardware/gpio.h>
#include <pico/binary_info.h>
#include <pico/stdlib.h>
#include <stdio.h>
#include <tusb.h>

// screen /dev/tty.usbmodemXXXXXX
// ctrl-a-d

#define PORT 0
#define SCK  2
#define TX   3 // SDO
#define RX   4 // SDI
#define CS   13
#define INT  14

volatile bool bar_ready = false;

void callback(uint pin, uint32_t events) {
  if (pin == INT) bar_ready = true;
  // printf("> GPIO %u %u\n", pin, events);
}

int main() {
  stdio_init_all();
  while (!tud_cdc_connected()) {
    sleep_ms(100);
  }

  uint32_t speed = gcis_spi0_init(GCIS_SPI_1MHZ, RX, TX, SCK);

  bi_decl(bi_3pins_with_func(RX, TX, SCK, GPIO_FUNC_SPI));

  lps22_io_t *press = NULL;
  uint32_t cnt      = 0;
  while (true) {
    press = lps22_spi_init(0, CS, LPS22_ODR_1HZ);
    if (press != NULL) break;
    printf("imu error, loop: %u\n", cnt++);
    sleep_ms(1000);
  }
  bi_decl(bi_1pin_with_name(CS, "SPI CS"));

  gpio_init(INT);
  gpio_set_dir(INT, GPIO_IN); // Set as input
  gpio_pull_down(INT);        // Enable internal pull-up resistor
  gpio_set_irq_enabled_with_callback(INT, GPIO_IRQ_EDGE_RISE, true, callback);
  bi_decl(bi_1pin_with_name(INT, "INT"));

  printf("//--- PRES/TEMP START ---//\n");
  printf(">> SPI%u @ %u bps\n", PORT, speed);
  printf(">> SDI(RX): %u SDO(TX): %u SCK: %u CS: %u\n", RX, TX, SCK, CS);

  // Sealevel: 1,013.25 hPa
  while (1) {
    if (bar_ready) {
      bar_ready = false;
      pt_t pt   = lps22_read(press);
      float alt = pressure_altitude(pt.pressure * 100.0);
      printf("Press: %10.3f Pa  Temp: %10.3f C  Alt: %10.3f\n",
             pt.pressure,
             pt.temperature,
             alt);
    }

    sleep_ms(10);
  }

  return 0;
}