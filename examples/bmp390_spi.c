#include <hardware/gpio.h>
#include <pico/binary_info.h>
#include <pico/stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <tusb.h> // wait for USB

#include "gci_sensors/algorithms.h"
#include "gci_sensors/bmp390.h"

#define PORT        0
#define SCK         2
#define TX          3 // SDO
#define RX          4 // SDI
#define CS          5
#define INT         15
#define BUFFER_SIZE 10

int main() {
  stdio_init_all();
  while (!tud_cdc_connected()) {
    sleep_ms(100);
  }

  uint32_t speed = gcis_spi0_init(GCIS_SPI_10MHZ, RX, TX, SCK);
  printf(">> spi instance: %u baudrate: %u\n", 0, speed);
  printf(">> spi SDI: %u SDO: %u SCK: %u CS: %u\n", TX, RX, SCK, CS);
  bi_decl(bi_3pins_with_func(RX, TX, SCK, GPIO_FUNC_SPI));

  bmp390_io_t *pt = NULL;
  while (true) {
    pt = bmp390_spi_init(PORT, CS, BMP390_ODR_50_HZ, BMP390_IIR_COEFF_3);
    if (pt != NULL) break;
    printf("bmp390 error\n");
    sleep_ms(1000);
  }

  printf("/// SPI PRESSURE/TEMPERATURE START ///\n");

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