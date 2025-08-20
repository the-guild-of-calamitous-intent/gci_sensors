#include <hardware/gpio.h>
#include <pico/binary_info.h>
#include <pico/stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <tusb.h> // wait for USB

#include "gci_sensors/algorithms.h"
#include "gci_sensors/bmp390.h"

#define PORT 0
#define SCK  2
#define SDO  3 // SDO
#define SDI  4 // SDI
#define CS   13
#define INT  14

volatile bool bar_ready = false;

void callback(uint pin, uint32_t event) {
  if ((pin == INT) && (event & GPIO_IRQ_EDGE_RISE)) bar_ready = true;
  // printf("x\n");
  // printf("> GPIO %u %u\n", pin, events);
}

int main() {
  stdio_init_all();
  while (!tud_cdc_connected()) {
    sleep_ms(100);
  }

  uint32_t speed = gcis_spi0_init(GCIS_SPI_1MHZ, SDI, SDO, SCK);
  printf(">> spi instance: %u baudrate: %u\n", 0, speed);
  printf(">> spi SDI: %u SDO: %u SCK: %u CS: %u\n", SDO, SDI, SCK, CS);
  bi_decl(bi_3pins_with_func(SDI, SDO, SCK, GPIO_FUNC_SPI));
  bi_decl(bi_1pin_with_name(CS, "SPI CS"));

  bmp390_io_t *pt = bmp390_create(SPI_INTERFACE, PORT, CS);
  pt->odr = BMP390_ODR_50_HZ;
  pt->iir = BMP390_IIR_COEFF_3;
  
  while (bmp390_init(pt) < 0) {
    printf("bmp390 error\n");
    sleep_ms(1000);
  }

  gpio_init(INT);
  gpio_set_dir(INT, GPIO_IN);
  gpio_pull_down(INT);
  bi_decl(bi_1pin_with_name(INT, "SPI INT"));
  gpio_set_irq_enabled_with_callback(INT, GPIO_IRQ_EDGE_RISE, true, &callback);

  printf("/// SPI PRESSURE/TEMPERATURE START ///\n");

  while (true) {
    sleep_ms(10);
    if (bar_ready == false) continue;
    bar_ready = false;

    pt_t i;
    if (bmp390_read(pt, &i) < 0) {
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