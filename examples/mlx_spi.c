#include "gci_sensors/mlx90393.h"
#include <hardware/gpio.h>
#include <pico/binary_info.h>
#include <pico/stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <tusb.h> // wait for USB

#define SCK         2
#define SDO         3 // data out
#define SDI         4 // data in
#define CS          5
#define INT         15
#define BUFFER_SIZE 10
// constexpr float M_PIf = 3.14159265358979323846f;

mlx90393_io_t *mag      = NULL;
volatile bool mag_ready = true;

void callback(uint pin, uint32_t event) {
  if ((pin == INT) && (event & GPIO_IRQ_EDGE_RISE)) mag_ready = true;
  // printf("> pin: %u   event: %u\n", pin, event);
}

void init_mag() {
  gpio_init(INT);
  gpio_set_dir(INT, GPIO_IN); // Set as input
  gpio_pull_down(INT);
  gpio_set_irq_enabled_with_callback(INT, GPIO_IRQ_EDGE_RISE, true, callback);

  bi_decl(bi_1pin_with_name(INT, "MAG INT"));

  while (true) {
    mag = mlx90393_spi_init(0, CS);
    if (mag != NULL || mag->ok == true) break;
    printf("mag error\n");
    sleep_ms(1000);
  }

  printf("//---- SPI MAGS START -----//\n");
}

void loop_read_mag() {
  uint64_t prev = 0;
  uint64_t cnt  = 0;

  while (1) {
    if (mag_ready) {
      mag_ready = false;

      uint64_t now = time_us_64();
      vec3f_t i    = mlx90393_read(mag);
      if (mag->ok == false) {
        printf("****************\n");
        printf("*** Bad read ***\n");
        printf("****************\n");
        sleep_ms(1);
        return;
      }

      uint64_t delta = now - prev;

      if (cnt++ % 30 == 0) {
        printf("-----------------------------\n");
        printf("Mags: %8.3f %8.3f %8.3f g\n", i.x, i.y, i.z);
        printf("Timestamp: %llu msec\n", now);
        printf("Delta: %llu usec   %llu msec   %.1fHz\n", delta, delta / 1000, 1E6 / (float)delta);
      }
      prev = now;
    }

    sleep_us(10);
  }
}

int main() {
  stdio_init_all();
  while (!tud_cdc_connected()) {
    sleep_ms(100);
  }

  uint32_t speed = gcis_spi0_init(GCIS_SPI_10MHZ, SDI, SDO, SCK);

  printf(">> spi instance: %u baudrate: %u\n", 0, speed);
  printf(">> spi SDI: %u SDO: %u SCK: %u CS: %u\n", SDO, SDI, SCK, CS);
  bi_decl(bi_3pins_with_func(SDI, SDO, SCK, GPIO_FUNC_SPI));
  bi_decl(bi_1pin_with_name(CS, "SPI CS"));

  init_mag();
  loop_read_mag();

  return 0;
}