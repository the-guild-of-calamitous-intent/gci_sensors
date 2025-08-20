#include "gci_sensors/icm42688p.h"
#include <hardware/gpio.h>
#include <pico/binary_info.h>
#include <pico/stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <tusb.h> // wait for USB

#define PORT 0
#define SCK  2
#define TX   3 // SDO
#define RX   4 // SDI
#define CS   11

icm42688p_io_t *imu = NULL;

int main() {
  stdio_init_all();
  while (!tud_cdc_connected())
    sleep_ms(100);

  uint32_t speed = gcis_spi0_init(GCIS_SPI_1MHZ, RX, TX, SCK);
  printf(">> SPI%u baudrate: %u\n", PORT, speed);
  printf(">> SDI: %u SDO: %u SCK: %u CS: %u\n", TX, RX, SCK, CS);
  bi_decl(bi_3pins_with_func(RX, TX, SCK, GPIO_FUNC_SPI));

  imu = icm42688p_create(SPI_INTERFACE, PORT, CS);
  imu->accel = ICM42688_ACCEL_16G;
  bi_decl(bi_1pin_with_name(CS, "SPI CS"));

  int err;
  while ((err = icm42688p_init(imu)) < 0) {
    printf("*** imu init error: %d ***\n", err);
    sleep_ms(2000);
  }

  printf("/// SPI ACCEL/GYRO START ///\n");
  printf(">> Init IMU:\n");
  printf(">> Core: %u\n", get_core_num());

  uint64_t prev = 0;

  while (1) {
    uint64_t now = time_us_64();
    imuf_t i;
    if (icm42688p_read(imu, &i) < 0) {
      printf("*** Bad read ***\n");
      sleep_ms(1000);
      continue;
    }

    uint64_t delta = now - prev;

    printf("-----------------------------\n");
    printf("Accels: %8.3f %8.3f %8.3f g\n", i.a.x, i.a.y, i.a.z);
    printf("Gyros:  %8.3f %8.3f %8.3f dps\n", i.g.x, i.g.y, i.g.z);
    printf("Temperature: %.1f C\n", i.temperature);
    printf("Timestamp: %llu msec\n", now / 1000);
    printf("Delta: %llu usec   %llu msec   %.1fHz\n", delta, delta / 1000, 1E6 / (float)delta);

    prev = now;

    sleep_ms(100);
  }

  return 0;
}