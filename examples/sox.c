#include "gci_sensors/lsm6dsox.h"
#include <hardware/gpio.h>
#include <pico/binary_info.h>
#include <pico/stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <tusb.h> // wait for USB

#define SCL  17
#define SDA  16
#define PORT 0
#define SCK  2
#define TX   3 // SDO
#define RX   4 // SDI
#define CS   11

lsm6dsox_io_t *imu = NULL;

void init_imu() {
  uint32_t cnt = 0;
  while (true) {
    int err = lsm6dsox_init(imu, LSM6DSOX_XL_16_G, LSM6DSOX_G_2000_DPS, LSM6DSOX_ODR_104_HZ);
    if (err == 0) break;
    printf("*** imu init error %u %d ***\n", cnt++, err);
    sleep_ms(2000);
  }

  printf("/// SPI ACCEL/GYRO START ///\n");
  printf(">> Init IMU:\n");
  printf(">> Core: %u\n", get_core_num());
}

int main() {
  stdio_init_all();
  while (!tud_cdc_connected()) {
    sleep_ms(100);
  }

#if 1
  uint32_t speed = gcis_spi0_init(GCIS_SPI_1MHZ, RX, TX, SCK);
  printf(">> SPI%u baudrate: %u\n", PORT, speed);
  printf(">> SDI: %u SDO: %u SCK: %u CS: %u\n", TX, RX, SCK, CS);
  bi_decl(bi_3pins_with_func(RX, TX, SCK, GPIO_FUNC_SPI));

  imu = lsm6dsox_create(SPI_INTERFACE, PORT, CS);
  bi_decl(bi_1pin_with_name(CS, "SPI CS"));
#else
  uint32_t speed = gcis_i2c_bus_init(PORT, GCIS_I2C_400KHZ, SDA, SCL);
  printf(">> I2C%u at %u bps\n", PORT, speed);
  printf(">> SDA: %u SCL: %u\n", SDA, SCL);
  bi_decl(bi_2pins_with_func(SDA, SCL, GPIO_FUNC_I2C)); // compile info

  imu = lsm6dsox_create(I2C_INTERFACE, PORT, LSM6DSOX_ADDRESS);
#endif
  init_imu();
  lsm6dsox_dump(imu);

  uint64_t prev = 0;

  while (1) {
    uint64_t now = time_us_64();
    imuf_t i;
    if (lsm6dsox_read(imu, &i) < 0) {
      printf("*** Bad read ***\n");
      sleep_ms(1000);
      continue;
    }

    uint64_t delta = now - prev;

    printf("-----------------------------\n");
    printf("Accels: %8.3f %8.3f %8.3f g\n", i.a.x, i.a.y, i.a.z);
    printf("Gyros:  %8.3f %8.3f %8.3f dps\n", i.g.x, i.g.y, i.g.z);
    printf("Temperature: %.1f C\n", i.temperature);
    printf("Timestamp: %llu msec\n", now);
    printf("Delta: %llu usec   %llu msec   %.1fHz\n", delta, delta / 1000, 1E6 / (float)delta);

    prev = now;

    sleep_ms(1000);
  }

  return 0;
}