#include "gci_sensors/lis3mdl.h"
#include "gci_sensors/lsm6dsox.h"
#include <hardware/gpio.h>
#include <pico/binary_info.h>
#include <pico/stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <tusb.h> // wait for USB

constexpr pin_t SCK = 2;
constexpr pin_t TX  = 3; // SDO
constexpr pin_t RX  = 4; // SDI
constexpr pin_t CS  = 5;
constexpr pin_t INT = 15;

constexpr uint32_t BUFFER_SIZE = 10;
// constexpr float M_PIf = 3.14159265358979323846f;

int main() {
  stdio_init_all();
  while (!tud_cdc_connected()) {
    sleep_ms(100);
  }

  uint32_t speed = gcis_spi0_init(GCIS_SPI_1MHZ, RX, TX, SCK);
  // gcis_spi_init_cs(CS);

  printf(">> spi instance: %u baudrate: %u\n", 0, speed);
  printf(">> spi SDI: %u SDO: %u SCK: %u CS: %u\n", TX, RX, SCK, CS);

  bi_decl(bi_3pins_with_func(RX, TX, SCK, GPIO_FUNC_SPI));
  bi_decl(bi_1pin_with_name(CS, "SPI CS"));

  // Initialize the GPIO pin
  gpio_init(INT);
  gpio_set_dir(INT, GPIO_IN); // Set as input
  gpio_pull_up(INT);          // Enable internal pull-up resistor
  bi_decl(bi_1pin_with_name(INT, "IMU INT1"));

  lsm6dsox_io_t *imu = NULL;
  uint32_t cnt       = 0;
  while (true) {
    imu = lsm6dsox_spi_init(
        0, CS,
        ACCEL_RANGE_16_G, GYRO_RANGE_2000_DPS,
        RATE_208_HZ);
    if (imu != NULL || imu->ok == true) break;
    printf("imu error, loop: %u\r", cnt++);
    sleep_ms(2000);
  }

  printf("/// ACCEL/GYRO START ///\n");

  while (true) {
    lsm6dsox_t i = lsm6dsox_read(imu);
    if (imu->ok == false) {
      printf("*** Bad read ***\n");
      sleep_ms(1000);
      continue;
    }

    printf("-----------------------------\n");
    printf("Accels: %f %f %f g\n", i.a.x, i.a.y, i.a.z);
    printf("Gyros: %f %f %f dps\n", i.g.x, i.g.y, i.g.z);
    printf("Temperature: %f C\n", i.temperature);
    printf("Timestamp: %llu msec\n", time_us_64());

    sleep_ms(500);
  }
}