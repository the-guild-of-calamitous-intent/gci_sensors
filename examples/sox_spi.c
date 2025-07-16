#include <hardware/gpio.h>
#include <pico/binary_info.h>
#include <pico/stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <tusb.h> // wait for USB
#include "gci_sensors/lsm6dsox.h"
// #include "gci_sensors/lis3mdl.h"

#define SCK 2
#define TX  3 // SDO
#define RX  4 // SDI
#define CS  5
#define INT 15
#define BUFFER_SIZE 10
// constexpr float M_PIf = 3.14159265358979323846f;

bool imu_ready = true;

void callback(uint pin, uint32_t event) {
  imu_ready = true;
  // printf("> pin: %u   event: %u\n", pin, event);
}

int main() {
  stdio_init_all();
  while (!tud_cdc_connected()) {
    sleep_ms(100);
  }

  uint32_t speed = gcis_spi0_init(GCIS_SPI_10MHZ, RX, TX, SCK);
  // gcis_spi_init_cs(CS);

  printf(">> spi instance: %u baudrate: %u\n", 0, speed);
  printf(">> spi SDI: %u SDO: %u SCK: %u CS: %u\n", TX, RX, SCK, CS);

  bi_decl(bi_3pins_with_func(RX, TX, SCK, GPIO_FUNC_SPI));
  bi_decl(bi_1pin_with_name(CS, "SPI CS"));

  // Initialize the GPIO pin
  // gpio_init(INT);
  // gpio_set_dir(INT, GPIO_IN); // Set as input
  // gpio_pull_up(INT);          // Enable internal pull-up resistor
  // gpio_set_irq_enabled_with_callback(pin, GPIO_IRQ_EDGE_FALL, true, callback);
  gcis_spi_init_cs(CS);
  bi_decl(bi_1pin_with_name(INT, "IMU INT1"));

  gpio_set_irq_enabled_with_callback(INT, GPIO_IRQ_EDGE_RISE, true, callback);

  lsm6dsox_io_t *imu = NULL;
  // uint32_t cnt       = 0;
  while (true) {
    imu = lsm6dsox_spi_init(
        0, CS,
        ACCEL_RANGE_16_G, GYRO_RANGE_2000_DPS,
        RATE_6660_HZ);
        // RATE_208_HZ);
    if (imu != NULL || imu->ok == true) break;
    printf("imu error\n");
    sleep_ms(2000);
  }

  printf("/// ACCEL/GYRO START ///\n");
  uint64_t prev = 0;
  uint64_t cnt = 0;

  while (true) {
    if (imu_ready) {
      imu_ready = false;

      uint64_t now = time_us_64();
      lsm6dsox_t i = lsm6dsox_read(imu);
      if (imu->ok == false) {
        printf("****************\n");
        printf("*** Bad read ***\n");
        printf("****************\n");
        sleep_ms(1000);
        break;
      }

      uint64_t delta = now - prev;

      if (cnt++ % 30 == 0) {
        printf("-----------------------------\n");
        printf("Accels: %8.3f %8.3f %8.3f g\n", i.a.x, i.a.y, i.a.z);
        printf(" Gyros: %8.3f %8.3f %8.3f dps\n", i.g.x, i.g.y, i.g.z);
        printf("Temperature: %.1f C\n", i.temperature);
        printf("Timestamp: %llu msec\n", now);
        printf("Delta: %llu usec   %llu msec   %.1fHz\n", delta, delta/1000, 1E6 / (float)delta);
      }
      prev = now;
    }
    // printf("\r");

    sleep_us(10);
    // tight_loop_contents(); // DON'T USE THIS
  }
}