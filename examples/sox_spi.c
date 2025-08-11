#include "gci_sensors/lsm6dsox.h"
#include <hardware/gpio.h>
#include <pico/binary_info.h>
#include <pico/stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <tusb.h> // wait for USB

#define SCL 17
#define SDA 16
#define PORT 0
#define SCK         2
#define TX          3 // SDO
#define RX          4 // SDI
#define CS          11
// #define INT         15
// #define TYPE SPI_INTERFACE
// #define TYPE I2C_INTERFACE

// lsm6dsox_io_t imu_hw;
// lsm6dsox_io_t *imu = &imu_hw;

lsm6dsox_io_t *imu = NULL;


// volatile bool imu_ready = true;

// static
// void callback(uint pin, uint32_t event) {
//   if (event == GPIO_IRQ_EDGE_RISE && pin == INT) imu_ready = true;
//   // printf("> pin: %u   event: %u\n", pin, event);
// }

// bool is_gpio_interrupt_enabled(const uint gpio, const uint32_t event) {
//   uint offset = 0;
//   if (event == GPIO_IRQ_LEVEL_LOW) offset = 0;
//   else if (event == GPIO_IRQ_LEVEL_HIGH) offset = 1;
//   else if (event == GPIO_IRQ_EDGE_FALL) offset = 2;
//   else if (event == GPIO_IRQ_EDGE_RISE) offset = 3;

//   // Calculate the correct interrupt enable register
//   // PROC1_INTE0 covers GPIO 0-7, PROC1_INTE1 covers GPIO 8-15, etc.
//   const uint reg_index  = gpio / 8;                // Register index (0 for GPIO 0-7, 1 for 8-15, etc.)
//   const uint bit_offset = (gpio % 8) * 4 + offset; // Each GPIO has 4 bits (LEVEL_LOW, LEVEL_HIGH, EDGE_LOW, EDGE_HIGH)
//   volatile uint32_t reg = *(volatile uint32_t *)(IO_BANK0_BASE + IO_BANK0_PROC1_INTE0_OFFSET + (reg_index * 4));
//   return (reg & (1u << bit_offset)) != 0;
// }

void init_imu() {
  // imu = lsm6dsox_create(TYPE, PORT, CS);

  uint32_t cnt = 0;
  while (true) {
    // Grok says normal maneuvers (0.5-2G) and agressive maneuvers (3-6G)
    // and extreme flying (8-10G)
    int err = lsm6dsox_init(imu,LSM6DSOX_XL_16_G, LSM6DSOX_G_2000_DPS,LSM6DSOX_ODR_416_HZ);
    // printf("init_imu: imu: %d comm: %d\n", imu, imu->comm);
    // printf("post-lsm6dsox_spi_init\n");
    if (err == 0) break;
    printf("*** imu init error %u %d ***\n", cnt++, err);
    sleep_ms(2000);
  }

  bi_decl(bi_1pin_with_name(CS, "SPI CS"));
  printf("init_imu: imu: %d comm: %d\n", imu, imu->comm);

  // gpio_init(INT);
  // gpio_set_dir(INT, GPIO_IN); // Set as input
  // gpio_pull_down(INT);          // Enable internal pull-up resistor
  // gpio_set_irq_enabled_with_callback(INT, GPIO_IRQ_EDGE_RISE, true, callback);
  // bi_decl(bi_1pin_with_name(INT, "IMU INT1"));

  printf("/// SPI ACCEL/GYRO START ///\n");
  printf(">> Init IMU:\n");
  printf(">> Core: %u\n", get_core_num());
  // printf(">> INT %u: LOW[%d] HIGH[%d] RISE[%d] FALL[%d]\n",
  //   INT,
  //   is_gpio_interrupt_enabled(INT, GPIO_IRQ_LEVEL_LOW),
  //   is_gpio_interrupt_enabled(INT, GPIO_IRQ_LEVEL_HIGH),
  //   is_gpio_interrupt_enabled(INT, GPIO_IRQ_EDGE_RISE),
  //   is_gpio_interrupt_enabled(INT, GPIO_IRQ_EDGE_FALL));
  // printf(">> IRQ Priority: %u\n", irq_get_priority(INT));
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
#else
  uint32_t speed = gcis_i2c_bus_init(PORT, GCIS_I2C_400KHZ, SDA, SCL);
  printf(">> I2C%u at %u bps\n", PORT, speed);
  printf(">> SDA: %u SCL: %u\n", SDA, SCL);
  bi_decl(bi_2pins_with_func(SDA, SCL, GPIO_FUNC_I2C)); // compile info

  imu = lsm6dsox_create(I2C_INTERFACE, PORT, LSM6DSOX_ADDRESS);
#endif
  init_imu();
  // lsm6dsox_dump(imu);
  
  uint64_t prev = 0;
  uint64_t cnt  = 0;

  while (1) {
    // imu_ready = true;
    // if (imu_ready) {
    //   imu_ready = false;

      uint64_t now = time_us_64();
      imuf_t i;
      if (lsm6dsox_read(imu, &i) < 0) {
        printf("*** Bad read ***\n");
        sleep_ms(1000);
        continue;
      }

      uint64_t delta = now - prev;

      if (cnt++ % 3 == 0) {
        printf("-----------------------------\n");
        printf("Accels: %8.3f %8.3f %8.3f g\n", i.a.x, i.a.y, i.a.z);
        printf("Gyros:  %8.3f %8.3f %8.3f dps\n", i.g.x, i.g.y, i.g.z);
        printf("Temperature: %.1f C\n", i.temperature);
        printf("Timestamp: %llu msec\n", now);
        printf("Delta: %llu usec   %llu msec   %.1fHz\n", delta, delta / 1000, 1E6 / (float)delta);
      }
      prev = now;
    // }

    sleep_ms(1000);
  }

  return 0;
}