#include "gci_sensors/lsm6dsox.h"
#include <hardware/gpio.h>
#include <pico/binary_info.h>
#include <pico/stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <tusb.h> // wait for USB

#define SCK 2
#define TX 3 // SDO
#define RX 4 // SDI
#define CS 5
#define INT 15
#define BUFFER_SIZE 10
// constexpr float M_PIf = 3.14159265358979323846f;

lsm6dsox_io_t *imu = NULL;
// uint64_t prev      = 0;
// uint64_t cnt       = 0;
volatile bool imu_ready = true;

// void callback(uint pin, uint32_t event) {
//   imu_ready = true;
//   // printf("> pin: %u   event: %u\n", pin, event);
// }

static void my_irq_handler(void) {
  volatile uint32_t event = gpio_get_irq_event_mask(INT);
  // printf("irq event: %d\n", event);
  if (event & GPIO_IRQ_EDGE_RISE) {
    gpio_acknowledge_irq(INT, GPIO_IRQ_EDGE_RISE);
    // handle the IRQ
    imu_ready = true;
  }
}

bool is_gpio_interrupt_enabled(const uint gpio, const uint32_t event) {
  // // Read the interrupt enable register for the GPIO pin
  // uint32_t inten = *(volatile uint32_t *)(IO_BANK0_BASE + IO_BANK0_PROC1_INTE0_OFFSET + (gpio / 8) * 4);
  // uint shift = (gpio % 8) * 4; // Each GPIO has 4 bits (LEVEL_LOW, LEVEL_HIGH, EDGE_LOW, EDGE_HIGH)
  // return (inten & (1u << shift)) != 0;

  uint offset = 0;
  if (event == GPIO_IRQ_LEVEL_LOW) offset = 0;
  else if (event == GPIO_IRQ_LEVEL_HIGH) offset = 1;
  else if (event == GPIO_IRQ_EDGE_FALL) offset = 2;
  else if (event == GPIO_IRQ_EDGE_RISE) offset = 3;

  // Calculate the correct interrupt enable register
  // PROC1_INTE0 covers GPIO 0-7, PROC1_INTE1 covers GPIO 8-15, etc.
  const uint reg_index  = gpio / 8;                // Register index (0 for GPIO 0-7, 1 for 8-15, etc.)
  const uint bit_offset = (gpio % 8) * 4 + offset; // Each GPIO has 4 bits (LEVEL_LOW, LEVEL_HIGH, EDGE_LOW, EDGE_HIGH)
  volatile uint32_t reg = *(volatile uint32_t *)(IO_BANK0_BASE + IO_BANK0_PROC1_INTE0_OFFSET + (reg_index * 4));
  return (reg & (1u << bit_offset)) != 0;
}

void init_imu() {
  // gpio_set_irq_enabled_with_callback(INT, GPIO_IRQ_EDGE_RISE, true, callback);

  gpio_set_irq_enabled(INT, GPIO_IRQ_EDGE_RISE, true);
  gpio_add_raw_irq_handler(INT, my_irq_handler);
  irq_set_enabled(IO_IRQ_BANK0, true);

  // Initialize the GPIO pin
  // gpio_init(INT);
  // gpio_set_dir(INT, GPIO_IN); // Set as input
  // gpio_pull_up(INT);          // Enable internal pull-up resistor
  // gpio_set_irq_enabled_with_callback(pin, GPIO_IRQ_EDGE_FALL, true, callback);
  // gcis_spi_init_cs(CS, SPI_CS_PULLDOWN);
  bi_decl(bi_1pin_with_name(CS, "SPI CS"));
  bi_decl(bi_1pin_with_name(INT, "IMU INT1"));

  // uint32_t cnt       = 0;
  while (true) {
    imu = lsm6dsox_spi_init(
        0, CS,
        LSM6DSOX_XL_16_G, LSM6DSOX_G_2000_DPS,
        LSM6DSOX_ODR_416_HZ);
    // ODR_208_HZ);
    if (imu != NULL || imu->ok == true) break;
    printf("imu error\n");
    sleep_ms(2000);
  }

  printf("/// SPI ACCEL/GYRO START ///\n");
  printf(">> Init IMU:\n");
  printf(">> Core: %u\n", get_core_num());
  // printf(">> IRQ Enabled: %d\n", irq_is_enabled(INT));
  // printf(">> IRQ handler: %d\n", irq_has_handler(INT));
  // printf(">> IRQ Shared Handler: %d\n", irq_has_shared_handler(INT));
  // printf(">> NVIC IO_IRQ_BANK0 Enabled: %d\n", irq_is_enabled(IO_IRQ_BANK0));
  // printf(">> GPIO %u interrupt RISE enabled: %d\n", INT,is_gpio_interrupt_enabled(INT, GPIO_IRQ_EDGE_RISE));
  // printf(">> GPIO %u interrupt FALL enabled: %d\n", INT,is_gpio_interrupt_enabled(INT, GPIO_IRQ_EDGE_FALL));
  // printf(">> GPIO %u interrupt LOW enabled: %d\n", INT,is_gpio_interrupt_enabled(INT, GPIO_IRQ_LEVEL_LOW));
  // printf(">> GPIO %u interrupt HIGH enabled: %d\n", INT,is_gpio_interrupt_enabled(INT, GPIO_IRQ_LEVEL_HIGH));

  printf(">> INT %u: LOW[%d] HIGH[%d] RISE[%d] FALL[%d]\n",
         INT,
         is_gpio_interrupt_enabled(INT, GPIO_IRQ_LEVEL_LOW),
         is_gpio_interrupt_enabled(INT, GPIO_IRQ_LEVEL_HIGH),
         is_gpio_interrupt_enabled(INT, GPIO_IRQ_EDGE_RISE),
         is_gpio_interrupt_enabled(INT, GPIO_IRQ_EDGE_FALL));
  // printf(">> IRQ handler: %d\n", gpio_get_irq_enabled(INT, GPIO_IRQ_EDGE_RISE));
  // printf(">> IRQ Shared Handler: %d\n", irq_has_shared_handler(IO_IRQ_BANK0));
  printf(">> IRQ Priority: %u\n", irq_get_priority(INT));
}

void loop_read_imu() {
  uint64_t prev = 0;
  uint64_t cnt  = 0;

  while (1) {
    if (imu_ready) {
      imu_ready = false;

      uint64_t now = time_us_64();
      lsm6dsox_t i = lsm6dsox_read(imu);
      if (imu->ok == false) {
        printf("****************\n");
        printf("*** Bad read ***\n");
        printf("****************\n");
        sleep_ms(1);
        return;
      }

      uint64_t delta = now - prev;

      if (cnt++ % 30 == 0) {
        printf("-----------------------------\n");
        printf("Accels: %8.3f %8.3f %8.3f g\n", i.a.x, i.a.y, i.a.z);
        printf(" Gyros: %8.3f %8.3f %8.3f dps\n", i.g.x, i.g.y, i.g.z);
        printf("Temperature: %.1f C\n", i.temperature);
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

  uint32_t speed = gcis_spi0_init(GCIS_SPI_10MHZ, RX, TX, SCK);
  // gcis_spi_init_cs(CS);

  printf(">> spi instance: %u baudrate: %u\n", 0, speed);
  printf(">> spi SDI: %u SDO: %u SCK: %u CS: %u\n", TX, RX, SCK, CS);
  bi_decl(bi_3pins_with_func(RX, TX, SCK, GPIO_FUNC_SPI));

  init_imu();
  loop_read_imu();

  return 0;
}